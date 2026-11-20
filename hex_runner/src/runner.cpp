#include <hex_runner/runner.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace hex_runner {
double MonotonicNow() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
std::string JsonText(const Json::Value& value) { Json::StreamWriterBuilder writer; writer["indentation"]=""; return Json::writeString(writer,value); }
std::string PhaseName(Phase phase) {
  switch(phase) {
    case Phase::Waiting:return "WAITING"; case Phase::Initializing:return "INITIALIZING";
    case Phase::Ready:return "READY"; case Phase::Running:return "RUNNING";
    case Phase::Paused:return "PAUSED"; case Phase::Completed:return "COMPLETED"; case Phase::Fault:return "FAULT";
  }
  return "";
}
void FeedbackCache::Motor(const JointMatrix& q,const JointMatrix& tau,uint64_t sample,double now,bool valid) {
  const auto previous=status_.find("motor");
  if (valid && previous!=status_.end() && previous->second.sample==sample) return;
  value_.q.leftCols<3>()=q; value_.tau=tau; value_.motor_sample=sample;
  status_["motor"]={now,valid,sample};
}
void FeedbackCache::Servo(const Ratios& q,double now,bool valid) { value_.q.col(3)=q; status_["servo"]={now,valid,0}; }
void FeedbackCache::Magnetic(const Ratios& attached,const Flags& enabled,double now,bool valid) {
  value_.attached=attached; value_.enabled=enabled; status_["magnetic"]={now,valid,0};
}
void FeedbackCache::Gravity(const Vec3& direction,double now,bool valid) { value_.gravity=direction; status_["gravity"]={now,valid,0}; }
std::optional<Feedback> FeedbackCache::Snapshot(double now,double timeout,std::string& reason,Json::Value& ages) const {
  ages=Json::Value(Json::objectValue); reason.clear();
  for (const auto& item:status_) ages[item.first]=std::max(0.,now-item.second.received);
  for (const char* name:{"motor","servo","magnetic","gravity"}) {
    const auto it=status_.find(name);
    if (it==status_.end()) { reason=std::string("missing ")+name; return {}; }
    if (!it->second.valid) { reason=std::string("invalid ")+name; return {}; }
    if (std::max(0.,now-it->second.received)>timeout) { reason=std::string("stale ")+name; return {}; }
  }
  return value_;
}
bool FeedbackCache::MotorReady(double now,double timeout) const {
  const auto it=status_.find("motor"); return it!=status_.end() && it->second.valid && std::max(0.,now-it->second.received)<=timeout;
}
Runner::Runner(const RunnerConfig& c):config(c),kin(c.robot_urdf),expert(kin,c.expert) {
  initial_q_=kin.InitialJoints(c.initial_values,c.initial_feet,c.ik_branch);
  expert.SetInit(initial_q_); expert.state.adhesions=c.magnetic;
  ExportSimInfo(); std::cout << "[RUNNER] WAITING：等待初始化" << std::endl;
}
Command Runner::PrepareInitialization() {
  auto goal=expert.SetInit(initial_q_); expert.state.adhesions=config.magnetic;
  goal.adhesions=config.magnetic; goal.servo_enabled.fill(true); gait_switch_count_=0;
  ExportSimInfo(); return goal;
}
void Runner::ExportSimInfo() const {
  if (config.to_sim_info.empty()) return;
  YAML::Node data;
  for (const auto& name:leg_names) data["leg_names"].push_back(name);
  data["expert_type"]="climb"; data["robot_urdf"]=config.robot_urdf;
  auto append_matrix=[](YAML::Node node,const auto& m) {
    for (int i=0;i<m.rows();++i) { YAML::Node row; for (int j=0;j<m.cols();++j) row.push_back(m(i,j)); node.push_back(row); }
  };
  append_matrix(data["initial_state"]["W_T_R"],config.initial_pose);
  append_matrix(data["initial_state"]["q"],initial_q_);
  for (bool v:config.magnetic) data["initial_state"]["magnetic"].push_back(v);
  data["initial_state"]["servo_enabled"]=true;
  data["reference_path"]=YAML::Node(YAML::NodeType::Sequence);
  const auto parent=std::filesystem::path(config.to_sim_info).parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);
  std::ofstream file(config.to_sim_info);
  if (!file) throw std::runtime_error("Cannot write initial-state YAML: "+config.to_sim_info);
  file << data << '\n';
}
void Runner::SetPhase(Phase next,const std::string& reason) {
  if (phase!=next || !reason.empty()) {
    Json::Value event; event["phase"]=PhaseName(next); event["reason"]=reason; events_.append(event);
    std::cout << "[RUNNER] " << PhaseName(phase) << " -> " << PhaseName(next);
    if (!reason.empty()) std::cout << ": " << reason;
    std::cout << std::endl;
  }
  phase=next;
}
std::pair<bool,std::string> Runner::Request(const std::string& action,bool ready) {
  if (action=="disable_actuators" || action=="release_magnets") {
    pending_disable_ |= action=="disable_actuators"; pending_release_ |= action=="release_magnets";
    return {true,action+" queued"};
  }
  if (!pending_.empty()) return {false,"another operation is pending"};
  if (action=="start" || action=="pause") return {false,action+" applies only to complex"};
  if (action!="reset") return {false,"unknown operation"};
  if (!ready && !config.staged) return {false,"device feedback is not ready"};
  pending_=action; return {true,action+" queued"};
}
bool Runner::SetVelocity(const Vec3& v) {
  if (!v.allFinite()) throw std::invalid_argument("Velocity must be finite");
  if (motor_servo_disabled || magnetic_forced_off || (phase!=Phase::Ready && phase!=Phase::Running)) return false;
  velocity=v; return true;
}
void Runner::Tick(const std::optional<Feedback>& feedback,const std::string& unavailable) {
  output=OutputEvent::None;
  try {
    // 显式失能/释放优先处理，不通过力矩混合延迟执行。
    if (pending_disable_ || pending_release_) {
      if (pending_disable_) output=OutputEvent::Disable;
      motor_servo_disabled |= pending_disable_; magnetic_forced_off |= pending_release_;
      pending_disable_=pending_release_=false; pending_.clear(); initialization.reset(); velocity.setZero();
      if (phase!=Phase::Fault) SetPhase(command ? Phase::Paused : Phase::Waiting);
      return;
    }
    const auto action=pending_; pending_.clear();
    if (action=="reset") {
      if (config.staged) {
        Command goal; goal.q=initial_q_; goal.adhesions=config.magnetic; goal.servo_enabled.fill(true);
        initialization.emplace(config,expert,goal,expert.state); command.reset(); gait_switch_count_=0;
      } else {
        if (!feedback) throw std::runtime_error("reset lost feedback: "+unavailable);
        command=PrepareInitialization(); initialization.reset();
      }
      motor_servo_disabled=magnetic_forced_off=false; velocity.setZero(); failure_reason.clear();
      if (config.staged) SetPhase(Phase::Initializing);
      else { last_motor_sample_=feedback->motor_sample; ++reset_count; SetPhase(Phase::Ready); output=OutputEvent::Reset; return; }
    }
    if (phase==Phase::Initializing) {
      const auto result=initialization->Step(feedback,unavailable);
      if (result.command && result.command->tau.cwiseAbs().maxCoeff()>config.expert.torque_limit+1e-6)
        throw std::runtime_error("initialization exceeded configured torque limit");
      command=result.command; output=result.output;
      if (result.complete) { ++reset_count; last_motor_sample_=feedback->motor_sample; SetPhase(Phase::Ready); }
      return;
    }
    if (!command || motor_servo_disabled || magnetic_forced_off) return;
    if (!feedback && phase!=Phase::Fault) throw std::runtime_error(unavailable.empty() ? "device feedback unavailable" : unavailable);
    if (phase==Phase::Fault || phase==Phase::Paused || phase==Phase::Completed) return;
    SetPhase((velocity.array()!=0).any() ? Phase::Running : Phase::Ready);
    if (phase!=Phase::Running || feedback->motor_sample==last_motor_sample_) return;
    last_motor_sample_=feedback->motor_sample;
    const int old=expert.state.stance_group;
    const auto next=expert.RequestSingleStep(feedback->gravity,feedback->q,feedback->tau,feedback->attached,velocity);
    gait_switch_count_+=old!=expert.state.stance_group;
    if (!next.q.allFinite() || !next.tau.allFinite() || next.tau.cwiseAbs().maxCoeff()>config.expert.torque_limit+1e-6)
      throw std::runtime_error("expert exceeded configured torque limit or produced invalid command");
    command=next; ++compute_count; output=OutputEvent::Compute;
  } catch(const std::exception& error) {
    output=OutputEvent::None; failure_reason=error.what(); SetPhase(Phase::Fault,failure_reason);
  }
}
Json::Value Runner::Diagnostics() const {
  Json::Value d;
  d["motor_servo_disabled"]=motor_servo_disabled; d["magnetic_forced_off"]=magnetic_forced_off;
  d["expert_type"]="climb"; d["pose_source"]="gravity"; d["implementation"]="cpp";
  d["operating_mode"]=hex_expert::ModeName(config.expert.operating_mode);
  d["failure_reason"]=failure_reason; d["reset_count"]=Json::UInt64(reset_count); d["compute_count"]=Json::UInt64(compute_count);
  d["command"]=ToJson(velocity); Flags disabled{};
  d["servo_enabled"]=motor_servo_disabled ? ToJson(disabled) : command ? ToJson(command->servo_enabled) : Json::Value();
  d["gaits"]=ToJson(expert.state.gaits); d["adhesions"]=ToJson(expert.state.adhesions);
  d["gait_switch_count"]=gait_switch_count_; d["leg_names"]=ToJson(leg_names);
  d["estimated_contact_force"]=ToJson(expert.state.contact_force); d["normal_foot_velocity"]=ToJson(expert.state.normal_velocity);
  d["swing_reached"]=ToJson(expert.state.swing_reached); d["contact_count"]=ToJson(expert.state.contact_count);
  d["foot_target_leg"]=ToJson(expert.state.targets); d["foot_measured_leg"]=ToJson(expert.state.feet);
  d["workspace_ok"]=ToJson(expert.CheckWorkspace(expert.state.targets)); d["clearance_ok"]=ToJson(expert.CheckLegClearance(expert.state.targets));
  if (initialization) { const auto init=initialization->Diagnostics(); for (const auto& key:init.getMemberNames()) d[key]=init[key]; }
  return d;
}
Json::Value Runner::Report() const {
  Json::Value r; r["phase"]=PhaseName(phase); r["reset_count"]=Json::UInt64(reset_count);
  r["compute_count"]=Json::UInt64(compute_count); r["failure_reason"]=failure_reason; r["events"]=events_; r["expert"]=Diagnostics(); return r;
}
}  // namespace hex_runner
