#include <hex_runner/runner.hpp>
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace hex_runner {
std::string StageName(InitStage stage) {
  switch(stage) {
    case InitStage::WaitFeedback:return "WAIT_FEEDBACK"; case InitStage::WaitRelease:return "WAIT_RELEASE";
    case InitStage::Lift:return "LIFT"; case InitStage::MoveTarget:return "MOVE_TARGET";
    case InitStage::WaitAttach:return "WAIT_ATTACH"; case InitStage::Complete:return "COMPLETE";
  }
  return "";
}
Initialization::Initialization(const RunnerConfig& config,ExpertClimb& expert,const Command& goal,const ClimbState& target)
    : config_(config),expert_(expert),goal_(goal),target_state_(target) {
  waiting_reason_="waiting for fresh motor/servo/magnetic/pose feedback";
}
void Initialization::SetStage(InitStage next) {
  if (next!=stage) std::cout << "[INIT] " << StageName(next) << std::endl;
  stage=next;
}
void Initialization::Segment(const Angles& from,const Angles& target,InitStage next) {
  start_=from; target_=target;
  // 五次 smoothstep 的最大导数为 1.875，所有关节共用时长保证峰值速度 ≤ 1 rad/s。
  duration=std::max(config_.expert.control_dt,1.875*(target-start_).cwiseAbs().maxCoeff());
  elapsed=0; confirmed=0; SetStage(next);
}
bool Initialization::Arrived(const Angles& q) {
  motor_error_=(q.leftCols<3>()-target_.leftCols<3>()).cwiseAbs().maxCoeff();
  servo_error_=(q.col(3)-target_.col(3)).cwiseAbs().maxCoeff();
  confirmed=(*motor_error_<=.1 && *servo_error_<=.1 && elapsed>=duration) ? confirmed+1 : 0;
  return confirmed>=10;
}
Initialization::Result Initialization::Step(const std::optional<Feedback>& f,const std::string& unavailable) {
  if (stage==InitStage::WaitFeedback || stage==InitStage::WaitRelease) {
    if (!zero_sent_) { zero_sent_=true; if (f) zero_sample_=f->motor_sample; return {{},OutputEvent::Zero,false}; }
    if (!f) { waiting_reason_=unavailable.empty() ? "waiting for complete fresh feedback" : unavailable; return {command_,OutputEvent::Zero,false}; }
    if (zero_sample_ && f->motor_sample==*zero_sample_) {
      waiting_reason_="waiting for a new motor sample after zero torque"; return {command_,OutputEvent::Zero,false};
    }
    SetStage(InitStage::WaitRelease); waiting_legs_.clear();
    for (int i=0;i<6;++i) if (!(static_cast<float>(f->attached[i])<static_cast<float>(config_.expert.magnetic_release_below)) || f->enabled[i]) waiting_legs_.push_back(i);
    waiting_reason_=waiting_legs_.empty() ? "" : "waiting for all magnets to release";
    if (!waiting_legs_.empty()) return {command_,OutputEvent::Zero,false};
    expert_.SetInit(goal_.q); expert_.state.adhesions=goal_.adhesions;
    target_state_=expert_.state;
    // 仅从实测姿态起步；模型和目标在 reset 时已准备好。
    const auto lift=expert_.kin.InitialJoints(expert_.SwingStartLeg(),true,config_.ik_branch);
    Segment(f->q,lift,InitStage::Lift);
    Command out; out.q=f->q; out.servo_enabled.fill(true); command_=out;
    return {command_,OutputEvent::Initialize,false};
  }
  if (!f) throw std::runtime_error("initialization lost feedback: "+unavailable);
  // 每 tick 输出，不能因电机反馈恰好尚未更新而丢失插值周期。
  if (stage==InitStage::Lift || stage==InitStage::MoveTarget) {
    elapsed=std::min(duration,elapsed+config_.expert.control_dt);
    const double u=elapsed/duration,blend=10*u*u*u-15*u*u*u*u+6*u*u*u*u*u;
    Command out; out.q=start_+(target_-start_)*blend; out.servo_enabled.fill(true);
    if (stage==InitStage::MoveTarget) out.tau=expert_.InitializationFeedforward(f->gravity,f->q,goal_.adhesions);
    command_=out; waiting_reason_="waiting for measured joint targets";
    if (Arrived(f->q)) {
      if (stage==InitStage::Lift) Segment(out.q,goal_.q,InitStage::MoveTarget);
      else { SetStage(InitStage::WaitAttach); command_->adhesions=goal_.adhesions; confirmed=0; }
    }
    return {command_,OutputEvent::Initialize,false};
  }
  if (stage==InitStage::WaitAttach) {
    waiting_legs_.clear();
    for (int i=0;i<6;++i) {
      const float ratio=f->attached[i];
      const bool ok=goal_.adhesions[i] ? ratio>static_cast<float>(config_.expert.magnetic_attach_above) && f->enabled[i] :
        ratio<static_cast<float>(config_.expert.magnetic_release_below) && !f->enabled[i];
      if (!ok) waiting_legs_.push_back(i);
    }
    waiting_reason_=waiting_legs_.empty() ? "" : "waiting for configured magnet feedback";
    command_=goal_; command_->tau=expert_.InitializationFeedforward(f->gravity,f->q,goal_.adhesions);
    const bool arrived=Arrived(f->q);
    if (waiting_legs_.empty() && arrived) {
      expert_.state=target_state_;
      // 复用同一前馈混合状态，从实际保持力矩平滑进入运动阶段。
      expert_.SeedFeedforward(command_->tau);
      SetStage(InitStage::Complete); waiting_reason_.clear(); return {command_,OutputEvent::Initialize,true};
    }
    return {command_,OutputEvent::Initialize,false};
  }
  return {command_,OutputEvent::None,true};
}
Json::Value Initialization::Diagnostics() const {
  Json::Value d; d["initialization_stage"]=StageName(stage); d["initialization_waiting_reason"]=waiting_reason_;
  Json::Value legs(Json::arrayValue); for (int i:waiting_legs_) legs.append(i);
  d["initialization_waiting_legs"]=legs; d["initialization_confirm_samples"]=confirmed;
  d["initialization_motor_error"]=motor_error_ ? Json::Value(*motor_error_) : Json::Value();
  d["initialization_servo_error"]=servo_error_ ? Json::Value(*servo_error_) : Json::Value();
  return d;
}
}  // namespace hex_runner
