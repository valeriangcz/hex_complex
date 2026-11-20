#pragma once
#include <hex_expert/expert_climb.hpp>
#include <jsoncpp/json/json.h>
#include <map>
#include <optional>

namespace hex_runner {
using namespace hex_kinematic;
using hex_expert::ClimbState;
using hex_expert::ExpertClimb;
using Command = hex_expert::ExpertOutput;

struct RunnerConfig {
  hex_expert::ClimbParameters expert;
  double magnetic_hz=20,feedback_timeout=.5,motor_kp=60,motor_kd=1;
  bool staged=false,initial_feet=false;
  std::string robot_urdf,to_sim_info,data_root;
  Vec3 velocity_scales=Vec3::Ones();
  Eigen::Matrix4d initial_pose=Eigen::Matrix4d::Identity();
  JointMatrix initial_values=JointMatrix::Zero();
  Flags magnetic{};
  std::array<int,6> ik_branch{};
  static RunnerConfig Load(const std::string& filename,const std::string& data_root="",const std::string& sim_info="");
};

struct Feedback {
  Angles q=Angles::Zero();
  JointMatrix tau=JointMatrix::Zero();
  Ratios attached=Ratios::Zero();
  Flags enabled{};
  Vec3 gravity=Vec3(0,0,-1);
  uint64_t motor_sample=0;
};
struct SampleStatus { double received=0; bool valid=false; uint64_t sample=0; };
class FeedbackCache {
 public:
  void Motor(const JointMatrix& q,const JointMatrix& tau,uint64_t sample,double now,bool valid);
  void Servo(const Ratios& q,double now,bool valid);
  void Magnetic(const Ratios& attached,const Flags& enabled,double now,bool valid);
  void Gravity(const Vec3& direction,double now,bool valid);
  std::optional<Feedback> Snapshot(double now,double timeout,std::string& reason,Json::Value& ages) const;
  bool MotorReady(double now,double timeout) const;
 private:
  Feedback value_;
  std::map<std::string,SampleStatus> status_;
};

enum class Phase { Waiting,Initializing,Ready,Running,Paused,Completed,Fault };
enum class OutputEvent { None,Zero,Initialize,Reset,Compute,Disable };
enum class InitStage { WaitFeedback,WaitRelease,Lift,MoveTarget,WaitAttach,Complete };
std::string PhaseName(Phase phase);
std::string StageName(InitStage stage);

class Initialization {
 public:
  struct Result { std::optional<Command> command; OutputEvent output=OutputEvent::None; bool complete=false; };
  Initialization(const RunnerConfig& config,ExpertClimb& expert,const Command& goal,const ClimbState& target);
  Result Step(const std::optional<Feedback>& feedback,const std::string& unavailable);
  Json::Value Diagnostics() const;
  InitStage stage=InitStage::WaitFeedback;
  double elapsed=0,duration=0;
  int confirmed=0;
 private:
  void Segment(const Angles& from,const Angles& target,InitStage next);
  void SetStage(InitStage next);
  bool Arrived(const Angles& q);
  const RunnerConfig& config_;
  ExpertClimb& expert_;
  Command goal_;
  ClimbState target_state_;
  std::optional<Command> command_;
  Angles start_,target_;
  std::optional<uint64_t> zero_sample_;
  bool zero_sent_=false;
  std::optional<double> motor_error_,servo_error_;
  std::string waiting_reason_;
  std::vector<int> waiting_legs_;
};

class Runner {
 public:
  explicit Runner(const RunnerConfig& config);
  std::pair<bool,std::string> Request(const std::string& action,bool ready);
  bool SetVelocity(const Vec3& velocity);
  void Tick(const std::optional<Feedback>& feedback,const std::string& unavailable="");
  Json::Value Diagnostics() const;
  Json::Value Report() const;
  Command PrepareInitialization();
  void ExportSimInfo() const;
  RunnerConfig config;
  Kinematic kin;
  ExpertClimb expert;
  Phase phase=Phase::Waiting;
  OutputEvent output=OutputEvent::None;
  std::optional<Command> command;
  std::optional<Initialization> initialization;
  Vec3 velocity=Vec3::Zero();
  bool motor_servo_disabled=false,magnetic_forced_off=false;
  uint64_t compute_count=0,reset_count=0;
  std::string failure_reason;
 private:
  void SetPhase(Phase next,const std::string& reason="");
  Angles initial_q_;
  uint64_t last_motor_sample_=0;
  std::string pending_;
  bool pending_disable_=false,pending_release_=false;
  int gait_switch_count_=0;
  Json::Value events_{Json::arrayValue};
};

// 诊断中的数组仍采用 JSON，兼容现有 Python 监控及 bag 分析工具。
template<class Derived> Json::Value ToJson(const Eigen::MatrixBase<Derived>& matrix) {
  Json::Value rows(Json::arrayValue);
  for (int i=0;i<matrix.rows();++i) {
    if (matrix.cols()==1) rows.append(matrix(i,0));
    else { Json::Value row(Json::arrayValue); for (int j=0;j<matrix.cols();++j) row.append(matrix(i,j)); rows.append(row); }
  }
  return rows;
}
template<class T,size_t N> Json::Value ToJson(const std::array<T,N>& values) {
  Json::Value out(Json::arrayValue); for (const auto& v:values) out.append(v); return out;
}
std::string JsonText(const Json::Value& value);
double MonotonicNow();
}  // namespace hex_runner
