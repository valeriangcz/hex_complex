#pragma once
#include <hex_kinematic/kinematic.hpp>
#include <cstdint>

namespace hex_expert {
using namespace hex_kinematic;
enum class OperatingMode { Suspended, Ground };
OperatingMode ParseOperatingMode(const std::string& name);
std::string ModeName(OperatingMode mode);

struct ClimbParameters {
  OperatingMode operating_mode = OperatingMode::Suspended;
  std::array<std::array<int,3>,2> groups{{{{0,1,5}},{{2,3,4}}}};
  int stance_group = 0, contact_confirm_steps = 10;
  double control_dt=.01, torque_limit=27, contact_force_threshold=7,
    contact_velocity_threshold=.03, torque_blend_time=.1, stance_height=.08,
    joint_position_gain=40, magnetic_attach_above=.85, magnetic_release_below=.05;
};
struct ExpertOutput {
  Angles q = Angles::Zero();
  JointMatrix tau = JointMatrix::Zero();
  Flags adhesions{}, servo_enabled{};
};
// 仅保存控制状态，初始化快照不会复制模型或配置。
struct ClimbState {
  Angles q_des=Angles::Zero();
  JointMatrix feet=JointMatrix::Zero(), targets=JointMatrix::Zero(), previous_feet=JointMatrix::Zero(),
    tau=JointMatrix::Zero(), contact_forces=JointMatrix::Zero(), blend_start=JointMatrix::Zero();
  Ratios contact_force=Ratios::Zero(), normal_velocity=Ratios::Zero();
  Flags gaits{}, adhesions{}, swing_reached{}, support_mask{};
  std::array<int,6> contact_count{};
  int stance_group=0, blend_remaining=0;
  uint64_t compute_count=0;
  bool initialized=false, previous_valid=false;
};

class ExpertClimb {
 public:
  ExpertClimb(const Kinematic& kinematic, const ClimbParameters& parameters);
  void Reset();
  ExpertOutput SetInit(const Angles& q);
  ExpertOutput RequestSingleStep(const Vec3& gravity, const Angles& q,
                                const JointMatrix& torque, const Ratios& attached,
                                const Vec3& velocity);
  JointMatrix InitializationFeedforward(const Vec3& gravity, const Angles& q, const Flags& adhesions);
  void SeedFeedforward(const JointMatrix& tau);
  JointMatrix ComputeQuasiStaticTauFF(const Vec3& gravity, const JointMatrix& q, const Vec3& com);
  Rows3 SolveSupportForces(const Rows3& arms, const Vec3& gravity) const;
  Flags ContactDetection(const JointMatrix& q, const JointMatrix& torque);
  Flags CheckWorkspace(const JointMatrix& feet) const;
  Flags CheckLegClearance(const JointMatrix& feet) const;
  JointMatrix SwingStartLeg() const;
  ExpertOutput Output() const;
  const Kinematic& kin;
  ClimbParameters config;
  ClimbState state;

 private:
  void PlanGait(const Vec3& command, const JointMatrix& q, const JointMatrix& torque, const Ratios& attached);
  JointMatrix ComputeFootTargets(const Vec3& command);
  int blend_steps_;
};
}  // namespace hex_expert
