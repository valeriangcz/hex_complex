#include <hex_expert/expert_climb.hpp>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace hex_expert {
namespace {
Mat3 skew(const Vec3& v) { Mat3 m; m << 0,-v.z(),v.y(), v.z(),0,-v.x(), -v.y(),v.x(),0; return m; }
Vec3 unitGravity(const Vec3& direction) {
  const Vec3 scaled=direction/direction.cwiseAbs().maxCoeff();
  return scaled/scaled.norm();
}
}
OperatingMode ParseOperatingMode(const std::string& name) {
  if (name=="suspended") return OperatingMode::Suspended;
  if (name=="ground") return OperatingMode::Ground;
  throw std::invalid_argument("operating_mode must be suspended or ground");
}
std::string ModeName(OperatingMode mode) { return mode==OperatingMode::Ground ? "ground" : "suspended"; }
ExpertClimb::ExpertClimb(const Kinematic& k,const ClimbParameters& p)
    : kin(k),config(p),blend_steps_(std::max(1,static_cast<int>(std::nearbyint(p.torque_blend_time/p.control_dt)))) { Reset(); }
void ExpertClimb::Reset() { state=ClimbState{}; state.stance_group=config.stance_group; }
ExpertOutput ExpertClimb::SetInit(const Angles& q) {
  Reset(); state.q_des=q; state.feet=kin.ForwardKin(q.leftCols<3>());
  state.targets=state.feet; state.previous_feet=state.feet;
  state.initialized=true; state.previous_valid=true;
  for (int leg:config.groups[state.stance_group]) state.gaits[leg]=state.adhesions[leg]=true;
  return Output();
}
ExpertOutput ExpertClimb::Output() const {
  ExpertOutput out; out.q=state.q_des; out.tau=state.tau; out.adhesions=state.adhesions;
  for (int i=0;i<6;++i) out.servo_enabled[i]=!state.gaits[i];
  return out;
}
JointMatrix ExpertClimb::SwingStartLeg() const {
  JointMatrix feet;
  feet << .18,.08,.04, .18,-.08,.04, .19,0,.04, .18,-.08,.04, .18,.08,.04, .19,0,.04;
  return feet;
}
ExpertOutput ExpertClimb::RequestSingleStep(const Vec3& gravity,const Angles& q,
    const JointMatrix& torque,const Ratios& attached,const Vec3& velocity) {
  if (!q.allFinite() || !torque.allFinite() || !gravity.allFinite() || gravity.cwiseAbs().maxCoeff()==0 ||
      !attached.allFinite() || attached.minCoeff()<0 || attached.maxCoeff()>1 || !velocity.allFinite())
    throw std::invalid_argument("ExpertClimb requires finite feedback and gravity direction");
  if (!state.initialized) SetInit(q);
  const JointMatrix motor_q=q.leftCols<3>();
  state.feet=kin.ForwardKin(motor_q);
  if ((velocity.array()!=0).any()) {
    PlanGait(velocity,motor_q,torque,attached);
    for (int i=0;i<6;++i) {
      const Vec3 dq=kin.DampInvJac(motor_q.row(i).transpose())*(state.targets.row(i)-state.feet.row(i)).transpose();
      state.q_des.row(i).head<3>()=motor_q.row(i)+config.joint_position_gain*config.control_dt*dq.transpose();
      state.q_des(i,0)=std::clamp(state.q_des(i,0),kin.thigh_limits(i,0),kin.thigh_limits(i,1));
      for (int j=1;j<3;++j) state.q_des(i,j)=std::clamp(state.q_des(i,j),kin.rest_limits(j-1,0),kin.rest_limits(j-1,1));
      state.q_des(i,3)=-state.q_des(i,1)-state.q_des(i,2)-pi/2;
    }
  } else state.contact_count.fill(0);
  const Vec3 com=config.operating_mode==OperatingMode::Ground ? kin.ForwardKinAll(q).com : Vec3::Zero();
  ComputeQuasiStaticTauFF(9.81*unitGravity(gravity),motor_q,com);
  ++state.compute_count;
  return Output();
}
Flags ExpertClimb::ContactDetection(const JointMatrix& q,const JointMatrix& torque) {
  Flags detected{};
  for (int i=0;i<6;++i) {
    const Vec3 qi=q.row(i).transpose();
    state.contact_force[i]=kin.CF_Estimate(torque.row(i).transpose(),qi).z();
    state.normal_velocity[i]=state.previous_valid ?
      (state.feet(i,2)-state.previous_feet(i,2))/config.control_dt : std::numeric_limits<double>::infinity();
    const bool contact=state.contact_force[i]<=-config.contact_force_threshold &&
      std::abs(state.normal_velocity[i])<=config.contact_velocity_threshold && !state.gaits[i] && state.swing_reached[i];
    state.contact_count[i]=contact ? state.contact_count[i]+1 : 0;
    detected[i]=state.contact_count[i]>=config.contact_confirm_steps;
    if (detected[i]) state.contact_count[i]=0;
    // 悬空没有真实落地压力，用测得高度完成测试；地面使用连续压力和低速度。
    if (config.operating_mode==OperatingMode::Suspended)
      detected[i]=!state.gaits[i] && state.swing_reached[i] && state.feet(i,2)<=-config.stance_height;
  }
  state.previous_feet=state.feet; state.previous_valid=true;
  return detected;
}
void ExpertClimb::PlanGait(const Vec3& command,const JointMatrix& q,const JointMatrix& torque,const Ratios& attached) {
  const auto landed=ContactDetection(q,torque);
  for (int i=0;i<6;++i) {
    if (landed[i]) { state.targets.row(i)=state.feet.row(i); state.gaits[i]=true; }
    state.adhesions[i]=state.gaits[i];
  }
  const auto next=ComputeFootTargets(command);
  const auto workspace=CheckWorkspace(next),clearance=CheckLegClearance(next);
  bool move_body=true;
  for (int i=0;i<6;++i) {
    if (state.swing_reached[i]) state.adhesions[i]=true;
    const bool swing=!state.gaits[i];
    if (swing && ((clearance[i] && workspace[i]) || !state.swing_reached[i])) state.targets.row(i)=next.row(i);
    if ((swing && (!clearance[i] || !workspace[i])) || state.gaits[i]) state.targets(i,2)=next(i,2);
    move_body &= swing || (workspace[i] && clearance[i]);
  }
  if (move_body) for (int i=0;i<6;++i) if (state.gaits[i]) state.targets.row(i).head<2>()=next.row(i).head<2>();
  const auto& next_group=config.groups[1-state.stance_group];
  const auto& old_group=config.groups[state.stance_group];
  bool attached_ok=true,released_ok=true;
  // 磁吸消息为 float32，阈值比较与 Python 的同精度行为一致。
  for (int i:next_group) attached_ok &= static_cast<float>(attached[i])>static_cast<float>(config.magnetic_attach_above) && state.gaits[i];
  if (attached_ok) {
    for (int i:old_group) {
      state.adhesions[i]=false;
      released_ok &= static_cast<float>(attached[i])<static_cast<float>(config.magnetic_release_below);
    }
    if (released_ok) {
      state.stance_group=1-state.stance_group; state.swing_reached.fill(false); state.contact_count.fill(0); state.gaits.fill(false);
      for (int i:next_group) state.gaits[i]=true;
    }
  }
}
JointMatrix ExpertClimb::ComputeFootTargets(const Vec3& command) {
  JointMatrix next;
  const auto start=SwingStartLeg();
  const double nominal=std::min(1.2,std::max(command.head<2>().norm()*1.5,std::abs(command.z())*1.2));
  for (int i=0;i<6;++i) {
    const Vec3 foot=state.targets.row(i).transpose(),body=kin.B2R(foot,i);
    const Vec3 delta=command*config.control_dt*(state.gaits[i] ? 1. : -1.);
    const double c=std::cos(delta.z()),s=std::sin(delta.z());
    Mat3 rotation; rotation << c,s,0, -s,c,0, 0,0,1;
    const Vec3 translation(delta.x(),delta.y(),0);
    Vec3 target=kin.R2B(rotation*(body-translation),i);
    state.swing_reached[i]=state.swing_reached[i] || foot.z()>=start(i,2)-.005;
    const double speed=nominal*std::clamp((body.z()+.16)/.12,.1,1.);
    if (state.gaits[i]) {
      const double error=foot.z()+config.stance_height,increment=config.control_dt*speed*.2;
      target.z()=std::abs(error)<increment ? -config.stance_height : foot.z()-(error>0 ? 1 : (error<0 ? -1 : 0))*increment;
    } else if (!state.swing_reached[i]) {
      const Vec3 difference=start.row(i).transpose()-foot;
      target=foot;
      if (difference.norm()>0) target+=difference/difference.norm()*std::min(config.control_dt*speed,difference.norm());
    } else target.z()=std::max(target.z()-speed*config.control_dt,-.2);
    next.row(i)=target.transpose();
  }
  return next;
}
Flags ExpertClimb::CheckWorkspace(const JointMatrix& feet) const {
  Flags good{};
  for (int i=0;i<6;++i) {
    const double r=feet.row(i).head<2>().norm(),raw=std::atan2(feet(i,1),feet(i,0))+pi/2;
    const double angle=raw-std::floor(raw/pi)*pi-pi/2;
    good[i]=r>=.14 && r<=.3 && feet(i,2)>=-.2 && feet(i,2)<=.1 && angle>=-pi/4 && angle<=pi/4;
  }
  return good;
}
Flags ExpertClimb::CheckLegClearance(const JointMatrix& feet) const {
  Flags good{};
  JointMatrix body; for (int i=0;i<6;++i) body.row(i)=kin.B2R(feet.row(i).transpose(),i).transpose();
  for (int base:{0,3}) {
    good[base]=(body.row(base).head<2>()-body.row(base+2).head<2>()).norm()>.06;
    good[base+1]=(body.row(base+1).head<2>()-body.row(base+2).head<2>()).norm()>.06;
    good[base+2]=good[base] && good[base+1];
  }
  return good;
}
Rows3 ExpertClimb::SolveSupportForces(const Rows3& arms,const Vec3& gravity) const {
  if (arms.rows()<3) return Rows3(0,3);
  Eigen::MatrixXd A(6,3*arms.rows());
  for (int i=0;i<arms.rows();++i) { A.block<3,3>(0,3*i).setIdentity(); A.block<3,3>(3,3*i)=skew(arms.row(i).transpose()); }
  const Eigen::JacobiSVD<Eigen::MatrixXd> rank(A);
  const auto singular=rank.singularValues();
  if (singular[5]<=singular[0]*std::max(A.rows(),A.cols())*std::numeric_limits<double>::epsilon()) return Rows3(0,3);
  Eigen::Matrix<double,6,1> b; b.head<3>()=-kin.total_mass*gravity; b.tail<3>().setZero();
  const Eigen::Matrix<double,6,6> gram=A*A.transpose();
  const Eigen::JacobiSVD<Eigen::Matrix<double,6,6>> svd(gram,Eigen::ComputeFullU|Eigen::ComputeFullV);
  const auto values=svd.singularValues();
  Eigen::Matrix<double,6,1> inverse=values.unaryExpr([&](double v){return v>1e-9*values[0] ? 1/v : 0.;});
  const Eigen::VectorXd force=A.transpose()*svd.matrixV()*inverse.asDiagonal()*svd.matrixU().transpose()*b;
  if (!force.allFinite() || (A*force-b).norm()>1e-5*std::max(1.,b.norm())) return Rows3(0,3);
  Rows3 out(arms.rows(),3); for (int i=0;i<arms.rows();++i) out.row(i)=force.segment<3>(3*i).transpose(); return out;
}
void ExpertClimb::SeedFeedforward(const JointMatrix& tau) {
  state.tau=tau; state.blend_start=tau; state.blend_remaining=blend_steps_;
}
JointMatrix ExpertClimb::ComputeQuasiStaticTauFF(const Vec3& gravity,const JointMatrix& q,const Vec3& com) {
  Flags support{}; std::vector<int> indices;
  JointMatrix target=JointMatrix::Zero(); state.contact_forces.setZero();
  for (int i=0;i<6;++i) { support[i]=state.gaits[i] && state.adhesions[i]; if (support[i]) indices.push_back(i); }
  if (config.operating_mode==OperatingMode::Ground && !indices.empty()) {
    Rows3 arms(indices.size(),3);
    for (size_t j=0;j<indices.size();++j) arms.row(j)=(kin.B2R(state.feet.row(indices[j]).transpose(),indices[j])-com).transpose();
    const auto force=SolveSupportForces(arms,gravity);
    if (force.rows()) for (size_t j=0;j<indices.size();++j) {
      const int i=indices[j]; state.contact_forces.row(i)=force.row(j);
      target.row(i)=(-kin.Jacobian(q.row(i).transpose()).transpose()*kin.RVectorToLeg(force.row(j).transpose(),i)).transpose();
    }
  }
  target=target.cwiseMax(-config.torque_limit).cwiseMin(config.torque_limit);
  if (support!=state.support_mask) { SeedFeedforward(state.tau); state.support_mask=support; }
  if (state.blend_remaining) {
    const double u=static_cast<double>(blend_steps_-state.blend_remaining+1)/blend_steps_;
    state.tau=(1-u)*state.blend_start+u*target; --state.blend_remaining;
  } else state.tau=target;
  state.tau=state.tau.cwiseMax(-config.torque_limit).cwiseMin(config.torque_limit);
  return state.tau;
}
JointMatrix ExpertClimb::InitializationFeedforward(const Vec3& gravity,const Angles& q,const Flags& adhesions) {
  if (config.operating_mode==OperatingMode::Suspended) { state.tau.setZero(); return state.tau; }
  const auto fk=kin.ForwardKinAll(q); state.feet=fk.feet; state.adhesions=adhesions;
  return ComputeQuasiStaticTauFF(9.81*unitGravity(gravity),q.leftCols<3>(),fk.com);
}
}  // namespace hex_expert
