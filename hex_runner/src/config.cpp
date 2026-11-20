#include <hex_runner/runner.hpp>
#include <Eigen/Geometry>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <cstdlib>
#include <set>
#include <stdexcept>

namespace hex_runner {
namespace {
namespace fs=std::filesystem;
std::string resolve(std::string value,const fs::path& base) {
  if (value.rfind("package://",0)==0) {
    value.erase(0,10); const auto slash=value.find('/');
    const auto package=ros::package::getPath(value.substr(0,slash));
    if (package.empty()) throw std::runtime_error("ROS package not found: "+value.substr(0,slash));
    return fs::weakly_canonical(fs::path(package)/value.substr(slash+1)).string();
  }
  if (value.rfind("~/",0)==0) value=std::string(std::getenv("HOME"))+value.substr(1);
  fs::path path(value); return fs::weakly_canonical(path.is_absolute() ? path : base/path).string();
}
template<class T> T read(const YAML::Node& node,const char* key,T fallback) {
  return node[key] ? node[key].as<T>() : fallback;
}
void requireKeys(const YAML::Node& node,const std::set<std::string>& keys,const std::string& label) {
  for (const auto& item:node) if (!keys.count(item.first.as<std::string>()))
    throw std::invalid_argument("Unknown "+label+" key: "+item.first.as<std::string>());
}
}  // namespace
RunnerConfig RunnerConfig::Load(const std::string& filename,const std::string& data_root,const std::string& sim_info) {
  const auto path=std::filesystem::absolute(filename),base=path.parent_path();
  const auto node=YAML::LoadFile(path.string()); RunnerConfig c;
  if (node["expert_type"].as<std::string>()!="climb") throw std::invalid_argument("C++ runner supports only six-leg climb; use implementation:=python for other modes");
  if (read<std::string>(node,"pose_source","gravity")!="gravity") throw std::invalid_argument("climb requires gravity feedback");
  c.expert.operating_mode=hex_expert::ParseOperatingMode(read<std::string>(node,"operating_mode","suspended"));
  const auto root=resolve(read<std::string>(node,"workspace_root","../../.."),base);
  c.data_root=data_root.empty() ? resolve("data",root) : resolve(data_root,std::filesystem::current_path());
  c.robot_urdf=resolve(read<std::string>(node,"robot_urdf","package://hex_kinematic/resources/hex_magnetic/urdf/hex_all.urdf"),base);
  c.to_sim_info=resolve(sim_info.empty() ? read<std::string>(node,"to_sim_info","package://hex_runner/config/to_sim_info.yaml") : sim_info,base);
  c.expert.control_dt=read<double>(node,"control_dt",.01);
  c.expert.torque_limit=read<double>(node,"motor_torque_limit",27);
  c.magnetic_hz=read<double>(node,"magnetic_hz",20);
  c.feedback_timeout=read<double>(node,"feedback_timeout",.5);
  c.motor_kp=read<double>(node,"motor_kp",60); c.motor_kd=read<double>(node,"motor_kd",1);
  for (double v:{c.expert.control_dt,c.expert.torque_limit,c.magnetic_hz,c.feedback_timeout})
    if (!std::isfinite(v) || v<=0) throw std::invalid_argument("Timing and torque limits must be positive and finite");
  if (!std::isfinite(c.motor_kp) || !std::isfinite(c.motor_kd) || c.motor_kp<0 || c.motor_kd<0)
    throw std::invalid_argument("Motor gains must be finite and nonnegative");
  const auto init=node["initialization"];
  requireKeys(init,{"method","staged"},"initialization");
  if (read<std::string>(init,"method","set_init")!="set_init") throw std::invalid_argument("climb requires set_init");
  c.staged=read<bool>(init,"staged",false);
  const auto e=node["expert_parameters"];
  requireKeys(e,{"magnetic_attach_above","magnetic_release_below","contact_force_threshold","contact_confirm_steps",
      "contact_velocity_threshold","torque_blend_time","stance_height","joint_position_gain"},"expert_parameters");
  c.expert.magnetic_attach_above=read<double>(e,"magnetic_attach_above",.85);
  c.expert.magnetic_release_below=read<double>(e,"magnetic_release_below",.05);
  c.expert.contact_force_threshold=read<double>(e,"contact_force_threshold",7);
  c.expert.contact_confirm_steps=read<int>(e,"contact_confirm_steps",10);
  c.expert.contact_velocity_threshold=read<double>(e,"contact_velocity_threshold",.03);
  c.expert.torque_blend_time=read<double>(e,"torque_blend_time",.1);
  c.expert.stance_height=read<double>(e,"stance_height",.08);
  c.expert.joint_position_gain=read<double>(e,"joint_position_gain",40);
  for (double v:{c.expert.contact_force_threshold,c.expert.contact_velocity_threshold,c.expert.torque_blend_time,
      c.expert.stance_height,c.expert.joint_position_gain})
    if (!std::isfinite(v) || v<=0) throw std::invalid_argument("Expert gains and timing must be positive and finite");
  if (c.expert.contact_confirm_steps<1 || c.expert.magnetic_release_below<0 ||
      c.expert.magnetic_release_below>=c.expert.magnetic_attach_above || c.expert.magnetic_attach_above>1)
    throw std::invalid_argument("Invalid contact confirmation or magnetic thresholds");
  const std::array<std::string,3> axes{"vx","vy","wz"};
  for (int i=0;i<3;++i) c.velocity_scales[i]=node["velocity_scales"][axes[i]].as<double>();
  if (!c.velocity_scales.allFinite()) throw std::invalid_argument("Velocity scales must be finite");
  const auto initial=node["initial_state"];
  if (!initial["servo_enabled"].as<bool>()) throw std::invalid_argument("Initial servos must be enabled");
  const auto groups=initial["groups"];
  if (groups.size()!=2 || groups[0].size()!=3 || groups[1].size()!=3) throw std::invalid_argument("climb requires two groups of three legs");
  std::set<int> used;
  for (int i=0;i<2;++i) for (int j=0;j<3;++j) {
    const int leg=groups[i][j].as<int>(); c.expert.groups[i][j]=leg; used.insert(leg);
  }
  if (used!=std::set<int>{0,1,2,3,4,5}) throw std::invalid_argument("Groups must partition legs 0..5");
  c.expert.stance_group=initial["stance_group"].as<int>();
  if (c.expert.stance_group<0 || c.expert.stance_group>1) throw std::invalid_argument("Invalid stance_group");
  const auto pose=initial["pose"],pos=pose["position"],rot=pose["orientation"];
  c.initial_pose.block<3,1>(0,3)=Vec3(pos["x"].as<double>(),pos["y"].as<double>(),pos["z"].as<double>());
  Eigen::Quaterniond quaternion(rot["w"].as<double>(),rot["x"].as<double>(),rot["y"].as<double>(),rot["z"].as<double>());
  if (!quaternion.coeffs().allFinite() || std::abs(quaternion.norm()-1)>1e-6 || !c.initial_pose.allFinite())
    throw std::invalid_argument("Initial pose requires finite coordinates and a unit quaternion");
  c.initial_pose.topLeftCorner<3,3>()=quaternion.toRotationMatrix();
  const auto mode=initial["mode"].as<std::string>();
  if (mode!="feet" && mode!="joints") throw std::invalid_argument("initial_state.mode must be feet or joints");
  c.initial_feet=mode=="feet";
  if (c.initial_feet && initial["frame"].as<std::string>()!="leg") throw std::invalid_argument("Initial feet must be in leg frame");
  const std::array<std::string,3> feet_keys{"x","y","z"},joint_keys{"thigh","knee","ankle"};
  for (int i=0;i<6;++i) {
    c.magnetic[i]=initial["magnetic"][leg_names[i]].as<bool>();
    c.ik_branch[i]=initial["ik_branch"][leg_names[i]].as<int>();
    const auto entry=initial[c.initial_feet ? "feet" : "q"][leg_names[i]];
    for (int j=0;j<3;++j) c.initial_values(i,j)=entry[(c.initial_feet ? feet_keys : joint_keys)[j]].as<double>();
  }
  return c;
}
}  // namespace hex_runner
