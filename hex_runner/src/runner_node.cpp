#include <hex_runner/runner.hpp>
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <dm_motor_fusion/MultiLegCommand.h>
#include <dm_motor_fusion/MultiLegState.h>
#include <feetech_servo/servo_cmd.h>
#include <feetech_servo/servo_state.h>
#include <geometry_msgs/Twist.h>
#include <magnetic/MagneticCmd.h>
#include <magnetic/MagneticState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
#include <tuple>

namespace hex_runner {
namespace {
constexpr std::array<int,6> wire_index{2,0,1,5,3,4};
std::array<const dm_motor_fusion::LegState*,6> legs(const dm_motor_fusion::MultiLegState& m) {
  return {&m.lb,&m.lf,&m.lm,&m.rb,&m.rf,&m.rm};
}
std::array<dm_motor_fusion::LegCommand*,6> legs(dm_motor_fusion::MultiLegCommand& m) {
  return {&m.lb,&m.lf,&m.lm,&m.rb,&m.rf,&m.rm};
}
}  // namespace
class RunnerNode {
 public:
  RunnerNode(ros::NodeHandle& nh,const RunnerConfig& config,std::string output_dir)
      : runner_(config),output_dir_(std::move(output_dir)) {
    if (output_dir_.rfind("~/",0)==0) output_dir_=std::string(std::getenv("HOME"))+output_dir_.substr(1);
    motor_pub_=nh.advertise<dm_motor_fusion::MultiLegCommand>("hex/motor/command",1);
    servo_pub_=nh.advertise<feetech_servo::servo_cmd>("hex/servo/command",1);
    magnetic_pub_=nh.advertise<magnetic::MagneticCmd>("hex/magnetic/command",1);
    diag_pub_=nh.advertise<diagnostic_msgs::DiagnosticArray>("hex/runner/diagnostics",1);
    const auto transport=ros::TransportHints().tcpNoDelay();
    subscribers_.push_back(nh.subscribe("hex/motor/state",1,&RunnerNode::Motor,this,transport));
    subscribers_.push_back(nh.subscribe("hex/servo/state",1,&RunnerNode::Servo,this,transport));
    subscribers_.push_back(nh.subscribe("hex/magnetic/state",1,&RunnerNode::Magnetic,this,transport));
    subscribers_.push_back(nh.subscribe("hex/perception/gravity",1,&RunnerNode::Gravity,this,transport));
    subscribers_.push_back(nh.subscribe("hex/cmd_vel",1,&RunnerNode::Velocity,this,transport));
    for (const std::string action:{"reset","start","pause","disable_actuators","release_magnets"}) {
      boost::function<bool(std_srvs::Trigger::Request&,std_srvs::Trigger::Response&)> callback=
        [this,action](std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
          std::string reason; Json::Value ages;
          const auto feedback=cache_.Snapshot(MonotonicNow(),runner_.config.feedback_timeout,reason,ages);
          const auto result=runner_.Request(action,feedback.has_value());
          response.success=result.first; response.message=result.second;
          if (!result.first && !feedback) response.message+=": "+reason;
          return true;
        };
      services_.push_back(nh.advertiseService("hex/runner/"+action,callback));
    }
  }
  void Spin() {
    using Clock=std::chrono::steady_clock;
    const auto dt=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(runner_.config.expert.control_dt));
    auto deadline=Clock::now();
    std::cout << "HEX_RUNNER_INITIALIZED" << std::endl;
    while (ros::ok()) {
      // 同一线程消费最新反馈、服务请求和控制状态，避免模型锁与后台计算竞争。
      ros::spinOnce(); Tick(); deadline+=dt;
      if (deadline>Clock::now()) std::this_thread::sleep_until(deadline);
      else deadline=Clock::now();  // 过期周期不追赶补算
    }
    if (!output_dir_.empty()) {
      std::filesystem::create_directories(output_dir_);
      const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      std::ofstream file(std::filesystem::path(output_dir_)/("runner_"+std::to_string(ns)+".json"));
      file << JsonText(runner_.Report()) << '\n';
    }
  }
 private:
  void Motor(const dm_motor_fusion::MultiLegState::ConstPtr& m) {
    JointMatrix q,tau,velocity; const auto fields=legs(*m); bool valid=!m->header.stamp.isZero();
    for (int i=0;i<6;++i) {
      q.row(i) << fields[i]->thigh_pos,fields[i]->knee_pos,fields[i]->ankle_pos;
      tau.row(i) << fields[i]->thigh_tau,fields[i]->knee_tau,fields[i]->ankle_tau;
      velocity.row(i) << fields[i]->thigh_vel,fields[i]->knee_vel,fields[i]->ankle_vel;
      for (int j=0;j<3;++j) valid &= m->valid[wire_index[i]*3+j];
    }
    cache_.Motor(q,tau,m->header.stamp.toNSec(),MonotonicNow(),valid && q.allFinite() && tau.allFinite() && velocity.allFinite());
  }
  void Servo(const feetech_servo::servo_state::ConstPtr& m) {
    Ratios q; q << m->lb,m->lf,m->lm,m->rb,m->rf,m->rm; q*=pi/180;
    bool valid=q.allFinite(); for (int i=0;i<6;++i) valid &= m->valid[wire_index[i]];
    cache_.Servo(q,MonotonicNow(),valid);
  }
  void Magnetic(const magnetic::MagneticState::ConstPtr& m) {
    Ratios attached; attached << m->attached_lb,m->attached_lf,m->attached_lm,m->attached_rb,m->attached_rf,m->attached_rm;
    const Flags enabled{m->enabled_lb!=0,m->enabled_lf!=0,m->enabled_lm!=0,m->enabled_rb!=0,m->enabled_rf!=0,m->enabled_rm!=0};
    const bool valid=m->valid_lb && m->valid_lf && m->valid_lm && m->valid_rb && m->valid_rf && m->valid_rm && attached.allFinite();
    cache_.Magnetic(attached,enabled,MonotonicNow(),valid);
  }
  void Gravity(const std_msgs::Float64MultiArray::ConstPtr& m) {
    Vec3 direction=Vec3::Zero(); bool valid=m->data.size()==3;
    if (valid) direction=Eigen::Map<const Vec3>(m->data.data());
    valid &= direction.allFinite() && direction.cwiseAbs().maxCoeff()>0;
    if (valid) { direction/=direction.cwiseAbs().maxCoeff(); direction.normalize(); }
    cache_.Gravity(direction,MonotonicNow(),valid);
  }
  void Velocity(const geometry_msgs::Twist::ConstPtr& m) {
    const Vec3 velocity(m->linear.x,m->linear.y,m->angular.z);
    if (!velocity.allFinite()) { ROS_WARN_THROTTLE(1.,"invalid velocity command"); return; }
    runner_.SetVelocity(velocity.cwiseProduct(runner_.config.velocity_scales));
  }
  void Tick() {
    const double now=MonotonicNow(); std::string reason; Json::Value ages;
    const auto feedback=cache_.Snapshot(now,runner_.config.feedback_timeout,reason,ages);
    runner_.Tick(feedback,reason);
    const bool disabled=runner_.motor_servo_disabled,off=runner_.magnetic_forced_off;
    const auto key=std::make_tuple(disabled,off,runner_.reset_count);
    if (key!=output_key_) { last_magnetic_=-std::numeric_limits<double>::infinity(); output_key_=key; }
    const auto event=runner_.output;
    if (event!=OutputEvent::None) {
      dm_motor_fusion::MultiLegCommand motor;
      motor.option=event==OutputEvent::Disable ? -1 : event==OutputEvent::Zero ? 0 : 1;
      if (!disabled && event!=OutputEvent::Zero) {
        const auto fields=legs(motor); const auto& command=runner_.command.value();
        for (int i=0;i<6;++i) {
          auto& leg=*fields[i];
          leg.thigh_pos=command.q(i,0); leg.knee_pos=command.q(i,1); leg.ankle_pos=command.q(i,2);
          leg.thigh_tau=command.tau(i,0); leg.knee_tau=command.tau(i,1); leg.ankle_tau=command.tau(i,2);
          leg.thigh_kp=leg.knee_kp=leg.ankle_kp=runner_.config.motor_kp;
          leg.thigh_kd=leg.knee_kd=leg.ankle_kd=runner_.config.motor_kd;
          // 保持现有 wire 行为：本次迁移不另加目标速度前馈。
          leg.thigh_vel=leg.knee_vel=leg.ankle_vel=0;
        }
      }
      motor_pub_.publish(motor);
      if (event!=OutputEvent::Zero) {
        feetech_servo::servo_cmd servo; servo.enable=!disabled;
        const std::array<uint8_t*,6> enables{&servo.enable_lb,&servo.enable_lf,&servo.enable_lm,&servo.enable_rb,&servo.enable_rf,&servo.enable_rm};
        const std::array<double*,6> positions{&servo.lb,&servo.lf,&servo.lm,&servo.rb,&servo.rf,&servo.rm};
        for (int i=0;i<6;++i) {
          *enables[i]=!disabled && runner_.command->servo_enabled[i];
          if (!disabled) *positions[i]=runner_.command->q(i,3)*180/pi;
        }
        servo_pub_.publish(servo);
      }
    }
    if ((runner_.command || off || runner_.phase==Phase::Initializing) && now-last_magnetic_>=1/runner_.config.magnetic_hz) {
      magnetic::MagneticCmd m; const std::array<uint8_t*,6> values{&m.lb,&m.lf,&m.lm,&m.rb,&m.rf,&m.rm};
      for (int i=0;i<6;++i) *values[i]=!off && runner_.command && runner_.command->adhesions[i];
      magnetic_pub_.publish(m); last_magnetic_=now;
    }
    if (now-last_diag_>=.1) {
      auto values=runner_.Diagnostics(); values["feedback_ready"]=feedback.has_value(); values["feedback_reason"]=reason;
      values["feedback_age"]=ages; values["motor_feedback_ready"]=cache_.MotorReady(now,runner_.config.feedback_timeout);
      diagnostic_msgs::DiagnosticStatus status; status.name="hex_runner"; status.hardware_id="hex/shared-controller";
      status.message=PhaseName(runner_.phase);
      status.level=runner_.phase==Phase::Fault ? diagnostic_msgs::DiagnosticStatus::ERROR : !feedback ? diagnostic_msgs::DiagnosticStatus::WARN : diagnostic_msgs::DiagnosticStatus::OK;
      for (const auto& key:values.getMemberNames()) {
        diagnostic_msgs::KeyValue value; value.key=key; value.value=JsonText(values[key]); status.values.push_back(value);
      }
      diagnostic_msgs::DiagnosticArray diag; diag.header.stamp=ros::Time::now(); diag.status.push_back(status);
      diag_pub_.publish(diag); last_diag_=now;
    }
  }
  Runner runner_;
  FeedbackCache cache_;
  std::string output_dir_;
  ros::Publisher motor_pub_,servo_pub_,magnetic_pub_,diag_pub_;
  std::vector<ros::Subscriber> subscribers_;
  std::vector<ros::ServiceServer> services_;
  double last_magnetic_=-std::numeric_limits<double>::infinity(),last_diag_=-std::numeric_limits<double>::infinity();
  std::tuple<bool,bool,uint64_t> output_key_{false,false,std::numeric_limits<uint64_t>::max()};
};
}  // namespace hex_runner

int main(int argc,char** argv) {
  ros::init(argc,argv,"hex_runner");
  try {
    ros::V_string args; ros::removeROSArgs(argc,argv,args);
    std::map<std::string,std::string> options{{"--config",""},{"--data-root",""},{"--output-dir",""},{"--to-sim-info",""}};
    for (size_t i=1;i<args.size();i+=2) {
      if (!options.count(args[i]) || i+1==args.size()) throw std::invalid_argument("Expected --config, --data-root, --output-dir or --to-sim-info followed by a value");
      options[args[i]]=args[i+1];
    }
    if (options["--config"].empty()) throw std::invalid_argument("--config is required");
    const auto config=hex_runner::RunnerConfig::Load(options["--config"],options["--data-root"],options["--to-sim-info"]);
    ros::NodeHandle nh; hex_runner::RunnerNode node(nh,config,options["--output-dir"]); node.Spin();
    return 0;
  } catch(const std::exception& error) { ROS_FATAL("hex_runner: %s",error.what()); return 1; }
}
