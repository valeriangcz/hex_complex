#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <array>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "hardware_interface/damiao.h"

namespace
{
enum class HighLevelMode : int
{
  Disable = -1,
  ZeroTorque = 0,
  PosVel = 1,
  TrajFollow = 2
};

struct MotorRuntime
{
  int logical_id{0};   // 1..18
  int physical_id{0};  // actual CAN id
  int mode_code{static_cast<int>(damiao::Control_Mode_Code::MIT)};
  bool enabled{false};
  bool has_command{false};
  double cmd_pos{0.0};
  double cmd_vel{0.0};
  double cmd_kp{0.0};
  double cmd_kd{0.0};
  double cmd_tau{0.0};
  std::shared_ptr<damiao::Motor> motor;
  int high_level_mode{static_cast<int>(HighLevelMode::Disable)};
};

struct LegGroup
{
  std::string name;
  std::array<int, 3> ids;
  std::array<double, 3> pn;
};

int toControlModeCode(const int mode)
{
  if (mode <= 1)
  {
    return static_cast<int>(damiao::Control_Mode_Code::MIT);
  }
  return static_cast<int>(damiao::Control_Mode_Code::POS_VEL);
}
}  // namespace

class DmMotorFusionNode
{
public:
  explicit DmMotorFusionNode(ros::NodeHandle& nh) : nh_(nh), pnh_("~")
  {
    pnh_.param<std::string>("can_interface", can_interface_, "can0");
    pnh_.param<int>("thread_priority", thread_priority_, 95);
    pnh_.param<double>("control_hz", control_hz_, 100.0);
    pnh_.param<double>("status_hz", status_hz_, 100.0);
    pnh_.param<int>("motor_type", motor_type_, static_cast<int>(damiao::DM4340));

    initMotors();

    lf_sub_ = nh_.subscribe("/LF/sita_des", 20, &DmMotorFusionNode::lfCallback, this);
    lm_sub_ = nh_.subscribe("/LM/sita_des", 20, &DmMotorFusionNode::lmCallback, this);
    lb_sub_ = nh_.subscribe("/LB/sita_des", 20, &DmMotorFusionNode::lbCallback, this);
    rf_sub_ = nh_.subscribe("/RF/sita_des", 20, &DmMotorFusionNode::rfCallback, this);
    rm_sub_ = nh_.subscribe("/RM/sita_des", 20, &DmMotorFusionNode::rmCallback, this);
    rb_sub_ = nh_.subscribe("/RB/sita_des", 20, &DmMotorFusionNode::rbCallback, this);

    lf_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/LF/sita_cur", 20);
    lm_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/LM/sita_cur", 20);
    lb_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/LB/sita_cur", 20);
    rf_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/RF/sita_cur", 20);
    rm_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/RM/sita_cur", 20);
    rb_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/RB/sita_cur", 20);

    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_hz_), &DmMotorFusionNode::controlLoop, this);
    status_timer_ = nh_.createTimer(ros::Duration(1.0 / status_hz_), &DmMotorFusionNode::publishStatus, this);
  }

private:
  void initMotors()
  {
    auto motor_type = static_cast<damiao::DM_Motor_Type>(motor_type_);
    for (int logical_id = 1; logical_id <= 18; ++logical_id)
    {
      const int physical_id = logical_id;
      const int master_id = 0x20 + physical_id;
      dm_data_.emplace(static_cast<uint16_t>(physical_id), damiao::DmActData{
                                                               .motorType = motor_type,
                                                               .mode = damiao::MIT_MODE,
                                                               .can_id = static_cast<uint16_t>(physical_id),
                                                               .mst_id = static_cast<uint16_t>(master_id),
                                                               .pos = 0,
                                                               .vel = 0,
                                                               .effort = 0,
                                                               .cmd_pos = 0,
                                                               .cmd_vel = 0,
                                                               .cmd_effort = 0,
                                                               .kp = 0,
                                                               .kd = 0});
    }

    controller_ = std::make_unique<damiao::Motor_Control>(can_interface_, &dm_data_);

    for (int logical_id = 1; logical_id <= 18; ++logical_id)
    {
      const int physical_id = logical_id;
      auto motor = std::make_shared<damiao::Motor>(
          static_cast<damiao::DM_Motor_Type>(motor_type_),
          damiao::MIT_MODE,
          static_cast<uint16_t>(physical_id),
          static_cast<uint16_t>(0x20 + physical_id));

      controller_->addMotor(motor);
      controller_->switchControlMode(*motor, damiao::Control_Mode_Code::MIT);
      controller_->disable(*motor);

      MotorRuntime runtime{};
      runtime.logical_id = logical_id;
      runtime.physical_id = physical_id;
      runtime.motor = motor;
      motors_.emplace(logical_id, runtime);
    }

    groups_ = {
        LegGroup{"LF", {1, 2, 3}, {1.0, 1.0, -1.0}},
        LegGroup{"LM", {4, 5, 6}, {1.0, 1.0, -1.0}},
        LegGroup{"LB", {7, 8, 9}, {1.0, 1.0, -1.0}},
        LegGroup{"RF", {10, 11, 12}, {1.0, -1.0, 1.0}},
        LegGroup{"RM", {13, 14, 15}, {1.0, -1.0, 1.0}},
        LegGroup{"RB", {16, 17, 18}, {1.0, -1.0, 1.0}},
    };

    ROS_INFO("dm_motor_fusion ready on %s, using single CAN bus and 18 motors", can_interface_.c_str());
  }

  void lfCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) { legCommandCallback(0, msg); }
  void lmCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) { legCommandCallback(1, msg); }
  void lbCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) { legCommandCallback(2, msg); }
  void rfCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) { legCommandCallback(3, msg); }
  void rmCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) { legCommandCallback(4, msg); }
  void rbCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) { legCommandCallback(5, msg); }

  void legCommandCallback(std::size_t group_idx, const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    // motor_control format:
    // [mode, pos1, pos2, pos3, vel1, vel2, vel3]
    std::lock_guard<std::mutex> lock(mutex_);
    if (group_idx >= groups_.size() || msg->data.size() < 7)
    {
      return;
    }
    auto& group = groups_[group_idx];
    const int mode = static_cast<int>(std::llround(msg->data[0]));

    for (std::size_t i = 0; i < 3; ++i)
    {
      auto& motor = motors_.at(group.ids[i]);
      applyModeFromCommand(motor, mode);
      motor.high_level_mode = mode;
      if (mode == static_cast<int>(HighLevelMode::Disable))
      {
        continue;
      }
      motor.cmd_pos = msg->data[1 + i] * group.pn[i];
      motor.cmd_vel = msg->data[4 + i];
      motor.cmd_tau = 0.0;
      motor.cmd_kp = 100.0;
      motor.cmd_kd = 1.0;
      motor.has_command = true;
    }
  }

  void applyModeFromCommand(MotorRuntime& motor, const int mode)
  {
    if (mode == static_cast<int>(HighLevelMode::Disable))
    {
      setMotorEnable(motor, false);
      motor.has_command = false;
      return;
    }

    setMotorEnable(motor, true);
    const int target_mode = toControlModeCode(mode);
    switchMotorMode(motor, target_mode);

    if (mode == static_cast<int>(HighLevelMode::ZeroTorque))
    {
      motor.cmd_pos = 0.0;
      motor.cmd_vel = 0.0;
      motor.cmd_kp = 0.0;
      motor.cmd_kd = 0.0;
      motor.cmd_tau = 0.0;
      motor.has_command = true;
    }
  }

  void switchMotorMode(MotorRuntime& motor, int mode_code)
  {
    mode_code = static_cast<int>(damiao::Control_Mode_Code::MIT);
    if (motor.mode_code == mode_code)
    {
      return;
    }
    controller_->switchControlMode(*motor.motor, static_cast<damiao::Control_Mode_Code>(mode_code));
    motor.mode_code = mode_code;
  }

  void setMotorEnable(MotorRuntime& motor, bool enable)
  {
    if (motor.enabled == enable)
    {
      return;
    }
    if (enable)
    {
      controller_->enable(*motor.motor);
    }
    else
    {
      controller_->disable(*motor.motor);
    }
    motor.enabled = enable;
  }

  void controlLoop(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& kv : motors_)
    {
      auto& motor = kv.second;
      if (!motor.enabled || !motor.has_command)
      {
        continue;
      }

      if (motor.mode_code == static_cast<int>(damiao::Control_Mode_Code::MIT))
      {
        controller_->control_mit(*motor.motor,
                                 static_cast<float>(motor.cmd_kp),
                                 static_cast<float>(motor.cmd_kd),
                                 static_cast<float>(motor.cmd_pos),
                                 static_cast<float>(motor.cmd_vel),
                                 static_cast<float>(motor.cmd_tau));
      }
      else if (motor.mode_code == static_cast<int>(damiao::Control_Mode_Code::POS_VEL))
      {
        controller_->control_pos_vel(*motor.motor,
                                     static_cast<float>(motor.cmd_pos),
                                     static_cast<float>(motor.cmd_vel));
      }
      else if (motor.mode_code == static_cast<int>(damiao::Control_Mode_Code::VEL))
      {
        controller_->control_vel(*motor.motor, static_cast<float>(motor.cmd_vel));
      }
    }
  }

  void publishStatus(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publishLegGroup(groups_[0], lf_pub_);
    publishLegGroup(groups_[1], lm_pub_);
    publishLegGroup(groups_[2], lb_pub_);
    publishLegGroup(groups_[3], rf_pub_);
    publishLegGroup(groups_[4], rm_pub_);
    publishLegGroup(groups_[5], rb_pub_);
  }

  void publishLegGroup(const LegGroup& group, ros::Publisher& pub)
  {
    // motor_control format:
    // [pos1, pos2, pos3, tau1, tau2, tau3, vel1, vel2, vel3]
    std_msgs::Float64MultiArray msg;
    msg.data.reserve(9);
    for (std::size_t i = 0; i < 3; ++i)
    {
      const auto& motor = motors_.at(group.ids[i]);
      msg.data.push_back(static_cast<double>(motor.motor->Get_Position()) * group.pn[i]);
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
      const auto& motor = motors_.at(group.ids[i]);
      msg.data.push_back(static_cast<double>(motor.motor->Get_tau()) * group.pn[i]);
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
      const auto& motor = motors_.at(group.ids[i]);
      msg.data.push_back(static_cast<double>(motor.motor->Get_Velocity()) * group.pn[i]);
    }
    pub.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber lf_sub_;
  ros::Subscriber lm_sub_;
  ros::Subscriber lb_sub_;
  ros::Subscriber rf_sub_;
  ros::Subscriber rm_sub_;
  ros::Subscriber rb_sub_;
  ros::Publisher lf_pub_;
  ros::Publisher lm_pub_;
  ros::Publisher lb_pub_;
  ros::Publisher rf_pub_;
  ros::Publisher rm_pub_;
  ros::Publisher rb_pub_;
  ros::Timer control_timer_;
  ros::Timer status_timer_;

  std::string can_interface_{"can0"};
  int thread_priority_{95};
  double control_hz_{500.0};
  double status_hz_{100.0};
  int motor_type_{static_cast<int>(damiao::DM4340)};

  std::mutex mutex_;
  std::array<LegGroup, 6> groups_;
  std::unordered_map<uint16_t, damiao::DmActData> dm_data_;
  std::unordered_map<int, MotorRuntime> motors_;
  std::unique_ptr<damiao::Motor_Control> controller_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dm_motor_fusion_node");
  ros::NodeHandle nh;
  DmMotorFusionNode node(nh);
  ros::spin();
  return 0;
}
