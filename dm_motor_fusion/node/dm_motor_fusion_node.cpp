#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <array>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>

#include <dm_motor_fusion/MultiLegCommand.h>
#include <dm_motor_fusion/MultiLegState.h>

#include "damiao.h"

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

// 关节顺序与消息字段对应：0=脚踝(ankle), 1=膝关节(knee), 2=大腿(thigh)
// 即组内 CAN ID 从小到大：如 LF 腿 1/2/3 = 脚踝/膝/大腿
double jointPos(const dm_motor_fusion::LegCommand& leg, std::size_t joint)
{
  switch (joint)
  {
    case 0:
      return leg.ankle_pos;
    case 1:
      return leg.knee_pos;
    default:
      return leg.thigh_pos;
  }
}

double jointVel(const dm_motor_fusion::LegCommand& leg, std::size_t joint)
{
  switch (joint)
  {
    case 0:
      return leg.ankle_vel;
    case 1:
      return leg.knee_vel;
    default:
      return leg.thigh_vel;
  }
}

double jointTau(const dm_motor_fusion::LegCommand& leg, std::size_t joint)
{
  switch (joint)
  {
    case 0:
      return leg.ankle_tau;
    case 1:
      return leg.knee_tau;
    default:
      return leg.thigh_tau;
  }
}

double jointKp(const dm_motor_fusion::LegCommand& leg, std::size_t joint)
{
  switch (joint)
  {
    case 0:
      return leg.ankle_kp;
    case 1:
      return leg.knee_kp;
    default:
      return leg.thigh_kp;
  }
}

double jointKd(const dm_motor_fusion::LegCommand& leg, std::size_t joint)
{
  switch (joint)
  {
    case 0:
      return leg.ankle_kd;
    case 1:
      return leg.knee_kd;
    default:
      return leg.thigh_kd;
  }
}

const dm_motor_fusion::LegCommand& legCommandOf(const dm_motor_fusion::MultiLegCommand& msg, std::size_t group_idx)
{
  switch (group_idx)
  {
    case 0:
      return msg.left_front;
    case 1:
      return msg.left_mid;
    case 2:
      return msg.left_back;
    case 3:
      return msg.right_front;
    case 4:
      return msg.right_mid;
    default:
      return msg.right_back;
  }
}

dm_motor_fusion::LegState& legStateOf(dm_motor_fusion::MultiLegState& msg, std::size_t group_idx)
{
  switch (group_idx)
  {
    case 0:
      return msg.left_front;
    case 1:
      return msg.left_mid;
    case 2:
      return msg.left_back;
    case 3:
      return msg.right_front;
    case 4:
      return msg.right_mid;
    default:
      return msg.right_back;
  }
}

void setJointState(dm_motor_fusion::LegState& leg, std::size_t joint, double pos, double vel, double tau)
{
  switch (joint)
  {
    case 0:
      leg.ankle_pos = pos;
      leg.ankle_vel = vel;
      leg.ankle_tau = tau;
      break;
    case 1:
      leg.knee_pos = pos;
      leg.knee_vel = vel;
      leg.knee_tau = tau;
      break;
    default:
      leg.thigh_pos = pos;
      leg.thigh_vel = vel;
      leg.thigh_tau = tau;
      break;
  }
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

    cmd_sub_ = nh_.subscribe("/sita_des", 20, &DmMotorFusionNode::commandCallback, this);
    state_pub_ = nh_.advertise<dm_motor_fusion::MultiLegState>("/sita_cur", 20);

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

    // 组内 ID 从小到大依次为：脚踝, 膝关节, 大腿
    groups_ = {
        LegGroup{"LF", {1, 2, 3}, {1.0, 1.0, -1.0}},
        LegGroup{"LM", {4, 5, 6}, {1.0, 1.0, -1.0}},
        LegGroup{"LB", {7, 8, 9}, {1.0, 1.0, -1.0}},
        LegGroup{"RF", {10, 11, 12}, {1.0, -1.0, 1.0}},
        LegGroup{"RM", {13, 14, 15}, {1.0, -1.0, 1.0}},
        LegGroup{"RB", {16, 17, 18}, {1.0, -1.0, 1.0}},
    };

    ROS_INFO("dm_motor_fusion ready on %s: cmd /sita_des, state /sita_cur, 18 motors", can_interface_.c_str());
  }

  void commandCallback(const dm_motor_fusion::MultiLegCommand::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int option = msg->option;
    current_option_ = option;

    for (std::size_t g = 0; g < groups_.size(); ++g)
    {
      const auto& leg = legCommandOf(*msg, g);
      for (std::size_t j = 0; j < 3; ++j)
      {
        auto& motor = motors_.at(groups_[g].ids[j]);
        applyModeFromCommand(motor, option);
        if (option == static_cast<int>(HighLevelMode::Disable))
        {
          continue;
        }
        // 零力矩模式已在 applyModeFromCommand 中置零，不使用消息数据
        if (option == static_cast<int>(HighLevelMode::ZeroTorque))
        {
          continue;
        }
        motor.cmd_pos = jointPos(leg, j) * groups_[g].pn[j];
        motor.cmd_vel = jointVel(leg, j);
        motor.cmd_tau = jointTau(leg, j);
        motor.cmd_kp = jointKp(leg, j);
        motor.cmd_kd = jointKd(leg, j);
        motor.has_command = true;
      }
    }
  }

  void applyModeFromCommand(MotorRuntime& motor, const int option)
  {
    if (option == static_cast<int>(HighLevelMode::Disable))
    {
      setMotorEnable(motor, false);
      motor.has_command = false;
      return;
    }

    setMotorEnable(motor, true);
    const int target_mode = toControlModeCode(option);
    switchMotorMode(motor, target_mode);

    if (option == static_cast<int>(HighLevelMode::ZeroTorque))
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
    dm_motor_fusion::MultiLegState msg;
    msg.option = current_option_;

    for (std::size_t g = 0; g < groups_.size(); ++g)
    {
      auto& leg = legStateOf(msg, g);
      for (std::size_t j = 0; j < 3; ++j)
      {
        const auto& motor = motors_.at(groups_[g].ids[j]);
        const double pn = groups_[g].pn[j];
        setJointState(leg, j,
                      static_cast<double>(motor.motor->Get_Position()) * pn,
                      static_cast<double>(motor.motor->Get_Velocity()) * pn,
                      static_cast<double>(motor.motor->Get_tau()) * pn);
      }
    }
    state_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cmd_sub_;
  ros::Publisher state_pub_;
  ros::Timer control_timer_;
  ros::Timer status_timer_;

  std::string can_interface_{"can0"};
  int thread_priority_{95};
  double control_hz_{500.0};
  double status_hz_{100.0};
  int motor_type_{static_cast<int>(damiao::DM4340)};

  int current_option_{static_cast<int>(HighLevelMode::Disable)};
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
