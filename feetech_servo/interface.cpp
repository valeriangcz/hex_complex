#include <ros/ros.h>
#include <feetech_servo/servo_cmd.h>
#include <feetech_servo/servo_state.h>
#include <SCSCL.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr std::size_t kServoCount = 6;
const char* const kLegNames[] = {"lf", "lm", "lb", "rf", "rm", "rb"};

// A wrong parameter type must not silently select a fallback hardware setting.
template <typename T>
T parameter(const ros::NodeHandle& nh, const std::string& name, const T& fallback)
{
    T value;
    if (!nh.hasParam(name))
        return fallback;
    if (!nh.getParam(name, value))
        throw std::runtime_error("Invalid parameter type: ~" + name);
    return value;
}

class ServoInterface
{
public:
    ServoInterface() : private_nh_("~")
    {
        const std::string port = parameter(private_nh_, "port", std::string("/dev/ttyUSB0"));
        const int baud = parameter(private_nh_, "baud_rate", 115200);
        const auto ids = parameter(private_nh_, "ids", std::vector<int>{1, 2, 3, 4, 5, 6});
        offsets_ = parameter(private_nh_, "offset_deg", std::vector<double>(kServoCount, 0.0));
        directions_ = parameter(private_nh_, "direction", std::vector<int>(kServoCount, 1));
        span_deg_ = parameter(private_nh_, "angle_span_deg", 300.0);
        raw_max_ = parameter(private_nh_, "raw_max", 1023);
        const int move_time = parameter(private_nh_, "move_time_ms", 1000);
        const int speed = parameter(private_nh_, "speed", 0);
        const int timeout = parameter(private_nh_, "read_timeout_ms", 20);
        const double state_hz = parameter(private_nh_, "state_hz", 20.0);
        if (!std::isfinite(state_hz) || state_hz < 0.0)
            throw std::runtime_error("state_hz must be finite and nonnegative");

        if (ids.size() != kServoCount || offsets_.size() != kServoCount ||
            directions_.size() != kServoCount)
            throw std::runtime_error("ids, offset_deg and direction must each contain six values");
        if (std::set<int>(ids.begin(), ids.end()).size() != kServoCount)
            throw std::runtime_error("Servo IDs must be unique");
        for (std::size_t i = 0; i < kServoCount; ++i)
        {
            if (ids[i] < 0 || ids[i] > 253 || !std::isfinite(offsets_[i]) ||
                (directions_[i] != 1 && directions_[i] != -1))
                throw std::runtime_error("Invalid ID, offset or direction for " + std::string(kLegNames[i]));
            ids_[i] = static_cast<u8>(ids[i]);
        }
        if (!std::isfinite(span_deg_) || span_deg_ <= 0.0 || raw_max_ < 1 || raw_max_ > 65535)
            throw std::runtime_error("angle_span_deg must be positive; raw_max must be in [1, 65535]");
        if (move_time < 0 || move_time > 65535 || speed < 0 || speed > 65535)
            throw std::runtime_error("move_time_ms and speed must be in [0, 65535]");
        // The vendor read routine stores this interval in timeval.tv_usec.
        if (timeout < 1 || timeout > 999)
            throw std::runtime_error("read_timeout_ms must be in [1, 999]");
        const std::vector<int> supported_baud{9600, 19200, 38400, 57600, 115200, 500000, 1000000};
        if (port.empty() || std::find(supported_baud.begin(), supported_baud.end(), baud) == supported_baud.end())
            throw std::runtime_error("Empty port or baud_rate unsupported by SCSerial::begin");

        times_.fill(static_cast<u16>(move_time));
        speeds_.fill(static_cast<u16>(speed));
        servo_.IOTimeOut = static_cast<unsigned long>(timeout);
        if (!servo_.begin(baud, port.c_str()))
        {
            servo_.end();
            throw std::runtime_error("Cannot open/configure servo port: " + port);
        }

        state_pub_ = nh_.advertise<feetech_servo::servo_state>("servo_state", 1);
        command_sub_ = nh_.subscribe("servo_cmd", 1, &ServoInterface::commandCallback, this);
        if (state_hz > 0.0)
            state_timer_ = nh_.createTimer(ros::Duration(1.0 / state_hz),
                [this](const ros::TimerEvent&) { publishState(); });
        ROS_INFO("SCS2332 interface on %s at %d baud; command=%s, state=%s",
                 port.c_str(), baud, command_sub_.getTopic().c_str(), state_pub_.getTopic().c_str());
    }

    ~ServoInterface()
    {
        servo_.end();
    }

private:
    void commandCallback(const feetech_servo::servo_cmd::ConstPtr& msg)
    {
        const std::array<bool, kServoCount> enabled{{
            msg->enable && msg->enable_lf, msg->enable && msg->enable_lm,
            msg->enable && msg->enable_lb, msg->enable && msg->enable_rf,
            msg->enable && msg->enable_rm, msg->enable && msg->enable_rb}};
        const std::array<double, kServoCount> angles{{msg->lf, msg->lm, msg->lb, msg->rf, msg->rm, msg->rb}};
        std::array<u8, kServoCount> active_ids;
        std::array<u16, kServoCount> positions, times, speeds;
        std::size_t count = 0;
        for (std::size_t i = 0; i < kServoCount; ++i)
        {
            if (!enabled[i])
                continue;  // Disabled legs have no position target to validate or write.
            const double absolute = offsets_[i] + directions_[i] * angles[i];
            if (!std::isfinite(angles[i]) || !std::isfinite(absolute) || absolute < 0.0 || absolute > span_deg_)
            {
                ROS_WARN_THROTTLE(1.0, "Rejecting entire command: %s enabled target is non-finite or outside absolute [0, %.3f] degrees",
                                  kLegNames[i], span_deg_);
                return;
            }
            active_ids[count] = ids_[i];
            positions[count] = static_cast<u16>(std::lround((absolute / span_deg_) * raw_max_));
            times[count] = times_[i];
            speeds[count] = speeds_[i];
            ++count;
        }

        // Validate all active targets before changing any torque-enable register.
        setTorqueEnabled(enabled);
        // Only actively controlled servos receive position writes.
        if (count > 0)
            servo_.SyncWritePos(active_ids.data(), static_cast<u8>(count), positions.data(), times.data(), speeds.data());
        publishState();
    }

    void setTorqueEnabled(const std::array<bool, kServoCount>& enabled)
    {
        std::array<u8, kServoCount> torque;
        for (std::size_t i = 0; i < kServoCount; ++i)
            torque[i] = enabled[i] ? 1 : 0;
        // Address only the six configured IDs, not every servo on the bus.
        servo_.syncWrite(ids_.data(), static_cast<u8>(kServoCount),
                         SCSCL_TORQUE_ENABLE, torque.data(), 1);
    }

    void publishState()
    {
        feetech_servo::servo_state state;
        state.header.stamp = ros::Time::now();
        const std::array<double*, kServoCount> fields{{&state.lf, &state.lm, &state.lb, &state.rf, &state.rm, &state.rb}};
        for (std::size_t i = 0; i < kServoCount; ++i)
        {
            const int raw = servo_.ReadPos(ids_[i]);
            state.valid[i] = raw >= 0 && raw <= raw_max_ && servo_.getState() == 0;
            *fields[i] = state.valid[i]
                ? directions_[i] * ((static_cast<double>(raw) / raw_max_) * span_deg_ - offsets_[i])
                : std::numeric_limits<double>::quiet_NaN();
            if (!state.valid[i])
                ROS_WARN_THROTTLE(1.0, "Invalid feedback: %s, id=%u, raw=%d, communication_error=%u, servo_status=%u",
                                  kLegNames[i], static_cast<unsigned>(ids_[i]), raw,
                                  static_cast<unsigned>(servo_.getLastError()), static_cast<unsigned>(servo_.getState()));
        }
        state_pub_.publish(state);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    SCSCL servo_;
    ros::Publisher state_pub_;
    ros::Subscriber command_sub_;
    ros::Timer state_timer_;
    std::array<u8, kServoCount> ids_;
    std::array<u16, kServoCount> times_;
    std::array<u16, kServoCount> speeds_;
    std::vector<double> offsets_;
    std::vector<int> directions_;
    double span_deg_;
    int raw_max_;
};
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feetech_servo_interface");
    try
    {
        ServoInterface interface;
        ros::spin();
    }
    catch (const std::exception& error)
    {
        ROS_FATAL("Servo interface failed: %s", error.what());
        return 1;
    }
    return 0;
}
