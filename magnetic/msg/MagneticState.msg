std_msgs/Header header

# 归一化传感器读数：0=标定最小值，1=标定最大值，不是吸附成功布尔量。
# 仅在对应 valid_* 为 true 时有效；吸附与释放由 expert 的统一阈值判定。
# 腿名：lf/lm/lb=左前/左中/左后，rf/rm/rb=右前/右中/右后。
float32 attached_lf
float32 attached_lm
float32 attached_lb
float32 attached_rf
float32 attached_rm
float32 attached_rb

# 实际通电状态：true=输出已导通，false=输出已断开；通电不代表吸附成功。
# 来自设备 io_state 反馈，不是 MagneticCmd 中的期望通断指令。
bool enabled_lf
bool enabled_lm
bool enabled_lb
bool enabled_rf
bool enabled_rm
bool enabled_rb

# 反馈有效性：true=该腿通电与吸附反馈均有效，false=未知或不可用。
# valid_* 为 false 时，不得将 attached_*=0 当作确认释放成功。
bool valid_lf
bool valid_lm
bool valid_lb
bool valid_rf
bool valid_rm
bool valid_rb
