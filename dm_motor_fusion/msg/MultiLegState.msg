std_msgs/Header header
# lf, lm, lb, rf, rm, rb; within each leg: ankle, knee, thigh.
# False until real CAN feedback is received, and again after feedback timeout.
bool[18] valid

# 六腿统一状态话题（替代原 /lf|lm|lb|rf|rm|rb/sita_cur 六个话题）
#
# option 为当前控制模式，含义同 MultiLegCommand.option：
#   -1 失能, 0 零力矩, 1 MIT, 2 位置速度
int8 option

LegState lf   # 左前腿（3 电机）
LegState lm     # 左中腿（3 电机）
LegState lb    # 左后腿（3 电机）
LegState rf  # 右前腿（3 电机）
LegState rm    # 右中腿（3 电机）
LegState rb   # 右后腿（3 电机）
