# 六腿统一状态话题（替代原 /LF|LM|LB|RF|RM|RB/sita_cur 六个话题）
#
# option 为当前控制模式，含义同 MultiLegCommand.option：
#   -1 失能, 0 零力矩, 1 MIT, 2 位置速度
int8 option

LegState left_front   # 左前腿（3 电机）
LegState left_mid     # 左中腿（3 电机）
LegState left_back    # 左后腿（3 电机）
LegState right_front  # 右前腿（3 电机）
LegState right_mid    # 右中腿（3 电机）
LegState right_back   # 右后腿（3 电机）
