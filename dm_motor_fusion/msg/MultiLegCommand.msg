# 六腿统一控制话题（替代原 /lf|lm|lb|rf|rm|rb/sita_des 六个话题）
#
# option 控制模式：
#   -1 失能 disable
#    0 零力矩 zero torque
#    1 MIT 模式（position/vel/tau + kp/kd 全参数控制）
#    2 位置速度模式（使用 position/vel，忽略 tau/kp/kd）
int8 OPTION_DISABLE     = -1
int8 OPTION_ZERO_TORQUE = 0
int8 OPTION_MIT         = 1
int8 OPTION_POS_VEL     = 2
int8 option

LegCommand lf   # 左前腿（3 电机）
LegCommand lm     # 左中腿（3 电机）
LegCommand lb    # 左后腿（3 电机）
LegCommand rf  # 右前腿（3 电机）
LegCommand rm    # 右中腿（3 电机）
LegCommand rb   # 右后腿（3 电机）
