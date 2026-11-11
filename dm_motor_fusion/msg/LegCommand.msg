# 单腿控制量：按从小到大排列，脚踝(ankle) → 膝关节(knee) → 大腿(thigh)
# 关节与 CAN ID 对应：LF=1/2/3, LM=4/5/6, LB=7/8/9, RF=10/11/12, RM=13/14/15, RB=16/17/18
# （每组内第一个 ID 为脚踝，第二个为膝关节，第三个为大腿）

# 脚踝
float64 ankle_pos    # 目标位置 (rad)
float64 ankle_vel    # 目标速度 (rad/s)
float64 ankle_tau    # 前馈力矩 (N·m)
float64 ankle_kp     # 位置环增益
float64 ankle_kd     # 速度环增益

# 膝关节
float64 knee_pos
float64 knee_vel
float64 knee_tau
float64 knee_kp
float64 knee_kd

# 大腿
float64 thigh_pos
float64 thigh_vel
float64 thigh_tau
float64 thigh_kp
float64 thigh_kd
