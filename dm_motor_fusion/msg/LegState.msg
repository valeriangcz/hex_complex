# 单腿测量状态：按从小到大排列，脚踝(ankle) → 膝关节(knee) → 大腿(thigh)

# 脚踝
float64 ankle_pos    # 当前位置 (rad)
float64 ankle_vel    # 当前速度 (rad/s)
float64 ankle_tau    # 当前力矩 (N·m)

# 膝关节
float64 knee_pos
float64 knee_vel
float64 knee_tau

# 大腿
float64 thigh_pos
float64 thigh_vel
float64 thigh_tau
