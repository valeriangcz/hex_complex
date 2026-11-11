# hex_expert

ExpertComplex 专家控制算法。运行时依赖注入运动学与可行性对象，不读取轨迹文件或创建仿真器。

`ExpertClimb` 是单环境 NumPy 速度指令爬行专家，复用当前 `Kinematic`
的正运动学、`DampInvJac`、URDF 质量模型和 `CF_Estimate`，不依赖 Torch
或 Isaac Lab，也不创建仿真器。

```python
import numpy as np
from spatialmath import SE3
from hex_kinematic import Kinematic
from hex_expert import ExpertClimb

kin = Kinematic(urdf_path="path/to/hex_all.urdf")
expert = ExpertClimb(kin, control_dt=0.01,
                     motor_torque_limits=27.0, adhesion_force_limit=300.0)
# 所有输入按 LB/LF/LM/RB/RF/RM 顺序。q_cur (6,4)，q_torque (6,3)，
# adhesion_force (6,) 为每足吸附力大小；cur_se3 为当前 W_T_R。
# 每个控制周期用真实反馈调用一次：
q_des, tau_ff, adhesions = expert.RequestSingleStep(
    cur_se3, q_cur, q_torque, adhesion_force,
    command=np.array([0.02, 0.0, 0.0]),  # 机身 vx、vy (m/s)、yaw_rate (rad/s)
)
```

返回 `(6,4)` 关节目标、`(6,3)` 前馈扭矩和 `(6,)` 吸附开关的独立副本。
`SetInit(q_init)` 可显式初始化；否则首次单步从 `q_cur` 初始化。
`Reset()` 清除历史，下一次单步重新初始化；`Reset(q_init)` 同时指定初始姿态。
零速度保持目标关节位置并继续计算重力前馈。与原版爬行专家一致，摆动腿目标
前馈为零，支撑切换时用 0.1 s 平滑过渡。

落脚不需要外部接触力。`CF_Estimate(q_torque, q_cur[:,:3])` 估计足端对环境
作用力，投影到腿局部落脚法向（默认 +z）。复用 `ExpertComplex` 的负向压力
和累计计数规则：默认 `<= -7 N` 累计 10 次判为接触，未命中不清除计数。
仅完成抬腿回位的摆动腿参与检测；停止指令、重置和步态组切换会清除计数。
`estimated_contact_force` 可检查法向估计值。阈值和次数可通过
`contact_force_threshold`、`contact_confirm_steps` 调整。

运行无仿真器测试：

```bash
python3 -m unittest discover -s src/hex_expert/tests -v
```
