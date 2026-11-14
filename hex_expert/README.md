# hex_expert

ExpertComplex 专家控制算法。运行时依赖注入运动学与可行性对象，不读取轨迹文件或创建仿真器。

`ExpertClimb` 是单环境 NumPy 速度指令爬行专家，复用当前 `Kinematic`
的正运动学、`DampInvJac`、URDF 质量模型和 `CF_Estimate`，不依赖 Torch
或 Isaac Lab，也不创建仿真器。

```python
import numpy as np
from hex_kinematic import Kinematic
from hex_expert import ExpertClimb

kin = Kinematic(urdf_path="path/to/hex_all.urdf")
expert = ExpertClimb(kin, control_dt=0.01,
                     motor_torque_limits=27.0)
# 所有输入按 lb/lf/lm/rb/rf/rm 顺序。q_cur (6,4)，q_torque (6,3)，
# attached (6,) 为每足归一化传感器读数，范围 [0, 1]；gravity_direction_body 为机身系向下的重力方向。
# 每个控制周期用真实反馈调用一次：
q_des, tau_ff, adhesions, servo_enabled = expert.RequestSingleStep(
    gravity_direction_body, q_cur, q_torque, attached,
    command=np.array([0.02, 0.0, 0.0]),  # 机身 vx、vy (m/s)、yaw_rate (rad/s)
)
```

climb 的重力方向输入为有限非零 `(3,)` 向量，内部归一化并乘以 9.81 m/s²；
不需要 SE3。水平机身 Z 轴朝上时方向为 `[0, 0, -1]`。complex 仍使用完整 SE3。

两个专家均返回 `(6,4)` 关节目标、`(6,3)` 前馈扭矩、`(6,)` 吸附开关、
`(6,)` 舵机使能的独立副本。舵机使能为 bool 数组 `~gaits`：支撑腿失能、摆动腿使能。
所有角度始终是正常弧度值，角度字段不承载失能信息；仿真和实物采用相同规则。
`SetInit(q_init)` 可显式初始化；否则首次单步从 `q_cur` 初始化。
`Reset()` 清除历史，下一次单步重新初始化；`Reset(q_init)` 同时指定初始姿态。
零速度保持目标关节位置并继续计算重力前馈。与原版爬行专家一致，摆动腿目标
前馈为零，支撑切换时用 0.1 s 平滑过渡。

落脚不需要外部接触力。`CF_Estimate(q_torque, q_cur[:,:3])` 估计足端对环境
作用力，投影到腿局部落脚法向（默认 +z）。climb 要求法向估计值 `<= -7 N`，
且实测足端法向速度绝对值 `<= 0.03 m/s`，连续满足 10 次才确认接触；中断即清零计数。
电机驱动扭矩还含惯性、重力和跟踪误差，不能仅凭负向等效力将快速下降的空中腿判为支撑。
仅完成抬腿回位的摆动腿参与检测；停止指令、重置和步态组切换会清除计数。
`estimated_contact_force` 可检查法向估计值。阈值和次数可通过
`contact_force_threshold`、`contact_confirm_steps`、`contact_velocity_threshold` 调整。
法向速度来自实测足端局部位置差分；该检查用于抑制动态误判，不是独立接触传感器。

抬腿是否到位使用实测足端高度；回位目标不越过抬腿端点。回位完成后保持横向目标并下降；
已吸附但尚待压力确认的足端随实测横向位置保持，不继续拖动机身。触地后以实测足端重新建立支撑目标，
支撑高度恢复独立于横向工作空间限制。下降减速因子保留非零下限，目标不低于工作空间 z=-0.2 m。
电机目标按运动学关节限位裁剪，foot 目标随后由裁剪后的 knee/ankle 计算。

运行无仿真器测试：

```bash
python3 -m unittest discover -s src/hex_expert/tests -v
```

吸附与释放由两个 expert 内部判断，六腿共用 `magnetic_attach_above`（默认 0.85）
和 `magnetic_release_below`（默认 0.05）。严格高于前者确认吸附，严格低于后者确认释放；
中间区和等号边界两者均不确认。阈值通过 runner 各模式 YAML 的 `expert_parameters` 传入，
分腿 ADC 标定与有效范围只在 magnetic 驱动 YAML 中设置。
