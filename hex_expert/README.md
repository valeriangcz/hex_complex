# hex_expert

ExpertComplex 专家控制算法。runner 传入资源路径和初态参数，由构造函数加载运动学、可行性缓存和轨迹并计算初态；也保留显式注入对象的低层接口用于测试。Expert 不创建仿真器。

`ExpertClimb` 是单环境 NumPy 速度指令爬行专家，复用当前 `Kinematic`
的正运动学、`DampInvJac`、URDF 质量模型和 `CF_Estimate`，不依赖 Torch
或 Isaac Lab，也不创建仿真器。

```python
import numpy as np
from hex_kinematic import Kinematic
from hex_expert import ExpertClimb

kin = Kinematic(urdf_path="path/to/hex_all.urdf")
expert = ExpertClimb(kin, groups=[[0, 1, 5], [2, 3, 4]], stance_group=0, control_dt=0.01,
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
climb 的 `SetInit(q_init)` 可显式初始化；否则首次单步从 `q_cur` 初始化。
climb 的 `Reset()` 清除历史，下一次单步重新初始化；`Reset(q_init)` 同时指定初始姿态。
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

## 单腿摆动轨迹查看器

```bash
source devel/setup.bash
rosrun hex_expert swing_traj_vis.py
rosrun hex_expert swing_traj_vis.py --leg-index 3 \
  --robot-config src/map_robot_voxles/config/hex_magnetic.yaml
```

`hex_expert.swing_traj_vis` 保留原版 B 系摆腿实验：完整可行体素、初始及优化 B 样条、
控制点、起终点法向、接触薄片、额外长方体障碍物及优化状态。仅加载机器人工作空间缓存，
无需地图、逆解或控制器实例；不替换 ExpertComplex 的正式摆腿算法。

模块顶部的 `LEG_CONFIGS` 配置起终点和法向，`SWING_LEG_INDEX` 选择已配置的腿（默认 0，另有 3）。
缓存路径和网格参数从机器人 YAML 读取并校验。支持 `--off-screen --screenshot /tmp/swing.png`。

runner 在 `hex_runner.config` 中解析初始参数；关节求解与环境选点由对应 Expert 完成。runner 配置中的 `initial_state.groups` 定义两组三腿索引，索引顺序为 lb、lf、lm、rb、rf、rm；`stance_group` 选择初始支撑组。两个专家均通过必填关键字参数 `groups`、`stance_group` 接收配置，重置后恢复该初始支撑组。分组字段直接使用，不做主动校验。

在完整工作空间中验证初始状态与分组配置：

```bash
source devel/setup.bash
python3 -m unittest discover -s src/hex_expert/tests -v
```

## complex 环境初始化

runner 直接创建 ExpertComplex，传入 robot_urdf、resources、initial_pose、ik_branch 等参数；
构造函数加载缓存和路径并调用 `SetInitBySE3()`。不提供 `from_config()`。
显式注入模型的低层调用仍采用 `LoadSE3(path)` → `SetInitBySE3()`；旧 `SetInit(q_init)`、
`Reset(q_init)` 接口已移除。再次调用 `SetInitBySE3()` 会清空运行历史并重新选点，
保留控制周期、速度、力矩限值和已加载路径。

```python
expert.LoadSE3(path)
q, tau_ff, adhesions = expert.SetInitBySE3(
    ik_branch=initial.ik_branch, initial_pose=SE3(initial.W_T_R)
)
```

`initial_pose` 默认是路径首个位姿，`ik_branch` 默认保留各腿当前分支。
支撑腿从环境可行候选中选择距工作空间边界最远的点，并按环境法向设置足舵机角度；
无当前分支可行点时直接报错，不切换分支。非支撑腿处于收腿参考位，foot 角为零。
支撑法向同步到 `B_support_n` 和 `B_landing_n`。未传实际 `attached` 反馈时，
初始化前馈为零，由后续 `RequestSingleStep` 计算。

complex YAML 的 `initial_state` 仅保留机身 `pose`、`groups`、`stance_group`、
`ik_branch`、`magnetic`、`motor_mode` 和 `servo_enabled`，不再支持 `mode/q/feet/frame`。
runner 从 Expert 取得计算结果并生成 `hex_runner/config/to_sim_info.yaml`，Expert 本身不写文件。
仿真只读取该文件获得初态和参考路径，不加载规划地图或调用专家。
`sim.launch data_root:=...` 只交给 runner 创建 Expert 时使用，reset 不重新加载缓存。
