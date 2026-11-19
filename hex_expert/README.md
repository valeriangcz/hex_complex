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

complex 可通过 runner 的 `initialization.method: set_init` 选择手动目标初始化，
`ExpertComplex.SetInit(q_init, initial_pose=.../initial_waypoint_index=...)` 只建立目标及重置历史；
两种方法都只计算最终初态。物理分段动作由独立的 `initialization.staged` 开关决定，默认 false。
complex 仓库 YAML 显式使用 `set_init_by_se3`、`staged: false`；改为 true 即可过渡到 SE3 求出的目标。
climb 的 SetInit 也只建立目标；staged 为 true 时，runner 执行零力矩等待、抬脚、目标插值和磁吸确认。

runner 直接创建 ExpertComplex，传入 robot_urdf、resources、initial_pose、ik_branch 等参数；
构造函数加载缓存和路径并调用 `SetInitBySE3()`。不提供 `from_config()`。
显式注入模型的低层调用采用 `LoadSE3(path)` → `SetInitBySE3()` 或手动 `SetInit(q_init)`；
不提供 `Reset(q_init)`。再次调用 `SetInitBySE3()` 会清空运行历史并重新选点，
保留控制周期、速度、力矩限值和已加载路径。

```python
expert.LoadSE3(path)
q, tau_ff, adhesions = expert.SetInitBySE3(
    ik_branch=initial.ik_branch, initial_pose=SE3(initial.W_T_R)
)
```

`initial_pose` 默认是路径首个位姿，`ik_branch` 默认保留各腿当前分支。
初始化支撑点和运行时摆腿落脚点均通过 `_SelectLandingPoint` 选择加权代价最小的可行候选，
代价相同时取候选顺序中的第一个；初始支撑腿按环境法向设置足舵机角度。
五项代价为：x_b3 与负表面法向的夹角除以 90°后平方、法向与连杆平面的夹角除以 20°后平方、
边界距离低于 5 cm 的不足量除以 3 cm 后平方、承载力低于 100 N 的不足量除以 10 N 后平方、
移动距离除以 0.1 m 后平方。以上阈值随 HexState 对应参数变化，距离尺度固定为 0.1 m。
默认权重为 `xb3: 1.0, plane: 2.0, boundary: 1.0, force: 1.0, distance: 0.2,
neighbor_end: 1.0, neighbor_ankle: 1.0`，
可通过构造参数或 runner YAML 的 `expert_parameters.landing_cost_weights` 覆盖部分权重。
权重须非负且至少一项为正。运行时距离从实测足端世界位置转换至目标机身系后计算；
初始化尚无实测足端，使用各腿收腿参考位置计算距离。最近一次评分各项和总成本保存在
`_last_landing_cost`，仅供诊断。代价选择仍沿用上游候选筛选。
新增相邻腿末端间距和 ankle 间距两项代价，分别由 `neighbor_end`、`neighbor_ankle` 加权。
邻接关系：lb/lf 比较 lm，rb/rf 比较 rm；lm 比较 lb、lf，rm 比较 rb、rf。
中间腿先取到两侧腿距离的平均值，再计算代价；末端和 ankle 分开处理。
末端间距低于 10 cm 的不足量除以 5 cm 后平方，ankle 间距低于 15 cm 的不足量除以
5 cm 后平方；尺度分别随末端硬间距、ankle 碰撞半径变化。超过对应间距后不再惩罚。
运行时相邻支撑腿取轨迹终点，其他腿取当前末端转换至目标机身系的位置；
ankle 根据对应末端和 IK 分支查缓存后转换至同一机身系。每轮参考位置固定，不随选点顺序改变。
初始化优先使用清空前的有效当前位置，首次初始化使用收腿参考位。
两个新权重均设为 0 时关闭新代价。诊断字段还包括 `neighbor_indices` 及候选与各邻腿的
`neighbor_end_distances`、`neighbor_ankle_distances`（未启用的距离计算可能为 None）。
摆腿按腿独立筛选和规划：至少一个候选即可尝试，未来交集为空时保留该腿上一轮非空候选，
其他腿继续前瞻。无候选或单腿数值计算失败时，只将该腿保留在当前足端的单点轨迹，
并继续计算其余腿；已完成的轨迹保留。失败腿及原因记录在
`_last_swing_planning_failures`，每轮摆腿规划重新清空。该行为不改变步态切换条件。
无当前分支可行点时直接报错，不切换分支。非支撑腿处于收腿参考位，foot 角为零。
支撑法向同步到 `B_support_n` 和 `B_landing_n`。未传实际 `attached` 反馈时，
初始化前馈为零，由后续 `RequestSingleStep` 计算。

complex 的 SetInitBySE3 路径中，`initial_state` 仅保留机身 `pose`、`groups`、`stance_group`、
`ik_branch`、`magnetic` 和 `servo_enabled`，不再支持 `mode/q/feet/frame`。
runner 从 Expert 取得计算结果并生成 `hex_runner/config/to_sim_info.yaml`，Expert 本身不写文件。
仿真只读取该文件获得初态和参考路径，不加载规划地图或调用专家。
`sim.launch data_root:=...` 只交给 runner 创建 Expert 时使用，reset 不重新加载缓存。

## 四足 ExpertQuadClimb

`from hex_expert import ExpertQuadClimb`。构造时传四足 URDF 和必填
`sequence=[0,3,1,2]`；不再接受 `groups`、`stance_group`。
sequence 必须包含四个不重复整数索引 0..3，数组顺序为 `lf/lb/rf/rb`。
关节、扭矩和磁吸输入分别为 `(4,4)`、`(4,3)`、`(4,)`，
返回目标、前馈、吸附开关和舵机使能。

初始化四脚支撑并开启磁吸，从 sequence 首项开始，逐腿执行
等待吸附→等待释放→抬起回位→下降→触地支撑。
触地使用连续压力及法向速度确认，磁吸反馈不能代替触地；
触地后四脚共同移动，全部确认吸附后才释放下一腿。
高点以实测三维距离不超过 5 mm 确认，下降时开启磁吸，摆动腿平面运动使用与支撑腿相反的速度指令。
`_ComputeFootTargets` 中的 `swing_scale=2.0` 控制摆动倍率，同时作用于抬腿回位、下降及摆动腿平面运动；支撑腿速度不变，抬腿与下降速度上限保留 1.2 m/s。
Reset/SetInit 恢复四脚支撑和首腿顺序，零速度保持当前阶段。
完整 ROS 与仿真入口见 hex_runner 的 `quad_climb` 模式。
