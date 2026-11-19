磁吸接口：`attached_*` 为归一化浮点读数，`enabled_*` 为实际通电状态。runner 原样传递读数，
两个 expert 使用本模式 YAML 的 `expert_parameters.magnetic_attach_above`（默认 0.85）和
`magnetic_release_below`（默认 0.05）作六腿统一判断；中间区不确认吸附或释放，不影响 valid。

## complex H5 记录

complex 仿真和实物共用记录功能，默认启用；可在 YAML 中设置
`recording: {enabled: false}` 关闭。其他专家不会创建记录文件。
依赖 `python3-h5py`（可通过 `rosdep install --from-paths src --ignore-src -y` 安装项目依赖）。
从节点启动到退出，一次运行生成一份 `${data_root}/h5_log/complex_时间_唯一标识.h5`；
`data_root` 默认是工作区的 `data`，也可使用 launch 的 `data_root` 参数指定。
reset 不分文件，暂停、失能和故障仍继续记录。

`feedback/{pose,motor,servo,magnetic}` 按消息到达保存源 ROS 时间戳 `stamp_ns`、
主机单调接收时间 `received_ns`、有效标记和原始反馈。
pose 保存世界系位置和 xyzw 四元数；motor 保存位置、传感器速度和力矩；
servo 保存角度；magnetic 保存吸附比例和使能。不会差分推算机身速度。
反馈未到达时对应数据集为空；无效数据仍保存，但 `valid=false`，不可当成有效测量。

`runner` 按控制周期保存状态、初始化阶段、reset/compute 计数、失能/吸附关闭标记和故障原因；
`planning` 保存规划落脚点 `landing_leg` / `landing_world`、支撑标记 `stance`、
六腿 `trajectory_index` / `trajectory_id`、`body_trajectory_id` 和摆动规划失败信息。
`stance=true` 为支撑腿，false 为摆动腿。规划信息与传感器测量分组保存。

`trajectories/feet` 与 `trajectories/body` 为轨迹版本表，含 `id, offset, length, received_ns`，
feet 表额外含 `leg`。用 offset 和 length 从 `feet_points` / `body_points` 切片重建整条轨迹；
长度为零表示轨迹清空。相同轨迹不重复保存，单腿内容变化独立产生新版本。
feet_points 为各腿基坐标系 B 下的 `(N,3)` 点，body_points 为世界系下的 `(N,4,4)` 机身位姿。
轨迹采样间隔见文件属性 `trajectory_dt`；腿顺序、配置、单位和坐标系也在属性中。

H5 单独线程分块追加，每秒 flush，正常退出会排空队列并关闭。
文件属性 `normal_close=true, incomplete=false` 表示正常完成；强制结束进程可能丢失最近尚未刷新的数据，
不保证文件可恢复。队列溢出、路径不可写或写盘失败会打印红色错误并停止记录，控制继续运行；
能继续写入文件时保存错误和 `incomplete=true`。记录线程不执行机器人控制。

## 感知输入

climb 固定配置 `pose_source: gravity`，订阅 `/hex/perception/gravity`：
`std_msgs/Float64MultiArray.data = [gx, gy, gz]`，表示机身坐标系指向物理下方的单位向量。
runner 校验长度、有限性与非零性，并归一化；不构造 SE3 或虚构平移。
expert 将单位方向乘以 9.81 m/s² 计算重力补偿。

complex 固定配置 `pose_source: se3`，订阅 `/hex/perception/pose`：
`geometry_msgs/PoseStamped`，frame_id=`world`，描述机身在世界中的完整位姿。
位置单位 m，姿态为单位四元数 xyzw。complex 不依赖重力 topic。
旧 `pose_source: imu` 与 `/hex/perception/imu` 接口已移除。

两种模式均要求电机、舵机、磁吸及本模式感知反馈有效且新鲜。
无效方向或超时沿用现有状态机：未初始化时等待，初始化后进入 FAULT 并保持上次输出；
不自动失能或释放磁铁。无 Header 的重力消息采用主机接收时间计算新鲜度，
发布端只发布新的有效四元数结果，不周期重发旧结果。

## 启动

IMU 独立启动（不启动执行器）：

```bash
roslaunch perception imu.launch
```

串口、频率、IMU 到机身旋转矩阵配置见 `perception/config/imu.yaml`。
设备波特率由用户设置，必须与 YAML 一致；默认 115200、100 Hz。
接入实机总 launch 时，设置已有的 `perception_launch` 参数为 `perception/launch/imu.launch` 的绝对路径。
自定义配置可单独启动 IMU，再启动 runner；避免同时运行两个串口读取节点。

仿真仍使用 `roslaunch hex_runner sim.launch mode:=climb`，模拟后端发布相同的重力方向接口。
`initial_state.pose` 仍用于定义仿真初态，不是 climb 的运行时输入。

假设备集成回归（均启动隔离的 ROS master，不访问真实执行器）：

```bash
python3 src/hex_runner/scripts/test_ros_integration.py --backend fake --mode climb
python3 src/hex_runner/scripts/test_ros_integration.py --backend fake --mode complex
```

## 排查 climb 摆腿不落地

climb 诊断提供 leg_names、swing_reached、contact_count、estimated_contact_force、
normal_foot_velocity、foot_target_leg、foot_measured_leg、workspace_ok、clearance_ok。
这些数组按 leg_names 对应；位置和法向速度均在各腿局部系，单位 m、m/s，力单位 N。
正常接触需连续受压且法向运动接近停止；仅看到负等效力不能认定脚已接触地面。
抬腿到位以实测位置确认；下降阶段保持横向落脚目标，防止高速横向推进导致吸附后反向拖动。
通过模拟后端的 contact_force_norm 和 adhesion_force 可进一步区分真实接触、磁吸确认与专家误判。
这些仿真物理量只作诊断，不作为实物专家的额外输入。

横向闭环回归可指定物理速度（测试脚本会转换成 runner 的原始轴值）：

```bash
python3 src/hex_runner/scripts/test_ros_integration.py --backend isaac --mode climb --device cpu \
  --climb-velocity 0 0.4 0 --motion-seconds 30 --min-gait-switches 4
```

实际可用速度还受跟踪增益、关节限位、摩擦和吸附状态影响；一次平面仿真不能替代实物速度标定。

## 初始化与仿真初态文件

目标计算方法和物理过渡独立配置。`method` 选择 Expert 如何计算最终初态，省略时统一为
`set_init`；`staged` 选择 runner 是否执行过渡，省略时统一为 `false`，不由专家类型或方法名推导。
仓库 climb YAML 显式开启过渡；complex YAML 使用 SE3 目标并关闭过渡；四足暂不支持开启过渡，
配置为 `staged: true` 时会在加载阶段报错。

设置 `staged: true` 时，收到 `/hex/runner/reset` 后才执行：
循环发送模式 0、关闭全部磁吸 → 等待新鲜完整反馈并确认全部释放 → 模式 1 插值到
程序内六脚抬起参考位 → 模式 1 插值到配置目标并调用现有专家前馈 →
按 `magnetic_hz` 循环设置配置磁吸 → 确认开启腿吸附、关闭腿释放后进入 READY。
没有完整反馈也允许提交此 reset，等待阶段仅发送模式 0 和磁吸关闭命令，不发送舵机目标。
首次反馈等待及吸附等待持续到满足条件；运动期间反馈失效则进入 FAULT。
初始化中拒绝运动指令，失能或释放磁吸可中断；恢复需重新 reset。

```yaml
initialization:
  method: set_init  # complex 也可以选择 set_init_by_se3
  staged: true     # false 直接应用最终目标；true 执行过渡
```

complex 的 `set_init` 和 `set_init_by_se3` 都可独立设置 staged。
`prepare_initialization()` 每次 reset 只准备一次目标，过渡结束由 `finish_initialization()`
提交缓存的最终状态并保留前馈，不会再次选点或求解。`staged: false` 时直接应用准备结果并进入 READY。

complex 选择 `set_init` 时，在现有机身 pose/waypoint_index、分组和 IK 分支之外，
手动目标复用 climb 的 `initial_state.mode: joints` 加 `q`，或 `mode: feet` 加 `frame: leg`、`feet`。
每腿字段按 lb/lf/lm/rb/rf/rm 命名；关节目标指定 thigh/knee/ankle，foot 角由专家计算。
示例（其他 initial_state 字段仍需提供）：

```yaml
initial_state:
  mode: feet
  frame: leg
  feet:
    lb: {x: 0.18, y: 0.08, z: -0.08}
    lf: {x: 0.18, y: -0.08, z: -0.08}
    lm: {x: 0.19, y: 0, z: -0.02}
    rb: {x: 0.18, y: -0.08, z: -0.02}
    rf: {x: 0.18, y: 0.08, z: -0.02}
    rm: {x: 0.19, y: 0, z: -0.08}
```

抬脚参考位内设于程序：四角腿局部坐标 x=0.18、y=±0.08、z=0.04 m，中腿为
(0.19,0,0.04) m。按当前 IK 分支求解，不从 YAML 读取。
插值为五次平滑曲线，最大关节速度 0.3 rad/s；每段结束需电机误差不超过 0.03 rad、
舵机误差不超过 0.05 rad，连续满足 10 个新电机反馈样本。重复样本不推进流程。
第二段按专家配置的初始步态和吸附目标计算现有前馈，实际磁吸命令仍保持关闭，
到达目标后才开启。无支撑目标时 climb 的现有前馈可能为零，不额外新增补偿算法。
诊断包含 initialization_stage、initialization_waiting_reason、initialization_waiting_legs、
initialization_motor_error 和 initialization_servo_error；reset_count 在成功完成时才增加。
四足保留直接初始化，complex 的 SetInitBySE3 函数内部逻辑不变。所有目标控制使用模式 1，
ROS 电机模式 2 已删除；模式 0 为零力矩，模式 -1 为失能。

runner 只解析 YAML 参数，直接创建 `ExpertComplex(...)` 或 `ExpertClimb(...)`。
Expert 在创建时完成模型加载和初态计算；complex 还加载唯一一套 voxel 与参考路径。
Adapter 不单独保存路径或环境，规划和完成判断使用 Expert 内部状态。
complex 继续保持配置的 IK 分支，按环境可行性和表面法向生成初始支撑落脚点与 foot 角度。
climb 的 joints/feet 配置由对应 Expert 求解。

runner 创建 Expert 后写入 `config/to_sim_info.yaml`，其中包括机身位姿、六腿关节角、
磁吸状态、电机/舵机使能和世界坐标参考路径。可用 `to_sim_info:=/path/to/file.yaml` 覆盖。
这是运行生成的普通 YAML，不需要手工填写；没有启动标识、版本、轮询或文件校验协议。

```bash
# 一条命令：先创建 Expert 并写文件，再启动仿真。
roslaunch hex_runner sim.launch mode:=complex headless:=true

# 也可以分开启动：先 runner，确认 HEX_RUNNER_INITIALIZED 后再启动 backend。
roslaunch hex_runner runner.launch mode:=complex
roslaunch hex_isaacgym backend.launch \
  config:=$PWD/src/hex_isaacgym/config/complex_surface4.yaml
```

`sim.launch` 的启动脚本转发 runner 输出，收到 `HEX_RUNNER_INITIALIZED` 后才启动 sim；
runner 初始化失败则不创建仿真。不靠旧文件是否存在判断就绪，也不轮询文件。
两个模式使用同样的文件接口。`data_root` 只交给 runner/Expert；sim 不读取 voxel 或原始轨迹文件。
sim 从该文件获取路径绘制数据，原 `/hex/reference_path` topic 已删除。

收到 reset 前，runner 不发送执行器命令。runner reset 复用 Expert 的模型和缓存，
SetInitBySE3/四足直接初始化后写文件并发送一次电机（option=1）与舵机目标；READY 阶段全部 foot 舵机使能，RUNNING 后恢复步态使能规则。
电机和舵机仅在 reset 成功、每次专家成功 compute（电机 option=1）或 disable_actuators（电机 option=-1、舵机全部失能）时各发送一次。
空闲、暂停、完成、故障及没有新电机反馈的 tick 不重复发送缓存目标；磁吸继续按 magnetic_hz 发布。
sim 物理 reset 重新读取文件并恢复初态和绘制路径。两种 reset 独立，不因文件修改自动重置物理机器人。
操作时先暂停控制，再完成需要的 runner 初始化文件更新，最后触发 sim reset；不要同时重写和读取该文件。
单独启动 sim 时，文件不存在或无法读取会直接报错退出。

## complex 初始位姿

可以用 `initial_state.waypoint_index: 0` 选择 `resources.se3_path` 中的原始路标点，
索引从 0 开始，同时取得位置和朝向，不受 dense 加密点数量影响。
runner 只传递索引；ExpertComplex 在轨迹加载后解析它。reset 使用相同选择，
仿真初态导出使用专家成功初始化的实际位姿。参考路径不会被裁剪。

```yaml
initial_state:
  waypoint_index: 0
  # 其余 groups、stance_group、ik_branch、magnetic、servo_enabled 保持原配置
```

也可以继续手写位姿，`pose` 与 `waypoint_index` 必须且只能提供一个：

```yaml
initial_state:
  pose:
    position: {x: 2.3104510167678747, y: 4.4763037345614, z: 0.18266166870934133}
    orientation: {x: 0.00008323880732471449, y: 0.10452843012500451, z: 0.9945215800370322, w: 0.0007919643604078812}
  # 其余初态字段保持原配置
```

`waypoint_index` 仅用于 complex，必须为非负整数；越界或所选位姿无法完成环境／IK
初始化时直接报错，不自动换点。climb 继续使用 `initial_state.pose`。

## 四足单腿循环攀爬

```bash
source devel/setup.bash
roslaunch hex_runner sim.launch mode:=quad_climb
# 仅运行控制入口：
roslaunch hex_runner runner.launch mode:=quad_climb
```

`config/quad_climb.yaml` 使用四足 URDF，编号固定为
`0=lf、1=lb、2=rf、3=rb`。通过 `initial_state.sequence: [0, 3, 1, 2]`
指定逐腿摆动顺序，从第一项开始循环；四足不再接受 `groups` 或 `stance_group`。
六足 climb、complex 仍使用原有分组配置。

初态四脚均为 stance、磁吸全开，足端高度为 `-stance_height`。
调用 `/hex/runner/reset` 后，向 `/hex/cmd_vel` 发送速度启动：
先等待四脚确认吸附，再释放首腿；释放确认后只有该腿摆动，其余三脚移动。
实测足端到达高点（距离不超过 5 mm）后开启磁吸并下降，下降时摆动腿的平面运动使用反向速度指令，与支撑腿相反。
摆动速度倍率为 2：抬腿回位、下降及摆动腿平面运动加速，支撑腿速度保持原值；抬腿和下降速度上限为 1.2 m/s。
连续受压且法向运动接近停止后确认触地，立即设为 stance，以实测触点重建目标，
四脚共同移动。四脚均确认吸附后再释放下一腿，触地同周期不会直接释放下一腿。
磁吸确认本身不会把空中脚设为 stance。

发送零速度保持目标和循环位置；恢复速度后继续，reset 从首腿重新开始。
诊断增加 `gait_phase`（WAIT_ATTACH、RELEASE、LIFT、DESCEND）、
`swing_leg_index`（启动等待时为 null；其后表示本次选中腿）、
`sequence`、`sequence_index`。`gait_switch_count` 表示实际开始单腿摆动的次数。

ROS 消息保留六腿字段，控制和反馈仅映射四条腿；中腿命令为零或关闭，
中腿无效反馈不阻止四足运行。仿真使用 `quad_plane.yaml`，实机驱动配置仍需单独适配。
舵机使能为 `~gaits`，支撑时被动、摆动时跟踪目标；重力补偿使用当前支撑腿，
保留限幅和平滑。`support_moment_residual_R` 为分配阶段、限幅前的机身系剩余力矩（N·m）。

```bash
python3 src/hex_runner/scripts/test_ros_integration.py --backend fake --mode quad_climb
# 五次开始摆动覆盖 0→3→1→2→0；CPU 运行速度可能低于实时。
python3 src/hex_runner/scripts/test_ros_integration.py --backend isaac --mode quad_climb \
  --device cpu --motion-seconds 180 --min-gait-switches 5
```
