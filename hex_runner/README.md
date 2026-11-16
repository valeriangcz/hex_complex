磁吸接口：`attached_*` 为归一化浮点读数，`enabled_*` 为实际通电状态。runner 原样传递读数，
两个 expert 使用本模式 YAML 的 `expert_parameters.magnetic_attach_above`（默认 0.85）和
`magnetic_release_below`（默认 0.05）作六腿统一判断；中间区不确认吸附或释放，不影响 valid。

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
重新初始化、写文件并发送关节目标；READY 阶段全部 foot 舵机使能，RUNNING 后恢复步态使能规则。
sim 物理 reset 重新读取文件并恢复初态和绘制路径。两种 reset 独立，不因文件修改自动重置物理机器人。
操作时先暂停控制，再完成需要的 runner 初始化文件更新，最后触发 sim reset；不要同时重写和读取该文件。
单独启动 sim 时，文件不存在或无法读取会直接报错退出。
