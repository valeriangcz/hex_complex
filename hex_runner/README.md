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
