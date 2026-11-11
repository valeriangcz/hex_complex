# hex_isaacgym

单环境 Isaac Gym 物理后端、执行器、磁吸、状态读取、完整 reset 和 ROS1 闭环协调器。Isaac Gym 必须先于 Torch 导入。

P4 后端只包含 ExpertComplex 所需的物理状态，不包含观测、奖励、episode buffer 或训练依赖。公开顺序固定为 `LB/LF/LM/RB/RF/RM`；后端根据 URDF 名称映射到物理张量，不依赖资产内的索引顺序。

真实 CPU PhysX 冒烟测试：

```bash
python3 src/hex_isaacgym/scripts/test_backend.py \
  --steps 5 --resets 2 --report data/runs/p4_backend_cpu.json
```

程序接口为 `reset(InitialState) -> RobotState`、`step(ExpertCommand) -> RobotState`、`get_state()` 和 `close()`。一次 `step` 固定推进四个 0.0025 秒物理步，即专家控制周期 0.01 秒。

ROS 闭环：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch hex_isaacgym expert_sim.launch \
  data_root:=/home/val/BIH_ws/hex_complex/data headless:=true device:=cpu
rosservice call /hex/reset
```

发布 `/hex/joint_states`、`/hex/base_pose`、`/hex/reference_path` 和 `/hex/diagnostics`。缓存数据通过 `data_root` 指定；安装空间只包含代码、URDF 和地图 mesh，不向包资源目录写运行产物。

节点启动只读取命令行 `args`，不再读取 `~device` 等 ROS 私有参数。
launch 的 `<arg>` 通过节点 `args` 传入，与直接运行 Python 脚本使用相同入口。
`device`、`path`、`data_root` 在 launch 中默认留空：省略时分别使用 backend YAML
的 `device`、场景 YAML 的 `resources.se3_path`、`workspace_root/data`；显式传入
时仅覆盖本次运行。运行报告默认保存到实际数据目录的 `runs/`，也可用
`output_dir:=...` 指定。`headless`、`publish_clock`、`wall_rate_hz` 是启动选项。

```bash
# 保留 YAML 中的设备和轨迹设置
roslaunch hex_isaacgym expert_sim.launch
# 临时覆盖，不修改 YAML
roslaunch hex_isaacgym expert_sim.launch device:=cuda headless:=false
# 直接启动使用同一套参数，布尔值也可显式传入 true/false
python3 src/hex_isaacgym/scripts/expert_sim_node.py \
  --scene-config src/hex_isaacgym/config/expert_complex_scene.yaml \
  --device cpu --headless true --wall-rate-hz 100
```

代码职责：`config.py` 解析配置及资源路径；`runtime.py` 组装对应专家和后端；
`coordinator.py` 管理 reset、settling 和步进；`adapters.py` 封装专家差异。
运动学与仿真统一读取 backend YAML 的 `asset.robot_urdf`，场景 YAML 不再重复声明。
复杂专家的 `v_max`、`w_max`、`stance_maxmove_t/w` 保留在 backend YAML 的
`control` 中，并在创建专家时绑定；爬行场景不配置这些未使用的轨迹参数。

将 `headless:=false` 可打开 Isaac Gym viewer。actor 在 viewer 创建前即设置为轨迹首位姿和专家初始关节角；绿色折线和十字标记显示加载的参考路径，相机自动看向首位姿。

## ExpertClimb 平面爬行

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch hex_isaacgym expert_climb.launch device:=cpu headless:=false
```

也可指定 `device:=cuda`。该模式使用无限水平平面，不需要轨迹、点云或体素缓存。
初始位姿、关节角、速度和专家参数位于 `config/expert_climb_scene.yaml`；
默认以机身速度 `[0.02, 0, 0]` 运行。控制周期和执行器限值从物理后端绑定，
不在专家参数中重复配置。

用一次消息更新机身 vx、vy（m/s）和 yaw_rate（rad/s）：

```bash
rostopic pub -1 /hex/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.01, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
# 停止移动并保持关节目标
rostopic pub -1 /hex/cmd_vel geometry_msgs/Twist '{}'
# 恢复初始位姿、步态与 YAML 初始速度
rosservice call /hex/reset
```

最后一次速度指令持续生效，没有超时归零；NaN/Inf 指令会拒绝并记录日志。
settling 期间保留收到的速度，结束后以实际关节反馈初始化专家。回调只更新
加锁的指令快照，计算与仿真均在控制循环中执行；专家输出沿用一个控制周期的
应用延迟。零指令不会进入 COMPLETED，爬行模式不检查路径偏离。

爬行模式发布关节、机身和诊断信息，不发布参考路径；诊断包含 `command`、
`gaits`、`adhesions`、`gait_switch_count`、`estimated_contact_force` 和
`contact_force_norm`。估计力沿局部落脚法向投影，负值表示足端对表面施压；
仿真接触力为反作用力的模长，两者用于对照，含义与符号不同。
接触估计继续使用 `CF_Estimate` 和当前 `DampInvJac`；`q_torque` 是物理后端
实际施加的 PD 加前馈限幅扭矩，并非独立力传感器反馈。

无 ROS 的 `build_coordinator(scene, ...)` 在爬行模式返回 `(coordinator, None)`；
可用 `coordinator.set_velocity_command([vx, vy, yaw_rate])` 更新速度。
默认 `expert_type: complex` 及原有 `expert_sim.launch` 保持兼容。
地形配置新增 `terrain.type: plane`，省略时仍使用 mesh 并要求 `map_mesh`。

验证：

```bash
python3 -m unittest discover -s src/hex_isaacgym/tests -v
python3 src/hex_isaacgym/scripts/test_climb.py --device cpu --report /tmp/climb_cpu.json
python3 src/hex_isaacgym/scripts/test_climb.py --device cuda --report /tmp/climb_cuda.json
python3 src/hex_isaacgym/scripts/test_climb_ros.py --report /tmp/climb_ros.json
python3 src/hex_isaacgym/scripts/test_closed_loop.py --report /tmp/complex_regression.json
```

平面物理测试检查两次支撑切换、正向位移、停止保持和 reset，并记录估计力与
真实接触力。ROS 测试启动独立 master（默认端口 11431），不使用当前 ROS master，
验证单次消息、指令保持、非法输入及 reset，完成后关闭其启动的进程。
