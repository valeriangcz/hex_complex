# control_stick：手柄控制

订阅 sensor_msgs/Joy，持续发布最后收到的速度到 hex/cmd_vel，并调用 runner 服务。不直接发布设备控制指令。
支持 climb 速度控制，以及 complex 的 reset/start/pause；索引均从0开始。

| 输入 | 操作 |
| --- | --- |
| buttons[0] | /hex/runner/start，complex 开始或恢复 |
| buttons[4] | /hex/runner/pause，complex 暂停并保留进度 |
| buttons[1] | /hex/runner/reset |
| buttons[3] | /hex/sim/reset，仅恢复仿真物理初态 |
| axes[0] | 原始 axes[0]，缩放由 runner 完成 |
| axes[1] | 原始 axes[1]，缩放由 runner 完成 |
| axes[2] | 原始 axes[2]，缩放由 runner 完成 |
| buttons[6] | /hex/runner/disable_actuators，电机与舵机断使能 |
| buttons[7] | /hex/runner/release_magnets，六腿全部断磁 |

## 启动与配置

```bash
source devel/setup.bash
# 默认启动当前 launch 配置的 joy_node
roslaunch control_stick climb.launch
# 已有 joy_node 时，避免重复启动
roslaunch control_stick climb.launch start_joy:=false
# 自定义私有ROS参数配置
roslaunch control_stick climb.launch config:=/absolute/path/climb.yaml
```

配套启动仿真或实物 runner。反馈就绪后按button 1初始化，再推动摇杆。
config/climb.yaml 定义 joy_topic、发布频率、三个轴索引和六个按钮索引。
默认20Hz发布原始轴数值，不做速度缩放、扳机转换或额外死区处理。
实际速度只由 hex_runner/config/climb.yaml 中 velocity_scales 决定。
例如原始vx=0.5、velocity_scales.vx=0.02，runner计算实际vx=0.01 m/s。
参数通过 launch 加载到节点私有命名空间；修改文件后重启手柄节点生效。

接收到新消息更新目标，之后持续以发布频率发送最后收到的速度，不再按消息间隔归零。
松开摇杆时需要 joy_node 发送实际零轴值，收到零值后才停止；断开手柄但手柄节点仍运行时保持最后速度。
轴越界缺失或NaN/Inf仍立即归零，并独立处理有效按钮。
runner 同样保留最新有效速度，即使手柄节点停止发布也不会按时间自动归零。
停止步态需发送零速度或调用 runner 的 pause（complex）、断使能、断磁等操作；设备反馈有效性检查仍保留。

按钮仅在按下边沿触发，长按不重复执行。失能和断磁优先于普通操作，二者可同时执行。普通操作同帧优先级为 sim_reset > reset > pause > start。
服务由工作线程调用，不阻塞输入与速度发布；服务不可用/拒绝时记录日志，需要重新按键重试。
过期或被新请求替代的排队 sim_reset/reset/start/pause 丢弃，不在未来自动补执行。
服务响应表示操作已排队，实际结果查看 /hex/runner/diagnostics。

## 锁存与恢复

- button 6：停止步态并清速度，持续发送 motor option=-1 和 servo enable=false，磁吸保持原指令。
- button 7：停止步态并清速度，六腿磁吸持续false，关节保持原目标与已有使能状态。
- 两项独立锁存。松开按钮、推摇杆、反馈恢复都不会解除锁存。WAITING/FAULT也可请求。
- button 1：要求反馈有效，关节目标和磁吸来自 hex_runner/config/climb.yaml 的 initial_state，
  前馈清零，成功构建目标后解除锁存。进入 READY 后保持该目标；开始步态前确认关节已到位。
- reset通过设备指令运动到预设关节姿态，不调用 /hex/sim/reset，也不瞬移机身。
  reset失败时不解除锁存，不重新使能或吸附。

## 验证

```bash
python3 -m unittest discover -s src/control_stick/tests -v
python3 src/control_stick/scripts/test_climb_joy.py
python3 src/hex_runner/scripts/test_ros_integration.py --backend fake --mode complex --joystick
```

集成测试使用独立ROS master、合成Joy和假设备，不控制真实机器人。
真实 /joy 采样目前超时，实际轴2中立值及设备按键布局尚未验证。

## 为什么看不到 runner 服务

source devel/setup.zsh 只配置环境，不启动节点。
control_stick 节点是服务客户端，只有 hex_runner 启动后才有 /hex/runner/reset 等服务。
可在另一终端启动仿真（含runner）：

```bash
roslaunch hex_runner sim.launch mode:=climb device:=cpu headless:=true
# 或仅启动runner，设备由其他节点提供：
roslaunch hex_runner runner.launch mode:=climb
```

runner 服务注册后，reset 仍要求有效设备反馈。只有 joy_node 和 climb_joy 时没有设备反馈来源。
get_loggers、set_logger_level 是 ROS 自动提供的日志级别管理服务，不是机器人控制服务。
rospy没有公开的init_node参数禁用这两个服务；disable_rosout只影响日志话题，不取消服务。
不修改系统ROS或第三方joy/rosout节点来删除它们。只看业务服务可使用：

```bash
rosservice list | rg '^/hex/'
```

## 速度配置的位置

climb实际速度只在 hex_runner/config/climb.yaml 中配置：

```yaml
velocity_scales:
  vx: 0.02
  vy: 0.02
  wz: 0.2
```

本包只负责轴和按钮索引、消息转发频率，不再包含速度缩放或joy_timeout。
`hex/cmd_vel` 保留 geometry_msgs/Twist 类型，但数值现在表示原始轴输入，不是m/s或rad/s。
runner收到后乘velocity_scales，内部专家仍使用m/s、rad/s；负缩放用于轴反向。
修改velocity_scales后重启runner，修改手柄映射后重启climb_joy。
complex的v_max/w_max是轨迹规划上限，舵机speed是设备参数，不属于这份climb速度指令。

## complex 的 start / pause

先启动 complex runner，再运行同一个 control_stick/climb.launch。
按钮1 reset，按钮0 start；按钮4 pause，随后再按按钮0继续原进度。
长按不重复调用，松开后再次按下才触发；complex 忽略摇杆速度。
climb 的 start/pause 服务仍返回“不适用”，climb 通过非零/零速度运行和保持。
pause 保留轨迹进度，可 start 恢复，也可直接 reset 重新初始化或断使能。
断使能、断磁后保持锁存，start 不能解除，必须成功 reset。

## 仿真物理复位按钮

buttons[3] 仅调用 hex/sim/reset，恢复配置中的机身位姿、关节和磁吸初态；不自动调用 runner reset。
buttons[1] 仍只调用 hex/runner/reset，两者独立。所有名称支持 namespace/remap。
使用时先暂停步态（complex 按4，climb 发送零摇杆），再按3；等仿真反馈更新后按1初始化控制器。
按住3不会重复复位；服务不可用时记录日志并丢弃本次请求，不会转而调用 runner reset，需再次按键重试。
