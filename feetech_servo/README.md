# feetech_servo：SCS2332 六舵机 ROS 接口

## 构建与运行

```bash
cd /home/val/BIH_ws/hex_complex
source /opt/ros/noetic/setup.bash
catkin_make
catkin_make install
source install/setup.bash
roslaunch feetech_servo servo.launch
```

可覆盖串口与波特率：

```bash
roslaunch feetech_servo servo.launch port:=/dev/ttyUSB0 baud_rate:=1000000
```

默认配置位于 `para/servo.yaml`；可用 `config:=/absolute/path/servo.yaml` 加载其他配置。
YAML 先加载，非空的 `port`、`baud_rate` 启动参数优先覆盖 YAML。

## 角度、零点及 ID

所有六元素数组顺序固定为 **lf、lm、lb、rf、rm、rb**。
`ids` 默认为 `[1,2,3,4,5,6]`，`direction` 每项必须为 `1` 或 `-1`。
`offset_deg` 每项表示：该腿指令为 0° 时，舵机对应的绝对角度，单位也是度。

```text
absolute_deg = offset_deg[i] + direction[i] * command_deg
raw = round(absolute_deg / angle_span_deg * raw_max)
feedback_deg = direction[i] * (raw / raw_max * angle_span_deg - offset_deg[i])
```

例如偏移 150°、方向 -1，发送 +30°，舵机绝对目标角度是 120°。
初始换算约定为 300° 对应刻度 1023，可通过 `angle_span_deg` 和 `raw_max` 调整。
SCS2332 不同规格资料存在量程差异，应按实物标定；默认零偏移仅为配置占位，
控制实际机构前请填写各腿零点、方向及实际串口波特率，并确认舵机处于位置控制。
节点根据总开关 `enable` 和各腿 `enable_lf` 等字段共同控制扭矩，不设置工作模式、ID、舵机波特率或 EEPROM；启动时不发送指令。

`move_time_ms` 默认为 1000，`speed` 默认为 0，采用时间控制；非零速度值为库的原始速度参数。
两者支持 0～65535，具体工作组合以舵机固件为准；`read_timeout_ms` 默认 20，支持 1～999。
软件只检查换算后的舵机量程，不代表机构各腿的机械运动极限。

## ROS 话题

订阅 `servo_cmd`，类型 `feetech_servo/servo_cmd`：一个总开关 `bool enable`、六个逐腿开关 `bool enable_lf/enable_lm/enable_lb/enable_rf/enable_rm/enable_rb` 和六个 `float64` 字段
`lf, lm, lb, rf, rm, rb`，单位为相对于机械零点的度。
例如以下命令只适用于已确认零点配置后的机构：

```bash
rostopic pub -1 /servo_cmd feetech_servo/servo_cmd   '{enable: true, enable_lf: true, enable_lm: true, enable_lb: true, enable_rf: true, enable_rm: true, enable_rb: true, lf: 0.0, lm: 0.0, lb: 0.0, rf: 0.0, rm: 0.0, rb: 0.0}'
# Disable torque on all six configured servos; angle fields are ignored.
rostopic pub -1 /servo_cmd feetech_servo/servo_cmd '{enable: false}'
rostopic echo /servo_state
```

发布 `servo_state`，类型 `feetech_servo/servo_state`：包含读取开始时间戳、
六个实测角度以及同顺序的 `bool[6] valid`。

每腿最终使能 = `enable && enable_<leg>`。验证所有使能腿的目标角度后，同步写六腿各自的扭矩开关，
仅向使能腿同步写位置；失能腿的角度忽略（即使该字段为 NaN 也不影响其他腿）。
`enable=false`：忽略全部角度，同步关闭六个舵机的扭矩，不写位置。
所有 bool 字段默认是 false。发送位置目标时，必须设置 `enable: true`，并为需要运动的腿设置 `enable_<leg>: true`。
两种指令执行后都会逐个读取位置并发布一次状态。
读到的是当时位置，不保证已经到位，也不是同时采样。同步写 API 没有逐舵机应答，
位置有效也不代表舵机已执行本次指令。
`state_hz` 默认20Hz独立轮询，无指令时也发布位置；设置0可恢复仅指令回调读取。订阅队列长度为 1，串口读写在单线程回调中串行执行，
连续读取超时会降低实际指令处理频率。

使能指令中的非法角度（NaN、Inf 或换算后越界）导致整条指令被拒绝，不发送扭矩或位置写入。
读取失败、舵机返回故障状态或原始位置越界时，对应角度为 NaN，`valid=false`，
其余舵机继续读取。话题使用相对名称，支持 namespace 和 remap。

消息新增逐腿 enable 字段，MD5 已变化；所有发布端、订阅端需重新编译并重启。字段名使用小写。

## VS Code 补全

工作空间 `.vscode/c_cpp_properties.json` 配置了源码、ROS、devel/include 和
install/include 路径。需要 Microsoft C/C++ 扩展；新增消息须先 `catkin_make` 生成头文件。

## 无硬件串口集成验证

构建后，在已 source 当前包环境的终端运行：

```bash
python3 src/feetech_servo/test/test_interface_pty.py
```

测试启动独立本地 ROS master 和伪串口，不连接真实舵机。

## 原始库独立编译

仍可在 `src` 子目录用 CMake 独立构建 `libSCServo.a`。
原厂示例保留于 `examples/`。

hex_runner 实物启动通过 remap 接入 /hex/servo/command 和 /hex/servo/state。

接入 hex_runner 时 state_hz 应大于0，以便 reset 前获取真实位置反馈。

runner reset 成功后先使能全部舵机并下发初态；专家运行后通过显式布尔数组选择各腿使能。
手柄断使能总开关优先于专家逐腿开关。失能表示关闭主动扭矩，机械摩擦和被动约束仍然存在。
