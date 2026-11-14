# magnetic 串口驱动与 ROS 接口

- `/hex/magnetic/command`：`MagneticCmd`，六腿字段为期望通断，true=打开。
- `/hex/magnetic/state`：`MagneticState`，`enabled_*` 为实际通电反馈；`attached_*` 为 float32 归一化霍尔读数（0～1），不是吸附成功布尔量；`valid_*` 表示对应 ADC 和 IO 反馈有效。Header 为完整帧接收时间。

## 配置与启动

先修改 `config/magnetic.yaml`。ADC 范围及标定值均为示例，必须实测标定：

- `port`、`baud_rate`：具体串口设备和波特率，默认 115200、8N1。
- `channel_order`：串口六个位置依次对应的腿名，命令与反馈共用此映射。
- `lower_bound/upper_bound`：六腿统一的原始 ADC 有效范围，包含边界。
- `calibration.<leg>.raw_min/raw_max`：每腿 0%/100% 对应的 ADC 值。

每腿计算 `clip((adc_raw - raw_min) / (raw_max - raw_min), 0, 1)`。
原始 ADC 在有效范围内且 IO 值为 0/1 时，该腿 valid=true。
超出标定区间但仍在有效范围内的读数截断至 0/1，仍有效；超出统一有效范围时 valid=false，限频记录腿名、ADC、有效范围和 IO 值。

```bash
roslaunch magnetic magnetic.launch port:=/dev/ttyUSB1
roslaunch hex_runner real.launch mode:=climb magnetic_launch:=$(rospack find magnetic)/launch/magnetic.launch
```

独立启动支持 `config:=/path/to/magnetic.yaml`、`port:=...` 和 `baud_rate:=...` 覆盖。
runner 的 `climb.yaml`、`complex.yaml` 在 `expert_parameters` 中各配置六腿统一阈值：

```yaml
magnetic_attach_above: 0.85
magnetic_release_below: 0.05
```

expert 在读数严格高于吸附阈值时确认吸附，严格低于释放阈值时确认释放；中间区（含等号）两者均不确认，但不影响 valid。仿真发布吸附力/最大吸附力，采用同一判定路径。
真正无效的反馈或串口超时仍由 runner 原有故障逻辑处理，FAULT 持续发布上一次控制指令。

## 串口协议与生命周期

接收示例：

```text
[adc_mv]:1641,1605,1649,1613,1622,1593; [adc_raw]:2037,1992,2047,2002,2013,1978; [io_state]:0,0,0,0,0,0; #
```

以 `#` 分帧，解析 adc_raw、io_state，忽略 adc_mv 和 RX/TX 日志。残缺或格式错误帧不发布。
发送格式为 `[IO_CMD]:0,0,0,0,0,0;`，1=打开，不追加换行。
只在收到 ROS 命令时发送；启动、退出不自动改变输出。串口无数据时不重复发布旧反馈，打开或读写失败时报错退出，不自动重连、重放命令。

`attached_*` 已从 bool 改为 float32，所有发布、订阅节点必须重新编译并重启；`MagneticCmd` 字段保持不变。

## 模拟成功反馈，同时真实收发串口

`magnetic_mock.launch` 替代真实 magnetic 节点运行，使用相同话题：

```bash
roslaunch magnetic magnetic_mock.launch port:=/dev/ttyUSB1
roslaunch hex_runner real.launch mode:=climb magnetic_launch:=$(rospack find magnetic)/launch/magnetic_mock.launch
```

支持 `config`、`port`、`baud_rate` 和 `state_hz`（默认 50 Hz）参数。
只使用配置中的串口参数及 `channel_order`，不依赖 ADC 标定值。

每收到一条 `/hex/magnetic/command` 消息，真实发送一次 `[IO_CMD]:...;` 并打印 TX 原始字节，
不定时发送或重发；上游持续发布命令时，每条消息仍会触发一次发送。
持续读取真实串口并打印 RX 原始字节（保留转义字符），不解析 ADC 或用它改变模拟状态。

ROS 状态独立定频发布：命令打开时 `attached_*=1.0、enabled_*=true`，关闭时为
`attached_*=0.0、enabled_*=false`，`valid_*` 始终为 true。这里 enabled 也按命令模拟，
不是设备实际通电确认。启动时六腿默认关闭；每次状态发布更新时间戳。
串口静默不影响模拟反馈，打开或读写失败则报错退出；启动、退出不发送通断命令。

伪终端集成验证（不会连接真实电磁铁）：

```bash
source devel/setup.bash
python3 src/magnetic/tests/serial_ros_check.py --mock
```
