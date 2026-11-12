# magnetic ROS 接口

本包生成 MagneticCmd 和 MagneticState，不提供硬件通信或假反馈节点。

- `/hex/magnetic/command`：magnetic/MagneticCmd，`lf/lm/lb/rf/rm/rb` 表示各腿通断指令。
- `/hex/magnetic/state`：magnetic/MagneticState，Header 加六腿实际吸附字段，
  `enabled_lf` 等表示实际通电状态，`valid_lf` 等表示该腿通电与吸附反馈均有效。

例如 `lf=false, valid_lf=true` 表示确认左前腿已释放；无数据时必须 `valid_lf=false`。
吸附字段来自实际检测，不是通电指令回显；不将无效数据当作释放成功。
所有腿均按字段名称访问，无需数组顺序转换。建议状态至少20Hz，Header记录采集时间。
旧 MultiLegCommand/MultiLegState 已删除，节点需重新编译并重启。

实际驱动完成后通过 hex_runner real.launch 的 magnetic_launch 参数接入。
仿真由 hex_isaacgym 提供相同协议。
