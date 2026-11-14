# hex_isaacgym

独立的单机器人 Isaac Gym 设备后端，不包含专家、路径执行或闭环协调器。
专家和操作入口已迁往 hex_runner，不保留 expert_sim/expert_climb 等旧入口。

```bash
source devel/setup.bash
roslaunch hex_isaacgym backend.launch device:=cpu headless:=true \
  initial_config:=$PWD/src/hex_runner/config/climb.yaml
# 配套控制器与完整用法
roslaunch hex_runner sim.launch mode:=climb
```

订阅 `/hex/motor/command`、`/hex/servo/command`、`/hex/magnetic/command`，
发布对应 state、`/hex/perception/gravity`、`/hex/perception/pose`。
协议与硬件相同，详见 hex_runner/README.md。回调只缓存命令，物理线程独占物理张量。
目标按100Hz实时推进，每次推进4个0.0025秒物理步；不依赖 runner 驱动物理步进。
计算超时不补算追赶；实际速率受设备性能影响，本机GPU管线回归慢于实时，/clock仍按实际物理步递增。

电机支持 option=-1失能、0零力矩、1 MIT，其他模式明确拒绝。
MIT 使用消息的 kp/kd、位置、速度、前馈，扭矩受 URDF 限制。
舵机每腿最终使能为 enable && enable_<leg>；失能腿的 foot 主动力矩为零，
使能腿继续位置 PD；被动关节约束仍按模型执行。
磁吸 attached_* 发布 clip(物理吸附力/最大吸附力, 0, 1)，不回显 enabled_*，不在仿真端进行阈值判定。
正常反馈 valid_* 均为 true；吸附/释放阈值在 runner 各模式配置中设置，由 expert 统一判断。
仿真施加的力与接触力只在 `/hex/sim/diagnostics` 发布，不作为虚构实物传感器输入。
IMU 输出 world 中姿态、body 系角速度和扣除重力后转换至 body 的比力；
reset 后首帧线加速度不可用，协方差首项为-1。完整位姿 frame_id=world，根链为body。

config/plane.yaml、complex_surface4.yaml 仅包含物理参数。
初态唯一来源为 initial_config 指定的 runner YAML 中的 initial_state，详细格式见 hex_runner/README.md。
组合 sim.launch 自动将 runner_config 同时传给后端；独立启动必须显式传 initial_config。
后端通过 hex_kinematic 的共用解析器读取关节或足端模式，不创建专家。
`/hex/sim/reset` (std_srvs/Trigger) 在物理循环恢复初态并清空执行器和传感器历史，
时间保持递增。操作顺序：暂停步态（complex 用 pause，climb 发零速度）→ sim reset → 等反馈 → runner reset。
可用 `publish_clock:=true` 发布 /clock；所有外部设备名称均支持 remap/namespace。

纯物理库接口：`reset(InitialState, adhesion_init=None)`、`step(ActuatorCommand)`、`get_state()`、`close()`。
ActuatorCommand 包含目标角、力矩、磁吸、motor_mode、六腿布尔数组 servo_enabled 及电机增益/速度。
servo_enabled 默认六腿全 true，显式传入时必须为 (6,) bool 数组；诊断同样发布该数组。
该接口仅供物理测试使用，runner 不依赖它。
Isaac Gym 必须先于 Torch 导入，loader 负责导入顺序。

```bash
python3 src/hex_isaacgym/scripts/test_backend.py --steps 5 --resets 2 --report /tmp/hex_backend.json
python3 -m unittest discover -s src/hex_isaacgym/tests -v
```

物理 reset 使用共享 initial_state.pose、直接指定或逆解得到的 q，以及 initial_state.magnetic。
启动和物理 reset 同时恢复这些目标及使能状态，不使用复位前的最后命令。
初始吸附腿立即恢复 suction_force_max，每腿三个作用点各分担1/3，其余腿为零。
复位当周期先发布恢复后的初态，不推进物理；下周期继续正常步进，仿真时间不回退。

执行器回调直接更新可变的 ActuatorCommand 字段，保留未涉及的通道。回调间使用锁；
物理循环持有同一对象引用，步进中读取到的模式可能更新，但已转换的目标张量通常下一周期更新。

重力 topic 为 `std_msgs/Float64MultiArray`，data 是由实际世界重力投影得到的机身系单位向下向量。
不再发布旧的 sensor_msgs/Imu topic。
