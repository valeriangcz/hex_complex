# dm_motor_fusion

六足机器人（电磁六足）达妙电机统一控制节点。通过单路 SocketCAN（`can0`）驱动 18 台达妙 DM 系列电机（6 条腿 × 3 关节），把原先 6 个控制话题 / 6 个状态话题合并为两个统一话题：

- 订阅 `/sita_des`（`dm_motor_fusion/MultiLegCommand`）— 六腿统一控制指令
- 发布 `/sita_cur`（`dm_motor_fusion/MultiLegState`）— 六腿统一测量状态

基于 ROS1（catkin），C++ 实现，控制环默认 100 Hz、状态上报默认 100 Hz。提供 Docker 一键部署（见仓库 `docker/README.md`）。

## 1. 话题接口

```
              /sita_des  (MultiLegCommand)
   上位机 ────────────────────────▶ dm_motor_fusion_node ──CAN──▶ 18×达妙电机
   上位机 ◀──────────────────────── dm_motor_fusion_node ◀──CAN──
              /sita_cur  (MultiLegState)
```

### 1.1 控制话题 `/sita_des`

`option` 为全局控制模式（六腿共用），每条腿独立给 3 个关节的控制量：

| option | 模式 | 使用的字段 |
| --- | --- | --- |
| `-1` (`OPTION_DISABLE`) | 失能 | 节点自动下发 disable，忽略所有字段 |
| `0` (`OPTION_ZERO_TORQUE`) | 零力矩 | 节点强制 kp/kd/tau=0（腿软），忽略消息数值 |
| `1` (`OPTION_MIT`) | MIT 全参数 | pos/vel/tau + kp/kd 全部生效 |
| `2` (`OPTION_POS_VEL`) | 位置速度 | pos/vel 生效，tau/kp/kd 忽略 |

### 1.2 每腿字段（LegCommand / LegState）

字段按关节从**小 CAN ID 到大 CAN ID**排列：脚踝 → 膝关节 → 大腿。

| 关节 | 控制量（LegCommand，5 项） | 测量量（LegState，3 项） |
| --- | --- | --- |
| 脚踝 ankle | `ankle_pos` `ankle_vel` `ankle_tau` `ankle_kp` `ankle_kd` | `ankle_pos` `ankle_vel` `ankle_tau` |
| 膝关节 knee | `knee_pos` `knee_vel` `knee_tau` `knee_kp` `knee_kd` | `knee_pos` `knee_vel` `knee_tau` |
| 大腿 thigh | `thigh_pos` `thigh_vel` `thigh_tau` `thigh_kp` `thigh_kd` | `thigh_pos` `thigh_vel` `thigh_tau` |

单位：`pos` rad、`vel` rad/s、`tau` N·m。`LegState` 为电机 CAN 反馈（含 pn 方向系数），没有 kp/kd 字段；`/sita_cur.option` 报告当前生效模式（初始 `-1`）。

### 1.3 腿-电机 CAN ID 映射

| 腿 | 字段名 | 脚踝 | 膝关节 | 大腿 | pn 方向系数（脚踝/膝/大腿） |
| --- | --- | --- | --- | --- | --- |
| 左前 LF | `left_front` | 1 | 2 | 3 | +1 / +1 / −1 |
| 左中 LM | `left_mid` | 4 | 5 | 6 | +1 / +1 / −1 |
| 左后 LB | `left_back` | 7 | 8 | 9 | +1 / +1 / −1 |
| 右前 RF | `right_front` | 10 | 11 | 12 | +1 / −1 / +1 |
| 右中 RM | `right_mid` | 13 | 14 | 15 | +1 / −1 / +1 |
| 右后 RB | `right_back` | 16 | 17 | 18 | +1 / −1 / +1 |

- CAN ID = 电机 ESC ID（1~18），反馈 CAN ID（master ID）= 0x21~0x32
- 位置指令按 pn 系数取符号后下发电机；状态反馈同样乘 pn 还原到机体坐标
- 若实际整机安装方向不同，需同步修改节点内 `groups_` 的 `pn`

## 2. 消息定义（msg/）

| 文件 | 说明 |
| --- | --- |
| `MultiLegCommand.msg` | `option` 常量 + 6 × `LegCommand` |
| `LegCommand.msg` | 单腿 3 关节控制量（15 字段） |
| `MultiLegState.msg` | `option` + 6 × `LegState` |
| `LegState.msg` | 单腿 3 关节测量量（9 字段） |

## 3. 节点参数（私有参数，`_参数:=值` 或 launch）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `can_interface` | `can0` | SocketCAN 接口名 |
| `motor_type` | `2`(DM4340) | 电机型号枚举（0=DM4310, 1=DM4310_48V, 2=DM4340, 3=DM4340_48V…） |
| `control_hz` | `100.0` | 控制发送频率 |
| `status_hz` | `100.0` | 状态发布频率 |
| `thread_priority` | `95` | CAN 接收线程优先级（预留） |

行为说明：CAN 接口不存在时节点不退出，每秒重试直到接口就绪；启动时对 18 台电机发送模式切换（MIT）与失能指令，收到 `/sita_des` 后按 `option` 使能。

## 4. 使用

### 4.1 编译（本机 ROS 环境）

```bash
mkdir -p catkin_ws/src && cp -r dm_motor_fusion catkin_ws/src/
cd catkin_ws && catkin_make
source devel/setup.bash
```

### 4.2 运行节点

```bash
rosrun dm_motor_fusion dm_motor_fusion_node _can_interface:=can0 _motor_type:=2
# 或使用 launch：
roslaunch dm_motor_fusion dm_motor.launch can_interface:=can0 motor_type:=2
```

### 4.3 测试脚本（正弦摆动）

```bash
# 所有电机 90° 正弦摆动，option=2 位置速度模式；连真机前务必确认安全！
rosrun dm_motor_fusion multi_motor_sin90.py _option:=2 _amplitude_deg:=90
# 可选参数：~publish_hz ~base_freq_hz ~freq_step_hz ~phase_step_deg ~kp ~kd
```

### 4.4 命令行发布示例（站立姿态，MIT）

```bash
rostopic pub -r 10 /sita_des dm_motor_fusion/MultiLegCommand "{
option: 1,
left_front:  {ankle_pos: -0.8, ankle_vel: 0.0, ankle_tau: 0.0, ankle_kp: 100.0, ankle_kd: 1.0, knee_pos: 1.6, knee_vel: 0.0, knee_tau: 0.0, knee_kp: 100.0, knee_kd: 1.0, thigh_pos: -0.8, thigh_vel: 0.0, thigh_tau: 0.0, thigh_kp: 100.0, thigh_kd: 1.0},
left_mid:    {ankle_pos: -0.8, ankle_vel: 0.0, ankle_tau: 0.0, ankle_kp: 100.0, ankle_kd: 1.0, knee_pos: 1.6, knee_vel: 0.0, knee_tau: 0.0, knee_kp: 100.0, knee_kd: 1.0, thigh_pos: -0.8, thigh_vel: 0.0, thigh_tau: 0.0, thigh_kp: 100.0, thigh_kd: 1.0},
left_back:   {ankle_pos: -0.8, ankle_vel: 0.0, ankle_tau: 0.0, ankle_kp: 100.0, ankle_kd: 1.0, knee_pos: 1.6, knee_vel: 0.0, knee_tau: 0.0, knee_kp: 100.0, knee_kd: 1.0, thigh_pos: -0.8, thigh_vel: 0.0, thigh_tau: 0.0, thigh_kp: 100.0, thigh_kd: 1.0},
right_front: {ankle_pos: -0.8, ankle_vel: 0.0, ankle_tau: 0.0, ankle_kp: 100.0, ankle_kd: 1.0, knee_pos: 1.6, knee_vel: 0.0, knee_tau: 0.0, knee_kp: 100.0, knee_kd: 1.0, thigh_pos: -0.8, thigh_vel: 0.0, thigh_tau: 0.0, thigh_kp: 100.0, thigh_kd: 1.0},
right_mid:   {ankle_pos: -0.8, ankle_vel: 0.0, ankle_tau: 0.0, ankle_kp: 100.0, ankle_kd: 1.0, knee_pos: 1.6, knee_vel: 0.0, knee_tau: 0.0, knee_kp: 100.0, knee_kd: 1.0, thigh_pos: -0.8, thigh_vel: 0.0, thigh_tau: 0.0, thigh_kp: 100.0, thigh_kd: 1.0},
right_back:  {ankle_pos: -0.8, ankle_vel: 0.0, ankle_tau: 0.0, ankle_kp: 100.0, ankle_kd: 1.0, knee_pos: 1.6, knee_vel: 0.0, knee_tau: 0.0, knee_kp: 100.0, knee_kd: 1.0, thigh_pos: -0.8, thigh_vel: 0.0, thigh_tau: 0.0, thigh_kp: 100.0, thigh_kd: 1.0}
}"
```

### 4.5 Docker 部署

见 [`docker/README.md`](../docker/README.md)：`ros:noetic` 镜像内自动 `catkin_make`，`network_mode: host` 直通主机 `can0`，compose 同时拉起 roscore。

## 5. 文件结构

```
dm_motor_fusion/
├── CMakeLists.txt            # 消息生成 + 节点编译
├── package.xml
├── launch/dm_motor.launch    # 节点启动（参数化）
├── msg/                      # 4 个消息定义
├── scripts/multi_motor_sin90.py  # 六腿正弦测试脚本
├── include/
│   ├── damiao.h              # 达妙电机协议封装（MIT/位置速度/速度控制、参数读写）
│   └── socketcan.h           # SocketCAN 收发封装（独立接收线程）
├── node/
│   └── dm_motor_fusion_node.cpp  # ROS 节点：话题收发、模式管理、控制环
└── src/
    ├── damiao.cpp
    └── socketcan.cpp
```

## 6. 注意事项

- ⚠️ 该节点会向真实电机发送使能与力矩/位置指令，联调时先小 kp、低位姿验证电机方向（尤其右侧三腿膝部 pn=−1），确认无误后再放大幅值。
- 消息中的 `kp/kd` 会直接下发到电机 MIT 模式寄存器，`option=1` 时请确保已给合理值（0 会使电机软掉）。
- `option=2` 目前节点内部仍按 MIT 模式发送（`switchMotorMode` 固定 MIT），等效于"用消息里的 kp/kd 做位置跟踪"。
