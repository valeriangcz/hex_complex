# perception

提供 WIT IMU 原生四元数串口读取和机身重力方向发布；完整定位由外部提供。

## 输出协议

- `/hex/perception/gravity`：`std_msgs/Float64MultiArray`，`data=[gx,gy,gz]`，
  机身坐标系下指向物理下方的单位向量，不是加速度计比力，也不是 m/s²。
  机身为 hex_magnetic URDF 根链；Z 轴朝上静置时为 `[0,0,-1]`。
  没有 Header，runner 以主机接收时间判定新鲜度。每条消息对应尚未发布的新四元数样本。
- `/hex/perception/pose`：`geometry_msgs/PoseStamped`，frame_id=`world`，
  表示机身在世界中的完整 SE3，位置 m、单位四元数 xyzw。本包不实现定位或虚构平移。

climb 只消费重力方向，complex 只消费完整 SE3。旧 `/hex/perception/imu` 已移除。

## 启动与配置

```bash
roslaunch perception imu.launch
# 自定义配置：
roslaunch perception imu.launch config:=/absolute/path/to/imu.yaml
```

默认配置在 `config/imu.yaml`：串口 `/dev/ttyUSB0`、115200 baud、100 Hz。
需要 numpy、PyYAML、pyserial；模块波特率必须由用户设置为 YAML 中的 baud。
节点不会扫描或更改波特率。独立 launch 不启动执行器；可通过 runner 的
`real.launch` 现有 `perception_launch` 参数接入。

`publish_hz` 同时控制模块四元数输出频率和 ROS 最大发布频率。
支持 0.2、0.5、1、2、5、10、20、50、100、200 Hz，并校验带宽。
`serial_timeout` 必须大于一个采样周期；低频输出需相应增大该值及 runner 的 feedback_timeout。
100 Hz 的四元数流需要约 11000 bit/s，9600 baud 不够。

`imu_to_body_rotation` 是 3×3 旋转矩阵 R_BI，按行填写；
**每列分别表示 IMU 的 x、y、z 单位轴在机身坐标系中的表示**。
节点验证正交性和 det=+1，不接受镜像或非旋转矩阵。单位阵仅适用于坐标轴完全对齐，
使用前必须根据安装修改。方向变换不需要平移：

```python
# q 来自模块 0x59 包，Hamilton w,x,y,z，归一化后构造 R_WI。
# R_WI 将 IMU 向量旋转到模块的世界参考系。
gravity_imu = R_WI.T @ np.array([0., 0., -1.])
gravity_body = R_BI @ gravity_imu
```

不经过欧拉角；q 与 -q 产生相同结果。安装后应在六面静置姿态核对物理重力方向。
模块内部姿态解算仍会受运动加速度、振动和磁场影响，本接口不代表已消除这些误差。

## 串口生命周期

独占打开串口后读取原输出配置，临时启用仅 `0x59` 四元数输出并设置目标频率，
读取寄存器核对后开始发布。不会发送 Flash 保存、标定或波特率修改命令。
正常结束或异常时尝试恢复原输出内容和频率，并读回核对；断线、强制杀进程时无法保证恢复。

解析器处理半包、粘包、垃圾字节及校验失败；无效四元数不发布、不刷新新鲜度。
四元数原始模长必须在 [0.9,1.1] 内，再归一化使用。
积压时发布最新样本，不重复旧结果以凑频率；持续无有效样本超过 serial_timeout 或串口故障时退出。
runner 随后按其反馈超时策略处理，不会从该节点收到虚构默认方向。

## 验证

```bash
source devel/setup.bash
python3 -m unittest discover -s src/perception/tests -v
```

测试覆盖原生包解析、安装矩阵列约定、竖直与倒置、实测数据回放、配置恢复、
100 Hz 新样本发布、无重复、无效样本超时与断线。实测数据位于工作空间
`data/imu_native`，安装环境缺少该记录时仅跳过回放测试。
