# perception 输出协议

本次定义接口，不实现雷达点云匹配定位、IMU硬件通信或假位姿节点。

- `/hex/perception/pose`：geometry_msgs/PoseStamped，frame_id=world，表示 world→body。
  body 是 hex_magnetic URDF 机身根链。位置 m，单位四元数 xyzw；时间戳为采集时间。
  对应未来雷达匹配定位；无可靠完整位姿时不发布虚构平移。
- `/hex/perception/imu`：sensor_msgs/Imu，frame_id=body，orientation 是 world 中机身姿态，
  angular_velocity 是 body 系 rad/s，linear_acceleration 是 body 系比力 m/s²。
  perception 负责标定并应用 IMU/雷达相对 body 的外参，runner 不接收原始传感器坐标系。
  未提供的字段按 ROS 约定设置 covariance[0]=-1。runner 的 IMU 模式要求有效 orientation。

建议至少100Hz提供IMU、20Hz提供定位，允许按实际频率调整runner的feedback_timeout。
climb 显式选择 imu 或 se3；complex 必须 se3，不自动回退到IMU或虚构零位置。
实际实现完成后通过 hex_runner real.launch 的 perception_launch 参数接入。
