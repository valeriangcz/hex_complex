# hex_kinematic

六足机器人正逆运动学、Jacobian、坐标变换和 URDF 质量模型。算法模块不依赖 ROS、地图、PyVista 或 Isaac Gym。

`Kinematic` 的腿顺序固定为 `lb, lf, lm, rb, rf, rm`。需要质量/质心计算时必须显式传入 `hex_all.urdf`。
