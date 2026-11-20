# hex_kinematic

C++ 库 `hex_kinematic_cpp` 使用 Eigen，公开头文件为 `hex_kinematic/kinematic.hpp`。
实现正逆运动学、雅可比及二阶导数、阻尼逆解、力估计、坐标变换和 URDF 质量模型；
当前原生控制入口仅覆盖六足。Python 和 Torch 模块保留，不依赖 C++ 绑定。

C++ 的 `Angles` 为 6×4、`JointMatrix` 为 6×3，均按行保存各腿数据；
`Rows3` 用于任意数量的运动学点。单点／方向变换接收 `Vector3d` 和腿索引，
左右腿的旋转及腿根平移与 Python 一致。质量参数在模型加载时缓存，质心计算复用每条腿的旋转。

```cpp
#include <hex_kinematic/kinematic.hpp>
hex_kinematic::Kinematic kin(urdf_path);
Eigen::Vector3d q(0.0, 0.6, -1.9);
auto foot = kin.ForwardKinReturn(q);
auto jacobian = kin.Jacobian(q);
auto branches = kin.InverseKin2Branches(foot);  // valid[0/1] 分别对应 ankle 正／负分支
```

六足机器人正逆运动学、Jacobian、坐标变换和 URDF 质量模型。算法模块不依赖 ROS、地图、PyVista 或 Isaac Gym。

`Kinematic` 的腿顺序固定为 `lb, lf, lm, rb, rf, rm`。需要质量/质心计算时必须显式传入 `hex_all.urdf`。
