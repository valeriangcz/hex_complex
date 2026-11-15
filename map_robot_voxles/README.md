# map_robot_voxles

环境点图、ESDF、机器人体素、整机可行性检查及共享 PyVista 场景组件。包名沿用项目约定中的 `voxles` 拼写。

在工作空间根目录 `source devel/setup.bash` 后：

```bash
# 地图独立四视图：无需机器人配置或缓存
rosrun map_robot_voxles view_map_robot.py --mode map \
  --map-config src/map_robot_voxles/config/complex_surface4.yaml

# 机器人完整工作空间：无需地图配置或缓存
rosrun map_robot_voxles view_map_robot.py --mode robot \
  --robot-config src/map_robot_voxles/config/hex_magnetic.yaml

# 整机可行性视图
rosrun map_robot_voxles view_map_robot.py --mode feasibility \
  --map-config src/map_robot_voxles/config/complex_surface4.yaml \
  --robot-config src/map_robot_voxles/config/hex_magnetic.yaml \
  --xyz 0 0 0 --rpy-deg 0 0 0
```

地图显示原始点、落脚表面、非落脚点及边界、叠加及法向四视图。
机器人模式默认完整显示，可用 `--stride 8` 降采样腿部工作空间。
各模式支持 `--off-screen --screenshot /tmp/scene.png`，渲染需要可用的 DISPLAY 或 Xvfb。

示例轨迹统一位于 `hex_se3/data/trajectories/`，共 5 个原项目 JSON，随 hex_se3 安装。
读取代码可使用 `map_robot_voxles.resources.package_resource` 定位源码或安装后的资源。
