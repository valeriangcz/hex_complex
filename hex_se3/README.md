# hex_se3

SE(3) 键盘标注、轨迹读写、shortcut、CPU/Torch 优化和原版风格可视化。

在工作空间根目录执行，先 `source devel/setup.bash`。示例轨迹位于
`src/hex_se3/data/trajectories/`，安装时随 hex_se3 一起安装。

```bash
# 原版多位姿静态场景，默认展示 5 个 teleop 位姿
rosrun hex_se3 se3_static_vis.py

# 多路径；每条路径显示所选来源的位姿
rosrun hex_se3 se3_static_vis.py --path \
  src/hex_se3/data/trajectories/shortcut_initial_se3_path_20261105_173348.json \
  src/hex_se3/data/trajectories/optimized_cma_es_torch_path_20261105_183455_665425.json \
  --source waypoints --indices 0 1

# 动画（原版默认 5 倍速度）
rosrun hex_se3 se3_animation_vis.py --path \
  src/hex_se3/data/trajectories/optimized_cma_es_torch_path_20261105_183455_665425.json

# 通用入口：直接调用两个查看器，默认 1 倍速度
rosrun hex_se3 view_path.py --mode static --source teleop --indices 0 1 2 3 4 \
  --path src/hex_se3/data/trajectories/feasibility_teleop_demo_20261105_220311.json
```

三个入口均支持 `--config` 指定场景 YAML，默认使用本包的 complex_surface4 配置。
静态模块顶部保留原版逐路径样式、位姿来源和索引、图层开关及固定相机配置；
默认隐藏路径线、标签和路径坐标轴，保留原版显示效果。

动画按键：Space 播放/暂停，左右箭头或 N/B 单步，R 重置，C 切换 Dense/Bezier，
F 跟随相机，L 切换可行落脚点，+/- 速度，H 帮助，Q/Esc 关闭。
Teleop 轨迹每个位姿间隔按 1 秒播放，不提供 Bezier 切换。
`view_path --stride` 是 `--landing-stride` 的兼容别名，仅控制背景落脚点采样。
