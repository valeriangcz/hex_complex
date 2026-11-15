"""单腿 B 系摆动轨迹的 PyVista 查看器。

修改下方配置后运行：python -m hex_expert.swing_traj_vis
起终点与曲线均在选中腿的局部 B 系下表达，长度单位 m。
显示半透明可行体素点云、初始/优化样条、控制点及起终点法向，不求逆解。
优化使用曲线平滑度、B 系工作空间距离场及长方体障碍物净空惩罚。
"""
from __future__ import annotations

from dataclasses import dataclass
from math import ceil
from pathlib import Path

import numpy as np
import pyvista as pv
from scipy.interpolate import BSpline
from scipy.ndimage import map_coordinates
from scipy.optimize import minimize

from map_robot_voxles.resources import package_resource
from map_robot_voxles import Voxels


# ======================== 用户配置 ========================
# index: 0 LB, 1 LF, 2 LM, 3 RB, 4 RF, 5 RM。
# 每项的起终点均在该腿 B 系下；仅显示 SWING_LEG_INDEX 选中的腿。
# 四组嵌套list按行一一对应；每增加一条轨迹，都要增加对应的起终点和法向。
LEG_CONFIGS = [
    dict(
        leg_index=0,
        start_B=[
            [0.06, 0.0, -0.18],  # 三条轨迹共用靠近腿根下方的起点
            [0.06, 0.0, -0.18],
            [0.06, 0.0, -0.18],
        ],
        start_norm=[
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
        ],
        end_B=[
            [0.3, 0.00, -0.1],  # 外侧下方
            [0.20, 0.25, 0.05],    # 外侧中间，+Y侧，法向-Y
            [0.18, -0.06, 0.2],   # 外侧上方
        ],
        end_norm=[
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ],
    ),
    dict(
        leg_index=3,
        start_B=[[0.20, -0.06, -0.18]],
        start_norm=[[0.0, 0.0, 1.0]],
        end_B=[[0.23, 0.06, -0.18]],
        end_norm=[[0.0, -1.0, 0.0]],
    ),
]
SWING_LEG_INDEX = LEG_CONFIGS[0]["leg_index"]  # 显示此腿的所有轨迹
RAISE_DISTANCE = 0.08  # 沿起点法向的初始抬脚距离，m
DOWN_DISTANCE = 0.08  # 沿终点法向的初始控制距离，m
MAX_FOOT_SPEED = 0.08  # 折线时间估计所用速度，m/s
SAMPLE_DT = 0.01  # 优化采样间隔，s
WORKSPACE_CLEARANCE = 0.02  # 距边界不足4cm时施加平方惩罚，m
OBSTACLE_ENABLED = True
CONTACT_PATCH_SIZE = 0.05  # 起终点表面的边长，5cm × 5cm
CONTACT_PATCH_THICKNESS = 0.005  # 薄片厚度5mm，实体位于接触法向反侧
# 额外障碍物：center_B为实体中心，size为局部三轴完整边长，normal_B为局部+Z方向。
# 增加字典即可放置更多；空列表表示只显示起终点薄片。
EXTRA_OBSTACLES = [
    # 当前第二条优化轨迹u=0.5处；加入障碍物后轨迹会重新优化绕行。
    # dict(center_B=[0.15, 0.08322, -0.01716], size=[0.05, 0.1, 0.05], normal_B=[0.0, 0.0, 1.0]),
]
OBSTACLE_CLEARANCE = 0.04  # 足端距长方体表面达到4cm后不惩罚
OBSTACLE_WEIGHT = 5.0
# 与现有腿部体素缓存的网格定义一致；读取选中腿的 B 系距离场和可行体素。
WORKSPACE_CACHE = None  # set from robot config before loading workspace
WORKSPACE_MIN_B = [-0.14, -0.38, -0.4]
WORKSPACE_VOXEL_SCALE = 0.01


WINDOW_SIZE = (1000, 800)
SHOW_FEASIBLE_VOXELS = True
# VOXEL_COLOR = "lightcoral"
color_list=["red","purple","blue","yellow","gray","cyan"]
VOXEL_COLOR = color_list[0]
VOXEL_OPACITY = 0.06  # 完整体素点云，透明度越小越便于观察内部曲线
VOXEL_POINT_SIZE = 30.0
SWING_SAMPLE_COUNT = 301
LEG_NAMES = ("LB", "LF", "LM", "RB", "RF", "RM")
# ==========================================================


@dataclass(frozen=True)
class SwingConfig:
    index: int
    start: np.ndarray
    start_norm: np.ndarray
    end: np.ndarray
    end_norm: np.ndarray


@dataclass(frozen=True)
class LegConfig:
    index: int
    swings: tuple[SwingConfig, ...]


@dataclass(frozen=True)
class BoxObstacle:
    center: np.ndarray
    size: np.ndarray
    rotation: np.ndarray  # 列向量为局部坐标轴在B系中的方向
    name: str
    contact: bool = False


@dataclass(frozen=True)
class SwingResult:
    initial: BSpline
    optimized: BSpline
    initial_cost: float
    optimized_cost: float
    success: bool
    message: str
    initial_obstacle_distance: float | None = None
    optimized_obstacle_distance: float | None = None


def _vector(value, name: str, normalize: bool = False) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"{name}: expected a finite (3,) vector")
    if normalize:
        length = np.linalg.norm(result)
        if length <= 1e-12:
            raise ValueError(f"{name}: normal must be nonzero")
        result = result / length
    return result


def _vectors(value, name: str, normalize: bool = False) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if (result.ndim != 2 or result.shape[1] != 3 or not len(result)
            or not np.isfinite(result).all()):
        raise ValueError(f"{name}: expected a nonempty finite (N,3) nested list")
    if normalize:
        lengths = np.linalg.norm(result, axis=1, keepdims=True)
        if np.any(lengths <= 1e-12):
            raise ValueError(f"{name}: normals must be nonzero")
        result = result / lengths
    return result


def _index(value, name: str, size: int) -> int:
    if (isinstance(value, (bool, np.bool_))
            or not isinstance(value, (int, np.integer)) or not 0 <= value < size):
        raise ValueError(f"{name}: expected an integer in 0..{size - 1}")
    return int(value)


def load_config():
    """先校验配置，避免加载机器人缓存后才报告输入错误。"""
    if not isinstance(LEG_CONFIGS, list) or not LEG_CONFIGS:
        raise ValueError("LEG_CONFIGS must be a nonempty list")
    legs = []
    for cfg in LEG_CONFIGS:
        leg = _index(cfg["leg_index"], "leg_index", 6)
        if any(item.index == leg for item in legs):
            raise ValueError(f"duplicate leg_index: {leg}")
        context = f"leg {leg}"
        starts = _vectors(cfg["start_B"], f"{context} start_B")
        start_norms = _vectors(cfg["start_norm"], f"{context} start_norm", True)
        ends = _vectors(cfg["end_B"], f"{context} end_B")
        end_norms = _vectors(cfg["end_norm"], f"{context} end_norm", True)
        if not (len(starts) == len(start_norms) == len(ends) == len(end_norms)):
            raise ValueError(f"{context}: start_B/start_norm/end_B/end_norm must have equal row counts")
        swings = tuple(SwingConfig(leg, start, start_n, end, end_n)
                       for start, start_n, end, end_n in zip(starts, start_norms, ends, end_norms))
        legs.append(LegConfig(leg, swings))
    selected = _index(SWING_LEG_INDEX, "SWING_LEG_INDEX", 6)
    if selected not in [leg.index for leg in legs]:
        raise ValueError("SWING_LEG_INDEX must appear in LEG_CONFIGS")
    if (isinstance(SWING_SAMPLE_COUNT, bool)
            or not isinstance(SWING_SAMPLE_COUNT, int) or SWING_SAMPLE_COUNT < 2):
        raise ValueError("SWING_SAMPLE_COUNT must be an integer >= 2")
    for name, value in (("RAISE_DISTANCE", RAISE_DISTANCE), ("DOWN_DISTANCE", DOWN_DISTANCE),
                        ("MAX_FOOT_SPEED", MAX_FOOT_SPEED), ("SAMPLE_DT", SAMPLE_DT),
                        ("WORKSPACE_VOXEL_SCALE", WORKSPACE_VOXEL_SCALE)):
        if not np.isscalar(value) or not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be positive and finite")
    if RAISE_DISTANCE > 0.1 or DOWN_DISTANCE > 0.1:
        raise ValueError("RAISE_DISTANCE and DOWN_DISTANCE must be <= 0.1")
    if (not np.isscalar(WORKSPACE_CLEARANCE) or not np.isfinite(WORKSPACE_CLEARANCE)
            or WORKSPACE_CLEARANCE < 0):
        raise ValueError("WORKSPACE_CLEARANCE must be nonnegative and finite")
    if not isinstance(OBSTACLE_ENABLED, (bool, np.bool_)):
        raise ValueError("OBSTACLE_ENABLED must be a boolean")
    for name, value in (("CONTACT_PATCH_SIZE", CONTACT_PATCH_SIZE),
                        ("CONTACT_PATCH_THICKNESS", CONTACT_PATCH_THICKNESS)):
        if not np.isscalar(value) or not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be positive and finite")
    for name, value in (("OBSTACLE_CLEARANCE", OBSTACLE_CLEARANCE),
                        ("OBSTACLE_WEIGHT", OBSTACLE_WEIGHT)):
        if not np.isscalar(value) or not np.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be nonnegative and finite")
    selected_leg = next(leg for leg in legs if leg.index == selected)
    return selected_leg


def load_workspace(leg: int):
    """只读摆动腿的局部距离场，不初始化控制器或环境。"""
    with np.load(WORKSPACE_CACHE) as cache:
        distances = np.asarray(cache["to_bound_dist"][..., leg], dtype=np.float64)
    lower = _vector(WORKSPACE_MIN_B, "WORKSPACE_MIN_B")
    bounds = np.column_stack([lower, lower + np.array(distances.shape) * WORKSPACE_VOXEL_SCALE])
    return Voxels(bounds, WORKSPACE_VOXEL_SCALE), distances


def _normal_frame(normal: np.ndarray) -> np.ndarray:
    """构造右手正交基，局部+Z与给定表面法向一致。"""
    n = _vector(normal, "obstacle normal", True)
    reference = np.eye(3)[np.argmin(np.abs(n))]
    tangent = np.cross(reference, n)
    tangent /= np.linalg.norm(tangent)
    return np.column_stack([tangent, np.cross(n, tangent), n])


def build_obstacles(leg: LegConfig) -> tuple[BoxObstacle, ...]:
    """起终点自动生成接触薄片，相同位置及法向只放一个，另加用户配置障碍物。"""
    if not OBSTACLE_ENABLED:
        return ()
    obstacles = []
    contacts = []
    for number, swing in enumerate(leg.swings, 1):
        for label, point, normal in (("Start", swing.start, swing.start_norm),
                                     ("End", swing.end, swing.end_norm)):
            if any(np.allclose(point, p, atol=1e-9, rtol=0)
                   and np.allclose(normal, n, atol=1e-9, rtol=0) for p, n in contacts):
                continue
            contacts.append((point, normal))
            obstacles.append(BoxObstacle(
                point - normal * CONTACT_PATCH_THICKNESS / 2.0,
                np.array([CONTACT_PATCH_SIZE, CONTACT_PATCH_SIZE, CONTACT_PATCH_THICKNESS]),
                _normal_frame(normal), f"{label} {number} surface", True,
            ))
    if not isinstance(EXTRA_OBSTACLES, list):
        raise ValueError("EXTRA_OBSTACLES must be a list")
    for number, cfg in enumerate(EXTRA_OBSTACLES, 1):
        center = _vector(cfg["center_B"], f"extra obstacle {number} center_B")
        size = _vector(cfg["size"], f"extra obstacle {number} size")
        if np.any(size <= 0):
            raise ValueError(f"extra obstacle {number}: size entries must be positive")
        rotation = _normal_frame(cfg.get("normal_B", [0, 0, 1]))
        obstacles.append(BoxObstacle(center, size, rotation, f"Extra {number}"))
    return tuple(obstacles)


def load_feasible_points(voxels: Voxels, leg: int) -> np.ndarray:
    """将选中腿任一 IK 分支可行的体素中心作为 B 系点云。"""
    with np.load(WORKSPACE_CACHE) as cache:
        feasible = cache["robot_reachable_legs"][..., leg].any(axis=1)
    if feasible.shape != (voxels.grid_size,):
        raise ValueError("feasible voxels and workspace grid have different sizes")
    return voxels.center[feasible]


def normal_spline(start, end, start_normal, end_normal, variables) -> BSpline:
    """8控制点三次B样条：抬落脚控制点之间使用3个自由连接控制点。"""
    # 变量顺序：raise_dist, ctrl1_xyz, ctrl2_xyz, ctrl3_xyz, down_dist。
    controls = np.vstack([
        start, start + variables[0] * start_normal, variables[1:4],
        variables[4:7], variables[7:10], end + variables[10] * end_normal, end, end,
    ])
    return BSpline([0, 0, 0, 0, 1/5, 2/5, 3/5, 4/5, 1, 1, 1, 1], controls, 3)


def sample_workspace_distance(voxels: Voxels, distances: np.ndarray,
                              points: np.ndarray) -> np.ndarray:
    """优化专用 float64 三线性采样，避免公共接口的 float32 坐标/输出量化。"""
    points = np.asarray(points, dtype=np.float64)
    grid_pos = ((points - voxels._bounds[:, 0].astype(np.float64))
                / np.float64(voxels.voxel_scale) - 0.5).T
    return map_coordinates(np.asarray(distances, dtype=np.float64), grid_pos,
                           output=np.float64, order=1, mode="grid-constant",
                           cval=-float(voxels.voxel_scale), prefilter=False)


def workspace_penalty(to_boundary: np.ndarray) -> float:
    """内部为正；达到4cm净空时惩罚归零，外部及净空不足时持续惩罚。"""
    deficit = np.maximum(WORKSPACE_CLEARANCE - np.asarray(to_boundary, dtype=np.float64), 0.0)
    return float(np.sum(deficit**2))


def obstacle_signed_distance(points: np.ndarray, obstacles: tuple[BoxObstacle, ...]) -> np.ndarray:
    """返回各点到全部有向长方体的最小有符号距离，外正内负，输入(N,3)。"""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("obstacle points must be a finite (N,3) array")
    distances = np.full(len(points), np.inf, dtype=np.float64)
    for obstacle in obstacles:
        local_points = (points - obstacle.center) @ obstacle.rotation
        q = np.abs(local_points) - obstacle.size / 2.0
        sdf = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
        distances = np.minimum(distances, sdf)
    return distances


def obstacle_penalty(distances: np.ndarray) -> float:
    """表面外的净空不足与内部穿透均按距离缺额平方惩罚。"""
    deficit = np.maximum(OBSTACLE_CLEARANCE - np.asarray(distances, dtype=np.float64), 0.0)
    return float(np.sum(deficit**2))


def optimize_swing(leg: SwingConfig, workspace, obstacles: tuple[BoxObstacle, ...] = ()) -> SwingResult:
    """只规划普通 B 系曲线，不调用完整 _SwingTrajCal 及其逆解流程。"""
    start_n, end_n = leg.start_norm, leg.end_norm
    raise_dist, down_dist = RAISE_DISTANCE, DOWN_DISTANCE
    raise_point = leg.start + raise_dist * start_n
    down_point = leg.end + down_dist * end_n
    voxels, distances = workspace
    integrate = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
    initial_values = np.r_[raise_dist, (3 * raise_point + down_point) / 4,
                           (raise_point + down_point) / 2,
                           (raise_point + 3 * down_point) / 4, down_dist].astype(np.float64)
    bounds = [(min(0.06, float(raise_dist)), 0.1)] + [(-0.32, 0.32)] * 9 + [
        (min(0.06, float(down_dist)), 0.1)]
    polyline = np.vstack([leg.start, raise_point, down_point, leg.end])
    duration = max(np.linalg.norm(np.diff(polyline, axis=0), axis=1).sum()
                   / (MAX_FOOT_SPEED * 2), SAMPLE_DT)
    u = np.linspace(0, 1, ceil(duration / SAMPLE_DT) + 1)

    def spline(values):
        return normal_spline(leg.start, leg.end, start_n, end_n, values)

    def cost(values):
        curve = spline(values)
        points = curve(u).T
        acc = curve.derivative(2)(u).T / duration**1.5
        smooth = integrate(np.sum(acc**2, axis=0), u)
        to_boundary = sample_workspace_distance(voxels, distances, points.T)
        workspace_cost = workspace_penalty(to_boundary)
        obstacle_cost = (obstacle_penalty(obstacle_signed_distance(points.T, obstacles))
                         if obstacles else 0.0)
        return float(0.2 * smooth + 3.0 * workspace_cost + OBSTACLE_WEIGHT * obstacle_cost)

    initial = spline(initial_values)
    initial_cost = cost(initial_values)
    if not np.isfinite(initial_cost):
        raise ValueError("initial swing cost is not finite")
    result = minimize(cost, initial_values, method="L-BFGS-B", bounds=bounds,
                      options={"maxiter": 50, "ftol": 1e-4, "gtol": 1e-4})
    finite = np.isfinite(result.x).all() and np.isfinite(result.fun)
    optimized = spline(result.x) if finite else initial
    message = str(result.message) if finite else "Non-finite result; using initial spline"
    display_u = np.linspace(0, 1, SWING_SAMPLE_COUNT)
    initial_distance = (float(obstacle_signed_distance(initial(display_u), obstacles).min())
                        if obstacles else None)
    optimized_distance = (float(obstacle_signed_distance(optimized(display_u), obstacles).min())
                          if obstacles else None)
    return SwingResult(initial, optimized, initial_cost,
                       cost(result.x) if finite else initial_cost,
                       bool(result.success and finite), message,
                       initial_distance, optimized_distance)


def obstacle_distance_text(result: SwingResult) -> str:
    if result.initial_obstacle_distance is None:
        return ""
    return (f"Min obstacle distance (sampled, m): {result.initial_obstacle_distance:.5f}"
            f" -> {result.optimized_obstacle_distance:.5f}\n")


def _cloud(plotter, points, **style):
    if len(points):
        plotter.add_mesh(pv.PolyData(np.asarray(points, dtype=float)),
                         render_points_as_spheres=True, **style)


def _line(plotter, points, **style):
    if len(points) >= 2:
        plotter.add_mesh(pv.lines_from_points(np.asarray(points, dtype=float)), **style)


class SwingTrajectoryVisualizer:
    """单腿摆动场景；build_scene 支持离屏渲染。"""

    def __init__(self, robot_config=None):
        from map_robot_voxles.cache_tools import load_config as read_config, resolve_path
        config = read_config(robot_config or package_resource("map_robot_voxles", "config/hex_magnetic.yaml"))
        global WORKSPACE_CACHE, WORKSPACE_MIN_B, WORKSPACE_VOXEL_SCALE
        WORKSPACE_CACHE = resolve_path(config, config["cache_dir"]) / "robot_reachable_legs.npz"
        grid = config.get("robot_voxels", {})
        WORKSPACE_MIN_B = np.asarray(grid.get("leg_xyz_range", [[-0.14, 0.38], [-0.38, 0.38], [-0.4, 0.4]]))[:, 0]
        WORKSPACE_VOXEL_SCALE = float(grid.get("voxel_scale", 0.01))
        with np.load(WORKSPACE_CACHE) as cache:
            shape = cache["to_bound_dist"].shape
        expected = np.rint((np.asarray(grid.get("leg_xyz_range", [[-0.14, 0.38], [-0.38, 0.38], [-0.4, 0.4]]))[:, 1] - WORKSPACE_MIN_B) / WORKSPACE_VOXEL_SCALE).astype(int)
        if shape != tuple(expected) + (6,):
            raise ValueError("workspace cache shape does not match robot config")
        self.swing_leg = load_config()
        self.obstacles = build_obstacles(self.swing_leg)
        workspace = load_workspace(self.swing_leg.index)
        self.feasible_points = (load_feasible_points(workspace[0], self.swing_leg.index)
                                if SHOW_FEASIBLE_VOXELS else np.empty((0, 3)))
        self.swings = tuple(optimize_swing(swing, workspace, self.obstacles) for swing in self.swing_leg.swings)

    def build_scene(self, off_screen=False) -> pv.Plotter:
        plot = pv.Plotter(off_screen=off_screen, window_size=WINDOW_SIZE,
                          title="Single leg swing (B)")
        try:
            plot.set_background("white")
            plot.add_axes()
            self._swing_scene(plot)
            plot.view_isometric()
            plot.camera.is_set = True
            plot.reset_camera()
            return plot
        except Exception:
            plot.close()
            raise

    def _swing_scene(self, plot):
        leg = self.swing_leg
        for number, box in enumerate(self.obstacles):
            obstacle = pv.Cube(x_length=box.size[0], y_length=box.size[1], z_length=box.size[2])
            transform = np.eye(4, dtype=np.float64)
            transform[:3, :3], transform[:3, 3] = box.rotation, box.center
            obstacle.transform(transform, inplace=True)
            plot.add_mesh(obstacle, name=f"obstacle_{number}",
                          color="seagreen" if box.contact else "slategray", opacity=0.65,
                          show_edges=True, edge_color="black", label=box.name)
        if len(self.feasible_points):
            plot.add_mesh(pv.PolyData(self.feasible_points), name="feasible_voxels",
                          style="points", color=VOXEL_COLOR, opacity=VOXEL_OPACITY,
                          point_size=VOXEL_POINT_SIZE, render_points_as_spheres=True)
        u = np.linspace(0, 1, SWING_SAMPLE_COUNT)
        status = [f"B frame | leg {leg.index} {LEG_NAMES[leg.index]}"]
        for number, (config, result) in enumerate(zip(leg.swings, self.swings), start=1):
            for name, curve, color in (
                ("Initial", result.initial, "darkorange"),
                ("Optimized", result.optimized, "royalblue"),
            ):
                _line(plot, curve(u), color=color, line_width=16, label=f"{number} {name}")
                _line(plot, curve.c, color=color, line_width=8, opacity=0.6)
                _cloud(plot, curve.c, color=color, point_size=12, opacity=0.6)
            _cloud(plot, np.vstack([config.start, config.end]), color="black", point_size=15)
            for point, normal in ((config.start, config.start_norm), (config.end, config.end_norm)):
                plot.add_mesh(pv.Arrow(start=point, direction=normal, scale=0.05), color="black")
            status.append(
                f"Swing {number}: cost {result.initial_cost:.6g} -> {result.optimized_cost:.6g}\n"
                + obstacle_distance_text(result)
                + f"Converged: {result.success} | {result.message}"
            )
        plot.add_text("\n".join(status), position="upper_left", font_size=10, color="black")
        plot.add_legend(bcolor="white", loc="lower right")


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default=str(package_resource("map_robot_voxles", "config/hex_magnetic.yaml")))
    parser.add_argument("--leg-index", type=int, choices=range(6))
    parser.add_argument("--off-screen", action="store_true")
    parser.add_argument("--screenshot")
    args = parser.parse_args(argv)
    global SWING_LEG_INDEX
    if args.leg_index is not None:
        SWING_LEG_INDEX = args.leg_index
    viewer = SwingTrajectoryVisualizer(args.robot_config)
    for number, result in enumerate(viewer.swings, start=1):
        print(f"Swing {number}: {result.message}")
        print(f"Cost: {result.initial_cost:.6g} -> {result.optimized_cost:.6g}")
        if result.initial_obstacle_distance is not None:
            print(obstacle_distance_text(result).rstrip())
    plot = viewer.build_scene(off_screen=args.off_screen or args.screenshot is not None)
    try:
        plot.show(screenshot=args.screenshot, interactive=not args.off_screen)
    finally:
        if not plot._closed:
            plot.close()


if __name__ == "__main__":
    main()
