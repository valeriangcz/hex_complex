"""Original-style PyVista SE(3) path feasibility viewer.

Loads optimized or teleop trajectories through trajectory_io and uses explicitly
configured local map/robot caches. No optimizer is imported or run.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from pathlib import Path
import time
from typing import Any, Sequence

import numpy as np
import pyvista as pv
from scipy.spatial.transform import Rotation
from spatialmath import SE3

from map_robot_voxles.resources import package_resource
from map_robot_voxles import HexState
from .trajectory_io import (
    PoseSeries, TrajectoryDocument as SE3PathData, load_trajectory,
    parse_pose_block as _parse_pose_block,
)


SHOW_INDEX = True  # Show 0-based indices in path_data.waypoints.

GREEN = [0.1, 0.8, 0.1]
YELLOW = [0.9, 0.7, 0.1]
RED = [0.9, 0.1, 0.1]
BODY_GRAY = [0.5, 0.5, 0.5]
PATH_BLUE = [0.1, 0.25, 0.9]
LEG_COLORS = (
    [0.9, 0.1, 0.1], [0.1, 0.65, 0.2], [0.1, 0.25, 0.9],
    [0.9, 0.75, 0.05], [0.75, 0.15, 0.9], [0.0, 0.75, 0.85],
)
DEFAULT_MAP_STL = str(
    package_resource("map_robot_voxles", "resources/maps/complex_surface4.STL")
)


def load_se3_path_json(path):
    """Load optimized or teleop paths through the shared trajectory reader."""
    document = load_trajectory(path)
    if document.dense is None:
        document = replace(document, dense=PoseSeries(
            np.arange(len(document.waypoints), dtype=float), document.waypoints))
    return document


class PathPlayback:
    """GUI-independent timeline shared by keyboard and timer callbacks."""

    def __init__(
        self, dense: PoseSeries, bezier: PoseSeries, source: str = "dense", speed: float = 1.0
    ) -> None:
        if source not in {"dense", "bezier"}:
            raise ValueError("source must be 'dense' or 'bezier'")
        if not np.isfinite(speed) or speed <= 0.0:
            raise ValueError("speed must be finite and positive")
        self.series = {"dense": dense, "bezier": bezier}
        self.source = source
        self.speed = float(speed)
        self.current_time = 0.0
        self.playing = False

    @property
    def active(self) -> PoseSeries:
        return self.series[self.source]

    @property
    def progress(self) -> float:
        return 0.0 if self.active.duration <= 0 else self.current_time / self.active.duration

    @property
    def pose(self) -> SE3:
        return self.active.pose_at(self.current_time)

    def reset(self) -> None:
        self.current_time = 0.0
        self.playing = False

    def step(self, direction: int) -> None:
        times = self.active.time
        if direction > 0:
            index = int(np.searchsorted(times, self.current_time + 1e-12, side="right"))
            self.current_time = float(times[min(index, len(times) - 1)])
        elif direction < 0:
            index = int(np.searchsorted(times, self.current_time - 1e-12, side="left")) - 1
            self.current_time = float(times[max(index, 0)])
        self.playing = False

    def advance(self, wall_seconds: float) -> None:
        if not self.playing:
            return
        self.current_time += max(0.0, float(wall_seconds)) * self.speed
        if self.current_time >= self.active.duration:
            self.current_time = self.active.duration
            self.playing = False

    def switch_source(self) -> None:
        ratio = float(np.clip(self.progress, 0.0, 1.0))
        self.source = "bezier" if self.source == "dense" else "dense"
        self.current_time = ratio * self.active.duration


@dataclass(frozen=True)
class FeasibilityStatus:
    body_valid: bool
    leg_counts: tuple[int, ...]
    level: str


def classify_feasibility(
    body_collision_free: np.ndarray,
    leg_feasible: np.ndarray,
    min_landing_per_leg: int = 3,
) -> FeasibilityStatus:
    """Collapse the planner masks into strong/weak/invalid display state."""
    body_valid = bool(np.asarray(body_collision_free, dtype=bool).all())
    mask = np.asarray(leg_feasible, dtype=bool)
    if mask.ndim != 3 or mask.shape[0] != 6:
        raise ValueError("leg_feasible must have shape (6, landing_count, branches)")
    counts = tuple(int(value) for value in mask.any(axis=-1).sum(axis=1))
    if body_valid and all(value >= min_landing_per_leg for value in counts):
        level = "strong"
    elif body_valid and all(value >= 1 for value in counts):
        level = "weak"
    else:
        level = "invalid"
    return FeasibilityStatus(body_valid, counts, level)


def _surface_flat_indices(voxels: Any, mask_flat: np.ndarray) -> np.ndarray:
    mask = np.asarray(mask_flat, dtype=bool).reshape(-1)
    candidate = np.flatnonzero(mask)
    if candidate.size == 0:
        return candidate
    grid = voxels.FlatIndex2GridIndex(candidate)
    offsets = np.asarray(
        ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)),
        dtype=np.int32,
    )
    boundary = np.zeros(candidate.size, dtype=bool)
    shape = np.asarray(voxels.grid_shape, dtype=np.int32)
    for offset in offsets:
        neighbor = grid + offset
        inside = ((neighbor >= 0) & (neighbor < shape)).all(axis=1)
        boundary |= ~inside
        if inside.any():
            neighbor_flat = voxels.GridIndex2FlatIndex(neighbor[inside])
            boundary[inside] |= ~mask[neighbor_flat]
    return candidate[boundary]


class SE3PathVisualizer:
    """Interactive PyVista scene for a loaded :class:`SE3PathData`."""

    def __init__(
        self,
        hex_state: HexState,
        path_data: SE3PathData,
        map_stl: str | Path = DEFAULT_MAP_STL,
        source: str = "dense",
        landing_stride: int = 10,
        curve_dt: float = 0.05,
        speed: float = 1.0,
        min_landing_per_leg: int = 3,
    ) -> None:
        if landing_stride < 1:
            raise ValueError("landing_stride must be at least 1")
        if min_landing_per_leg < 1:
            raise ValueError("min_landing_per_leg must be at least 1")
        self.hex_state = hex_state
        self.pointmap = hex_state.env_pointsmap_voxels
        self.path_data = path_data
        self.map_stl = Path(map_stl)
        if not self.map_stl.is_file():
            raise FileNotFoundError(f"map STL not found: {self.map_stl}")
        self.landing_stride = int(landing_stride)
        self.min_landing_per_leg = int(min_landing_per_leg)
        self.playback = PathPlayback(
            path_data.dense, (path_data.sample_bezier(curve_dt)
                             if path_data.bezier_segments else path_data.dense), source, speed
        )
        self.plotter: pv.Plotter | None = None
        self.follow_camera = False
        self.show_feasible_landings = True
        self._last_wall_time = time.monotonic()
        self._last_robot_position: np.ndarray | None = None
        self._timer_id: int | None = None
        self._timer_observer_id: int | None = None
        self._path_actor: Any = None
        self._status_actor: Any = None
        self._body_mesh: pv.PolyData | None = None
        self._body_actor: Any = None
        self._frame_mesh: pv.PolyData | None = None
        self._leg_meshes: list[pv.PolyData] = []
        self._leg_actors: list[Any] = []
        self._landing_meshes: list[pv.PolyData] = []
        self._landing_actors: list[Any] = []
        self._body_surface_R, self._leg_surfaces_R = self._robot_surfaces_R()

    def _robot_surfaces_R(self) -> tuple[np.ndarray, list[np.ndarray]]:
        robot = self.hex_state.robot_voxels
        body_voxels = robot.body_voxels
        body_surface = _surface_flat_indices(body_voxels, body_voxels.esdf_flat_for_env <= 0.0)
        body_points = np.asarray(body_voxels.center[body_surface], dtype=np.float64)
        within_force = robot.leg_voxels.mechanical_margin >= 90.0
        within_radius = np.linalg.norm(robot.leg_voxels.center, axis=1) >= 0.14
        legs: list[np.ndarray] = []
        for leg_index in range(6):
            reachable = robot.robot_reachable_legs[..., leg_index].any(axis=1)
            reachable &= within_force & within_radius
            surface = _surface_flat_indices(robot.leg_voxels, reachable)
            points_B = robot.leg_voxels.center[surface]
            points_R = robot.kin._B2R(points_B.T, leg_index).T
            legs.append(np.asarray(points_R, dtype=np.float64))
        return body_points, legs

    @staticmethod
    def _world_points(pose: SE3, points_R: np.ndarray) -> np.ndarray:
        if points_R.size == 0:
            return np.zeros((0, 3), dtype=np.float64)
        return np.asarray((pose * points_R.T).T, dtype=np.float64)

    def build_scene(self, off_screen: bool = False) -> pv.Plotter:
        """Build all static and mutable actors without entering the GUI loop."""
        self.plotter = pv.Plotter(off_screen=off_screen, window_size=(1440, 900))
        self.plotter.set_background("white")
        environment = pv.read(str(self.map_stl))
        self.plotter.add_mesh(
            environment, color="lightgray", opacity=0.08, show_edges=True,
            edge_color="silver", smooth_shading=False, name="environment",
        )
        landing = np.asarray(self.pointmap.landing_points[:: self.landing_stride])
        raw_points = getattr(self.pointmap, "_raw_points", None)
        nonlanding = np.asarray(self.pointmap.points[self.pointmap.landing_count:])
        for name, points, color, point_size, opacity in (
            ("raw_points", raw_points, "lightgray", 5.5, 0.18),
            ("static_landings", landing, "tomato", 6.0, 1.0),
            ("nonlanding_points", nonlanding, "black", 5.0, 1.0),
        ):
            if points is not None and len(points):
                self.plotter.add_mesh(
                    pv.PolyData(points), name=name, color=color,
                    point_size=point_size, opacity=opacity, render_points_as_spheres=True,
                )
        self._replace_path_actor()
        self._add_waypoints()
        self._add_robot_actors()
        self._status_actor = self.plotter.add_text(
            "", position="upper_left", font_size=10, color="#1a202c", name="status"
        )
        self.plotter.add_text(
            "Space play/pause | ←/→ step | C source | F camera | L landings | H help",
            position="lower_left", font_size=9, color="#4a5568", name="key_hint",
        )
        self._register_keys()
        self._update_frame()
        self.plotter.view_isometric()
        self.plotter.reset_camera()
        return self.plotter

    def _replace_path_actor(self) -> None:
        assert self.plotter is not None
        if self._path_actor is not None:
            self.plotter.remove_actor(self._path_actor, render=False)
        points = np.asarray([pose.t for pose in self.playback.active.poses])
        curve = (pv.lines_from_points(points, close=False) if len(points) > 1
                 else pv.PolyData(points))
        self._path_actor = self.plotter.add_mesh(
            curve, color=PATH_BLUE, line_width=10.0, render_lines_as_tubes=False,
            name="active_path",
        )

    def _add_waypoints(self) -> None:
        assert self.plotter is not None
        for waypoint_index, pose in enumerate(self.path_data.waypoints):
            frame = self._coordinate_frame(0.2, pose)
            self.plotter.add_mesh(
                frame, scalars="rgb", rgb=True, name=f"waypoint_{waypoint_index}",
            )
        if SHOW_INDEX and self.path_data.waypoints:
            positions = np.asarray([pose.t for pose in self.path_data.waypoints])
            self.plotter.add_point_labels(
                positions, [str(index) for index in range(len(positions))],
                font_size=24, text_color="black", point_size=0,
                shape=None, always_visible=True, name="waypoint_indices",
            )

    @staticmethod
    def _coordinate_frame(size: float, pose: SE3) -> pv.PolyData:
        """Build the labeler's gray origin sphere and RGB arrows at a pose."""
        frame = pv.Sphere(radius=0.06*size)
        frame["rgb"] = np.tile(np.asarray([0.5, 0.5, 0.5])*255, (frame.n_points, 1)).astype(np.uint8)
        for direction, color in zip(np.eye(3), np.eye(3)):
            arrow = pv.Arrow(
                direction=direction, scale=size, tip_length=0.2,
                tip_radius=0.06, shaft_radius=0.035,
            )
            arrow["rgb"] = np.tile(color*255, (arrow.n_points, 1)).astype(np.uint8)
            frame = frame.merge(arrow, merge_points=False)
        return frame.transform(pose.A, inplace=True)

    @staticmethod
    def _mutable_cloud(points: np.ndarray) -> pv.PolyData:
        # VTK cannot render a truly empty point set reliably. Hidden one-point
        # clouds are used until the first feasibility result is available.
        return pv.PolyData(points if len(points) else np.zeros((1, 3)))

    def _add_robot_actors(self) -> None:
        assert self.plotter is not None
        pose = self.playback.pose
        self._body_mesh = self._mutable_cloud(self._world_points(pose, self._body_surface_R))
        self._body_actor = self.plotter.add_mesh(
            self._body_mesh, color=BODY_GRAY, point_size=20.0, render_points_as_spheres=True,
            name="robot_body",
        )
        self._body_actor.SetVisibility(bool(len(self._body_surface_R)))
        self._frame_mesh = self._coordinate_frame(0.25, pose)
        self.plotter.add_mesh(self._frame_mesh, scalars="rgb", rgb=True, name="current_frame")
        for index, points_R in enumerate(self._leg_surfaces_R):
            mesh = self._mutable_cloud(self._world_points(pose, points_R))
            actor = self.plotter.add_mesh(
                mesh, color=LEG_COLORS[index], opacity=0.4, point_size=20.0,
                render_points_as_spheres=True, name=f"robot_leg_{index}",
            )
            actor.SetVisibility(bool(len(points_R)))
            landing_mesh = self._mutable_cloud(np.zeros((0, 3)))
            landing_actor = self.plotter.add_mesh(
                landing_mesh, color=LEG_COLORS[index], point_size=5.0,
                render_points_as_spheres=True, name=f"feasible_landing_{index}",
            )
            landing_actor.SetVisibility(False)
            self._leg_meshes.append(mesh)
            self._leg_actors.append(actor)
            self._landing_meshes.append(landing_mesh)
            self._landing_actors.append(landing_actor)

    @staticmethod
    def _set_actor_color(actor: Any, color: str | Sequence[float]) -> None:
        actor.prop.color = color

    def _query_feasibility(
        self, pose: SE3
    ) -> tuple[FeasibilityStatus, np.ndarray, np.ndarray, str | None]:
        try:
            _, landing_idx, body_mask, leg_mask = self.hex_state.RobotFeasiCheck(pose)
            status = classify_feasibility(
                body_mask, leg_mask, self.min_landing_per_leg
            )
            return status, np.asarray(landing_idx, dtype=np.int64), np.asarray(leg_mask), None
        except Exception as exc:  # Keep visualization responsive on malformed map data.
            status = FeasibilityStatus(False, (0, 0, 0, 0, 0, 0), "invalid")
            empty = np.zeros((6, 0, 2), dtype=bool)
            return status, np.zeros(0, dtype=np.int64), empty, str(exc)

    def _update_frame(self) -> None:
        if self.plotter is None or self._body_mesh is None:
            return
        pose = self.playback.pose
        status, landing_idx, leg_mask, error = self._query_feasibility(pose)
        self._body_mesh.points = self._world_points(pose, self._body_surface_R)
        body_color = GREEN if status.level == "strong" else (YELLOW if status.level == "weak" else RED)
        self._set_actor_color(self._body_actor, body_color)
        if self._frame_mesh is not None:
            self._frame_mesh.copy_from(self._coordinate_frame(0.25, pose))
        for index, (mesh, actor, points_R) in enumerate(
            zip(self._leg_meshes, self._leg_actors, self._leg_surfaces_R)
        ):
            mesh.points = self._world_points(pose, points_R)

            feasible = leg_mask[index].any(axis=-1) if leg_mask.shape[1] else np.zeros(0, bool)
            points = self.pointmap.landing_points[landing_idx[feasible]] if feasible.any() else np.zeros((0, 3))
            if len(points):
                self._landing_meshes[index].points = points
                self._landing_actors[index].SetVisibility(self.show_feasible_landings)
            else:
                self._landing_actors[index].SetVisibility(False)

        rpy = Rotation.from_matrix(pose.R).as_euler("xyz", degrees=True)
        state = status.level.upper()
        error_line = f"\nerror: {error}" if error else ""
        status_text = (
            f"{self.playback.source.upper()} | {state} | "
            f"t={self.playback.current_time:.2f}/{self.playback.active.duration:.2f}s "
            f"({100.0 * self.playback.progress:.1f}%) | speed={self.playback.speed:.2f}x\n"
            f"xyz=({pose.t[0]:.3f}, {pose.t[1]:.3f}, {pose.t[2]:.3f}) m | "
            f"rpy=({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}) deg\n"
            f"body={'free' if status.body_valid else 'collision'} | legs={list(status.leg_counts)}"
            f"{error_line}"
        )
        if hasattr(self._status_actor, "SetInput"):
            self._status_actor.SetInput(status_text)
        elif hasattr(self._status_actor, "SetText"):
            self._status_actor.SetText(0, status_text)

        if self.follow_camera:
            current = np.asarray(pose.t)
            if self._last_robot_position is not None:
                delta = current - self._last_robot_position
                camera = self.plotter.camera
                camera.position = tuple(np.asarray(camera.position) + delta)
                camera.focal_point = tuple(np.asarray(camera.focal_point) + delta)
            self._last_robot_position = current.copy()
        else:
            self._last_robot_position = np.asarray(pose.t).copy()
        self.plotter.render()

    def _register_keys(self) -> None:
        assert self.plotter is not None
        callbacks = {
            "space": self._toggle_play,
            "Right": lambda: self._step(1),
            "n": lambda: self._step(1),
            "Left": lambda: self._step(-1),
            "b": lambda: self._step(-1),
            "r": self._reset,
            "c": self._switch_source,
            "f": self._toggle_follow,
            "l": self._toggle_landings,
            "plus": lambda: self._change_speed(1.25),
            "equal": lambda: self._change_speed(1.25),
            "minus": lambda: self._change_speed(0.8),
            "h": self.print_help,
            "q": self._close,
            "Escape": self._close,
        }
        for key, callback in callbacks.items():
            self.plotter.add_key_event(key, callback)

    def _toggle_play(self) -> None:
        if self.playback.current_time >= self.playback.active.duration:
            self.playback.current_time = 0.0
        self.playback.playing = not self.playback.playing
        self._last_wall_time = time.monotonic()
        self._update_frame()

    def _step(self, direction: int) -> None:
        self.playback.step(direction)
        self._update_frame()

    def _reset(self) -> None:
        self.playback.reset()
        self._update_frame()

    def _switch_source(self) -> None:
        if not self.path_data.bezier_segments:
            return
        self.playback.switch_source()
        self._replace_path_actor()
        self._update_frame()

    def _toggle_follow(self) -> None:
        self.follow_camera = not self.follow_camera
        self._last_robot_position = np.asarray(self.playback.pose.t).copy()
        self._update_frame()

    def _toggle_landings(self) -> None:
        self.show_feasible_landings = not self.show_feasible_landings
        self._update_frame()

    def _change_speed(self, factor: float) -> None:
        self.playback.speed = float(np.clip(self.playback.speed * factor, 0.05, 20.0))
        self._update_frame()

    def _close(self) -> None:
        if self.plotter is not None:
            self.plotter.close()

    def _on_timer(self, _: int) -> None:
        now = time.monotonic()
        elapsed = now - self._last_wall_time
        self._last_wall_time = now
        if self.playback.playing:
            self.playback.advance(elapsed)
            self._update_frame()

    def _on_vtk_timer(self, _interactor: Any, _event: str) -> None:
        """Handle exactly one animation frame for each VTK TimerEvent.

        PyVista 0.44.2 ``add_timer_event`` runs all ``max_steps`` in a
        blocking ``while`` loop on the first event.  Registering the native
        repeating timer avoids starving window redraw and keyboard events.
        """
        self._on_timer(0)

    def _install_timer(self, duration_ms: int = 33) -> None:
        assert self.plotter is not None
        interactor = self.plotter.iren
        self._timer_observer_id = interactor.add_observer(
            "TimerEvent", self._on_vtk_timer
        )
        self._timer_id = interactor.create_timer(duration_ms, repeating=True)

    def _remove_timer(self) -> None:
        if self.plotter is None:
            return
        interactor = self.plotter.iren
        if interactor is None:
            return
        if self._timer_id is not None:
            try:
                interactor.destroy_timer(self._timer_id)
            except (AttributeError, RuntimeError):
                pass
            self._timer_id = None
        if self._timer_observer_id is not None:
            try:
                interactor.remove_observer(self._timer_observer_id)
            except (AttributeError, RuntimeError):
                pass
            self._timer_observer_id = None

    @staticmethod
    def print_help() -> None:
        print(
            "\nSE3 path viewer keys:\n"
            "  Space       auto play / pause\n"
            "  Right / N   next sample\n"
            "  Left / B    previous sample\n"
            "  R           reset to start\n"
            "  C           switch Dense / Bezier\n"
            "  F           fixed / follow camera\n"
            "  L           show / hide feasible landings\n"
            "  + / -       playback speed\n"
            "  H           print this help\n"
            "  Esc / Q     quit\n"
        )

    def run(self) -> None:
        plotter = self.build_scene(off_screen=False)
        self.print_help()
        self._last_wall_time = time.monotonic()
        self._install_timer(duration_ms=33)
        try:
            plotter.show(title="SE3 Path Feasibility Viewer")
        finally:
            self._remove_timer()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(package_resource("hex_se3", "config/complex_surface4.yaml")))
    parser.add_argument("--path", required=True, help="optimized or teleop SE3 path JSON")
    parser.add_argument("--source", choices=("dense", "bezier"), default="dense")
    parser.add_argument("--map-stl")
    parser.add_argument("--landing-stride", type=int, default=10)
    parser.add_argument("--curve-dt", type=float, default=0.05)
    parser.add_argument("--speed", type=float, default=5.0)
    parser.add_argument("--min-landing-per-leg", type=int, default=3)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    path_data = load_se3_path_json(args.path)
    from .runtime import build_hex_state, load_scene_config
    from map_robot_voxles.cache_tools import resolve_path
    state, map_config, _ = build_hex_state(load_scene_config(args.config))
    viewer = SE3PathVisualizer(
        state,
        path_data,
        map_stl=args.map_stl or resolve_path(map_config, map_config["source_mesh"]),
        source=args.source,
        landing_stride=args.landing_stride,
        curve_dt=args.curve_dt,
        speed=args.speed,
        min_landing_per_leg=args.min_landing_per_leg,
    )
    viewer.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
