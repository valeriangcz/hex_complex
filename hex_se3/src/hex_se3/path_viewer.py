"""Static and animated trajectory views using shared map/robot renderers."""

from dataclasses import dataclass
import time
import numpy as np

from map_robot_voxles.visualization import ActorLayer, FeasibilityRenderer, MapRenderer
from .trajectory_io import PoseSeries, TrajectoryDocument


@dataclass
class Playback:
    series: PoseSeries
    speed: float = 1.0
    current_time: float = 0.0
    playing: bool = False

    @property
    def pose(self):
        return self.series.pose_at(self.current_time)

    def advance(self, seconds):
        if self.playing:
            self.current_time = min(
                float(self.series.time[-1]),
                self.current_time + max(0.0, float(seconds)) * self.speed,
            )
            if self.current_time >= self.series.time[-1]:
                self.playing = False

    def step(self, direction):
        times = self.series.time
        if direction > 0:
            index = np.searchsorted(times, self.current_time + 1e-12, side="right")
            self.current_time = float(times[min(index, len(times) - 1)])
        else:
            index = np.searchsorted(times, self.current_time - 1e-12, side="left") - 1
            self.current_time = float(times[max(index, 0)])
        self.playing = False


class PathViewer:
    def __init__(self, state, document: TrajectoryDocument, *, mode="animation",
                 source="dense", curve_dt=0.05, speed=1.0, stride=16,
                 selected_indices=None):
        if mode not in ("static", "animation"):
            raise ValueError("mode must be static or animation")
        self.state = state
        self.document = document
        self.mode = mode
        self.stride = max(1, int(stride))
        self.selected_indices = selected_indices
        if source == "dense" and document.dense is not None:
            series = document.dense
        elif source == "bezier":
            series = document.sample_bezier(curve_dt)
        elif source in ("dense", "waypoints", "teleop"):
            poses = document.waypoints
            series = PoseSeries(np.arange(len(poses), dtype=float), poses)
        else:
            raise ValueError("unknown source: {}".format(source))
        self.playback = Playback(series, float(speed), float(series.time[0]))
        self.plotter = self.path_layer = self.feasibility = None
        self._timer_id = self._timer_observer = self._last_wall = None

    def _render_pose(self, pose, render=True):
        result = self.state.RobotFeasiCheck(pose)
        self.feasibility.render(self.state, pose, result,
                                env_stride=self.stride,
                                leg_voxel_stride=self.stride)
        if render:
            self.plotter.render()

    def build_scene(self, off_screen=False):
        import pyvista as pv
        self.plotter = pv.Plotter(off_screen=off_screen, window_size=(1280, 820))
        self.plotter.set_background("white")
        self.path_layer = ActorLayer(self.plotter, "path")
        self.feasibility = FeasibilityRenderer(self.plotter, "pose")
        MapRenderer(self.plotter, "background").render(
            self.state.env_pointsmap_voxels, stride=self.stride)
        positions = np.asarray([pose.t for pose in self.playback.series.poses])
        self.path_layer.line("line", positions, color="royalblue", line_width=5)
        self.path_layer.points("samples", positions, color="royalblue",
                               point_size=7, opacity=0.7)
        if self.mode == "static":
            indices = self.selected_indices
            if indices is None:
                indices = sorted(set((0, len(self.playback.series.poses) - 1)))
            for index in indices:
                if not 0 <= index < len(self.playback.series.poses):
                    raise IndexError("selected pose index out of range: {}".format(index))
                pose = self.playback.series.poses[index]
                result = self.state.RobotFeasiCheck(pose)
                FeasibilityRenderer(self.plotter, "static_{}".format(index)).render(
                    self.state, pose, result, env_stride=self.stride,
                    leg_voxel_stride=self.stride)
        else:
            self._render_pose(self.playback.pose, render=False)
            self._register_keys()
        self.plotter.show_axes()
        return self.plotter

    def _register_keys(self):
        self.plotter.add_key_event("space", self._toggle)
        for key in ("Right", "n"):
            self.plotter.add_key_event(key, lambda: self._step(1))
        for key in ("Left", "b"):
            self.plotter.add_key_event(key, lambda: self._step(-1))
        self.plotter.add_key_event("r", self._reset)
        self.plotter.add_key_event("plus", lambda: self._speed(1.25))
        self.plotter.add_key_event("minus", lambda: self._speed(0.8))
        self.plotter.add_key_event("q", self.plotter.close)

    def _toggle(self):
        self.playback.playing = not self.playback.playing
        self._last_wall = time.monotonic()

    def _step(self, direction):
        self.playback.step(direction)
        self._render_pose(self.playback.pose)

    def _reset(self):
        self.playback.current_time = float(self.playback.series.time[0])
        self.playback.playing = False
        self._render_pose(self.playback.pose)

    def _speed(self, factor):
        self.playback.speed *= factor
        print("playback speed={:.3g}x".format(self.playback.speed))

    def _timer(self, *_):
        now = time.monotonic()
        if self._last_wall is None:
            self._last_wall = now
        if self.playback.playing:
            self.playback.advance(now - self._last_wall)
            self._render_pose(self.playback.pose)
        self._last_wall = now

    def run(self):
        plotter = self.build_scene(False)
        if self.mode == "animation":
            self._timer_observer = plotter.iren.add_observer("TimerEvent", self._timer)
            self._timer_id = plotter.iren.create_timer(33, repeating=True)
            print("Space play/pause; Left/Right step; R reset; +/- speed; Q close")
        try:
            plotter.show(title="SE3 Path Viewer")
        finally:
            if self._timer_id is not None:
                plotter.iren.destroy_timer(self._timer_id)
            if self._timer_observer is not None:
                plotter.iren.remove_observer(self._timer_observer)
            plotter.close()
