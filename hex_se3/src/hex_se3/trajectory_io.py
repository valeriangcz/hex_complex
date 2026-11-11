"""Validated I/O for legacy teleop and schema-version-1 SE(3) paths."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence
import warnings

import numpy as np
from scipy.spatial.transform import Rotation
from spatialmath import SE3
from spatialmath.base import trlog


@dataclass(frozen=True)
class PoseSeries:
    time: np.ndarray
    poses: tuple[SE3, ...]

    def __post_init__(self):
        times = np.asarray(self.time, dtype=np.float64)
        if times.ndim != 1 or len(times) != len(self.poses) or not len(times):
            raise ValueError("PoseSeries time and poses must have equal nonzero lengths")
        if not np.isfinite(times).all():
            raise ValueError("PoseSeries time must be finite")
        object.__setattr__(self, "time", times)

    @property
    def duration(self):
        return float(self.time[-1] - self.time[0])

    def pose_at(self, query_time):
        value = float(np.clip(query_time, self.time[0], self.time[-1]))
        high = int(np.searchsorted(self.time, value, side="right"))
        if high == 0:
            return self.poses[0].copy()
        if high >= len(self.time):
            return self.poses[-1].copy()
        low = high - 1
        span = float(self.time[high] - self.time[low])
        u = 0.0 if span <= 0 else (value - float(self.time[low])) / span
        return se3_interpolate(self.poses[low], self.poses[high], u)


@dataclass(frozen=True)
class BezierSegment:
    degree: int
    duration: float
    control_poses: tuple[SE3, ...]


@dataclass(frozen=True)
class TrajectoryDocument:
    format: str
    metadata: Mapping[str, Any]
    waypoints: tuple[SE3, ...]
    dense: PoseSeries | None
    bezier_segments: tuple[BezierSegment, ...]
    protected_pose_indices: tuple[int, ...]
    raw: Mapping[str, Any]

    @property
    def poses(self):
        """Primary geometric path; teleop has no fabricated timestamps."""
        return self.waypoints

    def source(self, name):
        if name in ("teleop", "waypoints"):
            return self.waypoints
        if name == "dense_poses" and self.dense is not None:
            return self.dense.poses
        raise KeyError("trajectory source {!r} is unavailable".format(name))

    def sample_bezier(self, dt):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("curve dt must be finite and positive")
        times, poses = [], []
        elapsed = 0.0
        for segment_index, segment in enumerate(self.bezier_segments):
            intervals = max(1, int(math.ceil(segment.duration / dt)))
            for local_index in range(intervals + 1):
                if segment_index and local_index == 0:
                    continue
                u = local_index / intervals
                times.append(elapsed + u * segment.duration)
                poses.append(se3_de_casteljau(segment.control_poses, u))
            elapsed += segment.duration
        if not poses:
            raise ValueError("trajectory has no Bezier segments")
        return PoseSeries(np.asarray(times), tuple(poses))


def se3_interpolate(first, second, u):
    return first * SE3.Exp(float(u) * trlog((first.inv() * second).A, check=False))


def se3_de_casteljau(control_poses: Sequence[SE3], u: float):
    if not control_poses:
        raise ValueError("control_poses must not be empty")
    if not np.isfinite(u):
        raise ValueError("Bezier parameter must be finite")
    if u <= 0:
        return control_poses[0].copy()
    if u >= 1:
        return control_poses[-1].copy()
    work = [pose.copy() for pose in control_poses]
    for level in range(1, len(work)):
        for index in range(len(work) - level):
            work[index] = se3_interpolate(work[index], work[index + 1], u)
    return work[0]


def _array_field(block, field, path, ndim):
    if not isinstance(block, Mapping) or field not in block:
        raise ValueError("missing field: {}.{}".format(path, field))
    try:
        value = np.asarray(block[field], dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError("{}.{} must be numeric".format(path, field)) from exc
    if value.ndim != ndim or not np.isfinite(value).all():
        raise ValueError("{}.{} must be a finite {}-D array".format(path, field, ndim))
    return value


def parse_pose_block(value, path="poses"):
    if not isinstance(value, Mapping):
        raise ValueError("{} must be an object".format(path))
    translations = _array_field(value, "t", path, 2)
    angles = _array_field(value, "ang", path, 1)
    axes = _array_field(value, "vec", path, 2)
    if translations.shape[1:] != (3,) or axes.shape[1:] != (3,):
        raise ValueError("{}.t and .vec must have shape (N, 3)".format(path))
    if not (len(translations) == len(angles) == len(axes)) or not len(angles):
        raise ValueError("{}.t, .ang and .vec need equal nonzero lengths".format(path))
    result = []
    for index, (translation, angle, axis) in enumerate(zip(translations, angles, axes)):
        if abs(float(angle)) <= 1e-12:
            rotation = SE3()
        else:
            norm = float(np.linalg.norm(axis))
            if norm <= 1e-12:
                raise ValueError("{}.vec[{}] has zero norm for nonzero angle".format(path, index))
            rotation = SE3.AngleAxis(float(angle), axis / norm)
        result.append(SE3.Trans(translation) * rotation)
    return tuple(result)


def pose_block(poses: Sequence[SE3]):
    translations, angles, axes = [], [], []
    for pose in poses:
        matrix = np.asarray(pose.A, dtype=np.float64)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError("cannot serialize a non-finite SE(3) pose")
        angle, axis = pose.angvec()
        if abs(float(angle)) <= 1e-12:
            angle, axis = 0.0, np.zeros(3)
        else:
            axis = np.asarray(axis) / np.linalg.norm(axis)
        translations.append(np.asarray(pose.t).reshape(3).tolist())
        angles.append(float(angle))
        axes.append(np.asarray(axis).reshape(3).tolist())
    return {"t": translations, "ang": angles, "vec": axes}


def _matrix_poses(raw, axis_poses):
    if "W_T_R" not in raw:
        return axis_poses
    matrices = np.asarray(raw["W_T_R"], dtype=np.float64)
    if matrices.shape != (len(axis_poses), 4, 4) or not np.isfinite(matrices).all():
        raise ValueError("W_T_R must have shape (N, 4, 4) and finite values")
    poses = []
    for index, matrix in enumerate(matrices):
        if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
            raise ValueError("W_T_R[{}] has an invalid homogeneous row".format(index))
        rotation = matrix[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or np.linalg.det(rotation) < 0:
            raise ValueError("W_T_R[{}] has an invalid rotation".format(index))
        matrix_pose = SE3(matrix, check=False)
        delta = axis_poses[index].inv() * matrix_pose
        translation_error = np.linalg.norm(delta.t)
        rotation_error = Rotation.from_matrix(delta.R).magnitude()
        if translation_error > 1e-4 or rotation_error > 1e-4:
            warnings.warn(
                "W_T_R[{}] disagrees with t/ang/vec; t/ang/vec takes precedence".format(index),
                UserWarning,
            )
        poses.append(axis_poses[index])
    return tuple(poses)


def _protected(raw, count):
    value = raw.get("protected_pose_indices", [])
    if not isinstance(value, list) or any(
        isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < count
        for index in value
    ):
        raise ValueError("protected_pose_indices must contain valid 0-based integers")
    return tuple(sorted(set(value)))


def load_trajectory(path):
    path = Path(path).expanduser()
    try:
        with path.open("r", encoding="utf-8") as stream:
            root = json.load(stream)
    except json.JSONDecodeError as exc:
        raise ValueError("invalid JSON in {}: {}".format(path, exc)) from exc
    if not isinstance(root, Mapping):
        raise ValueError("trajectory root must be an object")
    metadata = root.get("metadata", {})
    if not isinstance(metadata, Mapping):
        raise ValueError("metadata must be an object")
    if root.get("schema_version") == 1:
        waypoints = parse_pose_block(root.get("waypoints"), "waypoints")
        dense_root = root.get("dense_poses")
        dense_poses = parse_pose_block(dense_root, "dense_poses")
        dense_time = _array_field(dense_root, "time", "dense_poses", 1)
        if len(dense_time) != len(dense_poses) or len(dense_time) < 2:
            raise ValueError("dense time and poses need equal lengths >= 2")
        if abs(float(dense_time[0])) > 1e-12 or np.any(np.diff(dense_time) <= 0):
            raise ValueError("dense_poses.time must start at zero and strictly increase")
        raw_segments = root.get("bezier_segments")
        if not isinstance(raw_segments, list) or not raw_segments:
            raise ValueError("bezier_segments must be a nonempty array")
        segments = []
        for index, raw_segment in enumerate(raw_segments):
            field = "bezier_segments[{}]".format(index)
            if not isinstance(raw_segment, Mapping):
                raise ValueError("{} must be an object".format(field))
            degree = raw_segment.get("degree")
            duration = raw_segment.get("duration")
            if isinstance(degree, bool) or not isinstance(degree, int) or degree < 1:
                raise ValueError("{}.degree must be a positive integer".format(field))
            if isinstance(duration, bool) or not isinstance(duration, (int, float)) or not np.isfinite(duration) or duration <= 0:
                raise ValueError("{}.duration must be finite and positive".format(field))
            controls = parse_pose_block(raw_segment.get("control_poses"), field + ".control_poses")
            if len(controls) != degree + 1:
                raise ValueError("{}.control_poses count must equal degree + 1".format(field))
            segments.append(BezierSegment(degree, float(duration), controls))
        if len(waypoints) != len(segments) + 1:
            raise ValueError("waypoint count must equal segment count + 1")
        return TrajectoryDocument(
            "schema_v1", deepcopy(metadata), waypoints,
            PoseSeries(dense_time, dense_poses), tuple(segments), (), deepcopy(root),
        )
    if "schema_version" in root:
        raise ValueError("unsupported schema_version: {!r}".format(root["schema_version"]))
    axis_poses = parse_pose_block(root, "teleop")
    poses = _matrix_poses(root, axis_poses)
    protected = _protected(root, len(poses))
    return TrajectoryDocument(
        "teleop", deepcopy(metadata), poses, None, (), protected, deepcopy(root),
    )


def write_trajectory(document, path):
    """Losslessly write a loaded document, including unknown metadata fields."""
    if not isinstance(document, TrajectoryDocument):
        raise TypeError("document must be TrajectoryDocument")
    output = Path(path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        json.dump(deepcopy(document.raw), stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return output
