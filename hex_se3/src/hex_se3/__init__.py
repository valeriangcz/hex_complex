"""SE(3) trajectory annotation, I/O, optimization and playback."""

from .trajectory_io import (
    BezierSegment, PoseSeries, TrajectoryDocument, load_trajectory,
    pose_block, se3_de_casteljau, se3_interpolate, write_trajectory,
)

__all__ = [
    "BezierSegment", "PoseSeries", "TrajectoryDocument", "load_trajectory",
    "pose_block", "se3_de_casteljau", "se3_interpolate", "write_trajectory",
]
