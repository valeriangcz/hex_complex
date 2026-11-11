"""Small, stable-name PyVista actor layer shared by migration applications."""

import numpy as np


def _pyvista():
    try:
        import pyvista as pv
    except ImportError as exc:
        raise ImportError("map_robot_voxles visualization requires pyvista") from exc
    return pv


def transform_points(transform, points):
    """Transform an ``(N, 3)`` point array with SE3 or a homogeneous matrix."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if points.size == 0:
        return points.copy()
    if hasattr(transform, "A"):
        matrix = np.asarray(transform.A, dtype=float)
    else:
        matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("transform must be SE3 or a (4, 4) matrix")
    homogeneous = np.column_stack((points, np.ones(points.shape[0])))
    return (matrix @ homogeneous.T).T[:, :3]


class ActorLayer:
    """Add, replace, hide and remove actors using deterministic names."""

    def __init__(self, plotter, prefix="map_robot"):
        self.plotter = plotter
        self.prefix = str(prefix).strip("/")
        self._names = set()

    def name(self, key):
        return "{}/{}".format(self.prefix, key) if self.prefix else str(key)

    @property
    def names(self):
        return tuple(sorted(self._names))

    def points(self, key, points, **style):
        pv = _pyvista()
        name = self.name(key)
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        if points.shape[0] == 0:
            self.remove(key)
            return None
        defaults = {
            "render_points_as_spheres": True,
            "point_size": 5,
            "reset_camera": False,
            "render": False,
        }
        defaults.update(style)
        actor = self.plotter.add_mesh(pv.PolyData(points), name=name, **defaults)
        self._names.add(name)
        return actor

    def mesh(self, key, mesh, **style):
        name = self.name(key)
        defaults = {"reset_camera": False, "render": False}
        defaults.update(style)
        actor = self.plotter.add_mesh(mesh, name=name, **defaults)
        self._names.add(name)
        return actor

    def line(self, key, points, **style):
        pv = _pyvista()
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        if points.shape[0] < 2:
            self.remove(key)
            return None
        defaults = {"line_width": 3, "reset_camera": False, "render": False}
        defaults.update(style)
        return self.mesh(key, pv.lines_from_points(points), **defaults)

    def visible(self, key, visible=True):
        name = self.name(key)
        actor = self.plotter.actors.get(name)
        if actor is None:
            return False
        actor.SetVisibility(bool(visible))
        return True

    def remove(self, key):
        name = self.name(key)
        removed = self.plotter.remove_actor(name, reset_camera=False, render=False)
        self._names.discard(name)
        return removed

    def clear(self):
        for name in tuple(self._names):
            self.plotter.remove_actor(name, reset_camera=False, render=False)
        self._names.clear()

