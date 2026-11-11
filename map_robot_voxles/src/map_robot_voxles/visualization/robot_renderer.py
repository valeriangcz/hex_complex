"""Robot workspace and whole-body feasibility renderers."""

import numpy as np

from .scene import ActorLayer, transform_points


LEG_COLORS = ("red", "green", "blue", "yellow", "magenta", "cyan")


class RobotVoxelRenderer:
    def __init__(self, plotter, prefix="robot"):
        self.layer = ActorLayer(plotter, prefix)

    def render(self, robot_voxels, W_T_R=None, leg_stride=1):
        matrix = np.eye(4) if W_T_R is None else W_T_R
        leg_stride = max(1, int(leg_stride))
        body = robot_voxels.body_voxels
        body_occ = body.esdf_flat_for_env <= 0.0
        self.layer.points(
            "body", transform_points(matrix, body.center[body_occ]),
            color="orange", point_size=9, opacity=0.65,
        )
        for index, color in enumerate(LEG_COLORS):
            reachable = robot_voxels.robot_reachable_legs[..., index].any(axis=1)
            points_B = robot_voxels.leg_voxels.center[reachable][::leg_stride]
            points_R = robot_voxels.kin._B2R(points_B.T, index).T
            self.layer.points(
                "workspace_{}".format(index), transform_points(matrix, points_R),
                color=color, point_size=4, opacity=0.20,
            )
        return self.layer

    def render_leg(self, leg_voxels, leg_index):
        if isinstance(leg_index, (bool, np.bool_)) or not isinstance(leg_index, (int, np.integer)):
            raise TypeError("leg_index must be an integer in [0, 5]")
        if not 0 <= int(leg_index) < 6:
            raise ValueError("leg_index must be in [0, 5]")
        visible = (
            leg_voxels.reachable_legs[:, :, leg_index].any(axis=1)
            & np.isfinite(leg_voxels.mechanical_margin)
        )
        points = leg_voxels.center[visible]
        if points.shape[0] == 0:
            raise ValueError("selected leg has no reachable voxels with finite margin")
        import pyvista as pv
        cloud = pv.PolyData(points)
        cloud["mechanical_margin"] = leg_voxels.mechanical_margin[visible]
        self.layer.mesh(
            "leg_margin", cloud, scalars="mechanical_margin", cmap="viridis",
            clim=(60, int(np.ceil(27.0 / leg_voxels.kin.l3))),
            render_points_as_spheres=True, point_size=20, opacity=0.4,
        )
        self.layer.points("leg_origin", np.array([[0.145, 0.0, 0.0]]), point_size=30)
        return self.layer


class FeasibilityRenderer:
    """Render supplied HexState results without rerunning feasibility checks."""

    def __init__(self, plotter, prefix="feasibility"):
        self.plotter = plotter
        self.map_layer = ActorLayer(plotter, prefix + "/map")
        self.robot_renderer = RobotVoxelRenderer(plotter, prefix + "/robot")
        self.result_layer = ActorLayer(plotter, prefix + "/result")

    def render(self, state, W_T_R, check_result, env_stride=1, leg_voxel_stride=8):
        if check_result is None:
            raise ValueError("check_result is required; rendering never runs feasibility implicitly")
        _, landing_idx, body_mask, leg_masks = check_result
        env_stride = max(1, int(env_stride))
        point_map = state.env_pointsmap_voxels
        self.map_layer.points(
            "surface", point_map.points[::env_stride], color="lightgray",
            point_size=3, opacity=0.18,
        )
        self.map_layer.points(
            "landing", point_map.landing_points[::env_stride], color="black",
            point_size=5, opacity=0.22,
        )
        self.robot_renderer.render(state.robot_voxels, W_T_R, leg_voxel_stride)

        body_points, _ = state._BodyEnvironmentDistances(W_T_R)
        body_mask = np.asarray(body_mask, dtype=bool)
        if body_mask.shape != (len(body_points),):
            raise ValueError("body clearance mask must match body surface samples")
        self.result_layer.points(
            "body_clear", body_points[body_mask], color="deepskyblue",
            point_size=11, opacity=0.85,
        )
        self.result_layer.points(
            "body_collision", body_points[~body_mask], color="darkred",
            point_size=15, opacity=1.0,
        )
        candidates = point_map.landing_points[np.asarray(landing_idx, dtype=np.int64)]
        leg_names = state.robot_voxels.leg_voxels._leg_names
        for index, color in enumerate(LEG_COLORS):
            feasible = np.asarray(leg_masks[index], dtype=bool).any(axis=1)
            self.result_layer.points(
                "{}_rejected".format(leg_names[index]), candidates[~feasible],
                color="dimgray", point_size=5, opacity=0.12,
            )
            self.result_layer.points(
                "{}_feasible".format(leg_names[index]), candidates[feasible],
                color=color, point_size=14, opacity=0.95,
            )
        return self

    @property
    def actor_names(self):
        return self.map_layer.names + self.robot_renderer.layer.names + self.result_layer.names

    def set_visible(self, visible):
        for name in self.actor_names:
            actor = self.plotter.actors.get(name)
            if actor is not None:
                actor.SetVisibility(bool(visible))

    def remove(self):
        self.map_layer.clear()
        self.robot_renderer.layer.clear()
        self.result_layer.clear()
