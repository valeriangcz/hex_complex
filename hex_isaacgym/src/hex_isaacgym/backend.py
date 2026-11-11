"""Single-environment Isaac Gym backend for ExpertComplex."""

from pathlib import Path

import numpy as np
import trimesh

from hex_kinematic.conventions import COMMAND_JOINT_NAMES, LEG_NAMES, MOTOR_JOINT_NAMES
from hex_kinematic.se3_utils import transform_from_xyzw, transform_to_xyzw

from .actuation import update_adhesion_forces
from .config import BackendConfig
from .loader import load_isaacgym
from .types import ExpertCommand, InitialState, RobotState


class IsaacGymBackend:
    """Minimal one-robot physics owner with no RL state or callbacks."""

    def __init__(self, config, headless=True, initial_state=None):
        if isinstance(config, (str, Path)):
            config = BackendConfig.from_yaml(config)
        if not isinstance(config, BackendConfig):
            raise TypeError("config must be BackendConfig or a YAML path")
        config.validate()
        self.config = config
        self.headless = bool(headless)
        self._closed = False
        self._sim_time = 0.0
        self._run_elapsed = 0.0
        self._last_state = None
        self._viewer = None
        self._reference_path_points = None
        if initial_state is not None and not isinstance(initial_state, InitialState):
            initial_state = InitialState(**initial_state)
        self._creation_initial_state = initial_state

        self.gymapi, self.gymtorch = load_isaacgym(config.isaacgym_python)
        # Isaac Gym must be loaded before this import.
        import torch
        self.torch = torch
        self.device, compute_device_id, use_gpu = self._parse_device(config.device)
        self.gym = self.gymapi.acquire_gym()
        self.sim = None
        try:
            self._create_sim(compute_device_id, use_gpu)
            self._add_map()
            self._create_robot()
            self.gym.prepare_sim(self.sim)
            self._acquire_tensors()
            if initial_state is not None:
                # Apply the complete articulated pose before a viewer exists.
                # This avoids displaying the asset's zero pose at the origin.
                self.reset(initial_state)
            if not self.headless:
                self._create_viewer()
        except Exception:
            self.close()
            raise

    @staticmethod
    def _parse_device(device):
        if device == "cpu":
            return "cpu", 0, False
        parts = device.split(":", 1)
        device_id = int(parts[1]) if len(parts) == 2 else 0
        return "cuda:{}".format(device_id), device_id, True

    def _create_sim(self, compute_device_id, use_gpu):
        params = self.gymapi.SimParams()
        params.dt = float(self.config.physics_dt)
        params.substeps = int(self.config.substeps)
        params.up_axis = self.gymapi.UP_AXIS_Z
        params.gravity = self.gymapi.Vec3(*self.config.gravity)
        params.use_gpu_pipeline = use_gpu
        params.physx.use_gpu = use_gpu
        params.physx.num_threads = 10
        params.physx.num_position_iterations = 8
        params.physx.num_velocity_iterations = 2
        params.physx.contact_offset = 0.01
        params.physx.max_depenetration_velocity = 0.5
        params.physx.bounce_threshold_velocity = 0.2
        graphics_device_id = -1 if self.headless else compute_device_id
        self.sim = self.gym.create_sim(
            compute_device_id,
            graphics_device_id,
            self.gymapi.SIM_PHYSX,
            params,
        )
        if self.sim is None:
            raise RuntimeError("Isaac Gym failed to create the PhysX simulation")

    def _add_map(self):
        if self.config.terrain_type == "plane":
            params = self.gymapi.PlaneParams()
            params.normal = self.gymapi.Vec3(0.0, 0.0, 1.0)
            params.distance = 0.0
            params.static_friction = float(self.config.static_friction)
            params.dynamic_friction = float(self.config.dynamic_friction)
            params.restitution = float(self.config.restitution)
            self.gym.add_ground(self.sim, params)
            return
        mesh = trimesh.load(str(self.config.map_mesh), force="mesh", process=False)
        if mesh.vertices.size == 0 or mesh.faces.size == 0:
            raise ValueError("map mesh is empty: {}".format(self.config.map_mesh))
        vertices = np.asarray(mesh.vertices, dtype=np.float32)
        triangles = np.asarray(mesh.faces, dtype=np.uint32)
        params = self.gymapi.TriangleMeshParams()
        params.nb_vertices = vertices.shape[0]
        params.nb_triangles = triangles.shape[0]
        params.static_friction = float(self.config.static_friction)
        params.dynamic_friction = float(self.config.dynamic_friction)
        params.restitution = float(self.config.restitution)
        self.gym.add_triangle_mesh(
            self.sim,
            vertices.flatten(order="C"),
            triangles.flatten(order="C"),
            params,
        )

    def _create_robot(self):
        options = self.gymapi.AssetOptions()
        options.default_dof_drive_mode = self.gymapi.DOF_MODE_EFFORT
        options.collapse_fixed_joints = False
        options.replace_cylinder_with_capsule = False
        options.flip_visual_attachments = False
        options.fix_base_link = False
        options.density = 0.001
        options.angular_damping = 0.0
        options.linear_damping = 0.0
        options.max_angular_velocity = 1000.0
        options.max_linear_velocity = 1000.0
        options.armature = 0.01
        options.thickness = 0.01
        options.disable_gravity = False

        urdf = self.config.robot_urdf
        asset = self.gym.load_asset(self.sim, str(urdf.parent), urdf.name, options)
        if asset is None:
            raise RuntimeError("Isaac Gym failed to load robot asset {}".format(urdf))
        self.asset = asset
        self.num_dofs = self.gym.get_asset_dof_count(asset)
        self.num_bodies = self.gym.get_asset_rigid_body_count(asset)
        self.dof_names = tuple(self.gym.get_asset_dof_names(asset))
        self.body_names = tuple(self.gym.get_asset_rigid_body_names(asset))
        if self.num_dofs != 42 or self.num_bodies != 55:
            raise ValueError(
                "hex_magnetic asset expected 42 DOFs and 55 bodies, got {} and {}".format(
                    self.num_dofs, self.num_bodies
                )
            )
        self.command_indices_np = self._dof_indices(COMMAND_JOINT_NAMES)
        self.motor_indices_np = self._dof_indices(MOTOR_JOINT_NAMES)
        self.foot_indices_np = self._body_indices(("toe",))[:, 0]
        self.magnetic_indices_np = self._body_indices(("suck", "toe", "empty"))

        shape_props = self.gym.get_asset_rigid_shape_properties(asset)
        for prop in shape_props:
            prop.friction = float(self.config.robot_friction)
            prop.restitution = float(self.config.restitution)
        self.gym.set_asset_rigid_shape_properties(asset, shape_props)

        lower = self.gymapi.Vec3(0.0, 0.0, 0.0)
        upper = self.gymapi.Vec3(0.0, 0.0, 0.0)
        self.env = self.gym.create_env(self.sim, lower, upper, 1)
        start = self.gymapi.Transform()
        if self._creation_initial_state is not None:
            position, quaternion = transform_to_xyzw(
                self._creation_initial_state.W_T_R
            )
            start.p = self.gymapi.Vec3(*position)
            start.r = self.gymapi.Quat(*quaternion)
        self.actor = self.gym.create_actor(
            self.env,
            asset,
            start,
            "hex_magnetic",
            0,
            int(self.config.self_collisions),
            0,
        )
        if self.actor < 0:
            raise RuntimeError("Isaac Gym failed to create the robot actor")

        dof_props = self.gym.get_asset_dof_properties(asset)
        dof_props["driveMode"].fill(self.gymapi.DOF_MODE_EFFORT)
        dof_props["stiffness"].fill(0.0)
        dof_props["damping"].fill(0.0)
        self.gym.set_actor_dof_properties(self.env, self.actor, dof_props)
        self.torque_limits_np = np.asarray(dof_props["effort"], dtype=np.float32).copy()
        if self._creation_initial_state is not None:
            dof_state = self.gym.get_actor_dof_states(
                self.env, self.actor, self.gymapi.STATE_ALL
            )
            dof_state["pos"].fill(0.0)
            dof_state["vel"].fill(0.0)
            dof_state["pos"][self.command_indices_np] = (
                self._creation_initial_state.q.reshape(-1)
            )
            dof_state["vel"][self.command_indices_np] = (
                self._creation_initial_state.q_dot.reshape(-1)
            )
            if not self.gym.set_actor_dof_states(
                self.env, self.actor, dof_state, self.gymapi.STATE_ALL
            ):
                raise RuntimeError("failed to apply initial articulated DOF state")

        kp = np.zeros(self.num_dofs, dtype=np.float32)
        kd = np.zeros(self.num_dofs, dtype=np.float32)
        for index, name in enumerate(self.dof_names):
            joint = name.rsplit("_", 1)[-1]
            if joint in MOTOR_JOINT_NAMES:
                kp[index], kd[index] = 60.0, 1.0
            elif joint == "foot":
                kp[index], kd[index] = 20.0, 0.3
            elif joint in ("ball1", "ball2", "suck"):
                kp[index], kd[index] = 5.0, 0.1
            else:
                raise ValueError("unexpected physical DOF name: {}".format(name))
        self.kp_np, self.kd_np = kp, kd

    @property
    def motor_torque_limits(self):
        """Asset effort limits in public (6,3) motor order."""
        return self.torque_limits_np[self.motor_indices_np].reshape(6, 3).copy()

    def _dof_indices(self, joint_parts):
        names = [
            "j_{}_{}".format(leg.lower(), joint)
            for leg in LEG_NAMES
            for joint in joint_parts
        ]
        missing = [name for name in names if name not in self.dof_names]
        if missing:
            raise ValueError("robot asset is missing DOFs: {}".format(missing))
        return np.asarray([self.dof_names.index(name) for name in names], dtype=np.int64)

    def _body_indices(self, body_parts):
        names = [
            ["l_{}_{}".format(leg.lower(), part) for part in body_parts]
            for leg in LEG_NAMES
        ]
        missing = [name for leg_names in names for name in leg_names if name not in self.body_names]
        if missing:
            raise ValueError("robot asset is missing rigid bodies: {}".format(missing))
        return np.asarray(
            [[self.body_names.index(name) for name in leg_names] for leg_names in names],
            dtype=np.int64,
        )

    def _acquire_tensors(self):
        torch = self.torch
        self.root_states = self.gymtorch.wrap_tensor(
            self.gym.acquire_actor_root_state_tensor(self.sim)
        ).view(-1, 13)
        self.dof_states = self.gymtorch.wrap_tensor(
            self.gym.acquire_dof_state_tensor(self.sim)
        ).view(1, self.num_dofs, 2)
        self.contact_forces = self.gymtorch.wrap_tensor(
            self.gym.acquire_net_contact_force_tensor(self.sim)
        ).view(1, self.num_bodies, 3)
        self.rigid_body_states = self.gymtorch.wrap_tensor(
            self.gym.acquire_rigid_body_state_tensor(self.sim)
        ).view(1, self.num_bodies, 13)
        self.rb_forces = torch.zeros_like(self.contact_forces)
        self.rb_positions = torch.zeros_like(self.contact_forces)
        local_positions = torch.tensor(
            (
                (0.0, 0.02, 0.009),
                (0.02 * np.sqrt(3.0), -0.01, 0.0),
                (-0.02 * np.sqrt(3.0), -0.01, 0.0),
            ),
            dtype=torch.float32,
            device=self.device,
        )
        magnetic = torch.as_tensor(self.magnetic_indices_np, dtype=torch.long, device=self.device)
        self.rb_positions[0, magnetic.reshape(-1), :] = local_positions.repeat(6, 1)
        self.command_indices = torch.as_tensor(self.command_indices_np, dtype=torch.long, device=self.device)
        self.motor_indices = torch.as_tensor(self.motor_indices_np, dtype=torch.long, device=self.device)
        self.foot_indices = torch.as_tensor(self.foot_indices_np, dtype=torch.long, device=self.device)
        self.magnetic_indices = magnetic
        self.kp = torch.as_tensor(self.kp_np, dtype=torch.float32, device=self.device)
        self.kd = torch.as_tensor(self.kd_np, dtype=torch.float32, device=self.device)
        self.torque_limits = torch.as_tensor(
            self.torque_limits_np, dtype=torch.float32, device=self.device
        )
        self.q_des_full = torch.zeros(self.num_dofs, dtype=torch.float32, device=self.device)
        self.last_torques = torch.zeros(self.num_dofs, dtype=torch.float32, device=self.device)
        self.previous_contacts = np.zeros(6, dtype=bool)
        self._refresh()

    def _create_viewer(self):
        self._viewer = self.gym.create_viewer(self.sim, self.gymapi.CameraProperties())
        if self._viewer is None:
            raise RuntimeError("failed to create Isaac Gym viewer (check DISPLAY)")
        if self._creation_initial_state is None:
            target = np.array((2.5, 3.4, 0.48), dtype=np.float64)
            camera = np.array((2.5, 4.2, 0.6), dtype=np.float64)
        else:
            target = self._creation_initial_state.W_T_R[:3, 3]
            camera = target + np.array((0.9, 1.0, 0.6), dtype=np.float64)
        self.gym.viewer_camera_look_at(
            self._viewer,
            None,
            self.gymapi.Vec3(*camera),
            self.gymapi.Vec3(*target),
        )
        self._rebuild_viewer_lines()

    def _refresh(self):
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_net_contact_force_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)

    def set_reference_path(self, points):
        """Store and, when available, draw an ``(N,3)`` reference path."""
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3 or not len(points):
            raise ValueError("reference path points must have shape (N, 3)")
        if not np.isfinite(points).all():
            raise ValueError("reference path points must be finite")
        self._reference_path_points = points.copy()
        self._rebuild_viewer_lines()

    def _rebuild_viewer_lines(self):
        if self._viewer is None:
            return
        self.gym.clear_lines(self._viewer)
        if self._reference_path_points is None:
            return
        points = self._reference_path_points
        if len(points) > 2000:
            indices = np.linspace(0, len(points) - 1, 2000).astype(np.int64)
            points = points[indices]
        line_vertices = []
        line_colors = []
        if len(points) > 1:
            line_vertices.extend(np.stack((points[:-1], points[1:]), axis=1))
            line_colors.extend(np.tile((0.0, 1.0, 0.0), (len(points) - 1, 1)))
        # Small three-axis crosses make isolated and sparse waypoints visible.
        marker_points = points
        if len(marker_points) > 500:
            indices = np.linspace(0, len(marker_points) - 1, 500).astype(np.int64)
            marker_points = marker_points[indices]
        marker_radius = 0.012
        for axis in np.eye(3, dtype=np.float32):
            line_vertices.extend(
                np.stack(
                    (marker_points - marker_radius * axis,
                     marker_points + marker_radius * axis),
                    axis=1,
                )
            )
            line_colors.extend(np.tile((0.0, 1.0, 0.0), (len(marker_points), 1)))
        if line_vertices:
            line_data = np.asarray(line_vertices, dtype=np.float32).reshape(-1, 2, 3)
            color_data = np.asarray(line_colors, dtype=np.float32).reshape(-1, 3)
            vertices = np.empty((len(line_data), 2), dtype=self.gymapi.Vec3.dtype)
            colors = np.empty(len(line_data), dtype=self.gymapi.Vec3.dtype)
            vertices["x"], vertices["y"], vertices["z"] = (
                line_data[:, :, 0], line_data[:, :, 1], line_data[:, :, 2]
            )
            colors["x"], colors["y"], colors["z"] = (
                color_data[:, 0], color_data[:, 1], color_data[:, 2]
            )
            self.gym.add_lines(
                self._viewer, self.env, len(vertices), vertices, colors
            )

    def get_rigid_body_positions(self):
        """Return body positions from the latest physics refresh.

        With the GPU pipeline, a reset is reflected here after the next
        simulation step, when PhysX updates the articulated body transforms.
        """
        self._require_open()
        return (
            self.rigid_body_states[0, :, :3]
            .detach().cpu().numpy().copy()
        )

    def reset(self, initial_state):
        self._require_open()
        if not isinstance(initial_state, InitialState):
            initial_state = InitialState(**initial_state)
        position, quaternion = transform_to_xyzw(initial_state.W_T_R)
        root = self.root_states[0]
        root.zero_()
        root[:3] = self.torch.as_tensor(position, dtype=root.dtype, device=self.device)
        root[3:7] = self.torch.as_tensor(quaternion, dtype=root.dtype, device=self.device)
        dof = self.dof_states[0]
        dof.zero_()
        dof[self.command_indices, 0] = self.torch.as_tensor(
            initial_state.q.reshape(-1), dtype=dof.dtype, device=self.device
        )
        dof[self.command_indices, 1] = self.torch.as_tensor(
            initial_state.q_dot.reshape(-1), dtype=dof.dtype, device=self.device
        )
        self.gym.set_actor_root_state_tensor(
            self.sim, self.gymtorch.unwrap_tensor(self.root_states)
        )
        self.gym.set_dof_state_tensor(
            self.sim, self.gymtorch.unwrap_tensor(self.dof_states)
        )
        self.q_des_full.zero_()
        self.q_des_full[self.command_indices] = dof[self.command_indices, 0]
        self.last_torques.zero_()
        self.rb_forces.zero_()
        self.previous_contacts[:] = False
        self._run_elapsed = 0.0
        # GPU setters are applied by the next simulate(). Refreshing here can
        # overwrite the reset buffers with the previous physics state.
        self._last_state = self._read_state()
        return self._last_state

    def step(self, command):
        self._require_open()
        if self._last_state is None:
            raise RuntimeError("reset(initial_state) must be called before step")
        if not isinstance(command, ExpertCommand):
            command = ExpertCommand(**command)
        torch = self.torch
        self.q_des_full.zero_()
        self.q_des_full[self.command_indices] = torch.as_tensor(
            command.q_des.reshape(-1), dtype=torch.float32, device=self.device
        )

        contacts_np = self.contact_forces[0, self.foot_indices].detach().cpu().numpy()
        magnetic_z = self.rb_forces[
            0, self.magnetic_indices.reshape(-1), 2
        ].view(6, 3).detach().cpu().numpy()
        updated_z, current_contacts = update_adhesion_forces(
            magnetic_z,
            contacts_np,
            self.previous_contacts,
            command.adhesions,
            self._run_elapsed < self.config.initial_buffer_time,
            force_delta=self.config.suction_force_delta,
            force_max=self.config.suction_force_max,
            contact_threshold=self.config.contact_threshold,
        )
        self.previous_contacts = current_contacts
        self.rb_forces.zero_()
        self.rb_forces[0, self.magnetic_indices.reshape(-1), 2] = torch.as_tensor(
            updated_z.reshape(-1), dtype=torch.float32, device=self.device
        )
        applied_forces = -self.rb_forces
        tau_ff = torch.as_tensor(
            command.tau_ff.reshape(-1), dtype=torch.float32, device=self.device
        )

        for _ in range(self.config.decimation):
            q = self.dof_states[0, :, 0]
            q_dot = self.dof_states[0, :, 1]
            torque = self.kp * (self.q_des_full - q) - self.kd * q_dot
            torque[self.motor_indices] += tau_ff
            torque = torch.clamp(torque, -self.torque_limits, self.torque_limits)
            self.last_torques.copy_(torque)
            self.gym.set_dof_actuation_force_tensor(
                self.sim, self.gymtorch.unwrap_tensor(torque)
            )
            self.gym.apply_rigid_body_force_at_pos_tensors(
                self.sim,
                self.gymtorch.unwrap_tensor(applied_forces),
                self.gymtorch.unwrap_tensor(self.rb_positions),
                self.gymapi.LOCAL_SPACE,
            )
            self.gym.simulate(self.sim)
            # Complete physics on both pipelines before reading state or
            # calling step_graphics() for the viewer.
            self.gym.fetch_results(self.sim, True)
            self.gym.refresh_dof_state_tensor(self.sim)

        self._sim_time += self.config.control_dt
        self._run_elapsed += self.config.control_dt
        self._refresh()
        self._draw_viewer()
        self._last_state = self._read_state()
        return self._last_state

    def _read_state(self):
        root = self.root_states[0].detach().cpu().numpy()
        W_T_R = transform_from_xyzw(root[:3], root[3:7])
        q = self.dof_states[0, self.command_indices, 0].detach().cpu().numpy().reshape(6, 4)
        q_dot = self.dof_states[0, self.command_indices, 1].detach().cpu().numpy().reshape(6, 4)
        q_torque = self.last_torques[self.motor_indices].detach().cpu().numpy().reshape(6, 3)
        magnetic_z = self.rb_forces[
            0, self.magnetic_indices.reshape(-1), 2
        ].detach().cpu().numpy().reshape(6, 3)
        adhesion_force = np.abs(magnetic_z.sum(axis=1))
        contact = self.contact_forces[0, self.foot_indices].detach().cpu().numpy()
        return RobotState(
            sim_time=self._sim_time,
            run_elapsed=self._run_elapsed,
            W_T_R=W_T_R,
            q=q,
            q_dot=q_dot,
            q_torque=q_torque,
            adhesion_force=adhesion_force,
            contact_forces=contact,
        )

    def get_state(self):
        self._require_open()
        if self._last_state is None:
            raise RuntimeError("backend has not been reset")
        # step() already refreshes physics; reset() writes these buffers
        # directly. A refresh between reset() and simulate() can be stale.
        self._last_state = self._read_state()
        return self._last_state

    def _draw_viewer(self):
        if self._viewer is None:
            return
        if self.gym.query_viewer_has_closed(self._viewer):
            raise RuntimeError("Isaac Gym viewer was closed")
        self.gym.step_graphics(self.sim)
        self.gym.draw_viewer(self._viewer, self.sim, True)

    def _require_open(self):
        if self._closed or self.sim is None:
            raise RuntimeError("Isaac Gym backend is closed")

    def close(self):
        if self._closed:
            return
        if getattr(self, "gym", None) is not None:
            if self._viewer is not None:
                self.gym.destroy_viewer(self._viewer)
                self._viewer = None
            if self.sim is not None:
                self.gym.destroy_sim(self.sim)
                self.sim = None
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

