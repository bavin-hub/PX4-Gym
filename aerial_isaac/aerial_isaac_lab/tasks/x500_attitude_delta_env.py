"""X500 attitude-delta task ported from the Aerial Gym attitude repo."""

from __future__ import annotations

import math

import torch

from isaaclab.utils import configclass

from aerial_isaac_lab.core import LeeAttitudeYawController
from aerial_isaac_lab.core.math import (
    euler_xyz_from_quat,
    quat_apply_inverse,
    quat_from_euler_xyz,
    wrap_to_pi,
)

from .x500_velocity_env import X500VelocityEnv, X500VelocityEnvCfg


@configclass
class X500AttitudeDeltaEnvCfg(X500VelocityEnvCfg):
    """Aerial Gym ``position_setpoint_task_attitude_delta_x500`` constants."""

    target_min_ratio = (0.5, 0.5, 0.5)
    target_max_ratio = (0.5, 0.5, 0.5)
    minimum_target_distance = 0.0
    target_sampling_attempts = 6
    target_reached_distance = 0.20
    resample_target_on_reach = False

    px4_hover_thrust = -0.6
    px4_min_thrust = -0.9
    px4_max_thrust = -0.4
    delta_thrust_scale = 0.1
    max_delta_roll_pitch = math.radians(5.0)
    max_delta_yaw = math.radians(5.0)


class X500AttitudeDeltaEnv(X500VelocityEnv):
    """Direct-RL port whose policy emits ``[d_thrust, d_roll, d_pitch, d_yaw]``."""

    cfg: X500AttitudeDeltaEnvCfg

    def __init__(self, cfg: X500AttitudeDeltaEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._controller = LeeAttitudeYawController(self.num_envs, self.device)
        self._raw_actions = torch.zeros_like(self._actions)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._previous_actions[:] = self._actions
        self._previous_distance[:] = self._position_error_w().norm(dim=-1)

        clipped_actions = actions.clamp(-1.0, 1.0)
        self._raw_actions[:] = clipped_actions

        roll, pitch, yaw = euler_xyz_from_quat(self._robot.data.root_quat_w)
        delta_thrust = clipped_actions[:, 0] * self.cfg.delta_thrust_scale
        px4_thrust = torch.clamp(
            self.cfg.px4_hover_thrust + delta_thrust,
            self.cfg.px4_min_thrust,
            self.cfg.px4_max_thrust,
        )
        hover_thrust_magnitude = abs(self.cfg.px4_hover_thrust)

        self._actions[:, 0] = torch.clamp((-px4_thrust / hover_thrust_magnitude) - 1.0, -1.0, 1.0)
        self._actions[:, 1] = roll + clipped_actions[:, 1] * self.cfg.max_delta_roll_pitch
        self._actions[:, 2] = pitch + clipped_actions[:, 2] * self.cfg.max_delta_roll_pitch
        self._actions[:, 3] = wrap_to_pi(yaw + clipped_actions[:, 3] * self.cfg.max_delta_yaw)

    def _apply_action(self) -> None:
        root_quat_wxyz = self._robot.data.root_quat_w
        root_lin_vel_w = self._robot.data.root_lin_vel_w
        root_ang_vel_b = quat_apply_inverse(root_quat_wxyz, self._robot.data.root_ang_vel_w)
        wrench_b = self._controller.compute(
            self._actions,
            root_quat_wxyz,
            root_ang_vel_b,
            self._mass,
            self._inertia_b,
        )
        motor_forces_b, motor_torques_b = self._allocator.allocate_wrench(wrench_b)
        self._motor_forces_b.zero_()
        self._motor_torques_b.zero_()
        self._motor_forces_b[:, self._motor_body_ids] = motor_forces_b
        self._motor_torques_b[:, self._motor_body_ids] = motor_torques_b

        body_lin_vel = quat_apply_inverse(root_quat_wxyz, root_lin_vel_w)
        drag_force = -self._linear_drag * body_lin_vel - self._quadratic_drag * body_lin_vel.norm(
            dim=-1, keepdim=True
        ) * body_lin_vel
        drag_torque = -self._angular_drag * root_ang_vel_b - self._angular_quadratic_drag * root_ang_vel_b.abs() * root_ang_vel_b
        self._motor_forces_b[:, self._root_body_id] += drag_force
        self._motor_torques_b[:, self._root_body_id] += drag_torque
        self._robot.permanent_wrench_composer.set_forces_and_torques(
            body_ids=torch.arange(self._robot.num_bodies, device=self.device),
            forces=self._motor_forces_b,
            torques=self._motor_torques_b,
        )

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        super()._reset_idx(env_ids)
        if not hasattr(self, "_raw_actions"):
            return
        if env_ids is None:
            env_ids = self._robot._ALL_INDICES
        self._raw_actions[env_ids] = 0.0

    def _get_rewards(self) -> torch.Tensor:
        pos_error_w = self._position_error_w()
        distance = pos_error_w.norm(dim=-1)
        root_quat_wxyz = self._robot.data.root_quat_w
        body_lin_vel = quat_apply_inverse(root_quat_wxyz, self._robot.data.root_lin_vel_w)
        body_ang_vel = quat_apply_inverse(root_quat_wxyz, self._robot.data.root_ang_vel_w)
        _, _, yaw = euler_xyz_from_quat(root_quat_wxyz)
        vehicle_quat_wxyz = quat_from_euler_xyz(torch.zeros_like(yaw), torch.zeros_like(yaw), yaw)
        pos_error_vehicle_frame = quat_apply_inverse(vehicle_quat_wxyz, pos_error_w)

        yaw_error = self._goal_yaw_error(pos_error_w, yaw)
        eps = 1.0e-6
        speed = body_lin_vel.norm(dim=-1)
        cruise_speed = 1.0
        brake_radius = 1.0
        goal_dir = pos_error_vehicle_frame / (distance.unsqueeze(-1) + eps)
        desired_speed = cruise_speed * torch.clamp(distance / brake_radius, 0.0, 1.0)
        desired_vel = desired_speed.unsqueeze(-1) * goal_dir
        vel_error = body_lin_vel - desired_vel
        velocity_tracking = torch.exp(-2.0 * (vel_error * vel_error).sum(dim=-1))
        goal_reward = torch.exp(-distance)
        yaw_reward = torch.exp(-2.0 * yaw_error * yaw_error)
        effort_penalty = (self._raw_actions * self._raw_actions).sum(dim=-1)
        action_diff = self._actions - self._previous_actions
        smoothness_penalty = 2.0 * action_diff[:, 0].square() + 0.5 * action_diff[:, 1:].square().sum(dim=-1)
        ang_rate_penalty = (body_ang_vel * body_ang_vel).sum(dim=-1)

        reward = (
            5.0 * velocity_tracking
            + 3.0 * goal_reward
            + 1.0 * yaw_reward
            - 0.05 * effort_penalty
            - 0.50 * smoothness_penalty
            - 0.10 * ang_rate_penalty
        )
        near_goal = torch.logical_and(distance < 0.20, speed < 0.10)
        reward = torch.where(near_goal, reward + 5.0, reward)
        return torch.where(distance > self.cfg.crash_distance, torch.full_like(reward, -100.0), reward)
