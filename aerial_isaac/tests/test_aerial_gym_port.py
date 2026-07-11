"""Pure-Torch checks for the simulator-independent Aerial Gym port."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import torch

from aerial_isaac_lab.core.attitude_control import LeeAttitudeYawController
from aerial_isaac_lab.core.math import (
    euler_xyz_from_quat,
    quat_apply,
    quat_from_euler_xyz,
    quat_wxyz_to_xyzw,
    wrap_to_pi,
)
from aerial_isaac_lab.core.motor import MultirotorAllocator
from aerial_isaac_lab.core.position_control import LeePositionController
from aerial_isaac_lab.core.velocity_control import LeeVelocityController
from aerial_isaac_lab.models import X500_MODEL


def test_quaternion_boundary_round_trip_and_rotation():
    yaw = torch.tensor([math.pi / 2.0])
    quat = quat_from_euler_xyz(torch.zeros_like(yaw), torch.zeros_like(yaw), yaw)
    assert torch.allclose(
        quat_wxyz_to_xyzw(quat), torch.tensor([[0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]])
    )
    rotated = quat_apply(quat, torch.tensor([[1.0, 0.0, 0.0]]))
    assert torch.allclose(rotated, torch.tensor([[0.0, 1.0, 0.0]]), atol=1.0e-6)
    _, _, recovered_yaw = euler_xyz_from_quat(quat)
    assert torch.allclose(recovered_yaw, yaw)


def test_x500_allocator_preserves_hover_thrust_and_motor_order():
    allocator = MultirotorAllocator(X500_MODEL, num_envs=2, dt=0.01, device=torch.device("cpu"))
    # Prime the latent motor state to make this a deterministic allocation check.
    allocator.motor_dynamics.current_thrust.zero_()
    wrench = torch.zeros((2, 6))
    wrench[:, 2] = 4.0
    forces, torques = allocator.allocate_wrench(wrench)
    assert forces.shape == (2, 4, 3)
    assert torques.shape == (2, 4, 3)
    assert torch.all(forces[..., 2] > 0.0)
    expected_torque_sign = -torch.tensor(X500_MODEL.motor_directions)
    assert torch.equal(torch.sign(torques[0, :, 2]), expected_torque_sign)


def test_x500_asset_is_self_contained_in_this_workspace():
    assert Path(X500_MODEL.urdf_path).is_file()


def test_wrapped_yaw_error_range():
    result = wrap_to_pi(torch.tensor([3.0 * math.pi, -3.0 * math.pi, 0.25]))
    assert torch.all(result <= math.pi)
    assert torch.all(result >= -math.pi)


def test_lee_velocity_controller_produces_finite_hover_wrench():
    controller = LeeVelocityController(num_envs=2, device=torch.device("cpu"))
    command = torch.zeros((2, 4))
    position = torch.zeros((2, 3))
    orientation = torch.tensor([[1.0, 0.0, 0.0, 0.0]]).repeat(2, 1)
    velocity = torch.zeros((2, 3))
    inertia = torch.eye(3).repeat(2, 1, 1)
    wrench = controller.compute(command, position, orientation, velocity, velocity, torch.ones(2), inertia)
    assert torch.isfinite(wrench).all()
    assert torch.all(wrench[:, 2] > 0.0)


def test_lee_position_controller_produces_finite_hover_wrench_at_setpoint():
    controller = LeePositionController(num_envs=2, device=torch.device("cpu"))
    command = torch.zeros((2, 4))
    position = torch.zeros((2, 3))
    orientation = torch.tensor([[1.0, 0.0, 0.0, 0.0]]).repeat(2, 1)
    velocity = torch.zeros((2, 3))
    inertia = torch.eye(3).repeat(2, 1, 1)
    wrench = controller.compute(command, position, orientation, velocity, velocity, torch.ones(2), inertia)
    assert torch.isfinite(wrench).all()
    assert torch.all(wrench[:, 2] > 0.0)


def test_lee_attitude_yaw_controller_hover_contract():
    controller = LeeAttitudeYawController(num_envs=2, device=torch.device("cpu"))
    command = torch.zeros((2, 4))
    orientation = torch.tensor([[1.0, 0.0, 0.0, 0.0]]).repeat(2, 1)
    velocity = torch.zeros((2, 3))
    inertia = torch.eye(3).repeat(2, 1, 1)
    mass = torch.full((2,), 2.0)
    wrench = controller.compute(command, orientation, velocity, mass, inertia)
    assert torch.isfinite(wrench).all()
    assert torch.allclose(wrench[:, 2], mass * 9.81)


def test_x500_urdf_uses_attitude_repo_inertials():
    root = ET.parse(X500_MODEL.urdf_path).getroot()
    inertials = {
        link.attrib["name"]: link.find("inertial")
        for link in root.findall("link")
    }
    base_inertia = inertials["base_link"].find("inertia").attrib
    prop_inertia = inertials["front_right_prop"].find("inertia").attrib
    assert float(inertials["base_link"].find("mass").attrib["value"]) == 2.0
    assert math.isclose(float(base_inertia["ixx"]), 0.02166666666666667)
    assert math.isclose(float(base_inertia["izz"]), 0.04000000000000001)
    assert math.isclose(float(inertials["front_right_prop"].find("mass").attrib["value"]), 0.016076923076923075)
    assert math.isclose(float(prop_inertia["iyy"]), 2.6115851691700804e-05)
