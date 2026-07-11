"""X500 values ported from Aerial Gym's ``X500Cfg`` and Lee controller config."""

from __future__ import annotations

from pathlib import Path

from .model_cfg import MultirotorModelCfg


_PROJECT_ROOT = Path(__file__).resolve().parents[2]
X500_URDF_PATH = str(_PROJECT_ROOT / "assets/robots/x500/model.urdf")

X500_MODEL = MultirotorModelCfg(
    urdf_path=X500_URDF_PATH,
    root_body_name="base_link",
    # This is Aerial Gym's application_mask order: FR, BL, FL, BR.
    motor_body_names=("front_right_prop", "back_left_prop", "front_left_prop", "back_right_prop"),
    motor_directions=(1.0, 1.0, -1.0, -1.0),
    allocation_matrix=(
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0),
        (1.0, 1.0, 1.0, 1.0),
        (-0.13, 0.13, 0.13, -0.13),
        (-0.13, 0.13, -0.13, 0.13),
        (-0.025, 0.025, -0.025, 0.025),
    ),
    thrust_constant=8.54858e-6,
    motor_tau_increasing=0.0125,
    motor_tau_decreasing=0.025,
    min_thrust=0.0,
    max_thrust=20.0,
    max_thrust_rate=100000.0,
    thrust_to_torque_ratio=0.025,
    rigid_linear_damping=0.02,
    rigid_angular_damping=0.02,
    aerodynamic_linear_damping=(0.0, 0.0, 0.0),
    aerodynamic_quadratic_damping=(0.0, 0.0, 0.0),
    aerodynamic_angular_linear_damping=(0.0, 0.0, 0.0),
    aerodynamic_angular_quadratic_damping=(0.0, 0.0, 0.0),
)
