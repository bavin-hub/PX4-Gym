"""Isaac Lab task registrations.

Import this module only after :class:`isaaclab.app.AppLauncher` has started the
simulation application.
"""

import gymnasium as gym

from .x500_attitude_delta_env import X500AttitudeDeltaEnv, X500AttitudeDeltaEnvCfg
from .x500_position_env import X500PositionEnv, X500PositionEnvCfg
from .x500_velocity_env import X500VelocityEnv, X500VelocityEnvCfg

X500_VELOCITY_TASK_ID = "AerialIsaac-X500-Velocity-v0"
X500_POSITION_TASK_ID = "AerialIsaac-X500-Position-v0"
X500_ATTITUDE_DELTA_TASK_ID = "AerialIsaac-X500-AttitudeDelta-v0"

# Backward-compatible name used by the existing velocity training script.
TASK_ID = X500_VELOCITY_TASK_ID

if X500_VELOCITY_TASK_ID not in gym.registry:
    gym.register(id=X500_VELOCITY_TASK_ID, entry_point="aerial_isaac_lab.tasks:X500VelocityEnv")

if X500_POSITION_TASK_ID not in gym.registry:
    gym.register(id=X500_POSITION_TASK_ID, entry_point="aerial_isaac_lab.tasks:X500PositionEnv")

if X500_ATTITUDE_DELTA_TASK_ID not in gym.registry:
    gym.register(id=X500_ATTITUDE_DELTA_TASK_ID, entry_point="aerial_isaac_lab.tasks:X500AttitudeDeltaEnv")

__all__ = [
    "TASK_ID",
    "X500_ATTITUDE_DELTA_TASK_ID",
    "X500_POSITION_TASK_ID",
    "X500_VELOCITY_TASK_ID",
    "X500AttitudeDeltaEnv",
    "X500AttitudeDeltaEnvCfg",
    "X500PositionEnv",
    "X500PositionEnvCfg",
    "X500VelocityEnv",
    "X500VelocityEnvCfg",
]
