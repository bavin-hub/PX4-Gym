"""Smoke-test the X500 position-setpoint Isaac Lab environment."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

parser = argparse.ArgumentParser()
parser.add_argument("--num-envs", type=int, default=4)
parser.add_argument("--steps", type=int, default=16)
parser.add_argument("--device", default="cuda:0")
parser.add_argument("--headless", action="store_true")
args, _ = parser.parse_known_args()

from isaaclab.app import AppLauncher

simulation_app = AppLauncher({"headless": args.headless, "device": args.device}).app

import gymnasium as gym
import torch

from aerial_isaac_lab.tasks import X500_POSITION_TASK_ID, X500PositionEnvCfg

env_cfg = X500PositionEnvCfg()
env_cfg.scene.num_envs = args.num_envs
env_cfg.sim.device = args.device
env = gym.make(X500_POSITION_TASK_ID, cfg=env_cfg)

obs, _ = env.reset()
print("obs", obs["policy"].shape)
for step in range(args.steps):
    actions = torch.empty((args.num_envs, 4), device=args.device).uniform_(-1.0, 1.0)
    obs, rewards, terminated, truncated, _ = env.step(actions)
    if step == args.steps - 1:
        print(
            "final",
            obs["policy"].shape,
            float(rewards.mean()),
            int(terminated.sum()),
            int(truncated.sum()),
        )

env.close()
simulation_app.close()
