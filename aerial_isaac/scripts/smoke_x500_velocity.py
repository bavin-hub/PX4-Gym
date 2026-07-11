"""Launch the X500 velocity task and validate its policy contract at runtime."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Running a script directly places ``scripts/`` rather than the repository
# root on sys.path.  Keep the smoke test usable without an editable install.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

parser = argparse.ArgumentParser()
parser.add_argument("--num-envs", type=int, default=16)
parser.add_argument("--steps", type=int, default=100)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--device", default="cuda:0")
args, unknown = parser.parse_known_args()

from isaaclab.app import AppLauncher

app_launcher = AppLauncher({"headless": args.headless, "device": args.device})
simulation_app = app_launcher.app

import torch

from aerial_isaac_lab.tasks import X500VelocityEnv, X500VelocityEnvCfg

cfg = X500VelocityEnvCfg()
cfg.scene.num_envs = args.num_envs
cfg.sim.device = args.device
env = X500VelocityEnv(cfg)
obs, _ = env.reset()
assert obs["policy"].shape == (args.num_envs, 18), obs["policy"].shape
assert torch.isfinite(obs["policy"]).all(), "initial observation contains non-finite values"

actions = torch.zeros((args.num_envs, 4), device=env.device)
for _ in range(args.steps):
    obs, reward, terminated, truncated, _ = env.step(actions)
    assert obs["policy"].shape[-1] == 18
    assert torch.isfinite(obs["policy"]).all(), "observation contains non-finite values"
    assert torch.isfinite(reward).all(), "reward contains non-finite values"

print(
    f"PASS: {args.num_envs} X500 environments, {args.steps} velocity-control steps, "
    f"mean reward={reward.mean().item():.4f}, done={(terminated | truncated).sum().item()}"
)
env.close()
simulation_app.close()
