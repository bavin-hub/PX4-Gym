# Isaac Lab Training And Evaluation

PX4-Gym trains and evaluates the Isaac Lab policies from `aerial_isaac/`.
These scripts use RL-Games and the Aerial Gym controller/task semantics ported
to Isaac Lab.

Tested stack:

- Isaac Sim `5.1.0`
- Isaac Lab `main`
- Python environment compatible with Isaac Sim `5.x`
- RL-Games installed through Isaac Lab or the active Python environment

## Tasks

The current Isaac Lab package registers:

- `AerialIsaac-X500-Position-v0`
- `AerialIsaac-X500-Velocity-v0`
- `AerialIsaac-X500-AttitudeDelta-v0`

The policy observation contract for these tasks is:

```text
[position_error_w(3), orientation_xyzw(4), body_linear_velocity_flu(3),
 body_angular_velocity_flu(3), previous_action(4), yaw_error_over_pi(1)]
```

This is an 18-dimensional observation.  Isaac Lab keeps quaternions in `wxyz`
inside the simulation boundary and converts to `xyzw` only for the policy
observation.

## Smoke Tests

Run these from `aerial_isaac/` after installing the package into the Isaac Lab
environment:

```bash
${ISAACSIM_PYTHON_EXE} scripts/smoke_x500_position.py --headless
${ISAACSIM_PYTHON_EXE} scripts/smoke_x500_velocity.py --headless
${ISAACSIM_PYTHON_EXE} scripts/smoke_x500_attitude_delta.py --headless
```

If you use an Isaac Lab virtual environment through Isaac Sim's launcher, set
`PYTHONEXE` first:

```bash
PYTHONEXE=/path/to/IsaacLab/env_isaaclab/bin/python \
  /path/to/isaacsim/python.sh scripts/smoke_x500_velocity.py --headless
```

## Training

Run training from `aerial_isaac/`.

Position policy:

```bash
${ISAACSIM_PYTHON_EXE} scripts/train_x500_position.py \
  --headless --device cuda:0 --num-envs 4096
```

Velocity policy:

```bash
${ISAACSIM_PYTHON_EXE} scripts/train_x500_velocity.py \
  --headless --device cuda:0 --num-envs 4096
```

Attitude-delta policy:

```bash
${ISAACSIM_PYTHON_EXE} scripts/train_x500_attitude_delta.py \
  --headless --device cuda:0 --num-envs 4096
```

For a short debug run, pass `--max-epochs` and lower `--num-envs`:

```bash
${ISAACSIM_PYTHON_EXE} scripts/train_x500_velocity.py \
  --headless --device cuda:0 --num-envs 16 --max-epochs 5
```

## Evaluation

Use the same training script with `--play`, one environment, and an absolute
checkpoint path.

Position policy:

```bash
${ISAACSIM_PYTHON_EXE} scripts/train_x500_position.py \
  --play --device cuda:0 --num-envs 1 \
  --checkpoint /absolute/path/to/checkpoint.pth
```

Velocity policy:

```bash
${ISAACSIM_PYTHON_EXE} scripts/train_x500_velocity.py \
  --play --device cuda:0 --num-envs 1 \
  --checkpoint /absolute/path/to/checkpoint.pth
```

Attitude-delta policy:

```bash
${ISAACSIM_PYTHON_EXE} scripts/train_x500_attitude_delta.py \
  --play --device cuda:0 --num-envs 1 \
  --checkpoint /absolute/path/to/checkpoint.pth
```

## RL-Games Configs

The task configs live in `aerial_isaac/config/`:

- `rl_games_x500_position.yaml`
- `rl_games_x500_velocity.yaml`
- `rl_games_x500_attitude_delta.yaml`

Each training script accepts `--config /path/to/config.yaml` if you want to
run a modified config without replacing the checked-in defaults.

## Controller Attribution

The low-level geometric controller is ported from
[ntnu-arl/aerial_gym_simulator](https://github.com/ntnu-arl/aerial_gym_simulator).
This repository ports the controller/task behavior from the Gym workflow to
Isaac Lab while keeping the deployment-facing policy contract stable.
