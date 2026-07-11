# Aerial Isaac Lab

This repository ports Aerial Gym task logic to Isaac Lab without replacing the
drone dynamics with Isaac Lab controller or actuator abstractions.  It includes
separate X500 velocity-setpoint, position-setpoint, and attitude-delta tasks.

The policy contract is deliberately stable and deployment-facing:

`[position_error_w(3), orientation_xyzw(4), body_linear_velocity_flu(3),`
` body_angular_velocity_flu(3), previous_action(4), yaw_error_over_pi(1)]`

It is an **18-dimensional** observation.  Isaac Lab data remains `wxyz` inside
the simulation boundary; conversion to `xyzw` happens only when producing the
policy observation.

Run a headless smoke test with the Isaac Lab Python environment:

```bash
PYTHONEXE=/home/bavin/airlab/IsaacLab/aerial_isaac/bin/python \
  /home/bavin/airlab/isaacsim/python.sh scripts/smoke_x500_velocity.py --headless
```

The X500 position-setpoint task keeps the same 18D observation and Aerial Gym
sim2real reward/target sampling, but the policy action is interpreted as
env-local `[x, y, z, yaw]`.  The first three raw action channels are clipped to
`[-1, 1]` and scaled to the 5 m Aerial Gym env box before reaching the
position controller; yaw is scaled to `[-pi, pi]`.

```bash
PYTHONEXE=/home/bavin/airlab/IsaacLab/aerial_isaac/bin/python \
  /home/bavin/airlab/isaacsim/python.sh scripts/smoke_x500_position.py --headless
```

The X500 attitude-delta task ports
`position_setpoint_task_attitude_delta_x500` from the attitude Aerial Gym repo.
The shared X500 URDF inertial values are overwritten with that repo's
`model_px4_inertia.urdf` values. Its raw action is
`[delta_thrust, delta_roll, delta_pitch, delta_yaw]` in `[-1, 1]`, converted to
PX4-hover-relative thrust and current-attitude-relative 5 degree deltas.

```bash
PYTHONEXE=/home/bavin/airlab/IsaacLab/aerial_isaac/bin/python \
  /home/bavin/airlab/isaacsim/python.sh scripts/smoke_x500_attitude_delta.py --headless
```
