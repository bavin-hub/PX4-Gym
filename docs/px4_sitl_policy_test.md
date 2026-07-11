# PX4 SITL Policy Testing

This guide runs a trained policy with PX4 `v1.14.3`, Gazebo SITL, Micro
XRCE-DDS Agent, and the ROS 2 `policy_test` package.

Use this after completing [Workspace Setup](workspace_setup.md).

## Before Running

Confirm the following:

- PX4-Autopilot is checked out at `v1.14.3`.
- `px4_msgs` is on `release/1.14`.
- `px4_ros_com` is on `release/v1.14`.
- PX4's `src/modules/uxrce_dds_client/dds_topics.yaml` has been replaced with
  this repository's `dds_topics.yaml`.
- The ROS 2 workspace builds successfully with `policy_test`.
- You have the RL-Games policy config and checkpoint paths for the selected
  controller.

## Terminal 1: Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

Leave this running.  PX4 and ROS 2 exchange `/fmu/in/*` and `/fmu/out/*`
topics through this agent.

## Terminal 2: PX4 Gazebo SITL

From the PX4-Autopilot checkout:

```bash
make px4_sitl gz_x500
```

Wait for PX4 and Gazebo to finish starting.  The vehicle should be ready before
starting a policy controller.

## Terminal 3: ROS 2 Policy Node

Source the ROS workspace:

```bash
cd ~/px4_ros_ws
source install/setup.bash
```

Run one controller at a time.  Each policy controller requires:

- `policy_config`: absolute path to the RL-Games YAML config.
- `checkpoint`: absolute path to the trained `.pth` checkpoint.
- `device`: inference device, defaulting to `cuda:0`.

Position policy:

```bash
ros2 run policy_test position_sim_controller --ros-args \
  -p policy_config:=/absolute/path/to/rl_games_x500_position.yaml \
  -p checkpoint:=/absolute/path/to/checkpoint.pth \
  -p device:=cuda:0
```

Velocity policy:

```bash
ros2 run policy_test velocity_sim_controller --ros-args \
  -p policy_config:=/absolute/path/to/rl_games_x500_velocity.yaml \
  -p checkpoint:=/absolute/path/to/checkpoint.pth \
  -p device:=cuda:0
```

Attitude-delta policy:

```bash
ros2 run policy_test delta_attitude_sp --ros-args \
  -p policy_config:=/absolute/path/to/rl_games_x500_attitude_delta.yaml \
  -p checkpoint:=/absolute/path/to/checkpoint.pth \
  -p device:=cuda:0
```

Actuator/motor policy:

```bash
ros2 run policy_test actuator_sim_controller --ros-args \
  -p policy_config:=/absolute/path/to/actuator_policy.yaml \
  -p checkpoint:=/absolute/path/to/checkpoint.pth \
  -p device:=cuda:0
```

Basic offboard takeoff/land smoke test:

```bash
ros2 run policy_test offb_control
```

## Topic Checks

Use these commands to confirm that PX4 and ROS 2 are connected:

```bash
ros2 topic list | grep /fmu
ros2 topic echo /fmu/out/vehicle_odometry
ros2 topic echo /fmu/out/vehicle_status
```

Expected policy inputs include PX4 odometry, vehicle status, global position,
and, for the attitude-delta controller, hover-thrust estimates.  Expected
outputs depend on the controller:

- Position and velocity policies publish `/fmu/in/trajectory_setpoint`.
- Attitude-delta policy publishes `/fmu/in/vehicle_attitude_setpoint`.
- Actuator policy publishes `/fmu/in/actuator_motors`.
- All policy nodes publish `/fmu/in/offboard_control_mode` and
  `/fmu/in/vehicle_command`.

## Notes

The ROS controllers convert PX4 NED/FRD state into the Isaac policy's NWU/FLU
observation frame before inference.  Keep the trained policy config,
checkpoint, observation size, and controller entry point matched; mixing a
position policy with a velocity or attitude controller will produce invalid
actions.

The checked-in `dds_topics.yaml` includes the PX4 topics needed by these
controllers.  Rebuild PX4 after replacing the file so the SITL client uses the
updated publication/subscription set.
