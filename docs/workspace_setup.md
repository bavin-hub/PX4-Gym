# Workspace Setup

This guide sets up the full workspace used by PX4-Gym:

- Isaac Sim `5.1.0`
- Isaac Lab `main`
- PX4-Autopilot `v1.14.3`
- Micro XRCE-DDS Agent
- ROS 2 workspace with `px4_msgs`, `px4_ros_com`, and `policy_test`
- PX4 `dds_topics.yaml` replaced with this repository's topic map

## Repository Layout

The main folders used by the workflows are:

- `aerial_isaac/`: Isaac Lab tasks, RL-Games configs, smoke tests, and training
  scripts.
- `policy_test/`: ROS 2 package that loads trained policies and publishes PX4
  offboard commands.
- `dds_topics.yaml`: PX4 uXRCE-DDS topic configuration used for the policy-test
  bridge.
- `docs/`: setup and workflow documentation.

## Isaac Sim And Isaac Lab

Install Isaac Sim `5.1.0` from the pre-built binaries, then install Isaac Lab
from the `main` branch.  The Isaac Lab binary installation page is:

<https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/binaries_installation.html>

The Isaac Lab guide assumes the Isaac Sim directory is available as
`${HOME}/isaacsim` on Linux and recommends exporting these variables:

```bash
export ISAACSIM_PATH="${HOME}/isaacsim"
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
```

Verify Isaac Sim first:

```bash
${ISAACSIM_PATH}/isaac-sim.sh
${ISAACSIM_PYTHON_EXE} -c "print('Isaac Sim configuration is now complete.')"
```

Clone and install Isaac Lab from `main`:

```bash
git clone https://github.com/isaac-sim/IsaacLab.git --branch main
cd IsaacLab
ln -s ${ISAACSIM_PATH} _isaac_sim
./isaaclab.sh --install rl_games
```

If you use a conda or uv environment for Isaac Lab, activate it before
installing this repository's Isaac Lab package.  From this repository:

```bash
cd aerial_isaac
python -m pip install -e .
```

If you use the Isaac Sim bundled Python directly:

```bash
cd aerial_isaac
${ISAACSIM_PYTHON_EXE} -m pip install -e .
```

Run a quick environment smoke test:

```bash
cd aerial_isaac
${ISAACSIM_PYTHON_EXE} scripts/smoke_x500_velocity.py --headless
```

## PX4 And Micro XRCE-DDS Agent

Clone PX4 at the tested version:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.14.3
git submodule update --init --recursive
```

Install PX4's host dependencies using the PX4 setup script for your platform,
then build the Gazebo SITL target you use for the X500 workflow.  A common
target is:

```bash
make px4_sitl gz_x500
```

Install Micro XRCE-DDS Agent:

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

Start the agent before running the ROS 2 policy node:

```bash
MicroXRCEAgent udp4 -p 8888
```

## DDS Topic Configuration

Replace PX4's uXRCE-DDS topic map with this repository's `dds_topics.yaml`.
For PX4 `v1.14.3`, the file is normally located at:

```text
PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
```

Copy the repository file into that location, then rebuild PX4 SITL:

```bash
cp /path/to/PX4-Gym/dds_topics.yaml \
  /path/to/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml

cd /path/to/PX4-Autopilot
make px4_sitl gz_x500
```

## ROS 2 Policy-Test Workspace

Create a ROS 2 workspace and clone the PX4 ROS repositories that match the PX4
`1.14` message set:

```bash
mkdir -p ~/px4_ros_ws/src
cd ~/px4_ros_ws/src

git clone https://github.com/PX4/px4_msgs.git --branch release/1.14
git clone https://github.com/PX4/px4_ros_com.git --branch release/v1.14
```

Add this repository's `policy_test` package to the same `src` folder.  A
symlink keeps local edits in this repository visible to the ROS workspace:

```bash
ln -s /path/to/PX4-Gym/policy_test ~/px4_ros_ws/src/policy_test
```

Install the Python packages used by the policy-test nodes:

```bash
python3 -m pip install --user rl-games PyYAML
python3 -m pip install --user torch --index-url https://download.pytorch.org/whl/cu128
```

Build and source the ROS workspace:

```bash
cd ~/px4_ros_ws
colcon build --symlink-install
source install/setup.bash
```

The policy-test nodes load the policy config and checkpoint from ROS
parameters.  Pass them when starting a controller:

```bash
ros2 run policy_test position_sim_controller --ros-args \
  -p policy_config:=/absolute/path/to/rl_games_x500_position.yaml \
  -p checkpoint:=/absolute/path/to/checkpoint.pth \
  -p device:=cuda:0
```
