# Building and running

Back to [Home](Home.md)

This page assumes you have already installed the prerequisites and cloned the
repository (see [Installation](installation.md)).

The repository contains two top-level folders:

- [`ros_ws`](../ros_ws) - the ROS 2 workspace with the driver, visualization,
  simulation and example controllers (C++ and Python).
- [`testing`](../testing) - plain, non-ROS smoke-test scripts (see
  [Writing controllers](writing-controllers.md#testing-folder)).

## Compilation

Source your ROS installation and build the workspace with `colcon`:

```bash
cd edubot/ros_ws

# For ROS 2 Humble users
source /opt/ros/humble/setup.bash

# For ROS 2 Jazzy users
source /opt/ros/jazzy/setup.bash

# Call build command
colcon build
```

## Sourcing the workspace

After a successful build, source the local workspace so ROS can find the EduBot
packages. Do this in every new terminal (after sourcing your ROS installation):

```bash
source install/setup.bash
```

## Running

Use the launch files below to start the simulation, the visualization, or the
real hardware driver. See [LeRobot nodes](lerobot-nodes.md) and
[Simulation and visualization](simulation-and-visualization.md) for what each one
does internally, and [Configuration](configuration.md) for the parameters they
load.

### Launch files (lerobot package)

| Command | Effect |
|---------|--------|
| `ros2 launch lerobot sim_position.launch.py` | Simulation and RViz in position control mode |
| `ros2 launch lerobot sim_velocity.launch.py` | Simulation and RViz in velocity control mode |
| `ros2 launch lerobot rviz.launch.py` | RViz only. **Launch something else** (sim, hw, or joint_slider) in another terminal to see the robot at a real pose. |
| `ros2 launch lerobot joint_slider.launch.py` | Joint position slider GUI (no RViz) |
| `ros2 launch lerobot hw_position.launch.py` | Hardware interface in position control mode |
| `ros2 launch lerobot hw_velocity.launch.py` | Hardware interface in velocity control mode |
| `ros2 launch lerobot hw_read.launch.py` | Passive read-only: publish joint states, torque disabled (move robot by hand) |

When you launch **RViz only** (`rviz.launch.py`), the robot is shown in the
default pose (all joints 0) until another node publishes joint states. To see the
robot at a real position, launch one of the following in a **second terminal**:
simulation (`sim_position` or `sim_velocity`), hardware (`hw_position`,
`hw_velocity`, or `hw_read`), or the joint slider (`joint_slider.launch.py`).

### Running controllers directly (without a launch file)

These publish commands on `joint_cmds`, so start a robot or simulation node first
(in position or velocity mode to match the controller). See
[Writing controllers](writing-controllers.md) for details.

| Command | Effect |
|---------|--------|
| `ros2 run controllers example_pos_traj` | C++ example position mode trajectory controller |
| `ros2 run controllers example_vel_traj` | C++ example velocity mode trajectory controller |
| `ros2 run python_controllers example_pos_traj` | Python example trajectory controller |
| `ros2 run python_controllers example_vel_traj` | Python velocity trajectory controller |

## Updating

The packages are continually being updated, so update every once in a while. Be
sure to update the submodule as well as the main repository:

```bash
git submodule update --recursive
git pull
```

If you maintain your own fork, see [Fork workflow](fork-workflow.md) for how to
sync with upstream without losing your work.
