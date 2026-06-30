# EduBot: The library for manipulators for Education

EduBot contains the drivers, visualization features and a simple simulation for
[LeRobot](https://github.com/huggingface/lerobot) (and, for legacy reasons, the
[Lynxmotion AL5A Arm](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/servo-erector-set-robots-kits/ses-v1-robots/ses-v1-arms/al5a/)
in its designated branch), all implemented on top of the
[ROS 2](https://docs.ros.org/en/humble/index.html) middleware.

## Documentation

Full documentation lives in the [docs wiki](docs/Home.md). Start there for an
overview, or jump straight to a topic:

- [Installation](docs/installation.md) - prerequisites, ROS 2 Humble/Jazzy, cloning (with submodules), USB access.
- [Building and running](docs/building-and-running.md) - `colcon build`, sourcing, launch files, and runnable controllers.
- [Architecture](docs/architecture.md) - the packages and how data flows between nodes.
- [Feetech servo driver](docs/feetech-driver.md) - the low-level driver, serial protocol, and control loop.
- [ROS interface](docs/ros-interface.md) - the `joint_cmds` / `joint_states` / `set_mode` contract.
- [LeRobot nodes](docs/lerobot-nodes.md) - the hardware, read-only, simulation, and path-publisher nodes.
- [Configuration](docs/configuration.md) - every YAML parameter and how to set the home position.
- [Simulation and visualization](docs/simulation-and-visualization.md) - the simulator, RViz, URDF, and mesh export.
- [Writing controllers](docs/writing-controllers.md) - C++ and Python controller patterns.
- [Fork workflow](docs/fork-workflow.md) - fork the repo, do assignments, and sync with upstream.
- [Troubleshooting](docs/troubleshooting.md) - common problems and fixes.

## Quickstart

```bash
# 1. Clone recursively (feetech_cpp_lib is a git submodule)
git clone --recurse-submodules https://github.com/bioMorphic-Intelligence-Lab/edubot
cd edubot/ros_ws

# 2. Source ROS 2 (humble or jazzy) and build
source /opt/ros/humble/setup.bash   # or: source /opt/ros/jazzy/setup.bash
colcon build

# 3. Source the workspace
source install/setup.bash

# 4. Launch the simulation + RViz
ros2 launch lerobot sim_position.launch.py
```

See [Installation](docs/installation.md) and
[Building and running](docs/building-and-running.md) for the full instructions,
including hardware setup and the `dialout` group.

## Issues

If any bugs or issues with this software occur, please use the
[Issues](https://github.com/BioMorphic-Intelligence-Lab/edubot/issues) pane within
this repository. Please write a short description of the issue and some steps to
reproduce it.

Good Luck!
