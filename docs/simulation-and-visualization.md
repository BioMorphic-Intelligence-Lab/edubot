# Simulation and visualization

Back to [Home](Home.md)

EduBot can run entirely in software: a simple kinematic simulator publishes joint
states, `robot_state_publisher` turns them into TF frames using the URDF, and
RViz renders the arm. This lets you develop and test controllers without
hardware. For how to start these, see
[Building and running](building-and-running.md).

## Launch files

| Launch file | Starts | Notes |
|-------------|--------|-------|
| [`sim_position.launch.py`](../ros_ws/src/lerobot/launch/sim_position.launch.py) | `lerobot_sim`, `robot_state_publisher`, `rviz2` | Position mode; loads `lerobot_pos_sim.yaml` |
| [`sim_velocity.launch.py`](../ros_ws/src/lerobot/launch/sim_velocity.launch.py) | same stack | Velocity mode; loads `lerobot_vel_sim.yaml` |
| [`rviz.launch.py`](../ros_ws/src/lerobot/launch/rviz.launch.py) | `robot_state_publisher`, `rviz2` only | No robot node; needs another node to publish joint states |
| [`joint_slider.launch.py`](../ros_ws/src/lerobot/launch/joint_slider.launch.py) | `joint_state_publisher_gui`, `robot_state_publisher` | Manual joint sliders; no RViz |

Common launch arguments for the sim launches:

| Arg | Default | Purpose |
|-----|---------|---------|
| `urdf_model` | `share/lerobot/urdf/lerobot.urdf` | URDF path |
| `rviz_config_file` | `share/lerobot/rviz/rviz_basic_settings.rviz` | RViz config |
| `use_robot_state_pub` | `True` | Toggle `robot_state_publisher` |
| `use_rviz` | `True` | Toggle RViz |
| `use_sim_time` | `True` | Use the simulation clock |

> The `path_publisher` node (end-effector trail) is commented out in the sim
> launch files. See [LeRobot nodes](lerobot-nodes.md#path_publisher---end-effector-trail).

## The `robot_sim` backend

The simulation logic lives in the reusable
[`robot_sim`](../ros_ws/src/robot_sim) library
([`src/robot_sim.cpp`](../ros_ws/src/robot_sim/src/robot_sim.cpp),
[`include/robot_sim/robot_sim.hpp`](../ros_ws/src/robot_sim/include/robot_sim/robot_sim.hpp)).
`RobotSim` is a subclass of [`robot_core::Robot`](ros-interface.md) that adds a
minimal kinematic model:

- **Position mode:** commanded positions are applied directly (instant).
- **Velocity mode:** the simulator integrates `qdot` at its loop rate (default
  25 Hz); the gripper opening is integrated the same way.

[`lerobot_sim`](lerobot-nodes.md#lerobot_sim---simulation-node) is the
LeRobot-specific entry point that links this library, sets the joint names, and
homes the arm.

## URDF model

The model is in [`urdf/lerobot.urdf`](../ros_ws/src/lerobot/urdf/lerobot.urdf)
(robot name `so_arm100`). The kinematic chain:

```
world --[fixed, pi yaw]-- base
  +- Shoulder_Rotation (revolute) -- shoulder
       +- Shoulder_Pitch (revolute) -- upper_arm
            +- Elbow (revolute) -- lower_arm
                 +- Wrist_Pitch (revolute) -- wrist
                      +- Wrist_Roll (revolute) -- gripper
                           +- [fixed] -- gripper_center   (EE frame, tracked by path_publisher)
                           +- Gripper (revolute) -- jaw
```

- **6 actuated joints**, matching the order used everywhere else
  (`Shoulder_Rotation`, `Shoulder_Pitch`, `Elbow`, `Wrist_Pitch`, `Wrist_Roll`,
  `Gripper`). See [ROS interface](ros-interface.md#joint-order).
- **Links:** `world`, `base`, `shoulder`, `upper_arm`, `lower_arm`, `wrist`,
  `gripper`, `gripper_center`, `jaw`.
- **Meshes:** referenced as `package://lerobot/meshes/*.stl` and installed from
  [`meshes/`](../ros_ws/src/lerobot/meshes).
- Approximate joint limits (rad): shoulder rotation +/-2; pitch / elbow /
  wrist pitch +/-1.57; wrist roll +/-pi; gripper -0.2 to 2.0.

## Exporting a posed mesh: `urdf_to_step.py`

[`scripts/urdf_to_step.py`](../ros_ws/src/lerobot/scripts/urdf_to_step.py) walks
the URDF kinematic tree at a given joint configuration, loads each visual mesh,
applies the link + visual transforms, and writes a single combined mesh. Despite
the name, it currently exports an **STL** file (the optional STEP libraries are
probed but not used).

Requirements: `trimesh` and `numpy`.

Usage:

```bash
# Default home pose -> lerobot_home.stl
python scripts/urdf_to_step.py urdf/lerobot.urdf lerobot_home.stl

# Custom configuration
python scripts/urdf_to_step.py urdf/lerobot.urdf out.stl --home-config Shoulder_Pitch=1.8326 ...
```

The default pose matches the sim home (0 deg, 105 deg, -70 deg, -60 deg, 0 deg,
gripper closed). Full details are in
[`scripts/README_urdf_to_step.md`](../ros_ws/src/lerobot/scripts/README_urdf_to_step.md).
