# Configuration

Back to [Home](Home.md)

All runtime parameters live in YAML files under
[`ros_ws/src/lerobot/config`](../ros_ws/src/lerobot/config) (for the robot/sim
nodes) and [`ros_ws/src/controllers/config`](../ros_ws/src/controllers/config)
(for the example controller). Each launch file loads the matching file; see
[Building and running](building-and-running.md) and
[LeRobot nodes](lerobot-nodes.md).

ROS 2 parameters are namespaced under the node name, then `ros__parameters`.

> **Rebuild after editing config.** The config files are installed into the
> package share folder at build time, so run `colcon build` again for changes to
> take effect.

## `robot_hw.yaml`

Loaded by [`lerobot_hw`](lerobot-nodes.md#lerobot_hw---real-hardware-driver) (node
name `lerobot`) via `hw_position.launch.py` / `hw_velocity.launch.py`. The launch
file additionally passes `mode: position` or `mode: velocity` inline.

File: [`config/robot_hw.yaml`](../ros_ws/src/lerobot/config/robot_hw.yaml)

| Key | Example value | Meaning |
|-----|---------------|---------|
| `serial_port` | `"/dev/ttyUSB0"` | USB serial device for the servo bus |
| `baud_rate` | `1000000` | Bus baud rate (must match the servo EEPROM setting) |
| `frequency` | `100.0` | Feetech driver control-loop rate (Hz) |
| `gripper_open` | `1.57079632679` | Gripper open angle (rad, here pi/2) |
| `gripper_closed` | `0.0` | Gripper closed angle (rad) |
| `max_speed` | `2.0` | Per-servo max speed clamp (rad/s) |
| `zero_positions` | `[2048, 2048, 2048, 2048, 2048, 2048]` | Raw encoder ticks used as the zero reference per servo (passed to `setHomePosition`) |
| `ids` | `[11, 12, 13, 14, 15, 16]` | Feetech servo IDs (5 arm joints + gripper) |
| `joint_signs` | `[-1.0, -1.0, -1.0, -1.0, -1.0, 1.0]` | Per-joint sign (+1/-1) so the hardware matches the URDF/sim convention. Order: Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Gripper |
| `home_position` | `[0.0, 1.5, -1.0, -0.5, 0.0]` | Logical home pose, joint angles in radians for the 5 arm joints |

It also inherits the [base `Robot` parameters](ros-interface.md#base-parameters)
(`f`, `pub_topic`, `sub_topic`, `mode`).

## `robot_read.yaml`

Loaded by [`lerobot_read`](lerobot-nodes.md#lerobot_read---passive-read-only-node)
via `hw_read.launch.py`.

File: [`config/robot_read.yaml`](../ros_ws/src/lerobot/config/robot_read.yaml)

| Key | Example value | Meaning |
|-----|---------------|---------|
| `serial_port` | `"/dev/ttyUSB0"` | USB serial device |
| `baud_rate` | `1000000` | Bus baud rate |
| `frequency` | `25.0` | Read + publish rate (Hz) |
| `ids` | `[11, 12, 13, 14, 15, 16]` | Servo IDs |
| `zero_positions` | `[2048, 2048, 2048, 2048, 2048, 2048]` | Encoder zero ticks |
| `joint_signs` | `[-1.0, -1.0, -1.0, -1.0, -1.0, 1.0]` | Per-joint sign to match URDF/sim |
| `publish_rate` | `50.0` | **Unused** by the node (it uses `frequency`) |
| `logging` | `false` | **Unused** by the node |

> Note: `publish_rate` and `logging` are present in the file but the node does not
> read them; the active rate is `frequency`.

## `lerobot_pos_sim.yaml` and `lerobot_vel_sim.yaml`

Loaded by [`lerobot_sim`](lerobot-nodes.md#lerobot_sim---simulation-node) via
`sim_position.launch.py` / `sim_velocity.launch.py`.

Files:
[`config/lerobot_pos_sim.yaml`](../ros_ws/src/lerobot/config/lerobot_pos_sim.yaml),
[`config/lerobot_vel_sim.yaml`](../ros_ws/src/lerobot/config/lerobot_vel_sim.yaml)

```yaml
lerobot_sim:
  ros__parameters:
    mode: "position"   # or "velocity" in lerobot_vel_sim.yaml
    home_position: [0.0, 1.8326, -1.2217, -1.0472, 0.0]
```

| Key | Value | Meaning |
|-----|-------|---------|
| `mode` | `"position"` / `"velocity"` | Control mode for the sim node |
| `home_position` | `[0.0, 1.8326, -1.2217, -1.0472, 0.0]` | Home pose (rad) for the 5 arm joints |

The other base parameters (`f`, `pub_topic`, `sub_topic`) use the code defaults
(`100`, `joint_states`, `joint_cmds`).

## `lerobot_params.yaml` (example controller)

Loaded by the example C++ controller launch file.

File: [`config/lerobot_params.yaml`](../ros_ws/src/controllers/config/lerobot_params.yaml)

```yaml
example_traj_lerobot:
  ros__parameters:
    # Home in rad: 0, 105 deg, -70 deg, -60 deg, 0 (matches URDF/sim)
    home: [0.0, 1.8326, -1.2217, -1.0472, 0.0]
```

| Key | Value | Meaning |
|-----|-------|---------|
| `home` | `[0.0, 1.8326, -1.2217, -1.0472, 0.0]` | Base pose (rad) the example controller oscillates around |

> Note: this file's namespace is `example_traj_lerobot`, but the example nodes are
> run directly with `ros2 run controllers example_pos_traj` / `example_vel_traj`,
> which use their own node names. See
> [Writing controllers](writing-controllers.md#caveats) for the stale launch-file
> details.

## Customizing the home position

The robot has a configurable **HOME** position that it moves to after start-up.
Each servo also has a configurable **zero position** that the driver uses as the
zero reference in radians. Both are set through the YAML files above:

- `zero_positions`: raw tick values passed to `FeetechServo::setHomePosition` for
  each servo ID, e.g. `zero_positions: [1950, 1950, 1950, 2048, 2048, 2048]`.
- `home_position`: joint angles in radians defining the logical home pose, e.g.
  `home_position: [0.0, 1.8326, -1.2217, -1.0472, 0.0]`.

For the hardware, edit these arrays in
[`robot_hw.yaml`](../ros_ws/src/lerobot/config/robot_hw.yaml) (and
[`robot_read.yaml`](../ros_ws/src/lerobot/config/robot_read.yaml) for the
read-only node); for the simulation, edit
[`lerobot_pos_sim.yaml`](../ros_ws/src/lerobot/config/lerobot_pos_sim.yaml) /
[`lerobot_vel_sim.yaml`](../ros_ws/src/lerobot/config/lerobot_vel_sim.yaml). Then
`colcon build` and re-launch.

## Wrong baud rate or USB port

If the robot does not respond, the `serial_port` and `baud_rate` above are the
first things to check. See
[Troubleshooting](troubleshooting.md#wrong-baud-rate-or-usb-port).
