# URDF to STL Converter

This script converts a URDF robot model to a single STL file with the robot in home configuration.

## Requirements

**Required:**
```bash
pip install trimesh numpy
```

## Usage

Basic usage:
```bash
cd /home/antbre/projects/edubot/ros_ws/src/lerobot
python scripts/urdf_to_step.py urdf/lerobot.urdf lerobot_home.stl
```

Or specify custom joint positions:
```bash
python scripts/urdf_to_step.py urdf/lerobot.urdf lerobot_home.stl \
    --home-config Shoulder_Rotation=0.0 Shoulder_Pitch=1.8326 Elbow=-1.2217 \
    Wrist_Pitch=-1.0472 Wrist_Roll=0.0 Gripper=0.0
```

## Default Home Configuration

The script uses the following default home configuration (from lerobot_sim.cpp):
- Shoulder_Rotation: 0.0 rad
- Shoulder_Pitch: 105° (1.8326 rad)
- Elbow: -70° (-1.2217 rad)
- Wrist_Pitch: -60° (-1.0472 rad)
- Wrist_Roll: 0.0 rad
- Gripper: 0.0 rad (closed)

## Output

The script generates a single STL file containing all robot parts assembled in the home configuration. All STL meshes are loaded, transformed according to their joint positions, and combined into one STL file.
