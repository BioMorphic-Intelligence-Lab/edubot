# EduBot: The 4-DoF manipulator for Education

This repository contains the drivers, visualization features and a simple simulation for the [Lynxmotion AL5A Arm](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/servo-erector-set-robots-kits/ses-v1-robots/ses-v1-arms/al5a/), all of which are implemented using the [ROS2](https://docs.ros.org/en/humble/index.html) middleware.

## Installation 

### Pre-requisites

To compile the pre-requisites are `ros2` and `boost`.
ROS2 humble (LTS) can be installed for ubuntu (>= 22.04) as explained in the [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). 
The easiest way is to add the sources and install via apt

        # Add the ROS 2 GPG key with apt.
        sudo apt update && sudo apt install curl -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 

        # Add the repository to your sources list.
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

        # Update the sources
        sudo apt update && sudo apt upgrade

        # Finally install ros
        sudo apt install ros-humble-desktop

Make sure the additional ros libraries are installed

        sudo apt install ros-humble-xacro ros-humble-joint-state-publisher

The required `boost` libraries are installed via

        sudo apt install libboost-all-dev

Finally, clone this repository recursively

        git clone --recurse-submodules https://github.com/bioMorphic-Intelligence-Lab/edubot

### Compilation

The repository contains two folders: `cpp_impl` and `python_impl`. The former contains the driver, visualization, simulation and a simple control example of the robot while the latter only contains the control example. 
You can choose which language you would like to write your controller in using the code provided as an initial guideline.
To compile each of the packages navigate into the folder, source your `ros` installation and call the `colcon` compilation, e.g.

        cd cpp_impl
        source /opt/ros/humble/setup.bash
        colcon build

### Running

Once the package is compiled and sourced via

      source cpp_impl/install/setup.bash

You can run start the simulation or the driver for the robot with the following commands

 Command                            |  Effect 
------------------------------------|---------------------------------------------------
`ros2 launch edubot sim.launch.py`  |  Launches the simulation and `rviz` visualization
`ros2 launch edubot rviz.launch.py` |  Launches the `rviz` visualization and a joint position interface which lets you play with the robot
`ros2 run edubot robot_hw`          |  Starts the Hardware driver for the robot
`ros2 run controllers example_traj` |  Starts the an controller that commands a periodic example trajectory

## Issues

If any issues with this software occur, please don't hesitate to use the [Issues](https://github.com/BioMorphic-Intelligence-Lab/edubot/issues) pane within this repository.

Good Luck!



