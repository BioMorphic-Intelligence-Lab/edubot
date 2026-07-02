# Installation

Back to [Home](Home.md)

This page covers installing the prerequisites and cloning the EduBot repository.
Once installed, continue with [Building and running](building-and-running.md).

## Prerequisites

To compile EduBot you need [ROS 2](https://docs.ros.org/en/humble/index.html) and
`boost`. Pick the ROS 2 distribution that matches your Ubuntu version:

- Ubuntu 22.04 -> ROS 2 Humble
- Ubuntu 24.04 -> ROS 2 Jazzy

If you are running inside a virtual machine, also read the VM setup notes in
[Troubleshooting](troubleshooting.md#virtual-machine-users).

### Ubuntu 22.04: install ROS 2 Humble

Full instructions are in the
[ROS 2 Humble documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
The easiest way is to add the apt sources and install:

```bash
# Add the ROS 2 GPG key with apt.
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update the sources
sudo apt update && sudo apt upgrade

# Finally install ROS and ROS dev tools
sudo apt install ros-humble-desktop ros-dev-tools
```

Then install the additional ROS libraries and Python dependencies:

```bash
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
# Install pip if not yet installed
sudo apt-get install python3-pip
# Install python pkg dependency
pip install catkin_pkg
```

### Ubuntu 24.04: install ROS 2 Jazzy

Full instructions are in the
[ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
The easiest way is to add the apt sources and install:

```bash
# Add the ROS 2 GPG key with apt.
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update the sources
sudo apt update

# Finally install ROS and ROS dev tools
sudo apt install ros-dev-tools ros-jazzy-desktop
```

Then install the additional ROS libraries and Python dependencies:

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui
# Install pip if not yet installed
sudo apt-get install python3-pip
# Install python pkg dependency (global)
sudo apt-get install python-catkin_pkg
```

If your installation complains about GPG keys, run:

```bash
sudo rm /etc/apt/sources.list.d/ros2.sources
```

### Boost (both 22.04 and 24.04)

The required `boost` libraries are installed via:

```bash
sudo apt install libboost-all-dev
```

## Cloning the repository

The driver library [`feetech_cpp_lib`](https://github.com/BioMorphic-Intelligence-Lab/feetech_cpp_lib) is a git
**submodule** (it points at the
[`feetech_cpp_lib`](https://github.com/BioMorphic-Intelligence-Lab/feetech_cpp_lib)
repository on the `leRobot_mods` branch, as declared in
[`.gitmodules`](../.gitmodules)). For that reason you must clone **recursively** so
the submodule is fetched too:

```bash
git clone --recurse-submodules https://github.com/bioMorphic-Intelligence-Lab/edubot
```

> If you already cloned without `--recurse-submodules`, run
> `git submodule update --init --recursive` from inside the repository to fetch it.

## USB access for the hardware

To let ROS access the USB serial port connected to the EduBot, add your user to
the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
```

Log out and back in (or reboot) for the group change to take effect.

## Next steps

- [Building and running](building-and-running.md) - compile the workspace and launch the robot or simulation.
- [Troubleshooting](troubleshooting.md) - if the USB port is not detected or you hit permission errors.
