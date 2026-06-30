# Troubleshooting

Back to [Home](Home.md)

Common problems and how to fix them. If none of these help, see
[Reporting issues](#reporting-issues) at the bottom.

## BRLTTY interferes with the servo driver

Ubuntu sometimes installs a package called `brltty` automatically, which grabs
USB serial devices and interferes with the servo driver. Remove it with:

```bash
sudo apt purge brltty
```

## Wrong baud rate or USB port

The USB port name and baud rate are defined in the config file
([`lerobot/config/robot_hw.yaml`](../ros_ws/src/lerobot/config/robot_hw.yaml), and
[`robot_read.yaml`](../ros_ws/src/lerobot/config/robot_read.yaml) for the
read-only node). Check which port was assigned with:

```bash
ls /dev/ttyUSB*
```

This should match the `serial_port` value in the YAML file, and `baud_rate` must
match the servos' configured baud. **Rebuild** (`colcon build`) after editing the
config so the change takes effect. See [Configuration](configuration.md).

## WSL: USB passthrough

If you are using WSL, the device may not appear because USB is not passed through
to WSL by default. Install and use `usbipd` on Windows to bind the port and
attach it to WSL, then check `ls /dev/ttyUSB*` inside WSL.

## Virtual machine users

Download and install both **VirtualBox** and the **VirtualBox Extension Pack**
from [their website](https://www.virtualbox.org/wiki/Downloads). Ensure both have
the same version. If a VM is already running, power it off before installing the
Extension Pack.

In **Virtual Machine Settings > USB**, check **Enable USB Controller** and the
matching USB type (e.g. **USB 3.0 (xHCI) Controller**).

Then, in this order: (1) connect the robot's USB connector, (2) power the robot
motors, (3) power on the virtual machine. Every new terminal must source ROS and
the workspace, and you must have built with `colcon` (see
[Building and running](building-and-running.md)).

Verify the VM can see the USB device:

```bash
ls -l /dev/ttyUSB*
```

If you get a **permission denied** error, add your user to the `dialout` group:

```bash
sudo usermod -a $USER -G dialout
```

Log out and back in, then try again (and remember to re-source).

> **If the USB still cannot be read:** unplug everything and kill all ROS
> processes, then (1) power the virtual machine, (2) power ON the robot emergency
> switch, (3) plug in the USB connector, (4) plug in power for the motors.

An example setup walkthrough is available
[on YouTube](https://www.youtube.com/watch?v=h-EOHbVqsJg).

## Reporting issues

If you hit a bug not covered here, please open an issue on the
[Issues](https://github.com/BioMorphic-Intelligence-Lab/edubot/issues) tracker
with a short description and steps to reproduce it.

Good luck!
