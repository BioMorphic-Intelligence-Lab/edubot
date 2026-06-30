# Feetech servo driver (`feetech_cpp_lib`)

Back to [Home](Home.md)

[`feetech_cpp_lib`](../ros_ws/src/feetech_cpp_lib) is the low-level C++ driver
that talks to the **Feetech STS/SCS TTL servos** over a USB serial adapter. It is
a git submodule (see [Installation](installation.md#cloning-the-repository)) and
is built as a library, not a ROS node. The hardware ROS node
[`lerobot_hw`](lerobot-nodes.md#lerobot_hw---real-hardware-driver) links against it.

Key files:

- [`include/feetech_cpp_lib/feetech_lib.hpp`](../ros_ws/src/feetech_cpp_lib/include/feetech_cpp_lib/feetech_lib.hpp) - the `FeetechServo` class and supporting types.
- [`src/feetech_lib.cpp`](../ros_ws/src/feetech_cpp_lib/src/feetech_lib.cpp) - the implementation and serial protocol.
- [`include/feetech_cpp_lib/kalman_filter.hpp`](../ros_ws/src/feetech_cpp_lib/include/feetech_cpp_lib/kalman_filter.hpp) - the per-joint Kalman filter.
- [`include/feetech_cpp_lib/boost_timer.hpp`](../ros_ws/src/feetech_cpp_lib/include/feetech_cpp_lib/boost_timer.hpp) - the fixed-rate control-loop timer.
- [`include/feetech_cpp_lib/serial_logger.hpp`](../ros_ws/src/feetech_cpp_lib/include/feetech_cpp_lib/serial_logger.hpp) - optional serial traffic logger.

## Responsibilities

- Open and configure a serial port (Boost.Asio) and talk to one or many servos.
- Run a periodic control loop (`FeetechServo::execute()`) on a dedicated thread.
- Read joint state (position, velocity, current/effort) with a single **sync read**.
- Write targets with **sync write** (position or velocity mode).
- Handle multi-turn position unwrapping, homing, gear ratios and joint direction.
- Optionally filter position + velocity per joint with a Kalman filter.
- Optionally log raw serial TX/RX as hex.

## The `FeetechServo` class

### Construction and lifecycle

```cpp
FeetechServo(std::string port="/dev/ttyUSB0", long baud=1000000,
             double frequency=250, const std::vector<uint8_t>& servo_ids={1},
             bool homing=true, bool logging=false, bool read_only=false);
~FeetechServo();
bool execute();   // periodic loop: read state, then write commands
bool close();
```

`execute()` is invoked automatically by the [BoostTimer](#boosttimer-the-control-loop)
at the configured frequency.

### Sending references (the usual control path)

Higher-level code sets thread-safe reference values; the next `execute()` cycle
sends them to the bus:

```cpp
void setReferencePosition(uint8_t id, double rad);
void setReferenceVelocity(uint8_t id, double rad_per_s);
void setReferenceAcceleration(uint8_t id, double rad_per_s2);
```

### Reading cached state

After each read cycle, filtered values are cached and exposed as vectors:

```cpp
std::vector<double> getCurrentPositions();    // rad (Kalman-filtered if enabled)
std::vector<double> getCurrentVelocities();   // rad/s
std::vector<double> getCurrentTemperatures(); // deg C
std::vector<double> getCurrentCurrents();     // A
std::vector<double> getCurrentPWMs();
```

### Calibration and configuration

| Method | Purpose |
|--------|---------|
| `setDriverSettings(...)` / `getDriverSettings()` | Apply/read runtime settings (baud, loop rate, Kalman + slew-rate tuning) |
| `setHomePosition(...)` / `resetHomePosition()` | Set the raw-tick zero reference per servo |
| `setGearRatio(...)` / `getGearRatio()` | Horn-to-output gear ratio |
| `setMaxSpeed(...)` / `getMaxSpeed()` | Per-servo speed clamp (rad/s) |
| `setVelocityDirection(...)` | Per-joint sign flip (+/-1) |
| `setOperatingMode(...)` | Host-side `DriverMode` (also reconfigures the firmware) |
| `ping(id)` / `stopAll()` | Connectivity check / emergency stop (zero velocity to all) |

### Supporting types

| Type | Role |
|------|------|
| `DriverMode` | Host-side mode: `POSITION`, `VELOCITY`, `PWM`, `STEP`, `CONTINUOUS_POSITION`, `UNPOWERED` |
| `STSMode` | Firmware register mode written to the servo |
| `ServoType` | `STS` vs `SCS` (byte order / sign-bit decoding differ) |
| `UNITS` | `COUNTS`, `RAD`, `DEG` |
| `DriverSettings` | Port, baud, loop rate, Kalman/slew-rate/velocity-gain tuning |
| `ServoData` | Per-servo runtime state, including a `JointKalman` instance |
| `STSRegisters` | Register address constants (e.g. `TARGET_POSITION=0x2A`, `CURRENT_POSITION=0x38`) |

## Serial protocol

The protocol is **Dynamixel-compatible** but uses Feetech-specific register
addresses. Frames have the form:

```
[0xFF][0xFF][ID][LENGTH][INSTRUCTION][PARAMS...][CHECKSUM]
```

The checksum is `~(ID + LENGTH + INSTRUCTION + sum(PARAMS))`.

### Instructions

| Code | Name | Use |
|------|------|-----|
| `0x01` | PING | Connectivity check |
| `0x02` | READ | Read a register block |
| `0x03` | WRITE | Immediate write |
| `0x04` | REGWRITE | Buffered write (committed by ACTION) |
| `0x05` | ACTION | Apply a buffered REGWRITE |
| `0x82` | SYNCREAD | Read the same registers from many IDs in one transaction |
| `0x83` | SYNCWRITE | Write the same register block to many IDs |
| `0xFE` | BROADCAST_ID | Target ID for sync operations |

### Reading state

Each cycle issues a single **SYNCREAD** spanning from `CURRENT_POSITION` (`0x38`)
through `CURRENT_CURRENT` (`0x45`). Each servo responds in turn; the driver
validates the header + checksum, decodes the 16-bit values (the layout differs
between STS and SCS servos), converts ticks to radians using **4096 ticks per
revolution** (with gear ratio and direction applied), and performs multi-turn
unwrapping.

### Writing targets

`execute()` builds command lists based on the current `DriverMode`:

- **Position modes** -> `SYNCWRITE` to `TARGET_POSITION` (`0x2A`).
- **Velocity mode** -> a host-side slew-rate limit is applied, then `SYNCWRITE` to
  `RUNNING_SPEED` (`0x2E`).

Sync writes are fire-and-forget (no per-servo response). Individual writes can be
buffered (`REGWRITE`) and later committed with `triggerAction()` (`ACTION`).

### Timing and concurrency

A mutex guards all bus access. After transmitting, the driver waits roughly one
byte-time per byte of the packet (plus margin) before reading the response, and
reads use an async timeout.

## JointKalman: state filtering

Each servo owns a [`JointKalman`](../ros_ws/src/feetech_cpp_lib/include/feetech_cpp_lib/kalman_filter.hpp)
filter (a linear, constant-velocity Kalman filter with state `[position, velocity]`):

```cpp
void update(double z_pos, double z_vel, double dt,
            double sigma_a, double r_pos, double r_vel);
double position() const;
double velocity() const;
```

When `DriverSettings::ekf_enabled` is true (the default), the raw measured
position and velocity are fused into smoothed estimates, which is what
`getCurrentPositions()` / `getCurrentVelocities()` return. The time step `dt` is
measured from a steady clock (robust to timer jitter). Tunables:
`ekf_accel_noise`, `ekf_pos_noise`, `ekf_vel_noise`.

## BoostTimer: the control loop

[`BoostTimer`](../ros_ws/src/feetech_cpp_lib/include/feetech_cpp_lib/boost_timer.hpp)
runs a callback at a fixed rate on a dedicated thread using
`boost::asio::steady_timer`:

```cpp
BoostTimer(double frequency, std::function<void()> callback);
```

`FeetechServo` constructs one to call `execute()` periodically. Ticks are
scheduled relative to the previous target time, so the loop does not drift even
when a callback occasionally runs long.

## ServoSerialLogger

When logging is enabled, [`ServoSerialLogger`](../ros_ws/src/feetech_cpp_lib/include/feetech_cpp_lib/serial_logger.hpp)
writes a hex dump of all serial traffic to a file (`logTx` / `logRx`), which is
useful when debugging the bus.

## Defaults quick reference

| Setting | Default |
|---------|---------|
| Serial port | `/dev/ttyUSB0` |
| Baud rate | 1 Mbps |
| Encoder resolution | 4096 ticks/revolution |
| Kalman filter (EKF) | enabled |
| Max servos per sync operation | 35 |

> The control-loop frequency differs by entry point: the `FeetechServo`
> constructor defaults to 250 Hz, while `DriverSettings` defaults to 100 Hz and
> the `lerobot` config sets it explicitly (see [Configuration](configuration.md)).

## See also

- [LeRobot nodes](lerobot-nodes.md) - how `lerobot_hw` and `lerobot_read` use this driver.
- [Configuration](configuration.md) - the serial port, baud rate, IDs, zero positions, etc.
- [Troubleshooting](troubleshooting.md) - serial/baud/USB problems.
