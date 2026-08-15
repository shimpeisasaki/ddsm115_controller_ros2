# DDSM115 ROS 2 Controller

C++ ROS 2 package for controlling DDSM115 motors over an RS-485 serial bus. It provides motor ID setup and discovery tools, a multi-motor velocity controller, and a differential-drive robot node.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- A USB-to-RS-485 adapter available as `/dev/ttyUSB0` by default
- A user account with access to the serial device

No Python virtual environment or pip packages are required.

Add the current user to the serial-device group if needed, then log out and back in:

```sh
sudo usermod -aG dialout "$USER"
```

## Build

From the ROS 2 workspace root:

```sh
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ddsm115_controller --symlink-install
source install/setup.bash
```

Source both setup files in each new terminal before running a node.

## Motor ID Setup

Connect only one motor to the RS-485 bus while assigning its ID:

```sh
ros2 run ddsm115_controller set_motor_id \
	--ros-args -p usb_dev:=/dev/ttyUSB0
```

Enter the desired ID and restart the motor power. The differential-drive node expects the left motor to use ID 1 and the right motor to use ID 2.

Check the connected IDs with `max_check` set to the highest ID to scan:

```sh
ros2 run ddsm115_controller check_motor_id \
	--ros-args -p usb_dev:=/dev/ttyUSB0 -p max_check:=1
```

## Velocity Control

For a single-motor test, connect only motor ID 1 and start the test launch file:

```sh
ros2 launch ddsm115_controller single_motor_test.launch.py
```

To select another serial device:

```sh
ros2 launch ddsm115_controller single_motor_test.launch.py usb_dev:=/dev/ttyUSB1
```

Start the motor interface. For one motor with ID 1:

```sh
ros2 run ddsm115_controller velocity_control \
	--ros-args -p usb_dev:=/dev/ttyUSB0 -p max_check:=1
```

Parameters:

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `usb_dev` | string | `/dev/ttyUSB0` | RS-485 serial device |
| `max_check` | integer | `10` | Highest motor ID scanned at startup |
| `command_timeout` | double | `2.0` | Seconds without an RPM command before commanding zero RPM |

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/ddsm115/rpm_cmd` | `std_msgs/msg/Int16MultiArray` | RPM commands indexed by motor ID minus one |
| `/ddsm115/brake` | `std_msgs/msg/Bool` | Enables or releases braking for all detected motors |

### Services

| Service | Type | Description |
| --- | --- | --- |
| `/ddsm115/set_freewheel` | `std_srvs/srv/SetBool` | Enables zero-current freewheel mode for all detected motors or restores velocity mode at zero RPM |

Enable freewheel mode before moving the robot by hand:

```sh
ros2 service call /ddsm115/set_freewheel std_srvs/srv/SetBool "{data: true}"
```

The controller continues publishing motor RPM feedback, so the differential-drive node continues updating odometry. Restore velocity control before commanding motion:

```sh
ros2 service call /ddsm115/set_freewheel std_srvs/srv/SetBool "{data: false}"
```

For example, command ID 1 at 10 RPM:

```sh
ros2 topic pub -r 5 /ddsm115/rpm_cmd \
	std_msgs/msg/Int16MultiArray "{data: [10]}"
```

Stop the motor explicitly before terminating the controller:

```sh
ros2 topic pub --once /ddsm115/rpm_cmd \
	std_msgs/msg/Int16MultiArray "{data: [0]}"
```

For IDs 1 and 3, include a placeholder for ID 2: `{data: [100, 0, -100]}`. If no RPM command arrives for two seconds, the controller commands zero RPM unless braking is enabled.

RPM commands are limited to the DDSM115 speed-loop range of -330 to 330 RPM. Entries omitted from a command array are set to zero.

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/ddsm115/rpm_fb` | `std_msgs/msg/Int16MultiArray` | RPM feedback indexed by motor ID minus one |
| `/ddsm115/cur_fb` | `std_msgs/msg/Float32MultiArray` | Current feedback in amperes |
| `/ddsm115/temp_fb` | `std_msgs/msg/Int8MultiArray` | Motor winding temperature feedback |
| `/ddsm115/error` | `std_msgs/msg/Int8MultiArray` | Motor error bitmask |
| `/ddsm115/online_id` | `std_msgs/msg/Int8MultiArray` | IDs detected during startup |

Error bits are `1` for sensor error, `2` for overcurrent, `4` for phase error, `8` for stall, and `16` for troubleshooting error.

## Differential-Drive Robot

Run `velocity_control` for IDs 1 and 2, then start the robot node in another terminal:

```sh
ros2 run ddsm115_controller velocity_control \
	--ros-args -p usb_dev:=/dev/ttyUSB0 -p max_check:=2

ros2 run ddsm115_controller two_wheels_robot \
	--ros-args -p wheel_base:=0.255 -p R_wheel:=0.051 -p pub_tf:=true
```

Parameters:

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `wheel_base` | double | `0.255` | Distance between wheel centers in metres |
| `R_wheel` | double | `0.051` | Wheel radius in metres |
| `pub_tf` | boolean | `false` | Publishes the `odom` to `base_link` transform |
| `joystick_timeout` | double | `0.5` | Seconds without joystick input before manual mode commands zero RPM |

ROS interfaces:

| Direction | Topic | Type | Description |
| --- | --- | --- | --- |
| Subscribe | `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot linear and angular velocity command |
| Subscribe | `/joy` | `sensor_msgs/msg/Joy` | Joystick input used by the built-in manual mode |
| Subscribe | `/ddsm115/rpm_fb` | `std_msgs/msg/Int16MultiArray` | Left and right wheel feedback |
| Publish | `/ddsm115/rpm_cmd` | `std_msgs/msg/Int16MultiArray` | Left and right wheel RPM command |
| Publish | `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry in the `odom` frame |

The `base_link` origin represents the midpoint between the left and right wheel rotation centers projected onto the ground. When `pub_tf` is enabled, the node publishes `odom -> base_link`.

## Safety

Lift the drive wheels off the ground for initial tests and begin with a low RPM command. Use a physical emergency stop or motor power disconnect for hardware safety; ROS commands are not a substitute for an independent emergency-stop circuit.