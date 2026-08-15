# DDSM115 ROS 2 Controller

DDSM115 motors are controlled over RS-485 from ROS 2 Humble. The package provides motor discovery, ID setup, velocity control, differential-drive control, and a PyQt5 test GUI.

## Build

```sh
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ddsm115_controller --symlink-install
source install/setup.bash
```

## Configuration

Edit [config/robot.yaml](config/robot.yaml) for the serial device, motor IDs, motor directions, wheel dimensions, and GUI limits. The GUI can also edit this file with **Edit shared YAML**. It creates a `.bak` backup and requires a restart to apply changes.

The differential-drive node uses the standard wheel equations, limits commands proportionally to `330 RPM`, and treats stale RPM feedback as zero after `feedback_timeout`.

Motor IDs are scanned from 1 through `max_check`. The default `max_check: 2` supports either one connected motor or motors with IDs 1 and 2.

## Test GUI

Start the GUI and controller with one or two motors:

```sh
ros2 launch ddsm115_controller motor_test_gui.launch.py
```

Override the serial device or scan range when needed:

```sh
ros2 launch ddsm115_controller motor_test_gui.launch.py \
  usb_dev:=/dev/ttyUSB1 max_check:=2
```

The GUI provides:

- RPM, current, temperature, error, and online-state monitoring
- Single-motor mode with a vertical mouse-pad control
- Differential-drive mode with `linear.x` and `angular.z` mouse-pad control
- Physical joystick input with a neutral dead zone and gradual speed changes
- `Drive`, `Free`, and `Brake` motor states
- Odometry display from `/odom`
- A shared YAML editor for motor and robot parameters

Releasing the mouse button sends zero velocity. The maximum single-motor command is `-330` to `330 RPM`.

## Production Differential Drive

Start the controller and differential-drive node without the GUI:

```sh
ros2 launch ddsm115_controller robot.launch.py
```

This launch uses the same `config/robot.yaml` and scans motor IDs 1 and 2 by default. It also accepts `usb_dev:=`, `max_check:=`, `command_timeout:=`, and `config:=` overrides.

## Motor ID Tools

Connect only one motor when changing or checking an ID:

```sh
ros2 run ddsm115_controller set_motor_id \
  --ros-args -p usb_dev:=/dev/ttyUSB0

ros2 run ddsm115_controller check_motor_id \
  --ros-args -p usb_dev:=/dev/ttyUSB0 -p max_check:=2
```

Restart motor power after changing an ID. Use IDs 1 through 255; ID 0 is not used.

## Topics and Services

The controller subscribes to:

- `/ddsm115/rpm_cmd` (`std_msgs/msg/Int16MultiArray`)
- `/ddsm115/brake` (`std_msgs/msg/Bool`)

It publishes:

- `/ddsm115/rpm_fb` (`std_msgs/msg/Int16MultiArray`)
- `/ddsm115/cur_fb` (`std_msgs/msg/Float32MultiArray`)
- `/ddsm115/temp_fb` (`std_msgs/msg/Int8MultiArray`)
- `/ddsm115/error` (`std_msgs/msg/Int8MultiArray`)
- `/ddsm115/online_id` (`std_msgs/msg/Int8MultiArray`)

It provides `/ddsm115/set_freewheel` (`std_srvs/srv/SetBool`). Set `data: true` for freewheel and `data: false` to restore velocity mode at zero RPM.

## Safety

Begin with the drive wheels lifted and low RPM. Keep the physical emergency-stop switch accessible. ROS commands are not a replacement for an independent power cutoff.
