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

Use a different shared configuration file when needed:

```sh
ros2 launch ddsm115_controller motor_test_gui.launch.py \
  config:=/path/to/robot.yaml
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

This launch uses the same `config/robot.yaml` and scans motor IDs 1 and 2 by default. Pass `config:=/path/to/robot.yaml` to use another complete configuration. Communication settings are read only from that YAML so GUI edits and production behavior cannot diverge.

## Nav2 Integration

`robot.launch.py` provides the mobile-base interfaces expected by Nav2:

- subscribes to `/cmd_vel`
- publishes `/odom`
- publishes `odom -> base_link` TF by default
- disables the built-in joystick command source by default
- stops motor commands after 0.5 seconds without a new command

Localization or SLAM must provide `map -> odom`; this package must remain the only publisher of `odom -> base_link`. Keep the Nav2 base frame consistent with `base_frame` in `config/robot.yaml`. Tune the pose and twist covariance values from measurements before using `robot_localization` or relying on localization quality.

`/ddsm115/online_id` contains motors that recently returned valid protocol responses. After three consecutive failures an ID is removed, and missing IDs are rescanned once per second.

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

- `/ddsm115/rpm_fb` (`std_msgs/msg/Int16MultiArray`, every `motor_update_period`)
- `/ddsm115/cur_fb` (`std_msgs/msg/Float32MultiArray`, every `current_publish_period`)
- `/ddsm115/temp_fb` (`std_msgs/msg/Int8MultiArray`, every `temperature_publish_period`)
- `/ddsm115/error` (`std_msgs/msg/Int8MultiArray`, every `status_publish_period`)
- `/ddsm115/online_id` (`std_msgs/msg/UInt8MultiArray`, every `status_publish_period`)

RPM command replies already contain RPM, current, temperature, and error in one fixed-length
frame. The publish-period parameters reduce ROS topic traffic; they do not change that frame.
No additional detail query is sent during normal control. `update_period` controls the `/odom`
and `odom -> base_link` TF rate.

It provides `/ddsm115/set_freewheel` (`std_srvs/srv/SetBool`). Set `data: true` for freewheel and `data: false` to restore velocity mode at zero RPM.

## Safety

Begin with the drive wheels lifted and low RPM. Keep the physical emergency-stop switch accessible. ROS commands are not a replacement for an independent power cutoff.
