# pynput_teleop_drive_keyboard
---
## Description
```
This package allows you to use the arrow key to easily steer and drive 
arkerman steering based robots in ROS2.

It publishes drive_velocity and steer_angle(in deg) commands
to the topic named /drive_cmd and uses the std_msgs.msg.Float64MultiArray
messages -> .data{drive_velocity, steer_angle}

It depends on the python pynput module, hence the name of the package.

Before using the package, you must first install the pynput module using pip.
Download or clone the repo in your ROS2 workspace, build,
source and then run.
```

Install the python pynput module via command line using any of the following
commands:
```shell
$ pip3 install pynput
```
or
```shell
$ pip install pynput
```

## Launch
to run basically (speed and steer_angle value defaults to 0.5 and 30 respectively):
```shell
$ ros2 run pynput_arkerman_drive_keyboard pynput_arkerman_drive_keyboard
```

one can also initially set a default speed and steer_angle value:
```shell
$ ros2 run pynput_arkerman_drive_keyboard pynput_arkerman_drive_keyboard 0.4 45
```

## Usage

```
------------------------------------------------
Drive and steer with arrow keys:

 [up/left]     [up]     [up/right]
                |
  [left] ---------------- [right]
                |
[down/left]   [down]   [down/right]

stops when no arrow key is pressed
-------------------------------------------------


w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only steer angle by +/- 5 deg

ALT to reset speed

CTRL-C to quit
```