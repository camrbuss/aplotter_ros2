# aplotter_ros2
Pen Plotter using ODrive motor control and ROS2

This repository is dependend on a ROS2 CAN ODrive package found at https://github.com/camrbuss/ros2_odrive_can

![CAD](https://lh3.googleusercontent.com/pw/ACtC-3clGLWjGjPLQd7VhQk3QP3K9Lg39KJp0tQEz6EgUK5Ak4WE-YtDJ8ZNHEzONkK3RGVDRFE0xoxDykFvw1JDTLQJV9iui-XEQ9JTUxxphp_TFRbhiN1gXHQWvIN8GwDhQNKsty9ygs0llwD1Yx70qwJe6w=w1168-h902-no?authuser=0)


## Microsoft Xbox 360 Wired Controller for Linux Interface

The controls needed to  achieve velocity control are sequential. The left bumper correspond to axis 1 and the right bumber to axis 0.

| ID | Button Combination | Function |
|-|-|-|
| 1 | <Left/Right> Bumper + X | E-Stop |
| 2 | <Left/Right> Bumper + Y | Calibrate Axis |
| 3 | <Left/Right> Bumper + A | Home Axis |
| 4 | <Left/Right> Bumper + B | Request Close Loop Control |
| 5 | <Left/Right> Bumper + Back | Clear Axis Error |
| 6 | Menu | Start Command Loop |
| 7 | <Left/Right> Bumper + D-Pad Left | Set Velocity Control Mode |
| 8 | D-Pad Up | Increment Max Velocity |
| 9 | D-Pad Down | Decrement Max Velocity |
| 10 | Left Joystick | X, Y Velocity Control |
| 11 | Left Joystick Button | Toggle Pen |

### Typical Sequence

1. ID2
2. ID3
3. ID4
4. ID4
5. ID7
6. ID6
7. ID10