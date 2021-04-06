# aplotter_ros2
Pen Plotter using ODrive motor control and ROS2

This repository is dependend on a ROS2 CAN ODrive package found at https://github.com/camrbuss/ros2_odrive_can

![CAD](https://lh3.googleusercontent.com/pw/ACtC-3clGLWjGjPLQd7VhQk3QP3K9Lg39KJp0tQEz6EgUK5Ak4WE-YtDJ8ZNHEzONkK3RGVDRFE0xoxDykFvw1JDTLQJV9iui-XEQ9JTUxxphp_TFRbhiN1gXHQWvIN8GwDhQNKsty9ygs0llwD1Yx70qwJe6w=w1168-h902-no?authuser=0)

[OnShape CAD Models](https://cad.onshape.com/documents/659b98b8e4e5f48dc0d1c1cf/w/8debc2aa847977d2ea1bd961/e/ff752f2c7fe58914142c1f60)

## Inverse Kinematics

![](./doc/geometry.png)
![](doc/matrix_equations0.png)
![](doc/matrix_equations1.png)

Implement in C++ with a(t) and b(t) coming in as discrete measurements, x'(t) and y'(t) being set by the user, and a'(t) and b'(t) being sent to the motor controllers.

``` cpp
  float j = -L3 * (std::pow(L1, 2) - std::pow(L2, 2) - std::pow(a, 2) + 2.0 * a * b - std::pow(b, 2));
  float k = A1 + std::acos((1.0 / 2.0) * (std::pow(L1, 2) - std::pow(L2, 2) + std::pow(a - b, 2)) / (L1 * (-a + b)));
  float l = 2.0 * L1 * sqrt(1 - 1.0 / 4.0 * std::pow(std::pow(L1, 2) - std::pow(L2, 2) + std::pow(a - b, 2), 2) / (std::pow(L1, 2) * std::pow(a - b, 2))) * std::pow(a - b, 2);
  float f = j * std::sin(k) / l;
  float g = j * std::cos(k) / l;

  this->a_vel_setpoint_ = x_vel_ - ((-1.0f + f) * y_vel_ / g);
  this->b_vel_setpoint_ = x_vel_ - ((f * y_vel_) / g);
```

## Microsoft Xbox 360 Wired Controller for Linux Interface

The controls needed to  achieve velocity control are sequential. The left bumper correspond to axis 1 and the right bumber to axis 0.

| ID | Button Combination | Function |
|-|-|-|
| 1 | <Left/Right> Bumper + X | E-Stop |
| 2 | <Left/Right> Bumper + Y | Calibrate Axis |
| 3 | <Left/Right> Bumper + A | Request Closed Loop Control |
| 4 | <Left/Right> Bumper + B | Home Axis |
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