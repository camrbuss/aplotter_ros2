#ifndef __APLOTTER_STATE_PUBLISHER_HPP
#define __APLOTTER_STATE_PUBLISHER_HPP

#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ros2_odrive_can/msg/odrive_status.hpp"
#include "ros2_odrive_can/srv/set_axis_requested_state.hpp"
#include "ros2_odrive_can/srv/set_controller_modes.hpp"
#include "ros2_odrive_can/srv/set_input_vel.hpp"
#include "ros2_odrive_can/srv/get_encoder_estimates.hpp"
#include "ros2_odrive_can/srv/odrive_estop.hpp"
#include "ros2_odrive_can/srv/clear_errors.hpp"

#define SRVCALLTIMEOUT std::chrono::milliseconds(1000)
#define VELOCITYINCREMENTAMOUNT 1000000

class APlotterStatePublisher : public rclcpp::Node
{
public:
  APlotterStatePublisher();
  ~APlotterStatePublisher();

private:
  void timer_callback();

  void odrive_status_callback(const ros2_odrive_can::msg::OdriveStatus::SharedPtr msg);

  void compute_jacobian(float a_pos, float b_pos, float a_vel, float b_vel);

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  void odrive_calibrate(int8_t axis);                                                // Y button
  void odrive_toggle_closed_loop_control(int8_t axis);                               // A button, toggle between closed loop and inactive(1)
  void odrive_home(int8_t axis);                                                     // B button
  void odrive_estop();                                                               // X button
  void odrive_clear_errors(int8_t axis);                                             // back button
  void odrive_set_control_mode(int8_t axis, int8_t control_mode, int8_t input_mode); // start button
  void odrive_adjust_max_velocity(int32_t amount);                                   // dpad up and down will increment and decrement speed
  void odrive_toggle_pen();                                                          // left joystick button to raise/lower pen
  void odrive_toggle_send_commands();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<ros2_odrive_can::msg::OdriveStatus>::SharedPtr odrive_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

  rclcpp::Clock ros_clock_;

  rclcpp::Client<ros2_odrive_can::srv::SetAxisRequestedState>::SharedPtr odrive_set_axis_requested_state_client_;
  rclcpp::Client<ros2_odrive_can::srv::GetEncoderEstimates>::SharedPtr odrive_get_encoder_estimates_client_;
  rclcpp::Client<ros2_odrive_can::srv::SetControllerModes>::SharedPtr odrive_set_controller_modes_client_;
  rclcpp::Client<ros2_odrive_can::srv::SetInputVel>::SharedPtr odrive_set_input_vel_client_;

  rclcpp::Client<ros2_odrive_can::srv::OdriveEstop>::SharedPtr odrive_estop_client_;
  rclcpp::Client<ros2_odrive_can::srv::ClearErrors>::SharedPtr odrive_clear_errors_client_;

  sensor_msgs::msg::JointState joint_state_msg_;
  // std::shared_ptr<ros2_odrive_can::srv::SetInputVel_Request> input_vel_request_;

  struct joy_data_t
  {
    // Joy stick of Xbox 360 with ROS Joy
    // a_ denotes an axis, b_ denotes a button
    int32_t b_a = 0;
    int32_t b_b = 0;
    int32_t b_x = 0;
    int32_t b_y = 0;
    int32_t b_left_bumper = 0;
    int32_t b_right_bumper = 0;
    int32_t b_back = 0;
    int32_t b_start = 0;
    int32_t b_power = 0;
    int32_t b_left_stick_button = 0;
    int32_t b_right_stick_button = 0;
    float a_left_stick_horizontal;
    float a_left_stick_vertical;
    float a_right_stick_horizontal;
    float a_right_stick_vertical;
    float a_right_trigger;
    float a_left_trigger;
    float a_cross_horizontal;
    float a_cross_vertical;
  };
  joy_data_t joy_current_state_;
  joy_data_t joy_previous_state_;

  float x_vel_ = 0.0;
  float y_vel_ = 0.0;

  float a_vel_setpoint_ = 0.0;
  float b_vel_setpoint_ = 0.0;

  float max_velocity_ = 10000000.0;

  bool is_odrive_alive_ = false;
  struct odrive_axis_state_t
  {
    int error = 0;
    int state = 0;
    bool is_calibrated = false;
    bool is_homed = false;
  };
  odrive_axis_state_t odrive_axis_[2];

  int calibration_count_ = 0;
};

#endif /* __APLOTTER_STATE_PUBLISHER_HPP */