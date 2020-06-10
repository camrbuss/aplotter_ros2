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

class APlotterStatePublisher : public rclcpp::Node
{
public:
  APlotterStatePublisher();
  ~APlotterStatePublisher();

private:
  void timer_callback();

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  void odrive_status_callback(const ros2_odrive_can::msg::OdriveStatus::SharedPtr msg);

  void compute_jacobian(float a_pos, float b_pos, float a_vel, float b_vel);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<ros2_odrive_can::msg::OdriveStatus>::SharedPtr odrive_subscription_;
  rclcpp::Clock ros_clock_;

  rclcpp::Client<ros2_odrive_can::srv::SetAxisRequestedState>::SharedPtr odrive_set_axis_requested_state_client_;
  rclcpp::Client<ros2_odrive_can::srv::GetEncoderEstimates>::SharedPtr odrive_get_encoder_estimates_client_;
  rclcpp::Client<ros2_odrive_can::srv::SetControllerModes>::SharedPtr odrive_set_controller_modes_client_;
  rclcpp::Client<ros2_odrive_can::srv::SetInputVel>::SharedPtr odrive_set_input_vel_client_;

  sensor_msgs::msg::JointState joint_state_msg_;
  std::shared_ptr<ros2_odrive_can::srv::SetInputVel_Request> input_vel_request_;

  float x_vel_ = 0.0;
  float y_vel_ = 0.0;

  float a_vel_setpoint_ = 0.0;
  float b_vel_setpoint_ = 0.0;

  bool is_odrive_alive_ = false;
  int odrive_axis0_error_ = 0;
  int odrive_axis1_error_ = 0;
  int odrive_axis0_state_ = 0;
  int odrive_axis1_state_ = 0;

  int calibration_count_ = 0;
};