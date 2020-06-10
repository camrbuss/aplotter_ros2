#include "aplotter_ros2/aplotter_state_publisher.hpp"

APlotterStatePublisher::APlotterStatePublisher() : Node("aplotter_state_publisher")
{
  publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&APlotterStatePublisher::timer_callback, this));
  timer_->cancel();
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&APlotterStatePublisher::joy_callback, this, std::placeholders::_1));
  odrive_subscription_ = this->create_subscription<ros2_odrive_can::msg::OdriveStatus>("/odrive/odrive_status", 5, std::bind(&APlotterStatePublisher::odrive_status_callback, this, std::placeholders::_1));

  odrive_set_axis_requested_state_client_ = this->create_client<ros2_odrive_can::srv::SetAxisRequestedState>("odrive/set_axis_requested_state");
  while (!odrive_set_axis_requested_state_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not create a Client for ros2_odrive_can::srv::SetAxisRequestedState");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for ros2_odrive_can::srv::etAxisRequestedState service");
    }
  }
  odrive_get_encoder_estimates_client_ = this->create_client<ros2_odrive_can::srv::GetEncoderEstimates>("odrive/get_encoder_estimates");
  while (!odrive_get_encoder_estimates_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not create a Client for ros2_odrive_can::srv::GetEncoderEstimates");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for ros2_odrive_can::srv::GetEncoderEstimates service");
    }
  }
  odrive_set_controller_modes_client_ = this->create_client<ros2_odrive_can::srv::SetControllerModes>("odrive/set_controller_modes");
  while (!odrive_set_controller_modes_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not create a Client for ros2_odrive_can::srv::SetControllerModes");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for ros2_odrive_can::srv::SetControllerModes service");
    }
  }
  odrive_set_input_vel_client_ = this->create_client<ros2_odrive_can::srv::SetInputVel>("odrive/set_input_vel");
  while (!odrive_set_input_vel_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not create a Client for ros2_odrive_can::srv::SetInputVel");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for ros2_odrive_can::srv::SetInputVel service");
    }
  }

  // TODO: Publish joint states for Rviz vizualization
  // TODO: Update STLs for Visualization
  joint_state_msg_.name.push_back("leftPJoint");
  joint_state_msg_.name.push_back("rightPJoint");
  joint_state_msg_.name.push_back("leftJoint");
  joint_state_msg_.name.push_back("rightJoint");
  joint_state_msg_.position.push_back(0.0);
  joint_state_msg_.position.push_back(0.0);
  joint_state_msg_.position.push_back(0.0);
  joint_state_msg_.position.push_back(0.0);
}

  APlotterStatePublisher::~APlotterStatePublisher()
  {
  }

void APlotterStatePublisher::timer_callback()
{
  // TODO: Allow for a way to do axis full calibration automatically
  // TODO: Allow for homing
  // TODO: Fix all the requests with not waiting or callback to signify success
  if (calibration_count_ == 0)
  {
    auto request = std::make_shared<ros2_odrive_can::srv::SetControllerModes::Request>();
    request->axis = 0;
    request->control_mode = 2; // Velocity Control
    request->input_mode = 1;   // Passthrough
    auto result_future = odrive_set_controller_modes_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Sent request for Velocity Input Mode");
    calibration_count_++;
  }

  else if ((calibration_count_ == 1) && this->is_odrive_alive_ && (this->odrive_axis0_state_ == 1) && (this->odrive_axis0_error_) == 0 && (this->odrive_axis0_state_ != 8))
  {
    auto request = std::make_shared<ros2_odrive_can::srv::SetAxisRequestedState::Request>();
    request->axis = 0;
    request->requested_state = 8; // Closed Loop Control
    auto result_future = odrive_set_axis_requested_state_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Sent request for Closed Loop Control");
    calibration_count_++;
  }

  // TODO: Add axis1 support
  else if (this->is_odrive_alive_ && (this->odrive_axis0_error_) == 0 && (this->odrive_axis0_state_ == 8))
  {
    auto input_vel_request_ = std::make_shared<ros2_odrive_can::srv::SetInputVel::Request>();
    input_vel_request_->axis = 0;
    input_vel_request_->input_vel = this->y_vel_;
    input_vel_request_->current_ff = 0;

    auto result_future = odrive_set_input_vel_client_->async_send_request(input_vel_request_);
    RCLCPP_INFO(this->get_logger(), "Sent velocity: %f", this->y_vel_);
  }

  // TODO: Add pos and vel get calls for inverse kinematics
  else
  {
    RCLCPP_INFO(this->get_logger(), "ODrive is not happy!");
  }
}

void APlotterStatePublisher::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  this->x_vel_ = -1000000.0 * msg->axes[0];
  this->y_vel_ = 10000000.0 * msg->axes[1];

  // RCLCPP_INFO(this->get_logger(), "x: '%3.3f' y: '%3.3f'", this->x_vel_, this->y_vel_);
}

void APlotterStatePublisher::odrive_status_callback(const ros2_odrive_can::msg::OdriveStatus::SharedPtr msg)
{
  // TODO: Should the ODrive status message be one message or stay as two?
  this->is_odrive_alive_ = msg->is_axis_alive;
  if (msg->axis == 0)
  {
    this->odrive_axis0_error_ = msg->axis_error;
    this->odrive_axis0_state_ = msg->axis_state;
  }
  else if (msg->axis == 1)
  {
    this->odrive_axis1_error_ = msg->axis_error;
    this->odrive_axis1_state_ = msg->axis_state;
  }

  if ((this->odrive_axis0_state_ == 1) && this->odrive_axis0_error_ == 0)
  {
    if (timer_->is_canceled())
    {
      timer_->reset();
      RCLCPP_INFO(this->get_logger(), "Starting Timer");
    }
  }
}

// TODO: Implement Inverse Kinematics
// Joystick will set End Effector X and Y Velocities. Jacobain will calculate
// The velocities to send to each ODrive Motor
void APlotterStatePublisher::compute_jacobian(float a_pos, float b_pos, float a_vel, float b_vel)
{
}