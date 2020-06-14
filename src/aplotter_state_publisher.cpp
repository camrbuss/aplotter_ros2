#include "aplotter_ros2/aplotter_state_publisher.hpp"

APlotterStatePublisher::APlotterStatePublisher() : Node("aplotter_state_publisher")
{

  publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&APlotterStatePublisher::timer_callback, this));
  timer_->cancel();
  odrive_subscription_ = this->create_subscription<ros2_odrive_can::msg::OdriveStatus>("/odrive/odrive_status", 5, std::bind(&APlotterStatePublisher::odrive_status_callback, this, std::placeholders::_1));
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&APlotterStatePublisher::joy_callback, this, std::placeholders::_1));

  odrive_set_axis_requested_state_client_ = this->create_client<ros2_odrive_can::srv::SetAxisRequestedState>("odrive/set_axis_requested_state");
  while (!odrive_set_axis_requested_state_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
      RCLCPP_ERROR(this->get_logger(), "Could not create a Client for ros2_odrive_can::srv::SetAxisRequestedState");
    else
      RCLCPP_INFO(this->get_logger(), "Waiting for ros2_odrive_can::srv::setAxisRequestedState service");
  }
  odrive_get_encoder_estimates_client_ = this->create_client<ros2_odrive_can::srv::GetEncoderEstimates>("odrive/get_encoder_estimates");
  while (!odrive_get_encoder_estimates_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
      RCLCPP_ERROR(this->get_logger(), "Could not create a Client for ros2_odrive_can::srv::GetEncoderEstimates");
    else
      RCLCPP_INFO(this->get_logger(), "Waiting for ros2_odrive_can::srv::GetEncoderEstimates service");
  }
  odrive_set_controller_modes_client_ = this->create_client<ros2_odrive_can::srv::SetControllerModes>("odrive/set_controller_modes");
  while (!odrive_set_controller_modes_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
      RCLCPP_ERROR(this->get_logger(), "Could not create a Client for ros2_odrive_can::srv::SetControllerModes");
    else
      RCLCPP_INFO(this->get_logger(), "Waiting for ros2_odrive_can::srv::SetControllerModes service");
  }
  odrive_set_input_vel_client_ = this->create_client<ros2_odrive_can::srv::SetInputVel>("odrive/set_input_vel");
  while (!odrive_set_input_vel_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
      RCLCPP_ERROR(this->get_logger(), "Could not create a Client for ros2_odrive_can::srv::SetInputVel");
    else
      RCLCPP_INFO(this->get_logger(), "Waiting for ros2_odrive_can::srv::SetInputVel service");
  }
  odrive_estop_client_ = this->create_client<ros2_odrive_can::srv::OdriveEstop>("odrive/odrive_estop");
  while (!odrive_estop_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
      RCLCPP_ERROR(this->get_logger(), "Could not create a Client for ros2_odrive_can::srv::OdriveEstop");
    else
      RCLCPP_INFO(this->get_logger(), "Waiting for ros2_odrive_can::srv::OdriveEstop service");
  }
  odrive_clear_errors_client_ = this->create_client<ros2_odrive_can::srv::ClearErrors>("odrive/clear_errors");
  while (!odrive_clear_errors_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
      RCLCPP_ERROR(this->get_logger(), "Could not create a Client for ros2_odrive_can::srv::ClearErrors");
    else
      RCLCPP_INFO(this->get_logger(), "Waiting for ros2_odrive_can::srv::ClearErrors service");
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

  if (odrive_axis_[0].error || odrive_axis_[1].error || (odrive_axis_[0].state != 8) || (odrive_axis_[1].state != 8))
  {
    RCLCPP_INFO(this->get_logger(), "Could not proceed in control loop!");
    RCLCPP_INFO(this->get_logger(), "A0 S: %i E: %i A1 S: %i E: %i", odrive_axis_[0].state, odrive_axis_[0].error, odrive_axis_[1].state, odrive_axis_[1].error);
  }
  else
  {
    // TOODO: adjust to control both axis in either position or velocity control
    odrive_set_input_velocity(0, this->y_vel_);
    odrive_set_input_velocity(1, this->x_vel_);

    RCLCPP_INFO(this->get_logger(), "Sent A0 Vel: %f A1 Vel: %f", this->y_vel_, this->y_vel_);
  }
}

void APlotterStatePublisher::odrive_status_callback(const ros2_odrive_can::msg::OdriveStatus::SharedPtr msg)
{
  // If an error is first flagged, stop the control loop and make both controllers inactive
  if ((msg->axis_error > 0) && (odrive_axis_[msg->axis].error != msg->axis_error))
  {
    RCLCPP_INFO(this->get_logger(), "A0 S: %i E: %i A1 S: %i E: %i", odrive_axis_[0].state, odrive_axis_[0].error, odrive_axis_[1].state, odrive_axis_[1].error);
    this->timer_->cancel();
    this->odrive_set_input_velocity(0, 0); // Set input Vel to 0
    this->odrive_set_input_velocity(1, 0);
    this->odrive_set_requested_state(0, 1); // Set Axis to idle
    this->odrive_set_requested_state(1, 1);
  }

  this->is_odrive_alive_ = msg->is_axis_alive;
  this->odrive_axis_[msg->axis].state = msg->axis_state;
  this->odrive_axis_[msg->axis].error = msg->axis_error;

  RCLCPP_DEBUG(this->get_logger(), "A0 S: %i E: %i A1 S: %i E: %i", odrive_axis_[0].state, odrive_axis_[0].error, odrive_axis_[1].state, odrive_axis_[1].error);
}

// TODO: Implement Inverse Kinematics
// Joystick will set End Effector X and Y Velocities. Jacobain will calculate
// The velocities to send to each ODrive Motor
void APlotterStatePublisher::compute_jacobian(float a_pos, float b_pos, float a_vel, float b_vel)
{
  // TODO implement real jacobian
  this->a_vel_setpoint_ = a_pos + a_vel;
  this->b_vel_setpoint_ = b_pos + b_vel;
}

void APlotterStatePublisher::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  this->joy_current_state_.b_a = msg->buttons[0];
  this->joy_current_state_.b_b = msg->buttons[1];
  this->joy_current_state_.b_x = msg->buttons[2];
  this->joy_current_state_.b_y = msg->buttons[3];
  this->joy_current_state_.b_left_bumper = msg->buttons[4];
  this->joy_current_state_.b_right_bumper = msg->buttons[5];
  this->joy_current_state_.b_back = msg->buttons[6];
  this->joy_current_state_.b_start = msg->buttons[7];
  this->joy_current_state_.b_power = msg->buttons[8];
  this->joy_current_state_.b_left_stick_button = msg->buttons[9];
  this->joy_current_state_.b_right_stick_button = msg->buttons[10];
  this->joy_current_state_.a_left_stick_horizontal = msg->axes[0];
  this->joy_current_state_.a_left_stick_vertical = msg->axes[1];
  this->joy_current_state_.a_right_stick_horizontal = msg->axes[2];
  this->joy_current_state_.a_right_stick_vertical = msg->axes[3];
  this->joy_current_state_.a_right_trigger = msg->axes[4];
  this->joy_current_state_.a_left_trigger = msg->axes[5];
  this->joy_current_state_.a_cross_horizontal = msg->axes[6];
  this->joy_current_state_.a_cross_vertical = msg->axes[7];

  // RCLCPP_INFO(this->get_logger(), "Current Y: %i, Previous Y: %i, Right Bumper: %i", joy_current_state_.b_y, joy_previous_state_.b_y, joy_current_state_.b_right_bumper);

  if (joy_current_state_.b_y && (joy_current_state_.b_y != joy_previous_state_.b_y) && joy_current_state_.b_right_bumper)
    this->odrive_set_requested_state(0, 3);

  if (joy_current_state_.b_y && (joy_current_state_.b_y != joy_previous_state_.b_y) && joy_current_state_.b_left_bumper)
    this->odrive_set_requested_state(1, 3);

  if (joy_current_state_.b_a && (joy_current_state_.b_a != joy_previous_state_.b_a) && joy_current_state_.b_right_bumper)
    this->odrive_toggle_closed_loop_control(0);

  if (joy_current_state_.b_a && (joy_current_state_.b_a != joy_previous_state_.b_a) && joy_current_state_.b_left_bumper)
    this->odrive_toggle_closed_loop_control(1);

  if (joy_current_state_.b_b && (joy_current_state_.b_b != joy_previous_state_.b_b) && joy_current_state_.b_right_bumper)
    this->odrive_set_requested_state(0, 11);

  if (joy_current_state_.b_b && (joy_current_state_.b_b != joy_previous_state_.b_b) && joy_current_state_.b_left_bumper)
    this->odrive_set_requested_state(1, 11);

  if (joy_current_state_.b_back && (joy_current_state_.b_back != joy_previous_state_.b_back) && joy_current_state_.b_right_bumper)
    this->odrive_clear_errors(0);

  if (joy_current_state_.b_back && (joy_current_state_.b_back != joy_previous_state_.b_back) && joy_current_state_.b_left_bumper)
    this->odrive_clear_errors(1);

  // TODO: Add in right button press of dpad for position control
  if ((joy_current_state_.a_cross_horizontal != joy_previous_state_.a_cross_horizontal) && (joy_current_state_.a_cross_horizontal > 0.5) && joy_current_state_.b_right_bumper)
    this->odrive_set_control_mode(0, 2, 1);

  if ((joy_current_state_.a_cross_horizontal != joy_previous_state_.a_cross_horizontal) && (joy_current_state_.a_cross_horizontal > 0.5) && joy_current_state_.b_left_bumper)
    this->odrive_set_control_mode(1, 2, 1);

  // TODO: Fix increment ammount to parameter
  if ((joy_current_state_.a_cross_vertical != joy_previous_state_.a_cross_vertical) && (joy_current_state_.a_cross_vertical > 0.5))
    this->odrive_adjust_max_velocity(VELOCITYINCREMENTAMOUNT);

  if ((joy_current_state_.a_cross_vertical != joy_previous_state_.a_cross_vertical) && (joy_current_state_.a_cross_vertical < -0.5))
    this->odrive_adjust_max_velocity(-VELOCITYINCREMENTAMOUNT);

  if (joy_current_state_.b_start && (joy_current_state_.b_start != joy_previous_state_.b_start))
    this->odrive_toggle_send_commands();

  if ((joy_current_state_.b_x != joy_previous_state_.b_x) && joy_current_state_.b_x)
    this->odrive_estop();

  if ((joy_current_state_.b_left_stick_button != joy_previous_state_.b_left_stick_button) && joy_current_state_.b_left_stick_button)
    this->odrive_toggle_pen();

  // TODO: Adjust the mapping to the joysticks
  this->x_vel_ = max_velocity_ * this->joy_current_state_.a_left_stick_horizontal;
  this->y_vel_ = max_velocity_ * this->joy_current_state_.a_left_stick_vertical;
  // RCLCPP_INFO(this->get_logger(), "x: '%3.3f' y: '%3.3f'", this->x_vel_, this->y_vel_);

  joy_previous_state_ = joy_current_state_;
}

  void APlotterStatePublisher::odrive_set_input_velocity(int8_t axis, float velocity){
    auto input_vel_request = std::make_shared<ros2_odrive_can::srv::SetInputVel::Request>();

    input_vel_request->axis = axis;
    input_vel_request->input_vel = velocity;
    input_vel_request->current_ff = 0;
    auto result_future = odrive_set_input_vel_client_->async_send_request(input_vel_request);
  }


void APlotterStatePublisher::odrive_set_requested_state(int8_t axis, int8_t state)
{
  RCLCPP_INFO(this->get_logger(), "ODrive Axis: %i Requested State: %i", axis, state);
  auto request = std::make_shared<ros2_odrive_can::srv::SetAxisRequestedState::Request>();
  request->axis = axis;
  request->requested_state = state;

  // TODO: Change this to use SharedFutureWithRequest to set is_calibrated after getting return value
  // TODO: Capturing this is giving seg faults
  // auto response_received_callback = [](rclcpp::Client<ros2_odrive_can::srv::SetAxisRequestedState>::SharedFuture future) {
  //   if (future.get().second->success)
  //     this->odrive_axis_[future.get().first->axis].is_calibrated = true;
  //   RCLCPP_INFO(this->get_logger(), "ODrive Calibrate returned: %i", future.get()->success);
  //   std::cout << "Service Response" << future.get()->success;
  // };

  // auto future_result = odrive_set_axis_requested_state_client_->async_send_request(request, response_received_callback);
  auto future_result = odrive_set_axis_requested_state_client_->async_send_request(request);
}

// A button, toggle between closed loop and inactive(1)
void APlotterStatePublisher::odrive_toggle_closed_loop_control(int8_t axis)
{
  // TODO: Check that the axis is calibrated before entering closed loop control
  int requested_state;
  if (odrive_axis_[axis].state == 1)
    requested_state = 8; // Closed Loop Control
  else if (odrive_axis_[axis].state == 8)
    requested_state = 1; // Idle
  
  odrive_set_requested_state(axis, requested_state);

  RCLCPP_INFO(this->get_logger(), "Requesting State: %i for ODrive Axis: %i", requested_state, axis);
}

// X button
void APlotterStatePublisher::odrive_estop()
{
  RCLCPP_INFO(this->get_logger(), "ODrive ESTOP");
  auto request = std::make_shared<ros2_odrive_can::srv::OdriveEstop::Request>();
  request->axis = 0;
  auto result_future = odrive_estop_client_->async_send_request(request);
}

// back button
void APlotterStatePublisher::odrive_clear_errors(int8_t axis)
{
  RCLCPP_INFO(this->get_logger(), "ODrive Clear Axis: %i", axis);
  auto request = std::make_shared<ros2_odrive_can::srv::ClearErrors::Request>();
  request->axis = axis;
  auto result_future = odrive_clear_errors_client_->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "A0 S: %i E: %i A1 S: %i E: %i", odrive_axis_[0].state, odrive_axis_[0].error, odrive_axis_[1].state, odrive_axis_[1].error);
}

// dpad left and right
void APlotterStatePublisher::odrive_set_control_mode(int8_t axis, int8_t control_mode, int8_t input_mode)
{
  auto request = std::make_shared<ros2_odrive_can::srv::SetControllerModes::Request>();
  request->axis = axis;
  request->control_mode = control_mode; // Velocity Control
  request->input_mode = input_mode;     // Passthrough
  auto result_future = odrive_set_controller_modes_client_->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "Sent request for Velocity Input Mode for Axis: %i", axis);
}

// dpad up and down will increment and decrement speed
void APlotterStatePublisher::odrive_adjust_max_velocity(int32_t amount)
{
  max_velocity_ = max_velocity_ + amount;
  RCLCPP_INFO(this->get_logger(), "ODrive Max Velocity is now: %f", max_velocity_);
}

// left joystick button to raise/lower pen
void APlotterStatePublisher::odrive_toggle_pen()
{
  // TODO: Implement this in hardware then software
  RCLCPP_INFO(this->get_logger(), "ODrive Toggle Pen");
}

void APlotterStatePublisher::odrive_toggle_send_commands()
{
  // TODO: Prevent starting the time unless there are no errors and we are in closed loop control
  if (this->timer_->is_canceled())
  {
    if (odrive_axis_[0].error || odrive_axis_[1].error || (odrive_axis_[0].state != 8) || (odrive_axis_[1].state != 8))
    {
      RCLCPP_INFO(this->get_logger(), "Could not enter control loop!");
      RCLCPP_INFO(this->get_logger(), "A0 S: %i E: %i A1 S: %i E: %i", odrive_axis_[0].state, odrive_axis_[0].error, odrive_axis_[1].state, odrive_axis_[1].error);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "ODrive Starting to Send Commands");
      this->timer_->reset();
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "ODrive No Longer Sending Commands");
    this->timer_->cancel();
  }
}