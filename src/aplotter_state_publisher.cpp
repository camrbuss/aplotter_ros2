#include "aplotter_ros2/aplotter_state_publisher.hpp"

APlotterStatePublisher::APlotterStatePublisher() : Node("aplotter_state_publisher")
{
  // TODO: Validate parameters
  // callback_handler = this->add_on_set_parameters_callback(std::bind(&APlotterStatePublisher::param_change_callback, this, std::placeholders::_1));

  this->declare_parameter<int64_t>("control_loop_frequency", 50);
  this->declare_parameter<float>("left_arm_length", 350);
  this->declare_parameter<float>("right_arm_pivot_length", 350);
  this->declare_parameter<float>("right_arm_full_length", 565);
  this->declare_parameter<float>("right_arm_offset_angle", 0.09);
  this->declare_parameter<float>("homed_joint_offset", 95);
  this->declare_parameter<float>("mm_per_rev", 12.5);

  this->get_parameter("control_loop_frequency", params_.control_loop_frequency);
  this->get_parameter("left_arm_length", params_.L2);
  this->get_parameter("right_arm_pivot_length", params_.L1);
  this->get_parameter("right_arm_full_length", params_.L3);
  this->get_parameter("right_arm_offset_angle", params_.A1);
  this->get_parameter("homed_joint_offset", params_.homed_joint_offset);
  this->get_parameter("mm_per_rev", params_.mm_per_rev);

  RCLCPP_INFO(this->get_logger(), "Param control_loop_frequency is: %i", params_.control_loop_frequency);
  RCLCPP_INFO(this->get_logger(), "Param left_arm_length is: %f", params_.L2);
  RCLCPP_INFO(this->get_logger(), "Param right_arm_pivot_length is: %f", params_.L1);
  RCLCPP_INFO(this->get_logger(), "Param right_arm_full_length is: %f", params_.L3);
  RCLCPP_INFO(this->get_logger(), "Param right_arm_offset_angle is: %f", params_.A1);
  RCLCPP_INFO(this->get_logger(), "Param homed_joint_offset is: %f", params_.homed_joint_offset);
  RCLCPP_INFO(this->get_logger(), "Param mm_per_rev is: %i", params_.mm_per_rev);

  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  aplotter_position_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("aplotter/aplotter_position", 10);
  aplotter_velocity_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("aplotter/aplotter_desired_velocity", 10, std::bind(&APlotterStatePublisher::planner_callback, this, std::placeholders::_1));
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

  // Initial joint states for Rviz vizualization
  joint_state_msg_.name.push_back("leftPJoint");
  joint_state_msg_.name.push_back("rightPJoint");
  joint_state_msg_.name.push_back("leftJoint");
  joint_state_msg_.name.push_back("rightJoint");
  joint_state_msg_.position.push_back(0.0);
  joint_state_msg_.position.push_back(0.0);
  joint_state_msg_.position.push_back(0.0);
  joint_state_msg_.position.push_back(0.0);

  aplotter_position_msg_.header.frame_id = "/base_link";

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / params_.control_loop_frequency), std::bind(&APlotterStatePublisher::timer_callback, this));
  timer_->reset();
}

APlotterStatePublisher::~APlotterStatePublisher()
{
}

void APlotterStatePublisher::timer_callback()
{
  odrive_get_encoder_estimates(0);
  odrive_get_encoder_estimates(1);
  compute_jacobian();
  if ((odrive_axis_[0].error || odrive_axis_[1].error || (odrive_axis_[0].state != 8) || (odrive_axis_[1].state != 8)) && send_commands_)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not send commands! A0 S: %i E: %i A1 S: %i E: %i", odrive_axis_[0].state, odrive_axis_[0].error, odrive_axis_[1].state, odrive_axis_[1].error);
  }
  else if (send_commands_)
  {
    odrive_set_input_velocity(0, this->b_vel_setpoint_);
    odrive_set_input_velocity(1, this->a_vel_setpoint_);
  }

  joint_state_msg_.header.stamp = ros_clock_.now();
  joint_state_msg_.position[0] = this->a_pos_mm_;
  joint_state_msg_.position[1] = this->b_pos_mm_;
  joint_state_msg_.position[2] = std::acos((std::pow(params_.L2, 2.0) + std::pow((joint_state_msg_.position[1] - joint_state_msg_.position[0]), 2.0) - std::pow(params_.L1, 2.0)) / (2.0 * params_.L2 * (joint_state_msg_.position[1] - joint_state_msg_.position[0])));
  joint_state_msg_.position[3] = -params_.A1 + M_PI - std::acos((std::pow(params_.L1, 2.0) + std::pow((joint_state_msg_.position[1] - joint_state_msg_.position[0]), 2.0) - std::pow(params_.L2, 2.0)) / (2.0 * params_.L1 * (joint_state_msg_.position[1] - joint_state_msg_.position[0])));
  joint_state_publisher_->publish(joint_state_msg_);

  aplotter_position_msg_.header.stamp = ros_clock_.now();
  float beta = joint_state_msg_.position[3];
  aplotter_position_msg_.point.x = this->b_pos_mm_ + params_.L3 * std::cos(beta);
  aplotter_position_msg_.point.y = params_.L3 * std::sin(beta);
  aplotter_position_msg_.point.z = 0;
  aplotter_position_publisher_->publish(aplotter_position_msg_);
}

void APlotterStatePublisher::odrive_status_callback(const ros2_odrive_can::msg::OdriveStatus::SharedPtr msg)
{
  // If an error is first flagged, stop the control loop and make both controllers inactive
  if ((msg->axis_error > 0) && (odrive_axis_[msg->axis].error != msg->axis_error))
  {
    RCLCPP_ERROR(this->get_logger(), "ODrive ERROR A0 S: %i E: %i A1 S: %i E: %i", odrive_axis_[0].state, odrive_axis_[0].error, odrive_axis_[1].state, odrive_axis_[1].error);
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

// Joystick will set End Effector X and Y Velocities. Jacobain will calculate
// The velocities to send to each ODrive Motor. See documentation for equations
void APlotterStatePublisher::compute_jacobian()
{
  float a = this->a_pos_rev_ * this->params_.mm_per_rev - this->params_.homed_joint_offset;
  float b = this->b_pos_rev_ * this->params_.mm_per_rev + this->params_.homed_joint_offset;
  a_pos_mm_ = a;
  b_pos_mm_ = b;

  float j = -params_.L3 * (std::pow(params_.L1, 2) - std::pow(params_.L2, 2) - std::pow(a, 2) + 2.0 * a * b - std::pow(b, 2));
  float k = params_.A1 + std::acos((1.0 / 2.0) * (std::pow(params_.L1, 2) - std::pow(params_.L2, 2) + std::pow(a - b, 2)) / (params_.L1 * (-a + b)));
  float l = 2.0 * params_.L1 * sqrt(1 - 1.0 / 4.0 * std::pow(std::pow(params_.L1, 2) - std::pow(params_.L2, 2) + std::pow(a - b, 2), 2) / (std::pow(params_.L1, 2) * std::pow(a - b, 2))) * std::pow(a - b, 2);
  float f = j * std::sin(k) / l;
  float g = j * std::cos(k) / l;

  this->a_vel_setpoint_ = x_vel_ - ((-1.0f + f) * y_vel_ / g);
  this->b_vel_setpoint_ = x_vel_ - ((f * y_vel_) / g);
  RCLCPP_DEBUG(this->get_logger(), "Units:mm/s Apos:%+.2f Avel:%+.2f Bpos:%+.2f Bvel:%+.2f Avelset:%+.2f Bvelset:%+.2f", a, a_vel_rev_s_, b, b_vel_rev_s_, this->a_vel_setpoint_, this->b_vel_setpoint_);
  this->a_vel_setpoint_ = this->a_vel_setpoint_ / this->params_.mm_per_rev; // mm/s back to rev/s
  this->b_vel_setpoint_ = this->b_vel_setpoint_ / this->params_.mm_per_rev;
}
void APlotterStatePublisher::planner_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!this->is_joy_vel_enabled_)
  {
    this->x_vel_ = msg->pose.orientation.x * max_velocity_;
    this->y_vel_ = msg->pose.orientation.y * max_velocity_;
  }
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

  // if ((joy_current_state_.a_cross_horizontal != joy_previous_state_.a_cross_horizontal) && (joy_current_state_.a_cross_horizontal > 0.5))
  //   this->send_commands_ = !this->send_commands_;

  if ((joy_current_state_.a_cross_horizontal != joy_previous_state_.a_cross_horizontal) && (joy_current_state_.a_cross_horizontal < -0.5))
  {
    this->is_joy_vel_enabled_ = !this->is_joy_vel_enabled_;
    RCLCPP_INFO(this->get_logger(), "Aplotter - Is Joystick in velocity control: %u", is_joy_vel_enabled_);
  }

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

  if (is_joy_vel_enabled_)
  {
    // Note X axis if flipped to what is expected from joy
    this->x_vel_ = -max_velocity_ * this->joy_current_state_.a_left_stick_horizontal;
    this->y_vel_ = max_velocity_ * this->joy_current_state_.a_left_stick_vertical;
  }

  joy_previous_state_ = joy_current_state_;
}

void APlotterStatePublisher::odrive_get_encoder_estimates(int8_t axis)
{
  auto get_estimate_request = std::make_shared<ros2_odrive_can::srv::GetEncoderEstimates::Request>();
  get_estimate_request->axis = axis;

  // TODO: Capturing this in the lamda causes a seg fault. Scrappy fix below
  // auto result_future = odrive_get_encoder_estimates_client_->async_send_request(std::move(get_estimate_request), [this](rclcpp::Client<ros2_odrive_can::srv::GetEncoderEstimates>::SharedFutureWithRequest future) {
  //   if (future.get().first->axis == 0)
  //   {
  //     this->b_pos_rev_ = future.get().second->pos_estimate;
  //     this->b_vel_rev_s_ = future.get().second->vel_estimate;
  //   }
  //   else if (future.get().first->axis == 1)
  //   {
  //     this->a_pos_rev_ = future.get().second->pos_estimate;
  //     this->a_vel_rev_s_ = future.get().second->vel_estimate;
  //   }
  // });
  auto result_future = odrive_get_encoder_estimates_client_->async_send_request(std::move(get_estimate_request), std::bind(&APlotterStatePublisher::odrive_get_encoder_estimates_response, this, std::placeholders::_1));
}

void APlotterStatePublisher::odrive_get_encoder_estimates_response(rclcpp::Client<ros2_odrive_can::srv::GetEncoderEstimates>::SharedFuture future)
{
  if (future.get()->axis_response == 0)
  {
    this->b_pos_rev_ = future.get()->pos_estimate;
    this->b_vel_rev_s_ = future.get()->vel_estimate;
  }
  else if (future.get()->axis_response == 1)
  {
    this->a_pos_rev_ = future.get()->pos_estimate;
    this->a_vel_rev_s_ = future.get()->vel_estimate;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid axis on encoder estimates respone");
    return;
  }
}

void APlotterStatePublisher::odrive_set_input_velocity(int8_t axis, float velocity)
{
  auto input_vel_request = std::make_shared<ros2_odrive_can::srv::SetInputVel::Request>();

  input_vel_request->axis = axis;
  input_vel_request->input_vel = velocity;
  input_vel_request->current_ff = 0;
  auto result_future = odrive_set_input_vel_client_->async_send_request(input_vel_request);
}

void APlotterStatePublisher::odrive_set_requested_state(int8_t axis, int8_t state)
{
  RCLCPP_INFO(this->get_logger(), "Set Requested State - Axis: %i Requested State: %i", axis, state);
  auto request = std::make_shared<ros2_odrive_can::srv::SetAxisRequestedState::Request>();
  request->axis = axis;
  request->requested_state = state;

  // TODO: Change this to use SharedFutureWithRequest to set is_calibrated after getting return value
  // TODO: Capturing this is giving seg faults
  // auto response_received_callback = [this](rclcpp::Client<ros2_odrive_can::srv::SetAxisRequestedState>::SharedFutureWithRequest future) {
  //   if (future.get().second->success)
  //     this->odrive_axis_[future.get().first->axis].is_calibrated = true;
  //   RCLCPP_INFO(this->get_logger(), "ODrive Calibrate returned: %i", future.get().second->success);
  //   std::cout << "Service Response" << future.get().second->success;
  // };

  // auto future_result = odrive_set_axis_requested_state_client_->async_send_request(request, response_received_callback);
  auto future_result = odrive_set_axis_requested_state_client_->async_send_request(request);
}

// A button, toggle between closed loop and inactive(1)
void APlotterStatePublisher::odrive_toggle_closed_loop_control(int8_t axis)
{
  // TODO: Check that the axis is calibrated before entering closed loop control
  int requested_state = -1;
  if (odrive_axis_[axis].state == 1)
    requested_state = 8; // Closed Loop Control
  else if (odrive_axis_[axis].state == 8)
    requested_state = 1; // Idle

  if (requested_state < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Toggle Closed Loop Control Failed due to ODrive Axis: %i state", axis);
  }
  else
  {
    odrive_set_requested_state(axis, requested_state);
    RCLCPP_INFO(this->get_logger(), "Requesting State: %i for ODrive Axis: %i", requested_state, axis);
  }
}

// X button
void APlotterStatePublisher::odrive_estop()
{
  RCLCPP_WARN(this->get_logger(), "ODrive ESTOP");
  auto request = std::make_shared<ros2_odrive_can::srv::OdriveEstop::Request>();
  request->axis = 0;
  auto result_future = odrive_estop_client_->async_send_request(request);
}

// back button
void APlotterStatePublisher::odrive_clear_errors(int8_t axis)
{
  auto request = std::make_shared<ros2_odrive_can::srv::ClearErrors::Request>();
  request->axis = axis;
  auto result_future = odrive_clear_errors_client_->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "ODrive Clear Axis: %i A0 S: %i E: %i A1 S: %i E: %i", axis, odrive_axis_[0].state, odrive_axis_[0].error, odrive_axis_[1].state, odrive_axis_[1].error);
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
void APlotterStatePublisher::odrive_adjust_max_velocity(float amount)
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
  if (!send_commands_ || odrive_axis_[0].error || odrive_axis_[1].error || (odrive_axis_[0].state != 8) || (odrive_axis_[1].state != 8))
  {
    if (odrive_axis_[0].error || odrive_axis_[1].error || (odrive_axis_[0].state != 8) || (odrive_axis_[1].state != 8))
    {
      RCLCPP_WARN(this->get_logger(), "Could not send commands! A0 S: %i E: %i A1 S: %i E: %i", odrive_axis_[0].state, odrive_axis_[0].error, odrive_axis_[1].state, odrive_axis_[1].error);
      send_commands_ = false;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "ODrive Starting to Send Commands at %i Hz", params_.control_loop_frequency);
      send_commands_ = true;
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "ODrive No Longer Sending Commands");
    send_commands_ = false;
  }
}