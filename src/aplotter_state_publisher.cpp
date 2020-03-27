#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class APlotterStatePublisher : public rclcpp::Node
{
public:
  APlotterStatePublisher()
      : Node("aplotter_state_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = this->create_wall_timer(
        20ms, std::bind(&APlotterStatePublisher::timer_callback, this));
    msg_.name.push_back("leftPJoint");
    msg_.name.push_back("rightPJoint");
    msg_.name.push_back("leftJoint");
    msg_.name.push_back("rightJoint");
    msg_.position.push_back(0.0);
    msg_.position.push_back(0.0);
    msg_.position.push_back(0.0);
    msg_.position.push_back(0.0);
  }

private:
  void timer_callback()
  {
    msg_.header.stamp = ros_clock_.now();
    msg_.position[0] = 50 * std::sin(count_ * 0.01) - 200;
    msg_.position[1] = 50 * std::sin(count_ * 0.02) + 200;
    msg_.position[2] = std::sin(count_ * 0.01);
    msg_.position[3] = std::sin(count_ * 0.02);
    count_++;
    publisher_->publish(msg_);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Clock ros_clock_;
  sensor_msgs::msg::JointState msg_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APlotterStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
