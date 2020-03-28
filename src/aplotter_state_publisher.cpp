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

    ll_ = 261;
    lr_ = 280.19;
  }

private:
  void timer_callback()
  {
    msg_.header.stamp = ros_clock_.now();
    xl_ = 70 * std::sin(count_ * 0.02) - 180;
    xr_ = 70 * std::sin(count_ * -0.02) + 180;
    msg_.position[0] = xl_;
    msg_.position[1] = xr_;

    al_ = std::acos((std::pow(ll_, 2.0) + std::pow((xr_ - xl_), 2.0) - std::pow(lr_, 2.0)) / (2.0 * ll_ * (xr_ - xl_)));
    ar_ = M_PI - std::acos((std::pow(lr_, 2.0) + std::pow((xr_ - xl_), 2.0) - std::pow(ll_, 2.0)) / (2.0 * lr_ * (xr_ - xl_)));
    msg_.position[2] = al_;
    msg_.position[3] = ar_;
    count_++;
    publisher_->publish(msg_);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Clock ros_clock_;
  sensor_msgs::msg::JointState msg_;
  size_t count_;

  double xl_ = 0.0;
  double xr_ = 0.0;
  double al_ = 0.0;
  double ar_ = 0.0;
  double ll_ = 0.0;
  double lr_ = 0.0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APlotterStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
