#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "aplotter_ros2/aplotter_state_publisher.hpp"
#include "aplotter_ros2/joy_subscriber.hpp"


int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  auto server = std::make_shared<APlotterStatePublisher>();
  exec.add_node(server);

  auto publisher = std::make_shared<JoySubscriber>();
  exec.add_node(publisher);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
