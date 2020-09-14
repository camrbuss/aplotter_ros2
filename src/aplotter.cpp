#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "aplotter_ros2/aplotter_state_publisher.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto aplotter_publisher = std::make_shared<APlotterStatePublisher>();
    exec.add_node(aplotter_publisher);
    exec.spin();
    rclcpp::shutdown();
    std::cout << "ROS2 Shutdown Smoothly\n";
    return 0;
}
