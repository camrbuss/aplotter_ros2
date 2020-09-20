#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "aplotter_ros2/aplotter_state_publisher.hpp"
#include "aplotter_ros2/aplotter_velocity_planner.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto aplotter_publisher = std::make_shared<APlotterStatePublisher>();
    exec.add_node(aplotter_publisher);
    auto aplotter_velocity_planner = std::make_shared<APlotterVelocityPlanner>();
    exec.add_node(aplotter_velocity_planner);
    exec.spin();
    rclcpp::shutdown();
    std::cout << "ROS2 Shutdown Smoothly\n";
    return 0;
}
