#ifndef __APLOTTER_VELOCITY_PLANNER_HPP
#define __APLOTTER_VELOCITY_PLANNER_HPP
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <fstream>
#include <deque>

class APlotterVelocityPlanner : public rclcpp::Node
{

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_velocity_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr aplotter_goal_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr aplotter_position_subscription_;

    geometry_msgs::msg::PoseStamped desired_velocity_msg_;
    geometry_msgs::msg::PointStamped goal_msg_;

    typedef struct coordinate_t
    {
        float x;
        float y;
    } coordinate;
    std::deque<coordinate_t> points_;
    float acceptable_error_ = 2.0;

    rclcpp::Clock ros_clock_;

    void position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    bool read_points(std::string filename);

public:
    APlotterVelocityPlanner(/* args */);
    ~APlotterVelocityPlanner();
};

#endif