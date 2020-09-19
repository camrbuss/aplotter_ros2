#ifndef __APLOTTER_VELOCITY_PLANNER_HPP
#define __APLOTTER_VELOCITY_PLANNER_HPP
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <fstream>
#include <deque>

class APlotterVelocityPlanner : public rclcpp::Node
{

private:
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_velocity_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr aplotter_position_subscription_;

    geometry_msgs::msg::Pose desired_velocity_msg_;

    typedef struct coordinate_t
    {
        float x;
        float y;
    } coordinate;
    std::deque<coordinate_t> points_;
    float acceptable_error_ = 2.0;

    void position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    bool read_points(std::string filename);

public:
    APlotterVelocityPlanner(/* args */);
    ~APlotterVelocityPlanner();
};

#endif