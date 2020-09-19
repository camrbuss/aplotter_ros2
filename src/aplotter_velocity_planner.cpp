#include "aplotter_ros2/aplotter_velocity_planner.hpp"

APlotterVelocityPlanner::APlotterVelocityPlanner(/* args */) : Node("aplotter_velocity_planner")
{
    this->declare_parameter<std::string>("coordinates_file", "launch/points.csv");
    this->declare_parameter<float>("aplotter_acceptable_error", 0.5);
    std::string file;
    this->get_parameter("coordinates_file", file);
    this->get_parameter("aplotter_acceptable_error", acceptable_error_);

    if (!this->read_points(file))
    {
        RCLCPP_FATAL(this->get_logger(), "APlotterVelocityPlanner failed to parse points file");
        this->~APlotterVelocityPlanner();
    }

    desired_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("aplotter/aplotter_desired_velocity", 10);
    aplotter_position_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>("aplotter/aplotter_position", 10, std::bind(&APlotterVelocityPlanner::position_callback, this, std::placeholders::_1));
}

APlotterVelocityPlanner::~APlotterVelocityPlanner()
{
}

bool APlotterVelocityPlanner::read_points(std::string filename)
{
    std::ifstream input_data_file_stream(filename, std::ifstream::in);
    std::string line;
    std::string value;
    while (std::getline(input_data_file_stream, line))
    {
        if (line.find(',') == std::string::npos)
        {
            return false;
        }
        std::istringstream valuess(line);
        bool is_first = true;
        APlotterVelocityPlanner::coordinate_t coord;
        while (std::getline(valuess, value, ','))
        {
            if (is_first)
            {
                coord.x = std::atof(value.c_str());
                is_first = false;
            }
            else
            {
                coord.y = std::atof(value.c_str());
            }
        }
        points_.push_back(coord);
        RCLCPP_INFO(this->get_logger(), "APlotterVelocityPlanner Point - x: %.3f y: %.3f", points_.back().x, points_.back().y);
    }
    input_data_file_stream.close();
    return true;
}

void APlotterVelocityPlanner::position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // Set Pose position to the same as the point of the end effector
    desired_velocity_msg_.position.x = msg->point.x;
    desired_velocity_msg_.position.y = msg->point.y;
    desired_velocity_msg_.position.z = 0;

    RCLCPP_INFO(this->get_logger(), "Current Position: X: %+.3f Y: %+.3f Goal Position: X: %+.3f Y: %+.3f", msg->point.x, msg->point.y, points_.front().x, points_.front().y);
    // Get vector pointing towards where we want to go
    float dx = points_.front().x - msg->point.x;
    float dy = points_.front().y - msg->point.y;
    float dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    // Normalize the vectors
    // desired_velocity_msg_.x = dx / dist;
    // desired_velocity_msg_.y = dy / dist;
    // desired_velocity_msg_.z = 0;
    desired_velocity_msg_.orientation.x = dx / dist;
    desired_velocity_msg_.orientation.y = dy / dist;
    desired_velocity_msg_.orientation.z = 0;
    desired_velocity_msg_.orientation.w = 0;

    // If we are not at the last point and we are within the acceptable error, get the next point
    if (points_.size() > 1 && dist < acceptable_error_)
    {
        points_.pop_front();
        RCLCPP_INFO(this->get_logger(), "APlotterVelocityPlanner - Moved to next point");
    }

    // If we are at the last point and we are in the acceptable error, send no velocity
    if (points_.size() == 1 && dist < acceptable_error_)
    {
        RCLCPP_INFO(this->get_logger(), "APlotterVelocityPlanner - Goal Reached!");
        desired_velocity_msg_.orientation.x = 0;
        desired_velocity_msg_.orientation.y = 0;
        desired_velocity_msg_.orientation.z = 0;
        desired_velocity_msg_.orientation.w = 0;
    }

    desired_velocity_publisher_->publish(desired_velocity_msg_);
}