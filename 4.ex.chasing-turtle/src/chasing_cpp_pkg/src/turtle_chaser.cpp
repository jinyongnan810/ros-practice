#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "chasing_interfaces/msg/target_positions.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleChaser : public rclcpp::Node
{
public:
    TurtleChaser() : Node("turtle_chaser")
    {
        target_subscription_ = create_subscription<chasing_interfaces::msg::TargetPositions>(
            "/spawned_target_positions", 10,
            [this](const chasing_interfaces::msg::TargetPositions::SharedPtr message)
            { target_positions_ = *message; });
        pose_subscription_ = create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleChaser::handle_pose, this, std::placeholders::_1));
        velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

private:
    void handle_pose(const turtlesim::msg::Pose::SharedPtr pose)
    {
        geometry_msgs::msg::Twist command;
        if (target_positions_.targets.empty())
        {
            selected_target_name_.clear();
            velocity_publisher_->publish(command);
            return;
        }

        auto selected_target = std::find_if(
            target_positions_.targets.begin(), target_positions_.targets.end(),
            [this](const auto &target)
            { return target.name == selected_target_name_; });

        if (selected_target == target_positions_.targets.end())
        {
            selected_target = std::min_element(
                target_positions_.targets.begin(), target_positions_.targets.end(),
                [&pose](const auto &left, const auto &right)
                {
                    const auto left_x = left.position.x - pose->x;
                    const auto left_y = left.position.y - pose->y;
                    const auto right_x = right.position.x - pose->x;
                    const auto right_y = right.position.y - pose->y;
                    return left_x * left_x + left_y * left_y < right_x * right_x + right_y * right_y;
                });
            selected_target_name_ = selected_target->name;
            RCLCPP_INFO(get_logger(), "Selected target %s", selected_target_name_.c_str());
        }

        const auto delta_x = selected_target->position.x - pose->x;
        const auto delta_y = selected_target->position.y - pose->y;
        const auto distance = std::hypot(delta_x, delta_y);
        const auto desired_heading = std::atan2(delta_y, delta_x);
        const auto heading_error = std::atan2(
            std::sin(desired_heading - pose->theta), std::cos(desired_heading - pose->theta));

        command.angular.z = 4.0 * heading_error;
        if (std::abs(heading_error) < 0.5)
        {
            command.linear.x = std::min(2.0, 1.5 * distance);
        }
        velocity_publisher_->publish(command);
    }

    rclcpp::Subscription<chasing_interfaces::msg::TargetPositions>::SharedPtr target_subscription_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    chasing_interfaces::msg::TargetPositions target_positions_;
    std::string selected_target_name_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleChaser>());
    rclcpp::shutdown();
    return 0;
}