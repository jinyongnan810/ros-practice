#ifndef ACTION_CPP_PKG__TURTLE_ACTION_SERVER_HPP_
#define ACTION_CPP_PKG__TURTLE_ACTION_SERVER_HPP_

#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "custom_interfaces/action/move_to_goal.hpp"
#include "action_cpp_pkg/visibility_control.h"

namespace action_cpp_pkg
{

class TurtleActionServerNode : public rclcpp::Node
{
public:
    using MoveToGoal = custom_interfaces::action::MoveToGoal;
    using GoalHandleMoveToGoal = rclcpp_action::ServerGoalHandle<MoveToGoal>;

    ACTION_CPP_PKG_PUBLIC
    explicit TurtleActionServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ACTION_CPP_PKG_PUBLIC
    virtual ~TurtleActionServerNode();

    ACTION_CPP_PKG_PUBLIC
    void stop_turtle();

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveToGoal::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveToGoal> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle);

    void execute(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle);

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp_action::Server<MoveToGoal>::SharedPtr action_server_;

    std::shared_ptr<GoalHandleMoveToGoal> active_goal_handle_{nullptr};
    std::mutex goal_mutex_;

    turtlesim::msg::Pose current_pose_;
    bool has_pose_{false};
};

}  // namespace action_cpp_pkg

#endif  // ACTION_CPP_PKG__TURTLE_ACTION_SERVER_HPP_
