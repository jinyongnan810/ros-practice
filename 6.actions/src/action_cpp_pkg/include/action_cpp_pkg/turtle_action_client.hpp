#ifndef ACTION_CPP_PKG__TURTLE_ACTION_CLIENT_HPP_
#define ACTION_CPP_PKG__TURTLE_ACTION_CLIENT_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/move_to_goal.hpp"
#include "action_cpp_pkg/visibility_control.h"

namespace action_cpp_pkg
{

class TurtleActionClientNode : public rclcpp::Node
{
public:
    using MoveToGoal = custom_interfaces::action::MoveToGoal;
    using GoalHandleMoveToGoal = rclcpp_action::ClientGoalHandle<MoveToGoal>;

    ACTION_CPP_PKG_PUBLIC
    explicit TurtleActionClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ACTION_CPP_PKG_PUBLIC
    void send_goal(float target_x, float target_y, float linear_velocity);

private:
    void timer_callback();

    void goal_response_callback(const GoalHandleMoveToGoal::SharedPtr & goal_handle);

    void feedback_callback(
        GoalHandleMoveToGoal::SharedPtr,
        const std::shared_ptr<const MoveToGoal::Feedback> feedback);

    void result_callback(const GoalHandleMoveToGoal::WrappedResult & result);

    rclcpp_action::Client<MoveToGoal>::SharedPtr client_;
    GoalHandleMoveToGoal::SharedPtr goal_handle_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr cancel_timer_;
};

}  // namespace action_cpp_pkg

#endif  // ACTION_CPP_PKG__TURTLE_ACTION_CLIENT_HPP_
