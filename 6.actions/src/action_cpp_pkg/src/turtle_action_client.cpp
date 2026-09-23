#include "action_cpp_pkg/turtle_action_client.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_cpp_pkg
{

TurtleActionClientNode::TurtleActionClientNode(const rclcpp::NodeOptions & options)
: Node("turtle_action_client", options)
{
    // Declare parameters
    this->declare_parameter<double>("target_x", 8.5);
    this->declare_parameter<double>("target_y", 8.5);
    this->declare_parameter<double>("linear_velocity", 2.0);
    this->declare_parameter<double>("cancel_after_sec", 0.0);

    // Create action client
    client_ = rclcpp_action::create_client<MoveToGoal>(this, "move_to_goal");

    RCLCPP_INFO(this->get_logger(), "Turtle Action Client component has been started!");

    // Trigger goal sending after a brief initialization delay
    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&TurtleActionClientNode::timer_callback, this));
}

void TurtleActionClientNode::send_goal(float target_x, float target_y, float linear_velocity)
{
    RCLCPP_INFO(this->get_logger(), "Waiting for action server '/move_to_goal'...");
    while (!client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action server.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Action server not available, waiting again...");
    }

    auto goal_msg = MoveToGoal::Goal();
    goal_msg.target_x = target_x;
    goal_msg.target_y = target_y;
    goal_msg.linear_velocity = linear_velocity;

    RCLCPP_INFO(this->get_logger(), "Sending goal: target=(%.2f, %.2f), velocity=%.2f",
                target_x, target_y, linear_velocity);

    auto send_goal_options = rclcpp_action::Client<MoveToGoal>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&TurtleActionClientNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&TurtleActionClientNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&TurtleActionClientNode::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_goal_options);
}

void TurtleActionClientNode::timer_callback()
{
    init_timer_->cancel();
    float target_x = this->get_parameter("target_x").as_double();
    float target_y = this->get_parameter("target_y").as_double();
    float linear_velocity = this->get_parameter("linear_velocity").as_double();
    send_goal(target_x, target_y, linear_velocity);
}

void TurtleActionClientNode::goal_response_callback(const GoalHandleMoveToGoal::SharedPtr & goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_WARN(this->get_logger(), "Goal was REJECTED by server!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal ACCEPTED by server! Navigating to target...");
    goal_handle_ = goal_handle;

    double cancel_after_sec = this->get_parameter("cancel_after_sec").as_double();
    if (cancel_after_sec > 0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Will request cancellation in %.1fs...", cancel_after_sec);
        cancel_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(cancel_after_sec),
            [this]() {
                cancel_timer_->cancel();
                if (goal_handle_)
                {
                    RCLCPP_INFO(this->get_logger(), "Sending cancel request to action server...");
                    client_->async_cancel_goal(goal_handle_);
                }
            });
    }
}

void TurtleActionClientNode::feedback_callback(
    GoalHandleMoveToGoal::SharedPtr,
    const std::shared_ptr<const MoveToGoal::Feedback> feedback)
{
    RCLCPP_INFO(
        this->get_logger(),
        "Feedback: remaining distance=%.2fm, current_pos=(%.2f, %.2f)",
        feedback->current_distance, feedback->current_x, feedback->current_y);
}

void TurtleActionClientNode::result_callback(const GoalHandleMoveToGoal::WrappedResult & result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(
                this->get_logger(),
                "Goal SUCCEEDED! Total distance: %.2fm, Elapsed time: %.2fs",
                result.result->total_distance, result.result->elapsed_time);
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(
                this->get_logger(),
                "Goal was CANCELED! Partial distance: %.2fm, Elapsed time: %.2fs",
                result.result->total_distance, result.result->elapsed_time);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(
                this->get_logger(),
                "Goal was ABORTED (preempted by another goal)! Partial distance: %.2fm",
                result.result->total_distance);
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Goal terminated with unknown status code.");
            break;
    }
}

}  // namespace action_cpp_pkg

RCLCPP_COMPONENTS_REGISTER_NODE(action_cpp_pkg::TurtleActionClientNode)
