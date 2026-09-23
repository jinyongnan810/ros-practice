#include "action_cpp_pkg/turtle_action_server.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_cpp_pkg
{

TurtleActionServerNode::TurtleActionServerNode(const rclcpp::NodeOptions & options)
: Node("turtle_action_server", options)
{
    // Coordinate boundary configuration (relaxed by default for open Gazebo worlds)
    this->declare_parameter<bool>("enforce_canvas_bounds", false);
    this->declare_parameter<double>("min_x", -20.0);
    this->declare_parameter<double>("max_x", 20.0);
    this->declare_parameter<double>("min_y", -20.0);
    this->declare_parameter<double>("max_y", 20.0);

    // Velocity message type: ROS 2 Jazzy uses TwistStamped on /cmd_vel for Gazebo Harmonic
    const char * distro_env = std::getenv("ROS_DISTRO");
    bool default_stamped = (distro_env == nullptr || std::string(distro_env) != "humble");
    this->declare_parameter<bool>("use_stamped_vel", default_stamped);
    use_stamped_vel_ = this->get_parameter("use_stamped_vel").as_bool();

    // 1. Publishers for velocity commands (dual support: Turtlesim & Gazebo / TurtleBot3)
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    if (use_stamped_vel_) {
        tb3_cmd_vel_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    } else {
        tb3_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    // 2. Subscribers for robot pose (dual support: Turtlesim Pose & Gazebo Odometry)
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10,
        std::bind(&TurtleActionServerNode::pose_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&TurtleActionServerNode::odom_callback, this, std::placeholders::_1));

    // 3. Action server
    action_server_ = rclcpp_action::create_server<MoveToGoal>(
        this,
        "move_to_goal",
        std::bind(&TurtleActionServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TurtleActionServerNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&TurtleActionServerNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Turtle Action Server component has been started! Action: /move_to_goal");
}

TurtleActionServerNode::~TurtleActionServerNode()
{
    stop_turtle();
}

void TurtleActionServerNode::stop_turtle()
{
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    if (use_stamped_vel_ && tb3_cmd_vel_stamped_pub_) {
        geometry_msgs::msg::TwistStamped stop_stamped;
        stop_stamped.header.stamp = this->now();
        stop_stamped.header.frame_id = "base_footprint";
        tb3_cmd_vel_stamped_pub_->publish(stop_stamped);
    } else if (tb3_cmd_vel_pub_) {
        tb3_cmd_vel_pub_->publish(stop_cmd);
    }
}

rclcpp_action::GoalResponse TurtleActionServerNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToGoal::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request: target=(%.2f, %.2f), velocity=%.2f",
                goal->target_x, goal->target_y, goal->linear_velocity);

    bool enforce_bounds = this->get_parameter("enforce_canvas_bounds").as_bool();
    double min_x = enforce_bounds ? 0.0 : this->get_parameter("min_x").as_double();
    double max_x = enforce_bounds ? 11.0 : this->get_parameter("max_x").as_double();
    double min_y = enforce_bounds ? 0.0 : this->get_parameter("min_y").as_double();
    double max_y = enforce_bounds ? 11.0 : this->get_parameter("max_y").as_double();

    // Check bounds
    if (goal->target_x < min_x || goal->target_x > max_x ||
        goal->target_y < min_y || goal->target_y > max_y)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Rejecting goal: target (%.2f, %.2f) out of bounds [%.1f, %.1f]!",
                    goal->target_x, goal->target_y, min_x, max_x);
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TurtleActionServerNode::handle_cancel(
    const std::shared_ptr<GoalHandleMoveToGoal> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TurtleActionServerNode::handle_accepted(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle)
{
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (active_goal_handle_ && active_goal_handle_->is_active())
        {
            RCLCPP_INFO(this->get_logger(), "Preempting previous active goal for new target...");
        }
        active_goal_handle_ = goal_handle;
    }

    // Execute asynchronously in a separate worker thread
    std::thread{std::bind(&TurtleActionServerNode::execute, this, goal_handle)}.detach();
}

void TurtleActionServerNode::execute(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToGoal::Feedback>();
    auto result = std::make_shared<MoveToGoal::Result>();

    float target_x = goal->target_x;
    float target_y = goal->target_y;
    float linear_velocity = goal->linear_velocity > 0.0f ? goal->linear_velocity : 2.0f;

    rclcpp::Rate loop_rate(10);
    auto start_time = this->now();
    float total_distance = 0.0f;
    auto prev_pose = current_pose_;

    RCLCPP_INFO(this->get_logger(), "Executing goal: moving to (%.2f, %.2f)...", target_x, target_y);

    while (rclcpp::ok())
    {
        // 1. Check if preempted by a newer goal request
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            if (goal_handle != active_goal_handle_)
            {
                RCLCPP_INFO(this->get_logger(), "Goal to (%.2f, %.2f) was preempted by a new goal.", target_x, target_y);
                result->success = false;
                result->total_distance = total_distance;
                result->elapsed_time = (this->now() - start_time).seconds();
                goal_handle->abort(result);
                return;
            }
        }

        // 2. Check if cancel was requested
        if (goal_handle->is_canceling())
        {
            stop_turtle();
            result->success = false;
            result->total_distance = total_distance;
            result->elapsed_time = (this->now() - start_time).seconds();
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled by client.");
            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                if (active_goal_handle_ == goal_handle)
                {
                    active_goal_handle_.reset();
                }
            }
            return;
        }

        // Wait until pose is available
        if (!has_pose_)
        {
            loop_rate.sleep();
            continue;
        }

        // 3. Accumulate traveled distance
        float dx = current_pose_.x - prev_pose.x;
        float dy = current_pose_.y - prev_pose.y;
        total_distance += std::hypot(dx, dy);
        prev_pose = current_pose_;

        // 4. Calculate distance and angle to target
        float delta_x = target_x - current_pose_.x;
        float delta_y = target_y - current_pose_.y;
        float distance = std::hypot(delta_x, delta_y);

        // 5. Check if goal is reached within tolerance
        if (distance < 0.1f)
        {
            stop_turtle();
            result->success = true;
            result->total_distance = total_distance;
            result->elapsed_time = (this->now() - start_time).seconds();
            goal_handle->succeed(result);
            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                if (active_goal_handle_ == goal_handle)
                {
                    active_goal_handle_.reset();
                }
            }
            RCLCPP_INFO(this->get_logger(), "Goal reached! Target: (%.2f, %.2f) | Distance: %.2fm | Time: %.2fs",
                        target_x, target_y, total_distance, result->elapsed_time);
            return;
        }

        // 6. Proportional steering and speed control
        float desired_heading = std::atan2(delta_y, delta_x);
        float heading_error = std::atan2(
            std::sin(desired_heading - current_pose_.theta),
            std::cos(desired_heading - current_pose_.theta));

        geometry_msgs::msg::Twist cmd;
        // Clamp angular velocity within physical limits (+-2.0 rad/s)
        cmd.angular.z = std::clamp(2.5f * heading_error, -2.0f, 2.0f);
        if (std::abs(heading_error) < 0.5f)
        {
            cmd.linear.x = std::min(linear_velocity, 1.5f * distance);
        }

        // Publish to Turtlesim (/turtle1/cmd_vel)
        cmd_vel_pub_->publish(cmd);

        // Publish to TurtleBot3 Gazebo (/cmd_vel)
        if (use_stamped_vel_ && tb3_cmd_vel_stamped_pub_) {
            geometry_msgs::msg::TwistStamped stamped_cmd;
            stamped_cmd.header.stamp = this->now();
            stamped_cmd.header.frame_id = "base_footprint";
            stamped_cmd.twist = cmd;
            tb3_cmd_vel_stamped_pub_->publish(stamped_cmd);
        } else if (tb3_cmd_vel_pub_) {
            tb3_cmd_vel_pub_->publish(cmd);
        }

        // 7. Publish feedback
        feedback->current_distance = distance;
        feedback->current_x = current_pose_.x;
        feedback->current_y = current_pose_.y;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }

    result->success = false;
    goal_handle->abort(result);
}

void TurtleActionServerNode::pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
{
    current_pose_ = *msg;
    has_pose_ = true;
}

void TurtleActionServerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pose_.x = static_cast<float>(msg->pose.pose.position.x);
    current_pose_.y = static_cast<float>(msg->pose.pose.position.y);

    // Convert orientation quaternion to 2D yaw angle (theta)
    const auto & q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    current_pose_.theta = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));

    has_pose_ = true;
}

}  // namespace action_cpp_pkg

RCLCPP_COMPONENTS_REGISTER_NODE(action_cpp_pkg::TurtleActionServerNode)
