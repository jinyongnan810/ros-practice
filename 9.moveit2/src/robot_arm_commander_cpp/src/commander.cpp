#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_arm_interface/action/control_gripper.hpp>
#include <robot_arm_interface/action/move_arm.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using moveit::planning_interface::MoveGroupInterface;
using MoveArm = robot_arm_interface::action::MoveArm;
using ControlGripper = robot_arm_interface::action::ControlGripper;
using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;
using GoalHandleControlGripper = rclcpp_action::ServerGoalHandle<ControlGripper>;

class Commander
{
public:
  explicit Commander(const std::shared_ptr<rclcpp::Node> &node)
      : node_(node),
        arm_(std::make_shared<MoveGroupInterface>(node_, "arm")),
        gripper_(std::make_shared<MoveGroupInterface>(node_, "gripper"))
  {
    arm_->setMaxVelocityScalingFactor(1.0);
    arm_->setMaxAccelerationScalingFactor(1.0);
    gripper_->setMaxVelocityScalingFactor(1.0);
    gripper_->setMaxAccelerationScalingFactor(1.0);

    move_arm_server_ = rclcpp_action::create_server<MoveArm>(
        node_,
        "move_arm",
        std::bind(
            &Commander::handleMoveArmGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Commander::handleMoveArmCancel, this, std::placeholders::_1),
        std::bind(&Commander::handleMoveArmAccepted, this, std::placeholders::_1));

    control_gripper_server_ = rclcpp_action::create_server<ControlGripper>(
        node_,
        "control_gripper",
        std::bind(
            &Commander::handleControlGripperGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Commander::handleControlGripperCancel, this, std::placeholders::_1),
        std::bind(&Commander::handleControlGripperAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(
        node_->get_logger(),
        "Commander ready. Action servers: /move_arm, /control_gripper");
  }

  bool goToNamedTarget(const std::string &name)
  {
    arm_->setStartStateToCurrentState();
    if (!arm_->setNamedTarget(name))
    {
      RCLCPP_ERROR(node_->get_logger(), "Unknown arm named target: '%s'.", name.c_str());
      return false;
    }
    return planAndExecute(*arm_, "Arm Named Goal: " + name);
  }

  bool goToJointTarget(const std::vector<double> &joints)
  {
    arm_->setStartStateToCurrentState();
    if (!arm_->setJointValueTarget(joints))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to set arm joint value target.");
      return false;
    }
    return planAndExecute(*arm_, "Arm Joint Goal");
  }

  bool goToPoseTarget(
      double x,
      double y,
      double z,
      double roll,
      double pitch,
      double yaw,
      bool cartesian_path = false)
  {
    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    return goToPoseTarget(target_pose, cartesian_path);
  }

  bool goToPoseTarget(const geometry_msgs::msg::Pose &pose, bool cartesian_path = false)
  {
    if (!cartesian_path)
    {
      arm_->setStartStateToCurrentState();
      arm_->setPoseTarget(pose);
      const bool success = planAndExecute(*arm_, "Arm Pose Goal");
      arm_->clearPoseTargets();
      return success;
    }

    return executeCartesianPath({pose});
  }

  bool goToGripperNamedTarget(const std::string &name)
  {
    gripper_->setStartStateToCurrentState();
    if (!gripper_->setNamedTarget(name))
    {
      RCLCPP_ERROR(node_->get_logger(), "Unknown gripper named target: '%s'.", name.c_str());
      return false;
    }
    return planAndExecute(*gripper_, "Gripper Named Goal: " + name);
  }

  bool goToGripperJointTarget(double position)
  {
    gripper_->setStartStateToCurrentState();
    // Target left_finger_joint by name so MoveIt handles the mimic right_finger_joint.
    if (!gripper_->setJointValueTarget("left_finger_joint", position))
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to set gripper joint value target (position: %.4f).", position);
      return false;
    }
    return planAndExecute(*gripper_, "Gripper Joint Goal");
  }

  bool openGripper()
  {
    return goToGripperNamedTarget("open");
  }

  bool closeGripper()
  {
    return goToGripperNamedTarget("close");
  }

private:
  template <typename ActionT>
  void finalizeGoal(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> &goal_handle,
      bool success,
      const std::string &success_msg,
      const std::string &failure_msg)
  {
    auto result = std::make_shared<typename ActionT::Result>();
    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "Goal canceled.";
      goal_handle->canceled(result);
      return;
    }

    result->success = success;
    result->message = success ? success_msg : failure_msg;
    if (success)
    {
      goal_handle->succeed(result);
    }
    else
    {
      goal_handle->abort(result);
    }
  }

  rclcpp_action::GoalResponse handleMoveArmGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveArm::Goal> goal)
  {
    (void)uuid;
    switch (goal->mode)
    {
    case MoveArm::Goal::MODE_NAMED:
      if (goal->named_target.empty())
      {
        RCLCPP_WARN(node_->get_logger(), "Rejecting MoveArm goal: empty named_target.");
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    case MoveArm::Goal::MODE_JOINT:
      if (goal->joint_positions.size() != arm_->getVariableCount())
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "Rejecting MoveArm goal: expected %u joint positions, got %zu.",
            arm_->getVariableCount(), goal->joint_positions.size());
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    case MoveArm::Goal::MODE_POSE:
    {
      const auto &q = goal->target_pose.orientation;
      const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (!std::isfinite(norm_sq) || norm_sq < 1e-6)
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "Rejecting MoveArm goal: invalid target_pose orientation quaternion.");
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    default:
      RCLCPP_WARN(node_->get_logger(), "Rejecting MoveArm goal: unknown mode %u.", goal->mode);
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse handleMoveArmCancel(
      const std::shared_ptr<GoalHandleMoveArm> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(node_->get_logger(), "Canceling MoveArm goal...");
    arm_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleMoveArmAccepted(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
  {
    std::thread{std::bind(&Commander::executeMoveArm, this, goal_handle)}.detach();
  }

  void executeMoveArm(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
  {
    std::lock_guard<std::mutex> lock(arm_mutex_);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveArm::Feedback>();

    feedback->status = "Planning and executing arm motion...";
    goal_handle->publish_feedback(feedback);

    bool success = false;
    switch (goal->mode)
    {
    case MoveArm::Goal::MODE_NAMED:
      success = goToNamedTarget(goal->named_target);
      break;
    case MoveArm::Goal::MODE_JOINT:
      success = goToJointTarget(goal->joint_positions);
      break;
    case MoveArm::Goal::MODE_POSE:
      success = goToPoseTarget(goal->target_pose, goal->cartesian_path);
      break;
    default:
      success = false;
      break;
    }

    finalizeGoal(
        goal_handle, success, "Arm motion succeeded.", "Arm planning or execution failed.");
  }

  rclcpp_action::GoalResponse handleControlGripperGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const ControlGripper::Goal> goal)
  {
    (void)uuid;
    switch (goal->mode)
    {
    case ControlGripper::Goal::MODE_NAMED:
      if (goal->named_target.empty())
      {
        RCLCPP_WARN(
            node_->get_logger(), "Rejecting ControlGripper goal: empty named_target.");
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    case ControlGripper::Goal::MODE_JOINT:
      if (!std::isfinite(goal->position))
      {
        RCLCPP_WARN(
            node_->get_logger(), "Rejecting ControlGripper goal: non-finite position.");
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    default:
      RCLCPP_WARN(
          node_->get_logger(), "Rejecting ControlGripper goal: unknown mode %u.", goal->mode);
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse handleControlGripperCancel(
      const std::shared_ptr<GoalHandleControlGripper> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(node_->get_logger(), "Canceling ControlGripper goal...");
    gripper_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleControlGripperAccepted(const std::shared_ptr<GoalHandleControlGripper> goal_handle)
  {
    std::thread{std::bind(&Commander::executeControlGripper, this, goal_handle)}.detach();
  }

  void executeControlGripper(const std::shared_ptr<GoalHandleControlGripper> goal_handle)
  {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ControlGripper::Feedback>();

    feedback->status = "Planning and executing gripper motion...";
    goal_handle->publish_feedback(feedback);

    bool success = false;
    switch (goal->mode)
    {
    case ControlGripper::Goal::MODE_NAMED:
      success = goToGripperNamedTarget(goal->named_target);
      break;
    case ControlGripper::Goal::MODE_JOINT:
      success = goToGripperJointTarget(goal->position);
      break;
    default:
      success = false;
      break;
    }

    finalizeGoal(
        goal_handle, success, "Gripper motion succeeded.",
        "Gripper planning or execution failed.");
  }

  bool planAndExecute(MoveGroupInterface &move_group, const std::string &label)
  {
    MoveGroupInterface::Plan plan;
    const auto plan_result = move_group.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
          node_->get_logger(), "[%s] Planning failed (error code: %d).", label.c_str(),
          plan_result.val);
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Plan succeeded. Executing...", label.c_str());
    const auto exec_result = move_group.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
          node_->get_logger(), "[%s] Execution failed (error code: %d).", label.c_str(),
          exec_result.val);
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Execution succeeded.", label.c_str());
    return true;
  }

  bool executeCartesianPath(
      const std::vector<geometry_msgs::msg::Pose> &waypoints,
      double eef_step = 0.01,
      double min_fraction = 0.9)
  {
    arm_->setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm_->computeCartesianPath(waypoints, eef_step, trajectory);
    RCLCPP_INFO(
        node_->get_logger(), "[Cartesian Path] Computed %.1f%% of requested path.",
        fraction * 100.0);

    if (fraction < min_fraction)
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "[Cartesian Path] Skipping execution because fraction < %.0f%%.", min_fraction * 100.0);
      return false;
    }

    MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory = trajectory;
    const auto exec_result = arm_->execute(cartesian_plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
          node_->get_logger(), "[Cartesian Path] Execution failed (error code: %d).",
          exec_result.val);
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[Cartesian Path] Execution succeeded.");
    return true;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MoveGroupInterface> arm_;
  std::shared_ptr<MoveGroupInterface> gripper_;
  std::mutex arm_mutex_;
  std::mutex gripper_mutex_;
  rclcpp_action::Server<MoveArm>::SharedPtr move_arm_server_;
  rclcpp_action::Server<ControlGripper>::SharedPtr control_gripper_server_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "commander",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  Commander commander(node);
  // MoveGroupInterface requires the node to spin while planning/executing.
  // std::thread worker([&commander, node]() {
  //     commander.goToNamedTarget("home");
  //     commander.openGripper();
  //     commander.goToJointTarget({0.5, -0.7, 1.0, 0.0, -0.3, 0.0});
  //     commander.closeGripper();
  //     commander.goToPoseTarget(0.55, 0.15, 0.35, 0.0, kHalfPi, 0.0, false);
  //     commander.goToPoseTarget(0.55, -0.15, 0.35, 0.0, kHalfPi, 0.0, true);
  //     commander.openGripper();
  //     commander.goToNamedTarget("home");

  //     RCLCPP_INFO(node->get_logger(), "Commander demo sequence finished.");
  //   });

  rclcpp::spin(node);
  // if (worker.joinable()) {
  //   worker.join();
  // }
  rclcpp::shutdown();
  return 0;
}
