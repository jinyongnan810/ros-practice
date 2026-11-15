#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using moveit::planning_interface::MoveGroupInterface;

namespace
{

constexpr double kHalfPi = 1.5707963267948966;

bool planAndExecute(
  MoveGroupInterface & move_group,
  const rclcpp::Logger & logger,
  const std::string & label)
{
  MoveGroupInterface::Plan plan;
  const auto plan_result = move_group.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "[%s] Planning failed (error code: %d).", label.c_str(), plan_result.val);
    return false;
  }

  RCLCPP_INFO(logger, "[%s] Plan succeeded. Executing...", label.c_str());
  const auto exec_result = move_group.execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "[%s] Execution failed (error code: %d).", label.c_str(), exec_result.val);
    return false;
  }

  RCLCPP_INFO(logger, "[%s] Execution succeeded.", label.c_str());
  return true;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "test_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto logger = node->get_logger();

  // MoveGroupInterface requires an active executor to receive /joint_states and action feedback.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  MoveGroupInterface arm(node, "arm");
  arm.setMaxVelocityScalingFactor(1.0);
  arm.setMaxAccelerationScalingFactor(1.0);

  RCLCPP_INFO(logger, "Planning frame: %s", arm.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", arm.getEndEffectorLink().c_str());

  // 1. Named goal (defined in config/six_axis_arm.srdf)
  arm.setStartStateToCurrentState();
  arm.setNamedTarget("home");
  planAndExecute(arm, logger, "Named Goal: home");

  // 2. Joint-space goal (joint_1 .. joint_6 in radians)
  const std::vector<double> joint_goal = {0.5, -0.7, 1.0, 0.0, -0.3, 0.0};
  arm.setStartStateToCurrentState();
  arm.setJointValueTarget(joint_goal);
  planAndExecute(arm, logger, "Joint Goal");

  // 3. Pose goal for tool0 (with +Z pointing forward along +X_world)
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, kHalfPi, 0.0);

  geometry_msgs::msg::Pose pose_goal;
  pose_goal.orientation = tf2::toMsg(orientation);
  pose_goal.position.x = 0.55;
  pose_goal.position.y = 0.15;
  pose_goal.position.z = 0.35;

  arm.setStartStateToCurrentState();
  arm.setPoseTarget(pose_goal);
  planAndExecute(arm, logger, "Pose Goal");
  arm.clearPoseTargets();

  // 4. Cartesian path from the current pose
  arm.setStartStateToCurrentState();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose waypoint = arm.getCurrentPose().pose;

  waypoint.position.z += 0.10;
  waypoints.push_back(waypoint);

  waypoint.position.y -= 0.30;
  waypoints.push_back(waypoint);

  waypoint.position.z -= 0.10;
  waypoints.push_back(waypoint);

  moveit_msgs::msg::RobotTrajectory trajectory;
  constexpr double kEefStep = 0.01;
  const double fraction = arm.computeCartesianPath(waypoints, kEefStep, trajectory);
  RCLCPP_INFO(logger, "[Cartesian Path] Computed %.1f%% of requested path.", fraction * 100.0);

  if (fraction >= 0.9) {
    MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory = trajectory;
    const auto exec_result = arm.execute(cartesian_plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "[Cartesian Path] Execution succeeded.");
    } else {
      RCLCPP_ERROR(logger, "[Cartesian Path] Execution failed (error code: %d).", exec_result.val);
    }
  } else {
    RCLCPP_WARN(logger, "[Cartesian Path] Skipping execution because fraction < 90%%.");
  }

  // Return to named home pose at the end
  arm.setStartStateToCurrentState();
  arm.setNamedTarget("home");
  planAndExecute(arm, logger, "Named Goal: home (return)");

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
