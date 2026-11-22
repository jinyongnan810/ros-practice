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
}  // namespace

class Commander
{
public:
  explicit Commander(const std::shared_ptr<rclcpp::Node> & node)
  : node_(node),
    arm_(std::make_shared<MoveGroupInterface>(node_, "arm")),
    gripper_(std::make_shared<MoveGroupInterface>(node_, "gripper"))
  {
    arm_->setMaxVelocityScalingFactor(1.0);
    arm_->setMaxAccelerationScalingFactor(1.0);
    gripper_->setMaxVelocityScalingFactor(1.0);
    gripper_->setMaxAccelerationScalingFactor(1.0);
  }

  bool goToNamedTarget(const std::string & name)
  {
    arm_->setStartStateToCurrentState();
    arm_->setNamedTarget(name);
    return planAndExecute(*arm_, "Arm Named Goal: " + name);
  }

  bool goToJointTarget(const std::vector<double> & joints)
  {
    arm_->setStartStateToCurrentState();
    arm_->setJointValueTarget(joints);
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

  bool goToPoseTarget(const geometry_msgs::msg::Pose & pose, bool cartesian_path = false)
  {
    if (!cartesian_path) {
      arm_->setStartStateToCurrentState();
      arm_->setPoseTarget(pose);
      const bool success = planAndExecute(*arm_, "Arm Pose Goal");
      arm_->clearPoseTargets();
      return success;
    }

    return executeCartesianPath({pose});
  }

  bool goToGripperNamedTarget(const std::string & name)
  {
    gripper_->setStartStateToCurrentState();
    gripper_->setNamedTarget(name);
    return planAndExecute(*gripper_, "Gripper Named Goal: " + name);
  }

  bool goToGripperJointTarget(double position)
  {
    gripper_->setStartStateToCurrentState();
    gripper_->setJointValueTarget(std::vector<double>{position});
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
  bool planAndExecute(MoveGroupInterface & move_group, const std::string & label)
  {
    MoveGroupInterface::Plan plan;
    const auto plan_result = move_group.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        node_->get_logger(), "[%s] Planning failed (error code: %d).", label.c_str(),
        plan_result.val);
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Plan succeeded. Executing...", label.c_str());
    const auto exec_result = move_group.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        node_->get_logger(), "[%s] Execution failed (error code: %d).", label.c_str(),
        exec_result.val);
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Execution succeeded.", label.c_str());
    return true;
  }

  bool executeCartesianPath(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double eef_step = 0.01,
    double min_fraction = 0.9)
  {
    arm_->setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm_->computeCartesianPath(waypoints, eef_step, trajectory);
    RCLCPP_INFO(
      node_->get_logger(), "[Cartesian Path] Computed %.1f%% of requested path.",
      fraction * 100.0);

    if (fraction < min_fraction) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[Cartesian Path] Skipping execution because fraction < %.0f%%.", min_fraction * 100.0);
      return false;
    }

    MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory = trajectory;
    const auto exec_result = arm_->execute(cartesian_plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "commander",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  Commander commander(node);

  // MoveGroupInterface requires the node to spin while planning/executing.
  std::thread worker([&commander, node]() {
      commander.goToNamedTarget("home");
      commander.openGripper();
      commander.goToJointTarget({0.5, -0.7, 1.0, 0.0, -0.3, 0.0});
      commander.closeGripper();
      commander.goToPoseTarget(0.55, 0.15, 0.35, 0.0, kHalfPi, 0.0, false);
      commander.goToPoseTarget(0.55, -0.15, 0.35, 0.0, kHalfPi, 0.0, true);
      commander.openGripper();
      commander.goToNamedTarget("home");

      RCLCPP_INFO(node->get_logger(), "Commander demo sequence finished.");
    });

  rclcpp::spin(node);
  if (worker.joinable()) {
    worker.join();
  }
  rclcpp::shutdown();
  return 0;
}
