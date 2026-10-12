#ifndef ROBOT_CONTROLLERS__WHEEL_POSITION_CONTROLLER_HPP_
#define ROBOT_CONTROLLERS__WHEEL_POSITION_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robot_controllers
{

/**
 * @brief Forward position targets to one joint through ros2_control.
 *
 * The `joints` parameter must contain exactly one name. The controller claims
 * that joint's position command and state interfaces and accepts one target in
 * radians in a Float64MultiArray on ~/commands. This preserves the command
 * format used by JointGroupPositionController and the position command GUI.
 *
 * Activation initializes the target from measured feedback. Subsequent updates
 * hold the latest accepted target; this controller does not interpolate a
 * trajectory or calculate motor effort. Position tracking is provided by the
 * hardware interface or Gazebo backend.
 */
class WheelPositionController : public controller_interface::ControllerInterface
{
public:
  /// Declare configuration parameters when the controller manager loads the plugin.
  controller_interface::CallbackReturn on_init() override;
  /// Request exclusive command access to the configured joint's position interface.
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  /// Request measured position feedback to initialize the activation hold target.
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  /// Validate the joint parameter and create the command subscription while inactive.
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  /// Validate loaned interfaces, hold the measured position, and begin accepting targets.
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  /// Stop accepting targets and writing commands; hardware torque is managed separately.
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  /// Release the subscription so a subsequent configure can recreate it.
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  /// Write the current target in the control loop; report failure to the controller manager.
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // Tag targets with their activation so buffered commands cannot cross reactivation.
  struct Command
  {
    double position{0.0};  // Absolute joint angle in radians.
    uint64_t epoch{0};  // Zero denotes a command from before the first activation.
  };

  // Full resource name (e.g. wheel_joint/position), resolved during configure.
  std::string interface_name_;
  // Initialized during activation, then read and updated only by the control loop.
  double target_position_{0.0};
  // Shared by lifecycle callbacks, the subscriber, and the control loop.
  std::atomic<bool> accepting_commands_{false};
  // Each activation invalidates targets received during earlier activations.
  std::atomic<uint64_t> activation_epoch_{0};
  // The subscriber writes outside the control loop; update() reads without waiting
  // for the subscriber to finish. Keep the payload fixed-size to avoid allocations.
  realtime_tools::RealtimeBuffer<Command> command_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscription_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS__WHEEL_POSITION_CONTROLLER_HPP_
