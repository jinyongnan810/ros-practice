#include "robot_controllers/wheel_position_controller.hpp"

#include <cmath>
#include <exception>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace robot_controllers
{

controller_interface::CallbackReturn WheelPositionController::on_init()
{
  try
  {
    // Match the standard position controller parameter so existing YAML can be reused.
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<double>("coefficient", 0.05);
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
WheelPositionController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, {interface_name_}};
}

controller_interface::InterfaceConfiguration
WheelPositionController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, {interface_name_}};
}

controller_interface::CallbackReturn WheelPositionController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  accepting_commands_.store(false);
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  if (joints.size() != 1 || joints.front().empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' must contain exactly one nonempty joint name.");
    return controller_interface::CallbackReturn::ERROR;
  }
  // Cache the blend weight during configuration, outside the realtime update loop.
  coefficient_ = get_node()->get_parameter("coefficient").as_double();
  if (!std::isfinite(coefficient_) || coefficient_ < 0.0 || coefficient_ > 1.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'coefficient' must be finite and in [0, 1].");
    return controller_interface::CallbackReturn::ERROR;
  }
  // The controller manager resolves these names and loans the interfaces on activation.
  interface_name_ = joints.front() + "/position";
  // The private topic resolves to /<controller_name>/commands. A depth of one
  // favors the most recent target rather than accumulating a queue of setpoints.
  command_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::QoS(1),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr message)
    {
      // Capture the activation before checking the gate. If a lifecycle transition
      // overlaps this callback, update() can reject its target using this tag.
      const auto epoch = activation_epoch_.load();
      if (!accepting_commands_.load())
      {
        return;
      }
      // Reject the whole message; retain the last valid target on malformed input.
      if (message->data.size() != 1 || !std::isfinite(message->data.front()))
      {
        RCLCPP_WARN(get_node()->get_logger(), "Ignoring command: expected one finite position.");
        return;
      }
      // Transfer only the validated scalar; never write hardware from the subscriber.
      command_buffer_.writeFromNonRT(Command{message->data.front(), epoch});
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WheelPositionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  accepting_commands_.store(false);
  // Refuse activation if the manager did not loan the exact resources requested.
  if (command_interfaces_.size() != 1 || state_interfaces_.size() != 1 ||
      command_interfaces_.front().get_name() != interface_name_ ||
      state_interfaces_.front().get_name() != interface_name_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected command and state interfaces for %s.",
      interface_name_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  // A zero target could move the platform unexpectedly. Start from feedback and
  // fail activation if that feedback is unavailable or not finite.
  const auto position = state_interfaces_.front().get_optional<double>();
  if (!position.has_value() || !std::isfinite(*position))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot activate without a finite measured position.");
    return controller_interface::CallbackReturn::ERROR;
  }
  target_position_ = *position;
  if (!command_interfaces_.front().set_value(target_position_))
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  // Invalidate buffered targets from previous activations without resetting a
  // buffer that a subscriber callback may still be writing.
  activation_epoch_.fetch_add(1);
  accepting_commands_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WheelPositionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Keep the last hardware target intact. Deactivation of this controller does
  // not disable motor torque; that belongs to the hardware component lifecycle.
  accepting_commands_.store(false);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WheelPositionController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  accepting_commands_.store(false);
  command_subscription_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type WheelPositionController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!accepting_commands_.load())
  {
    return controller_interface::return_type::OK;
  }
  // readFromRT() keeps the previous buffered value if the subscriber is writing.
  // Until a target from this activation arrives, retain the measured hold target.
  const auto * command = command_buffer_.readFromRT();
  if (command && command->epoch == activation_epoch_.load())
  {
    target_position_ = command->position;
  }
  // Blend the requested target with fresh measured feedback, not the previous
  // output command. Preserve target_position_ so repeated updates approach it.
  const auto position = state_interfaces_.front().get_optional<double>();
  if (!position.has_value() || !std::isfinite(*position))
  {
    return controller_interface::return_type::ERROR;
  }
  const double blended_position =
    coefficient_ * target_position_ + (1.0 - coefficient_) * (*position);
  if (!std::isfinite(blended_position))
  {
    return controller_interface::return_type::ERROR;
  }
  // No allocation, logging, or blocking subscriber operations in the control loop.
  return command_interfaces_.front().set_value(blended_position) ?
    controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

}  // namespace robot_controllers

// Register the class under the ControllerInterface base for controller-manager loading.
PLUGINLIB_EXPORT_CLASS(
  robot_controllers::WheelPositionController, controller_interface::ControllerInterface)
