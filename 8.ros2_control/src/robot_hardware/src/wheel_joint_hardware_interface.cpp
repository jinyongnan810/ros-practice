#include "robot_hardware/wheel_joint_hardware_interface.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace robot_hardware
{
  std::map<std::string, int> WheelJointHardwareInterface::default_motor_ids() const
  {
    return {{"wheel_joint", 5}};
  }
} // namespace robot_hardware

PLUGINLIB_EXPORT_CLASS(robot_hardware::WheelJointHardwareInterface, hardware_interface::SystemInterface)
