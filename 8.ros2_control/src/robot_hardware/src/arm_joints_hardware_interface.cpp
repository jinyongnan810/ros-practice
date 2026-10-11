#include "robot_hardware/arm_joints_hardware_interface.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace robot_hardware
{
  std::map<std::string, int> ArmJointsHardwareInterface::default_motor_ids() const
  {
    return {{"arm_joint_1", 6}, {"arm_joint_2", 7}};
  }
} // namespace robot_hardware

PLUGINLIB_EXPORT_CLASS(robot_hardware::ArmJointsHardwareInterface, hardware_interface::SystemInterface)
