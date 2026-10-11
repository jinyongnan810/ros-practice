#ifndef ROBOT_HARDWARE__ARM_JOINTS_HARDWARE_INTERFACE_HPP_
#define ROBOT_HARDWARE__ARM_JOINTS_HARDWARE_INTERFACE_HPP_

#include "robot_hardware/position_hardware_interface.hpp"

namespace robot_hardware
{
  class ArmJointsHardwareInterface : public PositionHardwareInterface
  {
  public:
    RCLCPP_SMART_PTR_DEFINITIONS(ArmJointsHardwareInterface)

  protected:
    std::map<std::string, int> default_motor_ids() const override;
  };
} // namespace robot_hardware

#endif // ROBOT_HARDWARE__ARM_JOINTS_HARDWARE_INTERFACE_HPP_
