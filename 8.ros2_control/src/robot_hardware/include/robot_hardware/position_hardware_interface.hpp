#ifndef ROBOT_HARDWARE__POSITION_HARDWARE_INTERFACE_HPP_
#define ROBOT_HARDWARE__POSITION_HARDWARE_INTERFACE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_component_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "robot_hardware/xl330_driver.hpp"

namespace robot_hardware
{
  // Common lifecycle implementation; each plugin owns its own serial connection.
  class PositionHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  protected:
    virtual std::map<std::string, int> default_motor_ids() const = 0;

  private:
    struct JointMotor
    {
      std::string name;
      int id;
    };
    std::vector<JointMotor> motors_;
    std::unique_ptr<XL330Driver> driver_;
    std::string port_name_{"/dev/ttyUSB0"};
    int baud_rate_{57600};
  };
} // namespace robot_hardware

#endif // ROBOT_HARDWARE__POSITION_HARDWARE_INTERFACE_HPP_
