#ifndef ROBOT_HARDWARE__BASE_LINK_HARDWARE_INTERFACE_HPP_
#define ROBOT_HARDWARE__BASE_LINK_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "robot_hardware/xl330_driver.hpp"

namespace robot_hardware
{

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class BaseLinkHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SMART_PTR_DEFINITIONS(BaseLinkHardwareInterface)

    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // Dynamixel motor driver
    std::unique_ptr<XL330Driver> driver_;
    std::string port_name_{"/dev/ttyUSB0"};
    int baud_rate_{57600};

    // Motor IDs for the 4 drive wheels
    int front_left_wheel_id_{1};
    int front_right_wheel_id_{2};
    int rear_left_wheel_id_{3};
    int rear_right_wheel_id_{4};

    // State buffers (size 4: FL, FR, RL, RR in rad and rad/s)
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;

    // Command buffers (velocity commands in rad/s for each wheel)
    std::vector<double> hw_commands_;
  };

} // namespace robot_hardware

#endif // ROBOT_HARDWARE__BASE_LINK_HARDWARE_INTERFACE_HPP_
