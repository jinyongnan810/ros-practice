#ifndef ARM_HARDWARE_INTERFACE_HPP
#define ARM_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "robot_arm_hardware/xl330_driver.hpp"

namespace arm_hardware
{

    class ArmHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        // Lifecycle node override
        hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        // SystemInterface override
        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
        hardware_interface::return_type
        read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type
        write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        struct MotorJoint
        {
            std::string name;
            int motor_id;
            bool has_velocity_state;
        };

        struct MimicJointInfo
        {
            std::string name;
            std::string mimicked_joint_name;
            double multiplier;
            double offset;
            bool has_velocity_state;
            bool mimicked_has_velocity_state;
        };

        void sync_states_from_driver(bool sync_commands);
        void update_mimic_states();

        std::shared_ptr<XL330Driver> driver_;
        std::vector<MotorJoint> motor_joints_;
        std::vector<MimicJointInfo> mimic_joints_;
        std::string port_;
        int baud_rate_{BAUDRATE};

    }; // class ArmHardwareInterface

} // namespace arm_hardware

#endif