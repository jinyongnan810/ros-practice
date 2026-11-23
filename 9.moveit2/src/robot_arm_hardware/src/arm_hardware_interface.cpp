#include "robot_arm_hardware/arm_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware
{
    namespace
    {
        int parse_integer(const std::string &value)
        {
            size_t end = 0;
            const int result = std::stoi(value, &end);
            if (end != value.size())
            {
                throw std::invalid_argument("Invalid integer: '" + value + "'");
            }
            return result;
        }

        bool has_interface(
            const std::vector<hardware_interface::InterfaceInfo> &interfaces,
            const std::string &name)
        {
            return std::any_of(
                interfaces.begin(), interfaces.end(),
                [&name](const auto &iface) { return iface.name == name; });
        }
    } // namespace

    hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        info_ = params.hardware_info;
        motor_joints_.clear();
        mimic_joints_.clear();

        try
        {
            if (info_.hardware_parameters.count("dynamixel_port"))
            {
                port_ = info_.hardware_parameters.at("dynamixel_port");
            }
            else if (info_.hardware_parameters.count("port_name"))
            {
                port_ = info_.hardware_parameters.at("port_name");
            }
            else
            {
                port_ = "/dev/ttyUSB0";
                RCLCPP_WARN(
                    get_logger(),
                    "Parameter 'dynamixel_port' not set, defaulting to '%s'.",
                    port_.c_str());
            }

            baud_rate_ = info_.hardware_parameters.count("baud_rate")
                             ? parse_integer(info_.hardware_parameters.at("baud_rate"))
                             : BAUDRATE;
            if (baud_rate_ <= 0)
            {
                throw std::invalid_argument("baud_rate must be positive");
            }

            std::map<std::size_t, hardware_interface::MimicJoint> mimic_by_index;
            for (const auto &mimic : info_.mimic_joints)
            {
                if (mimic.joint_index >= info_.joints.size() ||
                    mimic.mimicked_joint_index >= info_.joints.size())
                {
                    throw std::out_of_range("Mimic joint index out of bounds");
                }
                mimic_by_index[mimic.joint_index] = mimic;
            }

            std::set<int> used_ids;
            std::vector<std::size_t> unconfigured_indices;

            for (std::size_t i = 0; i < info_.joints.size(); ++i)
            {
                const auto &joint = info_.joints[i];
                const bool has_vel_state =
                    has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY);

                if (mimic_by_index.count(i))
                {
                    const auto &mimic = mimic_by_index.at(i);
                    const auto &mimicked_joint = info_.joints[mimic.mimicked_joint_index];
                    if (!has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION))
                    {
                        throw std::invalid_argument(
                            "Mimic joint '" + joint.name + "' must expose a position state interface");
                    }
                    mimic_joints_.push_back({
                        joint.name,
                        mimicked_joint.name,
                        mimic.multiplier,
                        mimic.offset,
                        has_vel_state,
                        has_interface(
                            mimicked_joint.state_interfaces, hardware_interface::HW_IF_VELOCITY),
                    });
                    continue;
                }

                if (!has_interface(joint.command_interfaces, hardware_interface::HW_IF_POSITION) ||
                    !has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION))
                {
                    throw std::invalid_argument(
                        "Joint '" + joint.name +
                        "' must expose position command and position state interfaces");
                }

                bool has_explicit_id = false;
                int motor_id = -1;
                const std::string hw_param_key = joint.name + "_motor_id";
                if (joint.parameters.count("motor_id"))
                {
                    motor_id = parse_integer(joint.parameters.at("motor_id"));
                    has_explicit_id = true;
                }
                else if (joint.parameters.count("dxl_id"))
                {
                    motor_id = parse_integer(joint.parameters.at("dxl_id"));
                    has_explicit_id = true;
                }
                else if (info_.hardware_parameters.count(hw_param_key))
                {
                    motor_id = parse_integer(info_.hardware_parameters.at(hw_param_key));
                    has_explicit_id = true;
                }

                if (has_explicit_id)
                {
                    if (motor_id < 0 || motor_id > 252 || !used_ids.insert(motor_id).second)
                    {
                        throw std::invalid_argument(
                            "Motor ID for joint '" + joint.name +
                            "' must be unique and in [0, 252]");
                    }
                }
                else
                {
                    unconfigured_indices.push_back(motor_joints_.size());
                }

                motor_joints_.push_back({joint.name, motor_id, has_vel_state});
            }

            int next_default_id = 1;
            for (const std::size_t idx : unconfigured_indices)
            {
                while (used_ids.count(next_default_id))
                {
                    ++next_default_id;
                }
                if (next_default_id > 252)
                {
                    throw std::out_of_range("Exceeded maximum Dynamixel ID (252)");
                }
                motor_joints_[idx].motor_id = next_default_id;
                used_ids.insert(next_default_id);
            }
        }
        catch (const std::exception &error)
        {
            RCLCPP_FATAL(
                get_logger(),
                "Failed to initialize ArmHardwareInterface: %s",
                error.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(
            get_logger(),
            "Configured %zu Dynamixel joint(s) and %zu mimic joint(s) on port '%s' (%d baud).",
            motor_joints_.size(), mimic_joints_.size(), port_.c_str(), baud_rate_);
        for (const auto &motor : motor_joints_)
        {
            RCLCPP_INFO(
                get_logger(),
                "  Joint '%s' -> Dynamixel ID %d",
                motor.name.c_str(), motor.motor_id);
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void ArmHardwareInterface::sync_states_from_driver(bool sync_commands)
    {
        for (const auto &motor : motor_joints_)
        {
            const double position = driver_->getPositionRadian(motor.motor_id);
            set_state(motor.name + "/position", position);
            if (motor.has_velocity_state)
            {
                set_state(motor.name + "/velocity", driver_->getVelocityRadianPerSec(motor.motor_id));
            }
            if (sync_commands)
            {
                set_command(motor.name + "/position", position);
            }
        }
        update_mimic_states();
    }

    void ArmHardwareInterface::update_mimic_states()
    {
        for (const auto &mimic : mimic_joints_)
        {
            const double parent_pos = get_state(mimic.mimicked_joint_name + "/position");
            set_state(mimic.name + "/position", mimic.multiplier * parent_pos + mimic.offset);
            if (mimic.has_velocity_state)
            {
                const double parent_vel = mimic.mimicked_has_velocity_state
                                              ? get_state(mimic.mimicked_joint_name + "/velocity")
                                              : 0.0;
                set_state(mimic.name + "/velocity", mimic.multiplier * parent_vel);
            }
        }
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        driver_ = std::make_shared<XL330Driver>(port_, baud_rate_);
        if (driver_->init() != 0)
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize XL330Driver on '%s'.", port_.c_str());
            driver_.reset();
            return hardware_interface::CallbackReturn::ERROR;
        }

        sync_states_from_driver(true);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_cleanup(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        driver_.reset();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        if (!driver_)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        sync_states_from_driver(true);
        for (const auto &motor : motor_joints_)
        {
            driver_->activateWithPositionMode(
                motor.motor_id, get_state(motor.name + "/position"));
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        if (driver_)
        {
            for (const auto &motor : motor_joints_)
            {
                driver_->deactivate(motor.motor_id);
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ArmHardwareInterface::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        if (!driver_)
        {
            return hardware_interface::return_type::ERROR;
        }

        sync_states_from_driver(false);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ArmHardwareInterface::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        if (!driver_)
        {
            return hardware_interface::return_type::ERROR;
        }

        for (const auto &motor : motor_joints_)
        {
            if (!std::isfinite(get_command(motor.name + "/position")))
            {
                return hardware_interface::return_type::ERROR;
            }
        }

        for (const auto &motor : motor_joints_)
        {
            driver_->setTargetPositionRadian(motor.motor_id, get_command(motor.name + "/position"));
        }

        return hardware_interface::return_type::OK;
    }

} // namespace arm_hardware

PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)
