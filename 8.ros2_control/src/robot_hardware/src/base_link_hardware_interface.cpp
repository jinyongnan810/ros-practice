#include "robot_hardware/base_link_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware
{

  CallbackReturn BaseLinkHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
  {
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Read hardware parameters if provided in URDF
    if (info_.hardware_parameters.find("port_name") != info_.hardware_parameters.end())
    {
      port_name_ = info_.hardware_parameters.at("port_name");
    }
    if (info_.hardware_parameters.find("baud_rate") != info_.hardware_parameters.end())
    {
      baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
    }

    // Read motor IDs from hardware parameters if configured
    if (info_.hardware_parameters.find("front_left_wheel_id") != info_.hardware_parameters.end())
    {
      front_left_wheel_id_ = std::stoi(info_.hardware_parameters.at("front_left_wheel_id"));
    }
    if (info_.hardware_parameters.find("front_right_wheel_id") != info_.hardware_parameters.end())
    {
      front_right_wheel_id_ = std::stoi(info_.hardware_parameters.at("front_right_wheel_id"));
    }
    if (info_.hardware_parameters.find("rear_left_wheel_id") != info_.hardware_parameters.end())
    {
      rear_left_wheel_id_ = std::stoi(info_.hardware_parameters.at("rear_left_wheel_id"));
    }
    if (info_.hardware_parameters.find("rear_right_wheel_id") != info_.hardware_parameters.end())
    {
      rear_right_wheel_id_ = std::stoi(info_.hardware_parameters.at("rear_right_wheel_id"));
    }

    // Read per-joint parameters (dxl_id) if defined in <joint> tags
    for (const auto &joint : info_.joints)
    {
      auto it = joint.parameters.find("dxl_id");
      if (it != joint.parameters.end())
      {
        int id = std::stoi(it->second);
        if (joint.name == "front_left_wheel_joint")
        {
          front_left_wheel_id_ = id;
        }
        else if (joint.name == "front_right_wheel_joint")
        {
          front_right_wheel_id_ = id;
        }
        else if (joint.name == "rear_left_wheel_joint")
        {
          rear_left_wheel_id_ = id;
        }
        else if (joint.name == "rear_right_wheel_joint")
        {
          rear_right_wheel_id_ = id;
        }
      }
    }

    // Validate command and state interfaces
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("BaseLinkHardwareInterface"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("BaseLinkHardwareInterface"),
            "Joint '%s' has '%s' command interface. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("BaseLinkHardwareInterface"),
            "Joint '%s' has %zu state interfaces found. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("BaseLinkHardwareInterface"),
            "Joint '%s' has '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("BaseLinkHardwareInterface"),
            "Joint '%s' has '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn BaseLinkHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("BaseLinkHardwareInterface"),
        "Configuring Dynamixel XL-330 driver on port '%s' at %d baud...",
        port_name_.c_str(), baud_rate_);

    driver_ = std::make_unique<XL330Driver>(port_name_);
    if (driver_->init() != 0)
    {
      RCLCPP_WARN(
          rclcpp::get_logger("BaseLinkHardwareInterface"),
          "Cannot open port '%s'. Proceeding in simulation / mock mode (no physical hardware).",
          port_name_.c_str());
      driver_.reset();
    }
    else
    {
      RCLCPP_INFO(
          rclcpp::get_logger("BaseLinkHardwareInterface"),
          "Successfully initialized Dynamixel driver on port '%s'.", port_name_.c_str());
    }

    // Initialize joint states and commands to 0.0
    for (const auto &joint : info_.joints)
    {
      set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, 0.0);
      set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
      set_command(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn BaseLinkHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("BaseLinkHardwareInterface"), "Activating hardware interface...");

    for (const auto &joint : info_.joints)
    {
      set_command(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    }

    if (driver_)
    {
      std::vector<int> motor_ids = {
          front_left_wheel_id_,
          front_right_wheel_id_,
          rear_left_wheel_id_,
          rear_right_wheel_id_};

      for (size_t i = 0; i < motor_ids.size() && i < info_.joints.size(); ++i)
      {
        driver_->activateWithVelocityMode(motor_ids[i]);
      }
      RCLCPP_INFO(
          rclcpp::get_logger("BaseLinkHardwareInterface"),
          "Successfully activated all 4 drive wheel actuators in velocity mode.");
    }
    else
    {
      RCLCPP_INFO(
          rclcpp::get_logger("BaseLinkHardwareInterface"),
          "Activated hardware interface in simulation / mock mode.");
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn BaseLinkHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("BaseLinkHardwareInterface"), "Deactivating hardware interface...");

    if (driver_)
    {
      std::vector<int> motor_ids = {
          front_left_wheel_id_,
          front_right_wheel_id_,
          rear_left_wheel_id_,
          rear_right_wheel_id_};

      for (size_t i = 0; i < motor_ids.size() && i < info_.joints.size(); ++i)
      {
        driver_->setTargetVelocityRadianPerSec(motor_ids[i], 0.0);
        driver_->deactivate(motor_ids[i]);
      }
      RCLCPP_INFO(
          rclcpp::get_logger("BaseLinkHardwareInterface"),
          "Successfully deactivated all 4 drive wheel actuators.");
    }

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type BaseLinkHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    /*
    // --- Real Hardware Mode (Physical Dynamixel XL-330 Encoders) ---
    std::vector<int> motor_ids = {
        front_left_wheel_id_,
        front_right_wheel_id_,
        rear_left_wheel_id_,
        rear_right_wheel_id_};

    for (size_t i = 0; i < motor_ids.size() && i < info_.joints.size(); ++i)
    {
      set_state(
          info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION,
          driver_->getPositionRadian(motor_ids[i]));
      set_state(
          info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY,
          driver_->getVelocityRadianPerSec(motor_ids[i]));
    }
    */

    // --- Simulation / Mock Mode (Numerical Integration: pos += vel * dt) ---
    for (const auto &joint : info_.joints)
    {
      const std::string pos_name = joint.name + "/" + hardware_interface::HW_IF_POSITION;
      const std::string vel_name = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;

      const double prev_position = get_state(pos_name);
      const double command_velocity = get_command(vel_name);

      const double new_position = prev_position + command_velocity * period.seconds();

      set_state(pos_name, new_position);
      set_state(vel_name, command_velocity);
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BaseLinkHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // /*
    // --- Real Hardware Mode (Physical Dynamixel XL-330) ---
    // if (driver_)
    // {
    std::vector<int> motor_ids = {
        front_left_wheel_id_,
        front_right_wheel_id_,
        rear_left_wheel_id_,
        rear_right_wheel_id_};

    for (size_t i = 0; i < motor_ids.size() && i < info_.joints.size(); ++i)
    {
      const double command_velocity =
          get_command(info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY);
      driver_->setTargetVelocityRadianPerSec(motor_ids[i], command_velocity);
    }
    // }
    // */

    return hardware_interface::return_type::OK;
  }

} // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    robot_hardware::BaseLinkHardwareInterface,
    hardware_interface::SystemInterface)
