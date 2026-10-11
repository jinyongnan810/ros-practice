#include "robot_hardware/position_hardware_interface.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware
{
  PositionHardwareInterface::CallbackReturn PositionHardwareInterface::on_init(
      const hardware_interface::HardwareComponentInterfaceParams &params)
  {
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    const auto defaults = default_motor_ids();
    if (info_.joints.size() != defaults.size())
    {
      RCLCPP_FATAL(rclcpp::get_logger("PositionHardwareInterface"),
          "Expected %zu position joints, received %zu.", defaults.size(), info_.joints.size());
      return CallbackReturn::ERROR;
    }
    motors_.clear();
    std::set<int> ids;
    std::set<std::string> names;
    try
    {
      const auto parse_integer = [](const std::string &value)
      {
        size_t end = 0;
        const int result = std::stoi(value, &end);
        if (end != value.size())
        {
          throw std::invalid_argument("Invalid integer: " + value);
        }
        return result;
      };
      if (info_.hardware_parameters.count("port_name"))
      {
        port_name_ = info_.hardware_parameters.at("port_name");
      }
      if (info_.hardware_parameters.count("baud_rate"))
      {
        baud_rate_ = parse_integer(info_.hardware_parameters.at("baud_rate"));
      }
      if (baud_rate_ <= 0)
      {
        throw std::invalid_argument("baud_rate must be positive");
      }
      for (const auto &joint : info_.joints)
      {
        const auto entry = defaults.find(joint.name);
        std::set<std::string> states;
        for (const auto &state : joint.state_interfaces)
        {
          states.insert(state.name);
        }
        if (entry == defaults.end() || !names.insert(joint.name).second ||
            joint.command_interfaces.size() != 1 ||
            joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
            joint.state_interfaces.size() != 2 ||
            states != std::set<std::string>{hardware_interface::HW_IF_POSITION,
                                           hardware_interface::HW_IF_VELOCITY})
        {
          RCLCPP_FATAL(rclcpp::get_logger("PositionHardwareInterface"),
              "Invalid joint '%s': expected position command and position/velocity states.",
              joint.name.c_str());
          return CallbackReturn::ERROR;
        }
        const int id = joint.parameters.count("dxl_id") ?
            parse_integer(joint.parameters.at("dxl_id")) : entry->second;
        if (id < 0 || id > 252 || !ids.insert(id).second)
        {
          throw std::invalid_argument("Motor IDs must be unique and in [0, 252]");
        }
        motors_.push_back({joint.name, id});
      }
    }
    catch (const std::exception &error)
    {
      RCLCPP_FATAL(rclcpp::get_logger("PositionHardwareInterface"),
          "Invalid hardware configuration: %s", error.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  PositionHardwareInterface::CallbackReturn PositionHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    driver_ = std::make_unique<XL330Driver>(port_name_, baud_rate_);
    if (driver_->init() != 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("PositionHardwareInterface"),
          "Cannot initialize port '%s'. Using mock mode.", port_name_.c_str());
      driver_.reset();
    }
    for (const auto &motor : motors_)
    {
      const double position = driver_ ? driver_->getPositionRadian(motor.id) : 0.0;
      set_state(motor.name + "/position", position);
      set_state(motor.name + "/velocity", driver_ ? driver_->getVelocityRadianPerSec(motor.id) : 0.0);
      set_command(motor.name + "/position", position);
    }
    return CallbackReturn::SUCCESS;
  }

  PositionHardwareInterface::CallbackReturn PositionHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (const auto &motor : motors_)
    {
      const double position = driver_ ? driver_->getPositionRadian(motor.id) :
          get_state(motor.name + "/position");
      set_state(motor.name + "/position", position);
      set_command(motor.name + "/position", position);
      if (driver_)
      {
        driver_->deactivate(motor.id);
        driver_->activateWithPositionMode(motor.id, position);
      }
    }
    return CallbackReturn::SUCCESS;
  }

  PositionHardwareInterface::CallbackReturn PositionHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    if (driver_)
    {
      for (const auto &motor : motors_)
      {
        driver_->deactivate(motor.id);
      }
    }
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type PositionHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    for (const auto &motor : motors_)
    {
      if (driver_)
      {
        set_state(motor.name + "/position", driver_->getPositionRadian(motor.id));
        set_state(motor.name + "/velocity", driver_->getVelocityRadianPerSec(motor.id));
      }
      else
      {
        const double command = get_command(motor.name + "/position");
        if (!std::isfinite(command))
        {
          return hardware_interface::return_type::ERROR;
        }
        const double previous = get_state(motor.name + "/position");
        set_state(motor.name + "/position", command);
        set_state(motor.name + "/velocity", period.seconds() > 0.0 ?
            (command - previous) / period.seconds() : 0.0);
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type PositionHardwareInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    for (const auto &motor : motors_)
    {
      if (!std::isfinite(get_command(motor.name + "/position")))
      {
        return hardware_interface::return_type::ERROR;
      }
    }
    if (driver_)
    {
      for (const auto &motor : motors_)
      {
        driver_->setTargetPositionRadian(motor.id, get_command(motor.name + "/position"));
      }
    }
    return hardware_interface::return_type::OK;
  }
} // namespace robot_hardware
