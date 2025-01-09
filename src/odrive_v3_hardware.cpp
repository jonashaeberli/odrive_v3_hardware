// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_v3_hardware/odrive_v3_hardware.hpp"
#include "rclcpp/rclcpp.hpp"

namespace odrive_v3_hardware
{
hardware_interface::CallbackReturn OdriveV3Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  try
    {
      auto transmission_param = info.hardware_parameters.at("transmission");
      transmission_ = static_cast<float>(std::stod(transmission_param));
      RCLCPP_INFO(rclcpp::get_logger("OdriveV3Hardware"), "Transmission: %f", transmission_);
    }
    catch (const std::out_of_range &)
    {
      RCLCPP_ERROR(rclcpp::get_logger("OdriveV3Hardware"), "Transmission parameter not found.");
      return CallbackReturn::ERROR;
    }
    catch (const std::invalid_argument &)
    {
      RCLCPP_ERROR(rclcpp::get_logger("OdriveV3Hardware"), "Invalid transmission parameter value.");
      return CallbackReturn::ERROR;
    }

  try
    {
      auto joint_zero_param = info.hardware_parameters.at("joint_zero");
      joint_zero_ = static_cast<float>(std::stod(joint_zero_param));
      RCLCPP_INFO(rclcpp::get_logger("OdriveV3Hardware"), "Zero: %f", joint_zero_);
    }
    catch (const std::out_of_range &)
    {
      RCLCPP_ERROR(rclcpp::get_logger("OdriveV3Hardware"), "Zero position parameter not found.");
      return CallbackReturn::ERROR;
    }
    catch (const std::invalid_argument &)
    {
      RCLCPP_ERROR(rclcpp::get_logger("OdriveV3Hardware"), "Invalid zero position parameter value.");
      return CallbackReturn::ERROR;
    }
  
  try
    {
      auto can_id_param = info.hardware_parameters.at("can_id");
      can_id_ = static_cast<uint16_t>(std::stoi(can_id_param));
      RCLCPP_INFO(rclcpp::get_logger("OdriveV3Hardware"), "can_id: %d", can_id_);
    }
    catch (const std::out_of_range &)
    {
      RCLCPP_ERROR(rclcpp::get_logger("OdriveV3Hardware"), "can_id parameter not found.");
      return CallbackReturn::ERROR;
    }
    catch (const std::invalid_argument &)
    {
      RCLCPP_ERROR(rclcpp::get_logger("OdriveV3Hardware"), "Invalid can_id parameter value.");
      return CallbackReturn::ERROR;
    }

  position_multiplication_factor_ = static_cast<float>(transmission_ / 6.283185307); // Rotations multiplied by this values results in the amount of rotations needed to get to a motor output angle specified in radians
  velocity_multiplication_factor_ = static_cast<float>((1000.0 / 6.283185307) * transmission_);


  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveV3Hardware::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("OdriveV3Hardware"),
  "Transitioning from state: %s", previous_state.label().c_str());
  
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OdriveV3Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OdriveV3Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // Position interface
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));

    // Velocity interface
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
  }

  return command_interfaces;
}


hardware_interface::CallbackReturn OdriveV3Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveV3Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OdriveV3Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): read robot states

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OdriveV3Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    punning_position.f = static_cast<float>(hw_commands_position_[i]) * position_multiplication_factor_ - joint_zero_;
    velocity = static_cast<int16_t>(hw_commands_velocity_[i] * velocity_multiplication_factor_);
    
    Hndl.SetInputPos(can_id_, punning_position.u, velocity, 0);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace odrive_v3_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(odrive_v3_hardware::OdriveV3Hardware, hardware_interface::ActuatorInterface)
