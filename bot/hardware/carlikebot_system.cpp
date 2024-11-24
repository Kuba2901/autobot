// Copyright 2021 ros2_control Development Team
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

#include "carlikebot/carlikebot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace carlikebot
{
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CarlikeBotSystemHardware"),
      "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      // RCLCPP_INFO(get_logger(), "Joint '%s' is a drive joint.", joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // ARDUINO CONNECTION
  std::string serial_device = info_.hardware_parameters["serial_device"];
  int32_t baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  int32_t timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  bool success = arduino_comm_.connect(serial_device, baud_rate, timeout_ms);
  if (!success)
  {
    RCLCPP_ERROR(rclcpp::get_logger("CarlikeBotSystemHardware"), "Failed to connect to Arduino.");
    return hardware_interface::CallbackReturn::ERROR;
  } else
    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully connected to Arduino.");

  hw_interfaces_["steering"] = Joint("virtual_front_wheel_joint");
  hw_interfaces_["traction"] = Joint("virtual_rear_wheel_joint");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

    if (joint.first == "traction")
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.second.state.velocity));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CarlikeBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    if (joint.first == "steering")
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.second.joint_name, hardware_interface::HW_IF_POSITION,
        &joint.second.command.position));
    }
    else if (joint.first == "traction")
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
        &joint.second.command.velocity));
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Activating ...please wait...");

  for (auto & joint : hw_interfaces_)
  {
    joint.second.state.position = 0.0;

    if (joint.first == "traction")
    {
      joint.second.state.velocity = 0.0;
      joint.second.command.velocity = 0.0;
    }
    else if (joint.first == "steering")
      joint.second.command.position = 0.0;
  }

  if (!arduino_comm_.isConnected())
  {
    RCLCPP_ERROR(rclcpp::get_logger("CarlikeBotSystemHardware"), "Arduino is not connected.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // LIGHTS
  is_running_ = true;
  light_control_node_ = rclcpp::Node::make_shared("light_control_node");
  
  light_command_subscription_ = light_control_node_->create_subscription<std_msgs::msg::String>(
    "light_command",
    10,
    std::bind(&CarlikeBotSystemHardware::light_command_callback, this, std::placeholders::_1)
  );

  // Start spinning in a separate thread
  spin_thread_ = std::thread([this]() {
    while (is_running_ && rclcpp::ok()) {
      rclcpp::spin_some(light_control_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Add small sleep to prevent CPU overuse
    }
  });

  RCLCPP_INFO(light_control_node_->get_logger(), "Subscribed to light_command topic");

  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Deactivating ...please wait...");

  arduino_comm_.disconnect();
  if (arduino_comm_.isConnected())
  {
    RCLCPP_ERROR(rclcpp::get_logger("CarlikeBotSystemHardware"), "Arduino failed to disconnect.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Stop spinning the light thread
    // Stop the spinning thread
  is_running_ = false;
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarlikeBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  hw_interfaces_["steering"].state.position = hw_interfaces_["steering"].command.position;
  hw_interfaces_["traction"].state.velocity = hw_interfaces_["traction"].command.velocity;
  hw_interfaces_["traction"].state.position +=
    hw_interfaces_["traction"].state.velocity * period.seconds();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type carlikebot ::CarlikeBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!arduino_comm_.isConnected())
  {
    RCLCPP_ERROR(rclcpp::get_logger("CarlikeBotSystemHardware"), "Arduino is not connected.");
    return hardware_interface::return_type::ERROR;
  }

  std::cout << "Steering position: " << hw_interfaces_["steering"].command.position << std::endl;
  arduino_comm_.setSteeringAngle(hw_interfaces_["steering"].command.position);
  arduino_comm_.setTractionVelocity(hw_interfaces_["traction"].command.velocity);
  return hardware_interface::return_type::OK;
}

void CarlikeBotSystemHardware::light_command_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "L,TOGGLE") {
    if (arduino_comm_.isConnected()) {
      arduino_comm_.toggleLightState();
    } else {
      RCLCPP_ERROR(light_control_node_->get_logger(), 
        "Cannot toggle lights - Arduino is not connected");
    }
  }
}


}  // namespace carlikebot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  carlikebot::CarlikeBotSystemHardware, hardware_interface::SystemInterface)