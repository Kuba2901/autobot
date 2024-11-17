// carlikebot_system.cpp
#include "carlikebot/carlikebot_system.hpp"

namespace carlikebot {

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // logger_ = rclcpp::get_logger("CarlikeBotSystemHardware");
  
  joints_["steering"] = Joint("virtual_front_wheel_joint");
  joints_["traction"] = Joint("virtual_rear_wheel_joint");

  // Initialize Arduino communication
  arduino_ = std::make_unique<ArduinoComm>();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Connect to Arduino
  const std::string port = info_.hardware_parameters["serial_port"];
  const int baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  
  if (!arduino_->connect(port, baud_rate)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (auto& joint : joints_) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.second.name, 
        hardware_interface::HW_IF_POSITION,
        &joint.second.position));
        
    if (joint.first == "traction") {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint.second.name,
          hardware_interface::HW_IF_VELOCITY,
          &joint.second.velocity));
    }
  }
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CarlikeBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (auto& joint : joints_) {
    if (joint.first == "steering") {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.second.name,
          hardware_interface::HW_IF_POSITION,
          &joint.second.position_command));
    } else if (joint.first == "traction") {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.second.name,
          hardware_interface::HW_IF_VELOCITY,
          &joint.second.velocity_command));
    }
  }
  
  return command_interfaces;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  // Initialize the Arduino
  if (!arduino_->initialize()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Reset all joint states and commands
  for (auto& joint : joints_) {
    joint.second.position = 0.0;
    joint.second.velocity = 0.0;
    joint.second.position_command = 0.0;
    joint.second.velocity_command = 0.0;
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  arduino_->stopMotors();
  arduino_->disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarlikeBotSystemHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // For now, just copy commands to states since we don't have encoder feedback
  joints_["steering"].position = joints_["steering"].position_command;
  joints_["traction"].velocity = joints_["traction"].velocity_command;
  
  // Update traction position based on velocity
  joints_["traction"].position += joints_["traction"].velocity * period.seconds();
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CarlikeBotSystemHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Send commands to Arduino
  if (!arduino_->setSteeringAngle(joints_["steering"].position_command) ||
      !arduino_->setTractionVelocity(joints_["traction"].velocity_command)) {
    return hardware_interface::return_type::ERROR;
  }
  
  return hardware_interface::return_type::OK;
}

}  // namespace carlikebot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  carlikebot::CarlikeBotSystemHardware, 
  hardware_interface::SystemInterface)