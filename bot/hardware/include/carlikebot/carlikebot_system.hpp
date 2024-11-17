// carlikebot_system.hpp
#ifndef CARLIKEBOT_SYSTEM_HPP_
#define CARLIKEBOT_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "carlikebot/arduino_comms.hpp"

namespace carlikebot {

class CarlikeBotSystemHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CarlikeBotSystemHardware);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware Handles
  struct Joint {
    std::string name;
    double position{0.0};
    double velocity{0.0};
    double position_command{0.0};
    double velocity_command{0.0};

    Joint() = default;
    explicit Joint(const std::string& joint_name) : name(joint_name) {}
  };

  std::unordered_map<std::string, Joint> joints_;
  std::unique_ptr<ArduinoComm> arduino_;
  rclcpp::Logger logger_;
};

}  // namespace carlikebot

#endif  // CARLIKEBOT_SYSTEM_HPP_