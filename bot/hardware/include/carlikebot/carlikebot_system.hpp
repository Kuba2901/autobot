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

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_11__CARLIKEBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_11__CARLIKEBOT_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Lights
#include "std_msgs/msg/string.hpp" // Include for String message
#include "rclcpp/rclcpp.hpp" // Include for rclcpp::Node
#include "rclcpp/subscription.hpp" // Include for creating a subscriber
#include <thread>

#include "carlikebot/arduino_comms.hpp"

namespace carlikebot
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  explicit Joint(const std::string & name) : joint_name(name)
  {
    state = JointValue();
    command = JointValue();
  }

  Joint() = default;

  std::string joint_name;
  JointValue state;
  JointValue command;
};
class CarlikeBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CarlikeBotSystemHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period
    ) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // LIGHTS
  rclcpp::Node::SharedPtr light_control_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr light_command_subscription_;
  std::thread spin_thread_;
  bool is_running_;
  void light_command_callback(const std_msgs::msg::String::SharedPtr msg);

private:
  // Parameters for the CarlikeBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // std::vector<std::tuple<std::string, double, double>>
  //   hw_interfaces_;  // name of joint, state, command
  std::map<std::string, Joint> hw_interfaces_;

  ArduinoComm arduino_comm_;
};

}  // namespace ros2_control_demo_example_11

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_11__CARLIKEBOT_SYSTEM_HPP_