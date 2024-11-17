// arduino_comms.hpp
#ifndef CARLIKEBOT_ARDUINO_COMMS_HPP_
#define CARLIKEBOT_ARDUINO_COMMS_HPP_

#include <memory>
#include <string>
#include <libserial/SerialPort.h>
#include "rclcpp/logger.hpp"
#include <math.h>

namespace carlikebot {

class ArduinoComm {
public:
  ArduinoComm() {}
  ~ArduinoComm();

  bool connect(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms);
  void disconnect();
  bool isConnected() const;
  
  // Motor control functions
  bool setSteeringAngle(double angle);  // angle in radians
  bool setTractionVelocity(double velocity);  // normalized velocity (-1.0 to 1.0)
  bool stopMotors();
  bool initialize();

private:
  bool sendCommand(const std::string& cmd);
  double mapToRange(double x, double in_min, double in_max, double out_min, double out_max);
  
  LibSerial::SerialPort serial_port_;
  int timeout_ms_;
  
  static constexpr double STEERING_MIN = -M_PI_2;  // -90 degrees
  static constexpr double STEERING_MAX = M_PI_2;   // +90 degrees
  static constexpr double VELOCITY_MIN = -1.0;
  static constexpr double VELOCITY_MAX = 1.0;
  static constexpr int PWM_MIN = 0;
  static constexpr int PWM_MAX = 255;
};

}  // namespace carlikebot

#endif  // CARLIKEBOT_ARDUINO_COMM_HPP_