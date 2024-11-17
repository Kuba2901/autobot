// arduino_comm.hpp
#ifndef CARLIKEBOT_ARDUINO_COMM_HPP_
#define CARLIKEBOT_ARDUINO_COMM_HPP_

#include <memory>
#include <string>
#include <libserial/SerialPort.h>
#include "rclcpp/logger.hpp"
#include <math.h>
#include <sstream>
#include <iostream>


namespace carlikebot {


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComm {
public:
  ArduinoComm();

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect() {
    stopMotors();
    serial_conn_.Close();
  }

  bool isConnected() const {
    return serial_conn_.IsOpen();
  }
  
  // Motor control functions
  void setSteeringAngle(double angle) {
    if (!isConnected()) return;
    
    int pwm = static_cast<int>(mapToRange(angle, STEERING_MIN, STEERING_MAX, PWM_MIN, PWM_MAX));
    
    std::stringstream cmd;
    cmd << "M," << pwm << ",0,0\n";  // Format: M,<steering_pwm>,0,0
  }

  void setTractionVelocity(double velocity) {
    if (!isConnected()) return;
    // Convert velocity to PWM value and direction
    int pwm = static_cast<int>(std::abs(mapToRange(velocity, VELOCITY_MIN, VELOCITY_MAX, PWM_MIN, PWM_MAX)));
    bool forward = velocity >= 0;
    
    std::stringstream cmd;
    cmd << "M,0," << pwm << "," << (forward ? "1" : "0") << "\n";  // Format: M,0,<traction_pwm>,<direction>
  }

  void stopMotors() {
    if (!isConnected()) return;
    sendCommand("STOP\n");
  }

  void initialize() {
    if (!isConnected()) return;
    sendCommand("INIT\n");
  }

  ~ArduinoComm() {
    if (isConnected()) {
      disconnect();
    }
  }

private:
  // CONSTANTS
  static constexpr double STEERING_MIN = -M_PI_2;  // -90 degrees
  static constexpr double STEERING_MAX = M_PI_2;   // +90 degrees
  static constexpr double VELOCITY_MIN = -1.0;
  static constexpr double VELOCITY_MAX = 1.0;
  static constexpr int PWM_MIN = 0;
  static constexpr int PWM_MAX = 255;

  // SERIAL CONNECTION PORT
  LibSerial::SerialPort serial_conn_;

  // CONNECTION TIMEOUT
  int timeout_ms_;

  std::string sendCommand(const std::string& cmd, bool print_output = false) {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(cmd);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << cmd << " Recv: " << response << std::endl;
    }
    return response;
  }

  double mapToRange(double x, double in_min, double in_max, double out_min, double out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
};

}  // namespace carlikebot

#endif  // CARLIKEBOT_ARDUINO_COMM_HPP_