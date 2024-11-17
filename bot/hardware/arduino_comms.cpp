// arduino_comm.cpp
#include "carlikebot/arduino_comms.hpp"
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

ArduinoComm::~ArduinoComm()
{
  if (isConnected()) {
    disconnect();
  }
}

bool ArduinoComm::connect(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms)
{
  try {
    timeout_ms_ = timeout_ms;
    serial_port_.Open(serial_device);
    serial_port_.SetBaudRate(convert_baud_rate(baud_rate));
    return true;
  } catch (const std::exception& e) {
    // RCLCPP_ERROR(logger_, "Failed to open serial port: %s", e.what());
    return false;
  } catch (...) {
    // RCLCPP_ERROR(logger_, "Failed to open serial port: unknown error");
    return false;
  }
}

void ArduinoComm::disconnect()
{
  if (serial_port_.IsOpen()) {
    stopMotors();
    serial_port_.Close();
  }
}

bool ArduinoComm::isConnected() const
{
  return serial_port_.IsOpen();
}

bool ArduinoComm::setSteeringAngle(double angle)
{
  if (!isConnected()) return false;
  
  // Convert angle to PWM value
  int pwm = static_cast<int>(mapToRange(angle, STEERING_MIN, STEERING_MAX, PWM_MIN, PWM_MAX));
  
  std::stringstream cmd;
  cmd << "M," << pwm << ",0,0\n";  // Format: M,<steering_pwm>,0,0
  return sendCommand(cmd.str());
}

bool ArduinoComm::setTractionVelocity(double velocity)
{
  if (!isConnected()) return false;
  
  // Convert velocity to PWM value and direction
  int pwm = static_cast<int>(std::abs(mapToRange(velocity, VELOCITY_MIN, VELOCITY_MAX, PWM_MIN, PWM_MAX)));
  bool forward = velocity >= 0;
  
  std::stringstream cmd;
  cmd << "M,0," << pwm << "," << (forward ? "1" : "0") << "\n";  // Format: M,0,<traction_pwm>,<direction>
  return sendCommand(cmd.str());
}

bool ArduinoComm::stopMotors()
{
  if (!isConnected()) return false;
  return sendCommand("STOP\n");
}

bool ArduinoComm::initialize()
{
  if (!isConnected()) return false;
  return sendCommand("INIT\n");
}

bool ArduinoComm::sendCommand(const std::string &cmd)
  {
    serial_port_.FlushIOBuffers(); // Just in case
    serial_port_.Write(cmd);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_port_.ReadLine(response, '\n', timeout_ms_);
      return (true);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
        return (false);
    }
  }


double ArduinoComm::mapToRange(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

}  // namespace carlikebot