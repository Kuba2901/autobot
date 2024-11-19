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
    initialize();
    return true;
  } catch (const std::exception& e) {
    return false;
  } catch (...) {
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

/*
STEERING PWM VALUES
[0-126]: full left
127: center
[128-255]: full right
*/
bool ArduinoComm::setSteeringAngle(double angle)
{
  std::stringstream cmd;

  if (!isConnected()) return false;

  // Convert angle to PWM value with wider range utilization
  int pwm = static_cast<int>(mapToRange(angle, STEERING_MIN, STEERING_MAX, 0, 255));
  
  // Print the values only if it changed
  if (pwm != this->current_steering_pwm_)
  {
    this->current_steering_pwm_ = pwm;
    std::cout << "Angle: " << angle << " PWM: " << pwm << std::endl;
  }
  cmd << "M," << pwm << "," << this->current_traction_pwm_ << "\n";  // // Format: M,<steering_pwm>,<traction_pwm>
  return sendCommand(cmd.str());
}

/*
TRACTION PWM VALUES
[0-126]: full reverse
127: stop
[128-255]: full forward
*/
bool ArduinoComm::setTractionVelocity(double velocity)
{
  std::stringstream cmd;
  int pwm;

  if (!isConnected()) return false;
  

  // Extract the PWM value from velocity
  if (velocity < 0)
    pwm = 0;
  else if (velocity > 0)
    pwm = 255;
  else
    pwm = 127;

  if (pwm != this->current_traction_pwm_)
  {
    this->current_traction_pwm_ = pwm;
    std::cout << "Velocity: " << velocity << " PWM: " << pwm << std::endl;
  }
  cmd << "M," << this->current_steering_pwm_ << "," << pwm << "\n";  // Format: M,<steering_pwm>,<traction_pwm>
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
    return (response.find("OK") != std::string::npos);
  }
  catch (const LibSerial::ReadTimeout&)
  {
    std::cerr << "The ReadByte() call has timed out." << std::endl;
    return false;
  }
}

double ArduinoComm::mapToRange(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

}  // namespace carlikebot