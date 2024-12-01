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
  std::stringstream cmd;
  if (serial_port_.IsOpen()) {
    stopMotors();
    cmd << "DISCONNECT" << "\n";
    sendCommand(cmd.str());
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

  std::cout << "Angle: " << angle << "\n";
  // Convert angle to PWM value with wider range utilization
  int pwm = static_cast<int>(mapToRange(angle, STEERING_MIN, STEERING_MAX, 0, 255));
  
  // Print the values only if it changed
  if (pwm != this->current_steering_pwm_)
    this->current_steering_pwm_ = pwm;
  std::cout << "M," << pwm << "," << this->current_traction_pwm_ << "\n";
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
  if (!isConnected()) return false;
  

  std::cout << "Velocity: " << velocity << "\n";
  // Extract the PWM value from velocity
  int pwm = static_cast<int>(mapToRange(velocity, VELOCITY_MIN, VELOCITY_MAX, 0, 255));
  if (pwm != this->current_traction_pwm_)
    this->current_traction_pwm_ = pwm;
  std::cout << "M," << this->current_steering_pwm_ << "," << pwm << "\n";
  cmd << "M," << this->current_steering_pwm_ << "," << pwm << "\n";  // Format: M,<steering_pwm>,<traction_pwm>
  return sendCommand(cmd.str());
}

bool ArduinoComm::stopMotors()
{
  std::stringstream cmd;
  if (!isConnected()) return false;
  cmd << "STOP" << "\n";;  // Only use \n instead of std::endl
  return sendCommand(cmd.str());
}

bool ArduinoComm::initialize()
{
  std::stringstream cmd;
  if (!isConnected()) return false;
  sleep(3);  // Wait for the Arduino to reset
  this->current_steering_pwm_ = 127;
  this->current_traction_pwm_ = 127;
  cmd << "INIT" << "\n";  // Only use \n instead of std::endl
  return sendCommand(cmd.str());
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

bool ArduinoComm::toggleLightState(void) {
  if (!isConnected()) return false;
  std::stringstream cmd;
  cmd << "L,TOGGLE" << "\n";
  return sendCommand(cmd.str());
}

}  // namespace carlikebot