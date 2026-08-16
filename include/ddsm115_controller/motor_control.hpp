#ifndef DDSM115_CONTROLLER__MOTOR_CONTROL_HPP_
#define DDSM115_CONTROLLER__MOTOR_CONTROL_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace ddsm115_controller
{

struct MotorFeedback
{
  int id{-1};
  std::uint8_t mode{0};
  float current{0.0F};
  std::int16_t rpm{0};
  std::uint8_t winding_temperature{0};
  std::uint8_t position{0};
  std::uint8_t error{0};
};

class MotorControl
{
public:
  explicit MotorControl(const std::string & device = "/dev/ttyUSB0");
  ~MotorControl();

  MotorControl(const MotorControl &) = delete;
  MotorControl & operator=(const MotorControl &) = delete;

  void set_id(std::uint8_t id);
  MotorFeedback send_current(std::uint8_t id, float current);
  MotorFeedback send_rpm(std::uint8_t id, std::int16_t rpm);
  MotorFeedback set_brake(std::uint8_t id);
  std::string set_drive_mode(std::uint8_t id, std::uint8_t mode);
  MotorFeedback get_motor_feedback(std::uint8_t id);

private:
  static std::array<std::uint8_t, 2> int16_to_bytes(std::int16_t value);
  static std::int16_t bytes_to_int16(std::uint8_t high, std::uint8_t low);
  static std::uint8_t crc8_maxim(const std::uint8_t * data, std::size_t size);
  static double map_value(
    double value, double in_min, double in_max, double out_min,
    double out_max);

  void write_packet(const std::uint8_t * data, std::size_t size);
  MotorFeedback read_reply(
    std::uint8_t id,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(10));

  int serial_fd_{-1};
};

}  // namespace ddsm115_controller

#endif  // DDSM115_CONTROLLER__MOTOR_CONTROL_HPP_
