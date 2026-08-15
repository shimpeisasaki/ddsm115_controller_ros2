#include "ddsm115_controller/motor_control.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#ifdef __linux__
#include <linux/serial.h>
#include <sys/ioctl.h>
#endif

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <system_error>

namespace ddsm115_controller
{

MotorControl::MotorControl(const std::string & device)
{
  serial_fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "Failed to open " + device);
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    const int error = errno;
    ::close(serial_fd_);
    throw std::system_error(error, std::generic_category(), "Failed to read serial settings");
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    const int error = errno;
    ::close(serial_fd_);
    throw std::system_error(error, std::generic_category(), "Failed to configure serial port");
  }

#ifdef TIOCSRS485
  serial_rs485 rs485{};
  rs485.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
  (void)::ioctl(serial_fd_, TIOCSRS485, &rs485);
#endif
  tcflush(serial_fd_, TCIOFLUSH);
}

MotorControl::~MotorControl()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
  }
}

std::array<std::uint8_t, 2> MotorControl::int16_to_bytes(std::int16_t value)
{
  const auto raw = static_cast<std::uint16_t>(value);
  return {static_cast<std::uint8_t>(raw >> 8), static_cast<std::uint8_t>(raw & 0xFF)};
}

std::int16_t MotorControl::bytes_to_int16(std::uint8_t high, std::uint8_t low)
{
  return static_cast<std::int16_t>((static_cast<std::uint16_t>(high) << 8) | low);
}

std::uint8_t MotorControl::crc8_maxim(const std::uint8_t * data, std::size_t size)
{
  std::uint8_t crc = 0;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x01U) !=
        0U ? static_cast<std::uint8_t>((crc >> 1) ^ 0x8CU) : static_cast<std::uint8_t>(crc >> 1);
    }
  }
  return crc;
}

double MotorControl::map_value(
  double value, double in_min, double in_max, double out_min, double out_max)
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void MotorControl::write_packet(const std::uint8_t * data, std::size_t size)
{
  std::size_t written = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (written < size) {
    const ssize_t result = ::write(serial_fd_, data + written, size - written);
    if (result > 0) {
      written += static_cast<std::size_t>(result);
      continue;
    }
    if (result < 0 && errno != EAGAIN && errno != EINTR) {
      throw std::system_error(errno, std::generic_category(), "Serial write failed");
    }

    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (remaining <= std::chrono::milliseconds::zero()) {
      throw std::runtime_error("Serial write timed out");
    }
    pollfd descriptor{serial_fd_, POLLOUT, 0};
    const int poll_result = ::poll(
      &descriptor, 1,
      static_cast<int>(std::max<std::int64_t>(1, remaining.count())));
    if (poll_result == 0) {
      throw std::runtime_error("Serial write timed out");
    }
    if (poll_result < 0 && errno != EINTR) {
      throw std::system_error(errno, std::generic_category(), "Serial write poll failed");
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      throw std::runtime_error("Serial port became unavailable while writing");
    }
  }
  if (tcdrain(serial_fd_) != 0) {
    throw std::system_error(errno, std::generic_category(), "Serial drain failed");
  }
}

void MotorControl::set_id(std::uint8_t id)
{
  std::array<std::uint8_t, 10> packet{0xAA, 0x55, 0x53, id, 0, 0, 0, 0, 0, 0};
  packet.back() = crc8_maxim(packet.data(), packet.size() - 1);
  for (int attempt = 0; attempt < 5; ++attempt) {
    write_packet(packet.data(), packet.size());
  }
}

void MotorControl::send_current(std::uint8_t id, float current)
{
  const float limited_current = std::clamp(current, -8.0F, 8.0F);
  const auto raw = static_cast<std::int16_t>(
    map_value(limited_current, -8.0, 8.0, -32767.0, 32767.0));
  const auto bytes = int16_to_bytes(raw);
  std::array<std::uint8_t, 10> packet{id, 0x64, bytes[0], bytes[1], 0, 0, 0, 0, 0, 0};
  packet.back() = crc8_maxim(packet.data(), packet.size() - 1);
  write_packet(packet.data(), packet.size());
  (void)read_reply(id);
}

void MotorControl::send_rpm(std::uint8_t id, std::int16_t rpm)
{
  constexpr std::int16_t minimum_rpm = -330;
  constexpr std::int16_t maximum_rpm = 330;
  const auto bytes = int16_to_bytes(std::clamp(rpm, minimum_rpm, maximum_rpm));
  std::array<std::uint8_t, 10> packet{id, 0x64, bytes[0], bytes[1], 0, 0, 0, 0, 0, 0};
  packet.back() = crc8_maxim(packet.data(), packet.size() - 1);
  write_packet(packet.data(), packet.size());
  (void)read_reply(id);
}

void MotorControl::send_degree(std::uint8_t id, double degrees)
{
  const auto raw = static_cast<std::int16_t>(map_value(degrees, 0.0, 360.0, 0.0, 32767.0));
  const auto bytes = int16_to_bytes(raw);
  std::array<std::uint8_t, 10> packet{id, 0x64, bytes[0], bytes[1], 0, 0, 0, 0, 0, 0};
  packet.back() = crc8_maxim(packet.data(), packet.size() - 1);
  write_packet(packet.data(), packet.size());
  (void)read_reply(id);
}

void MotorControl::set_brake(std::uint8_t id)
{
  std::array<std::uint8_t, 10> packet{id, 0x64, 0, 0, 0, 0, 0, 0xFF, 0, 0};
  packet.back() = crc8_maxim(packet.data(), packet.size() - 1);
  write_packet(packet.data(), packet.size());
  (void)read_reply(id);
}

std::string MotorControl::set_drive_mode(std::uint8_t id, std::uint8_t mode)
{
  const std::array<std::uint8_t, 10> packet{id, 0xA0, 0, 0, 0, 0, 0, 0, 0, mode};
  write_packet(packet.data(), packet.size());
  if (mode == 1) {
    return "Set " + std::to_string(id) + " as current (torque) mode";
  }
  if (mode == 2) {
    return "Set " + std::to_string(id) + " as velocity mode";
  }
  if (mode == 3) {
    return "Set " + std::to_string(id) + " as position mode";
  }
  return "Error " + std::to_string(mode) + " is unknown";
}

MotorFeedback MotorControl::get_motor_feedback(std::uint8_t id)
{
  std::array<std::uint8_t, 10> packet{id, 0x74, 0, 0, 0, 0, 0, 0, 0, 0};
  packet.back() = crc8_maxim(packet.data(), packet.size() - 1);
  write_packet(packet.data(), packet.size());
  return read_reply(id);
}

MotorFeedback MotorControl::read_reply(std::uint8_t id, std::chrono::milliseconds timeout)
{
  std::array<std::uint8_t, 10> buffer{};
  std::size_t used = 0;
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    pollfd descriptor{serial_fd_, POLLIN, 0};
    const int poll_result = ::poll(
      &descriptor, 1,
      static_cast<int>(std::max<std::int64_t>(1, remaining.count())));
    if (poll_result == 0) {
      break;
    }
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::system_error(errno, std::generic_category(), "Serial read poll failed");
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      break;
    }

    std::uint8_t byte = 0;
    const ssize_t count = ::read(serial_fd_, &byte, 1);
    if (count == 0 || (count < 0 && (errno == EAGAIN || errno == EINTR))) {
      continue;
    }
    if (count < 0) {
      break;
    }

    if (used == 0) {
      if (byte == id) {
        buffer[used++] = byte;
      }
      continue;
    }
    if (used == 1) {
      if (byte == 0x01 || byte == 0x02) {
        buffer[used++] = byte;
      } else {
        used = byte == id ? 1U : 0U;
        if (used == 1U) {
          buffer[0] = byte;
        }
      }
      continue;
    }

    buffer[used++] = byte;
    if (used == buffer.size()) {
      if (buffer.back() == crc8_maxim(buffer.data(), buffer.size() - 1)) {
        MotorFeedback feedback;
        feedback.id = buffer[0];
        feedback.mode = buffer[1];
        feedback.current =
          static_cast<float>(map_value(
            bytes_to_int16(buffer[2], buffer[3]), -32767.0, 32767.0,
            -8.0, 8.0));
        feedback.rpm = bytes_to_int16(buffer[4], buffer[5]);
        feedback.winding_temperature = buffer[6];
        feedback.position = buffer[7];
        feedback.error = buffer[8];
        return feedback;
      }
      used = 0;
    }
  }
  return {};
}

}  // namespace ddsm115_controller
