#include <chrono>
#include <memory>
#include <string>

#include "ddsm115_controller/motor_control.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

bool check_channel(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::string & device, int motor_id, int attempts,
  std::chrono::milliseconds reply_timeout)
{
  ddsm115_controller::MotorControl driver(device);
  driver.set_reply_timeout(reply_timeout);
  int replies = 0;
  for (int attempt = 0; attempt < attempts; ++attempt) {
    if (driver.get_motor_feedback(motor_id).id == motor_id) {
      ++replies;
    }
  }
  RCLCPP_INFO(
    node->get_logger(), "%s channel: motor ID %d replies: %d/%d (%s)",
    name.c_str(), motor_id, replies, attempts, device.c_str());
  return replies == attempts;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("check_motor_channels_node");
  const std::string left_device = node->declare_parameter(
    "left_usb_dev", std::string("/dev/ddsm115_left"));
  const std::string right_device = node->declare_parameter(
    "right_usb_dev", std::string("/dev/ddsm115_right"));
  const int left_motor_id = node->declare_parameter("left_motor_id", 2);
  const int right_motor_id = node->declare_parameter("right_motor_id", 1);
  const int attempts = node->declare_parameter("attempts", 10);
  const double reply_timeout = node->declare_parameter("reply_timeout", 0.020);
  if (left_device.empty() || right_device.empty() || left_device == right_device ||
    left_motor_id < 1 || right_motor_id < 1 || left_motor_id == right_motor_id ||
    attempts < 1 || reply_timeout <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid channel, motor ID, or timing parameter");
    rclcpp::shutdown();
    return 1;
  }
  const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(reply_timeout));
  if (timeout <= std::chrono::milliseconds::zero()) {
    RCLCPP_ERROR(node->get_logger(), "reply_timeout must be at least one millisecond");
    rclcpp::shutdown();
    return 1;
  }

  try {
    const bool left_ok = check_channel(
      node, "left", left_device, left_motor_id, attempts, timeout);
    const bool right_ok = check_channel(
      node, "right", right_device, right_motor_id, attempts, timeout);
    rclcpp::shutdown();
    return left_ok && right_ok ? 0 : 2;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
