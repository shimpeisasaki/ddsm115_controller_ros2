#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ddsm115_controller/motor_control.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("check_motor_id_node");
  const int max_check = node->declare_parameter("max_check", 10);
  const int attempts = node->declare_parameter("attempts", 3);
  const double reply_timeout = node->declare_parameter("reply_timeout", 0.020);
  const std::string device = node->declare_parameter("usb_dev", std::string("/dev/ttyUSB0"));
  if (max_check < 1 || max_check > 255 || attempts < 1 || reply_timeout <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Invalid max_check, attempts, or reply_timeout");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Start check_motor_id_node");
  RCLCPP_INFO(node->get_logger(), "max_check: %d", max_check);
  RCLCPP_INFO(node->get_logger(), "attempts: %d, reply_timeout: %.3f s", attempts, reply_timeout);
  RCLCPP_INFO(node->get_logger(), "usb_dev: %s", device.c_str());

  try {
    ddsm115_controller::MotorControl driver(device);
    driver.set_reply_timeout(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(reply_timeout)));
    std::vector<int> online_ids;
    std::vector<int> responses(max_check, 0);
    for (int attempt = 0; attempt < attempts; ++attempt) {
      for (int id = 1; id <= max_check; ++id) {
        const auto feedback = driver.get_motor_feedback(static_cast<std::uint8_t>(id));
        if (feedback.id >= 0) {
          ++responses[id - 1];
        }
      }
    }
    for (int id = 1; id <= max_check; ++id) {
      if (responses[id - 1] > 0) {
        online_ids.push_back(id);
      }
    }
    std::string result = "[";
    for (std::size_t index = 0; index < online_ids.size(); ++index) {
      result += (index == 0 ? "" : ", ") + std::to_string(online_ids[index]);
    }
    result += "]";
    RCLCPP_INFO(node->get_logger(), "Online ID is %s", result.c_str());
    for (int id = 1; id <= max_check; ++id) {
      if (responses[id - 1] > 0) {
        RCLCPP_INFO(
          node->get_logger(), "Motor %d replies: %d/%d", id, responses[id - 1], attempts);
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
