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
  const std::string device = node->declare_parameter("usb_dev", std::string("/dev/ttyUSB0"));
  RCLCPP_INFO(node->get_logger(), "Start check_motor_id_node");
  RCLCPP_INFO(node->get_logger(), "max_check: %d", max_check);
  RCLCPP_INFO(node->get_logger(), "usb_dev: %s", device.c_str());

  try {
    ddsm115_controller::MotorControl driver(device);
    std::vector<int> online_ids;
    for (int id = 1; id <= max_check; ++id) {
      const auto feedback = driver.get_motor_feedback(static_cast<std::uint8_t>(id));
      if (feedback.id >= 0) {
        online_ids.push_back(feedback.id);
      }
    }
    std::string result = "[";
    for (std::size_t index = 0; index < online_ids.size(); ++index) {
      result += (index == 0 ? "" : ", ") + std::to_string(online_ids[index]);
    }
    result += "]";
    RCLCPP_INFO(node->get_logger(), "Online ID is %s", result.c_str());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
