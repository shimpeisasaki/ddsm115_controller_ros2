#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include "ddsm115_controller/motor_control.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("set_motor_id_node");
  const std::string device = node->declare_parameter("usb_dev", std::string("/dev/ttyUSB0"));
  RCLCPP_INFO(node->get_logger(), "Start set_motor_id_node");
  RCLCPP_INFO(node->get_logger(), "usb_dev: %s", device.c_str());
  RCLCPP_WARN(node->get_logger(), "Make sure only one motor is connected to the bus");

  int id = 0;
  while (id < 1 || id > 255) {
    std::cout << "Input motor ID (1-255), then press [Enter]: " << std::flush;
    if (!(std::cin >> id)) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      id = 0;
    }
  }

  try {
    ddsm115_controller::MotorControl driver(device);
    driver.set_id(static_cast<std::uint8_t>(id));
    RCLCPP_INFO(node->get_logger(), "Motor ID is set to %d", id);
    RCLCPP_INFO(node->get_logger(), "Please restart the motor");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
