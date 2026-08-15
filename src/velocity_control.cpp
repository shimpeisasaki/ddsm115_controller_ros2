#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ddsm115_controller/motor_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

using namespace std::chrono_literals;

namespace ddsm115_controller
{

class VelocityControl : public rclcpp::Node
{
public:
  VelocityControl()
  : Node("velocity_control_node"),
    driver_(declare_parameter("usb_dev", std::string("/dev/ttyUSB0")))
  {
    const int max_check = declare_parameter("max_check", 10);
    RCLCPP_INFO(get_logger(), "Start velocity_control_node");
    RCLCPP_INFO(get_logger(), "max_check: %d", max_check);

    for (int id = 1; id <= max_check; ++id) {
      const auto feedback = driver_.get_motor_feedback(static_cast<std::uint8_t>(id));
      if (feedback.id >= 0) {
        online_ids_.push_back(feedback.id);
      }
    }
    if (online_ids_.empty()) {
      throw std::runtime_error("No motor detected");
    }
    const int last_id = *std::max_element(online_ids_.begin(), online_ids_.end());
    RCLCPP_INFO(get_logger(), "Detected %zu motor(s), last ID %d", online_ids_.size(), last_id);
    for (const int id : online_ids_) {
      RCLCPP_INFO(get_logger(), "%s", driver_.set_drive_mode(id, 2).c_str());
    }

    rpm_commands_.resize(std::max(10, last_id));
    rpm_feedback_.resize(last_id, 0);
    temperature_feedback_.resize(last_id, 0);
    current_feedback_.resize(last_id, 0.0F);
    errors_.resize(last_id, 0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    rpm_command_subscription_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "/ddsm115/rpm_cmd", qos,
      [this](const std_msgs::msg::Int16MultiArray & message) {
        const std::size_t count = std::min(message.data.size(), rpm_commands_.size());
        for (std::size_t index = 0; index < count; ++index) {
          rpm_commands_[index] = message.data[index];
        }
        last_command_time_ = std::chrono::steady_clock::now();
      });
    brake_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/ddsm115/brake", qos,
      [this](const std_msgs::msg::Bool & message) {brake_enabled_ = message.data;});
    rpm_publisher_ = create_publisher<std_msgs::msg::Int16MultiArray>("/ddsm115/rpm_fb", qos);
    current_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("/ddsm115/cur_fb", qos);
    temperature_publisher_ =
      create_publisher<std_msgs::msg::Int8MultiArray>("/ddsm115/temp_fb", qos);
    error_publisher_ = create_publisher<std_msgs::msg::Int8MultiArray>("/ddsm115/error", qos);
    online_id_publisher_ =
      create_publisher<std_msgs::msg::Int8MultiArray>("/ddsm115/online_id", qos);
    timer_ = create_wall_timer(10ms, std::bind(&VelocityControl::update, this));
  }

private:
  bool is_online(int id) const
  {
    return std::find(online_ids_.begin(), online_ids_.end(), id) != online_ids_.end();
  }

  void send_rpm_commands()
  {
    brake_enabled_ = false;
    for (std::size_t index = 0; index < rpm_commands_.size(); ++index) {
      const int id = static_cast<int>(index) + 1;
      if (rpm_commands_[index].has_value() && is_online(id)) {
        driver_.send_rpm(id, *rpm_commands_[index]);
      }
    }
  }

  void update()
  {
    try {
      if (std::chrono::steady_clock::now() - last_command_time_ > 2s) {
        for (const int id : online_ids_) {
          if (brake_enabled_) {
            driver_.set_brake(id);
          } else {
            rpm_commands_[id - 1] = 0;
          }
        }
        if (!brake_enabled_) {
          send_rpm_commands();
        }
      } else {
        send_rpm_commands();
      }

      for (const int id : online_ids_) {
        const auto feedback = driver_.get_motor_feedback(id);
        if (feedback.id >= 0) {
          rpm_feedback_[id - 1] = feedback.rpm;
          temperature_feedback_[id - 1] = static_cast<std::int8_t>(feedback.winding_temperature);
          current_feedback_[id - 1] = feedback.current;
          errors_[id - 1] = static_cast<std::int8_t>(feedback.error);
        }
      }

      std_msgs::msg::Int16MultiArray rpm_message;
      rpm_message.data = rpm_feedback_;
      rpm_publisher_->publish(rpm_message);
      std_msgs::msg::Float32MultiArray current_message;
      current_message.data = current_feedback_;
      current_publisher_->publish(current_message);

      if (std::chrono::steady_clock::now() - last_slow_publish_time_ > 100ms) {
        std_msgs::msg::Int8MultiArray temperature_message;
        temperature_message.data = temperature_feedback_;
        temperature_publisher_->publish(temperature_message);
        std_msgs::msg::Int8MultiArray error_message;
        error_message.data = errors_;
        error_publisher_->publish(error_message);
        std_msgs::msg::Int8MultiArray online_message;
        online_message.data.assign(online_ids_.begin(), online_ids_.end());
        online_id_publisher_->publish(online_message);
        last_slow_publish_time_ = std::chrono::steady_clock::now();
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", error.what());
    }
  }

  MotorControl driver_;
  std::vector<int> online_ids_;
  std::vector<std::optional<std::int16_t>> rpm_commands_;
  std::vector<std::int16_t> rpm_feedback_;
  std::vector<std::int8_t> temperature_feedback_;
  std::vector<float> current_feedback_;
  std::vector<std::int8_t> errors_;
  bool brake_enabled_{false};
  std::chrono::steady_clock::time_point last_command_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_slow_publish_time_{std::chrono::steady_clock::now()};
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rpm_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brake_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr rpm_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr current_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr temperature_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr error_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr online_id_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ddsm115_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ddsm115_controller::VelocityControl>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("velocity_control_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
