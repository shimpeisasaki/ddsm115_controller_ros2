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
#include "std_srvs/srv/set_bool.hpp"

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
    const double command_timeout = declare_parameter("command_timeout", 2.0);
    if (max_check < 1 || max_check > 255) {
      throw std::invalid_argument("max_check must be between 1 and 255");
    }
    if (command_timeout <= 0.0) {
      throw std::invalid_argument("command_timeout must be greater than zero");
    }
    command_timeout_ = std::chrono::duration<double>(command_timeout);
    RCLCPP_INFO(get_logger(), "Start velocity_control_node");
    RCLCPP_INFO(get_logger(), "max_check: %d", max_check);
    RCLCPP_INFO(get_logger(), "command_timeout: %.3f s", command_timeout);

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
        std::fill(rpm_commands_.begin(), rpm_commands_.end(), std::int16_t{0});
        const std::size_t count = std::min(message.data.size(), rpm_commands_.size());
        for (std::size_t index = 0; index < count; ++index) {
          rpm_commands_[index] = message.data[index];
        }
        last_command_time_ = std::chrono::steady_clock::now();
      });
    brake_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/ddsm115/brake", qos,
      [this](const std_msgs::msg::Bool & message) {set_brake_enabled(message.data);});
    freewheel_service_ = create_service<std_srvs::srv::SetBool>(
      "/ddsm115/set_freewheel",
      std::bind(
        &VelocityControl::set_freewheel, this, std::placeholders::_1,
        std::placeholders::_2));
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
    for (std::size_t index = 0; index < rpm_commands_.size(); ++index) {
      const int id = static_cast<int>(index) + 1;
      if (rpm_commands_[index].has_value() && is_online(id)) {
        driver_.send_rpm(id, *rpm_commands_[index]);
      }
    }
  }

  void set_brake_enabled(bool enabled)
  {
    try {
      if (enabled && freewheel_enabled_) {
        for (const int id : online_ids_) {
          driver_.send_current(id, 0.0F);
          RCLCPP_INFO(get_logger(), "%s", driver_.set_drive_mode(id, 2).c_str());
        }
        freewheel_enabled_ = false;
      }
      std::fill(rpm_commands_.begin(), rpm_commands_.end(), std::int16_t{0});
      brake_enabled_ = enabled;
      last_command_time_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(get_logger(), "Brake %s", enabled ? "enabled" : "released");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to change brake state: %s", error.what());
    }
  }

  void set_freewheel(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    try {
      if (request->data) {
        std::fill(rpm_commands_.begin(), rpm_commands_.end(), std::int16_t{0});
        brake_enabled_ = false;
        for (const int id : online_ids_) {
          RCLCPP_INFO(get_logger(), "%s", driver_.set_drive_mode(id, 1).c_str());
          driver_.send_current(id, 0.0F);
        }
        freewheel_enabled_ = true;
        response->message = "Freewheel enabled";
      } else {
        for (const int id : online_ids_) {
          driver_.send_current(id, 0.0F);
          RCLCPP_INFO(get_logger(), "%s", driver_.set_drive_mode(id, 2).c_str());
          driver_.send_rpm(id, 0);
        }
        std::fill(rpm_commands_.begin(), rpm_commands_.end(), std::int16_t{0});
        brake_enabled_ = false;
        freewheel_enabled_ = false;
        last_command_time_ = std::chrono::steady_clock::now();
        response->message = "Freewheel disabled; velocity mode restored at 0 RPM";
      }
      response->success = true;
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Failed to change freewheel mode: %s", error.what());
    }
  }

  void update()
  {
    try {
      if (freewheel_enabled_) {
        // Keep the zero-current command active without sending velocity or brake commands.
      } else if (brake_enabled_) {
        for (const int id : online_ids_) {
          driver_.set_brake(id);
        }
      } else if (std::chrono::steady_clock::now() - last_command_time_ > command_timeout_) {
        for (const int id : online_ids_) {
          rpm_commands_[id - 1] = 0;
        }
        send_rpm_commands();
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
  bool freewheel_enabled_{false};
  std::chrono::duration<double> command_timeout_{2.0};
  std::chrono::steady_clock::time_point last_command_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_slow_publish_time_{std::chrono::steady_clock::now()};
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rpm_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brake_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr freewheel_service_;
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
