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
#include "std_msgs/msg/u_int8_multi_array.hpp"
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
    const double motor_update_period = declare_parameter("motor_update_period", 0.01);
    const double online_timeout = declare_parameter("online_timeout", 0.5);
    const double current_publish_period = declare_parameter("current_publish_period", 1.0);
    const double temperature_publish_period =
      declare_parameter("temperature_publish_period", 1.0);
    const double status_publish_period = declare_parameter("status_publish_period", 0.5);
    if (max_check < 1 || max_check > 255) {
      throw std::invalid_argument("max_check must be between 1 and 255");
    }
    if (command_timeout <= 0.0) {
      throw std::invalid_argument("command_timeout must be greater than zero");
    }
    if (motor_update_period <= 0.0) {
      throw std::invalid_argument("motor_update_period must be greater than zero");
    }
    if (online_timeout <= 0.0) {
      throw std::invalid_argument("online_timeout must be greater than zero");
    }
    if (current_publish_period <= 0.0 || temperature_publish_period <= 0.0 ||
      status_publish_period <= 0.0)
    {
      throw std::invalid_argument("feedback publish periods must be greater than zero");
    }
    command_timeout_ = std::chrono::duration<double>(command_timeout);
    motor_update_period_ = std::chrono::duration<double>(motor_update_period);
    online_timeout_ = std::chrono::duration<double>(online_timeout);
    current_publish_period_ = std::chrono::duration<double>(current_publish_period);
    temperature_publish_period_ = std::chrono::duration<double>(temperature_publish_period);
    status_publish_period_ = std::chrono::duration<double>(status_publish_period);
    max_check_ = max_check;
    RCLCPP_INFO(get_logger(), "Start velocity_control_node");
    RCLCPP_INFO(get_logger(), "max_check: %d", max_check);
    RCLCPP_INFO(get_logger(), "command_timeout: %.3f s", command_timeout);
    RCLCPP_INFO(get_logger(), "motor_update_period: %.3f s", motor_update_period);
    RCLCPP_INFO(get_logger(), "online_timeout: %.3f s", online_timeout);
    RCLCPP_INFO(get_logger(), "current_publish_period: %.3f s", current_publish_period);
    RCLCPP_INFO(get_logger(), "temperature_publish_period: %.3f s", temperature_publish_period);
    RCLCPP_INFO(get_logger(), "status_publish_period: %.3f s", status_publish_period);

    for (int id = 1; id <= max_check; ++id) {
      const auto feedback = driver_.get_motor_feedback(static_cast<std::uint8_t>(id));
      if (feedback.id >= 0) {
        managed_ids_.push_back(feedback.id);
      }
    }
    if (managed_ids_.empty()) {
      throw std::runtime_error("No motor detected");
    }
    online_ids_ = managed_ids_;
    const int last_id = *std::max_element(managed_ids_.begin(), managed_ids_.end());
    RCLCPP_INFO(get_logger(), "Detected %zu motor(s), last ID %d", managed_ids_.size(), last_id);
    for (const int id : managed_ids_) {
      RCLCPP_INFO(get_logger(), "%s", driver_.set_drive_mode(id, 2).c_str());
    }

    rpm_commands_.resize(max_check);
    rpm_feedback_.resize(max_check, 0);
    temperature_feedback_.resize(max_check, 0);
    current_feedback_.resize(max_check, 0.0F);
    errors_.resize(max_check, 0);
    response_miss_counts_.resize(max_check, 0);
    last_response_times_.resize(max_check);
    const auto startup_time = std::chrono::steady_clock::now();
    for (const int id : managed_ids_) {
      last_response_times_[id - 1] = startup_time;
    }

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
      create_publisher<std_msgs::msg::UInt8MultiArray>("/ddsm115/online_id", qos);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(motor_update_period_),
      std::bind(&VelocityControl::update, this));
  }

private:
  bool is_managed(int id) const
  {
    return std::find(managed_ids_.begin(), managed_ids_.end(), id) != managed_ids_.end();
  }

  void configure_discovered_motor(int id)
  {
    managed_ids_.push_back(id);
    std::sort(managed_ids_.begin(), managed_ids_.end());
    response_miss_counts_[id - 1] = 0;
    last_response_times_[id - 1] = std::chrono::steady_clock::now();
    if (freewheel_enabled_) {
      RCLCPP_INFO(get_logger(), "%s", driver_.set_drive_mode(id, 1).c_str());
      (void)driver_.send_current(id, 0.0F);
    } else {
      RCLCPP_INFO(get_logger(), "%s", driver_.set_drive_mode(id, 2).c_str());
      if (brake_enabled_) {
        (void)driver_.set_brake(id);
      } else {
        (void)driver_.send_rpm(id, 0);
      }
    }
    RCLCPP_INFO(get_logger(), "Discovered motor ID %d", id);
  }

  void scan_missing_motor(const std::chrono::steady_clock::time_point & now)
  {
    if (now - last_rescan_time_ < 1s ||
      managed_ids_.size() >= static_cast<std::size_t>(max_check_))
    {
      return;
    }
    for (int offset = 0; offset < max_check_; ++offset) {
      const int id = (rescan_id_ + offset - 1) % max_check_ + 1;
      if (!is_managed(id)) {
        rescan_id_ = id % max_check_ + 1;
        last_rescan_time_ = now;
        try {
          const auto feedback = driver_.get_motor_feedback(id);
          if (feedback.id >= 0) {
            configure_discovered_motor(id);
          }
        } catch (const std::exception & error) {
          RCLCPP_DEBUG(get_logger(), "Motor %d rescan failed: %s", id, error.what());
        }
        return;
      }
    }
  }

  void store_fast_feedback(int id, const MotorFeedback & feedback)
  {
    rpm_feedback_[id - 1] = feedback.rpm;
    current_feedback_[id - 1] = feedback.current;
    temperature_feedback_[id - 1] =
      static_cast<std::int8_t>(feedback.winding_temperature);
    errors_[id - 1] = static_cast<std::int8_t>(feedback.error);
  }

  MotorFeedback exchange_with_motor(int id)
  {
    if (freewheel_enabled_) {
      return driver_.get_motor_feedback(id);
    }
    if (brake_enabled_) {
      return driver_.set_brake(id);
    }
    return driver_.send_rpm(id, rpm_commands_[id - 1].value_or(0));
  }

  void set_brake_enabled(bool enabled)
  {
    try {
      if (enabled && freewheel_enabled_) {
        for (const int id : managed_ids_) {
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
        for (const int id : managed_ids_) {
          RCLCPP_INFO(get_logger(), "%s", driver_.set_drive_mode(id, 1).c_str());
          driver_.send_current(id, 0.0F);
        }
        freewheel_enabled_ = true;
        response->message = "Freewheel enabled";
      } else {
        for (const int id : managed_ids_) {
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
    const auto update_start = std::chrono::steady_clock::now();
    try {
      scan_missing_motor(update_start);
      if (!freewheel_enabled_ && !brake_enabled_ &&
        update_start - last_command_time_ > command_timeout_)
      {
        for (const int id : managed_ids_) {
          rpm_commands_[id - 1] = 0;
        }
      }

      for (const int id : managed_ids_) {
        try {
          const auto feedback = exchange_with_motor(id);
          if (feedback.id >= 0) {
            store_fast_feedback(id, feedback);
            response_miss_counts_[id - 1] = 0;
            last_response_times_[id - 1] = update_start;
          } else {
            response_miss_counts_[id - 1] =
              std::min(response_miss_counts_[id - 1] + 1U, 3U);
          }
        } catch (const std::exception & error) {
          response_miss_counts_[id - 1] =
            std::min(response_miss_counts_[id - 1] + 1U, 3U);
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000, "Motor %d communication failed: %s",
            id, error.what());
        }
        if (response_miss_counts_[id - 1] >= 3) {
          rpm_feedback_[id - 1] = 0;
          current_feedback_[id - 1] = 0.0F;
          errors_[id - 1] = 0;
        }
      }
      online_ids_.clear();
      for (const int id : managed_ids_) {
        if (update_start - last_response_times_[id - 1] <= online_timeout_) {
          online_ids_.push_back(id);
        } else {
          rpm_feedback_[id - 1] = 0;
          current_feedback_[id - 1] = 0.0F;
          errors_[id - 1] = 0;
          temperature_feedback_[id - 1] = 0;
        }
      }

      std_msgs::msg::Int16MultiArray rpm_message;
      rpm_message.data = rpm_feedback_;
      rpm_publisher_->publish(rpm_message);

      const auto publish_time = std::chrono::steady_clock::now();
      if (publish_time - last_current_publish_time_ >= current_publish_period_) {
        std_msgs::msg::Float32MultiArray current_message;
        current_message.data = current_feedback_;
        current_publisher_->publish(current_message);
        last_current_publish_time_ = publish_time;
      }
      if (publish_time - last_temperature_publish_time_ >= temperature_publish_period_) {
        std_msgs::msg::Int8MultiArray temperature_message;
        temperature_message.data = temperature_feedback_;
        temperature_publisher_->publish(temperature_message);
        last_temperature_publish_time_ = publish_time;
      }
      if (publish_time - last_status_publish_time_ >= status_publish_period_) {
        std_msgs::msg::Int8MultiArray error_message;
        error_message.data = errors_;
        error_publisher_->publish(error_message);
        std_msgs::msg::UInt8MultiArray online_message;
        online_message.data.assign(online_ids_.begin(), online_ids_.end());
        online_id_publisher_->publish(online_message);
        last_status_publish_time_ = publish_time;
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", error.what());
    }
    const auto elapsed = std::chrono::steady_clock::now() - update_start;
    if (elapsed > motor_update_period_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Motor update overrun: %.3f ms (target %.3f ms)",
        std::chrono::duration<double, std::milli>(elapsed).count(),
        std::chrono::duration<double, std::milli>(motor_update_period_).count());
    }
  }

  MotorControl driver_;
  std::vector<int> managed_ids_;
  std::vector<int> online_ids_;
  std::vector<std::optional<std::int16_t>> rpm_commands_;
  std::vector<std::int16_t> rpm_feedback_;
  std::vector<std::int8_t> temperature_feedback_;
  std::vector<float> current_feedback_;
  std::vector<std::int8_t> errors_;
  std::vector<unsigned int> response_miss_counts_;
  std::vector<std::chrono::steady_clock::time_point> last_response_times_;
  bool brake_enabled_{false};
  bool freewheel_enabled_{false};
  int max_check_{10};
  int rescan_id_{1};
  std::chrono::duration<double> command_timeout_{2.0};
  std::chrono::duration<double> motor_update_period_{0.01};
  std::chrono::duration<double> online_timeout_{0.5};
  std::chrono::duration<double> current_publish_period_{1.0};
  std::chrono::duration<double> temperature_publish_period_{1.0};
  std::chrono::duration<double> status_publish_period_{0.5};
  std::chrono::steady_clock::time_point last_command_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_current_publish_time_{
    std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_temperature_publish_time_{
    std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_status_publish_time_{
    std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_rescan_time_{
    std::chrono::steady_clock::now()};
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rpm_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brake_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr freewheel_service_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr rpm_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr current_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr temperature_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr error_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr online_id_publisher_;
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
