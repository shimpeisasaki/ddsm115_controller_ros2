#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ddsm115_controller/feedback.hpp"
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

struct MotorChannel
{
  std::string name;
  int motor_id;
  std::unique_ptr<MotorControl> driver;
};

class VelocityControl : public rclcpp::Node
{
public:
  VelocityControl()
  : Node("velocity_control_node")
  {
    const std::string left_device = declare_parameter(
      "left_usb_dev", std::string("/dev/ddsm115_left"));
    const std::string right_device = declare_parameter(
      "right_usb_dev", std::string("/dev/ddsm115_right"));
    const int left_motor_id = declare_parameter("left_motor_id", 2);
    const int right_motor_id = declare_parameter("right_motor_id", 1);
    const double command_timeout = declare_parameter("command_timeout", 0.5);
    const double motor_update_period = declare_parameter("motor_update_period", 0.05);
    const double online_timeout = declare_parameter("online_timeout", 0.5);
    const double reconnect_restart_timeout =
      declare_parameter("reconnect_restart_timeout", 2.0);
    const double serial_reply_timeout = declare_parameter("serial_reply_timeout", 0.020);
    const double current_publish_period = declare_parameter("current_publish_period", 1.0);
    const double temperature_publish_period =
      declare_parameter("temperature_publish_period", 1.0);
    const double status_publish_period = declare_parameter("status_publish_period", 0.5);

    if (left_device.empty() || right_device.empty() || left_device == right_device) {
      throw std::invalid_argument(
              "left_usb_dev and right_usb_dev must be distinct non-empty paths");
    }
    if (left_motor_id < 1 || right_motor_id < 1 || left_motor_id == right_motor_id) {
      throw std::invalid_argument("left_motor_id and right_motor_id must be distinct positive IDs");
    }
    if (command_timeout <= 0.0 || motor_update_period <= 0.0 || online_timeout <= 0.0 ||
      reconnect_restart_timeout <= 0.0 ||
      serial_reply_timeout <= 0.0 || current_publish_period <= 0.0 ||
      temperature_publish_period <= 0.0 || status_publish_period <= 0.0)
    {
      throw std::invalid_argument("All timing parameters must be greater than zero");
    }

    const auto reply_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(serial_reply_timeout));
    if (reply_timeout <= std::chrono::milliseconds::zero()) {
      throw std::invalid_argument("serial_reply_timeout must be at least one millisecond");
    }

    channels_.push_back({"left", left_motor_id, std::make_unique<MotorControl>(left_device)});
    channels_.push_back({"right", right_motor_id, std::make_unique<MotorControl>(right_device)});
    for (auto & channel : channels_) {
      channel.driver->set_reply_timeout(reply_timeout);
    }

    command_timeout_ = std::chrono::duration<double>(command_timeout);
    motor_update_period_ = std::chrono::duration<double>(motor_update_period);
    online_timeout_ = std::chrono::duration<double>(online_timeout);
    reconnect_restart_timeout_ = std::chrono::duration<double>(reconnect_restart_timeout);
    current_publish_period_ = std::chrono::duration<double>(current_publish_period);
    temperature_publish_period_ = std::chrono::duration<double>(temperature_publish_period);
    status_publish_period_ = std::chrono::duration<double>(status_publish_period);
    maximum_motor_id_ = std::max(left_motor_id, right_motor_id);

    RCLCPP_INFO(get_logger(), "Start velocity_control_node");
    RCLCPP_INFO(
      get_logger(), "left channel: motor ID %d on %s", left_motor_id,
      left_device.c_str());
    RCLCPP_INFO(
      get_logger(), "right channel: motor ID %d on %s", right_motor_id,
      right_device.c_str());
    RCLCPP_INFO(get_logger(), "motor_update_period: %.3f s", motor_update_period);
    RCLCPP_INFO(get_logger(), "serial_reply_timeout: %.3f s", serial_reply_timeout);
    RCLCPP_INFO(
      get_logger(), "restart after all motors offline: %.3f s", reconnect_restart_timeout);

    rpm_commands_.resize(maximum_motor_id_);
    rpm_feedback_.resize(maximum_motor_id_, 0);
    rpm_feedback_valid_.resize(maximum_motor_id_, false);
    temperature_feedback_.resize(maximum_motor_id_, 0);
    current_feedback_.resize(maximum_motor_id_, 0.0F);
    errors_.resize(maximum_motor_id_, 0);
    last_response_times_.resize(maximum_motor_id_);

    const auto startup_time = std::chrono::steady_clock::now();
    for (auto & channel : channels_) {
      const auto feedback = channel.driver->get_motor_feedback(channel.motor_id);
      if (feedback.id != channel.motor_id) {
        throw std::runtime_error(
                "No reply from " + channel.name + " motor ID " +
                std::to_string(channel.motor_id));
      }
      store_feedback(channel.motor_id, feedback);
      last_response_times_[channel.motor_id - 1] = startup_time;
      RCLCPP_INFO(
        get_logger(), "%s", channel.driver->set_drive_mode(channel.motor_id, 2).c_str());
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
  void store_feedback(int motor_id, const MotorFeedback & feedback)
  {
    const auto index = static_cast<std::size_t>(motor_id - 1);
    rpm_feedback_[index] = feedback.rpm;
    current_feedback_[index] = feedback.current;
    temperature_feedback_[index] = static_cast<std::int8_t>(feedback.winding_temperature);
    errors_[index] = static_cast<std::int8_t>(feedback.error);
    rpm_feedback_valid_[index] = true;
  }

  MotorFeedback exchange_with_motor(MotorChannel & channel)
  {
    if (freewheel_enabled_) {
      return channel.driver->get_motor_feedback(channel.motor_id);
    }
    if (brake_enabled_) {
      return channel.driver->set_brake(channel.motor_id);
    }
    return channel.driver->send_rpm(
      channel.motor_id, rpm_commands_[channel.motor_id - 1].value_or(0));
  }

  void set_brake_enabled(bool enabled)
  {
    try {
      if (enabled && freewheel_enabled_) {
        for (auto & channel : channels_) {
          channel.driver->send_current(channel.motor_id, 0.0F);
          RCLCPP_INFO(
            get_logger(), "%s", channel.driver->set_drive_mode(channel.motor_id, 2).c_str());
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
      std::fill(rpm_commands_.begin(), rpm_commands_.end(), std::int16_t{0});
      if (request->data) {
        brake_enabled_ = false;
        for (auto & channel : channels_) {
          RCLCPP_INFO(
            get_logger(), "%s", channel.driver->set_drive_mode(channel.motor_id, 1).c_str());
          channel.driver->send_current(channel.motor_id, 0.0F);
        }
        freewheel_enabled_ = true;
        response->message = "Freewheel enabled";
      } else {
        for (auto & channel : channels_) {
          channel.driver->send_current(channel.motor_id, 0.0F);
          RCLCPP_INFO(
            get_logger(), "%s", channel.driver->set_drive_mode(channel.motor_id, 2).c_str());
          channel.driver->send_rpm(channel.motor_id, 0);
        }
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
    if (!freewheel_enabled_ && !brake_enabled_ &&
      update_start - last_command_time_ > command_timeout_)
    {
      std::fill(rpm_commands_.begin(), rpm_commands_.end(), std::int16_t{0});
    }

    for (auto & channel : channels_) {
      const auto index = static_cast<std::size_t>(channel.motor_id - 1);
      try {
        const auto feedback = exchange_with_motor(channel);
        if (feedback.id == channel.motor_id) {
          store_feedback(channel.motor_id, feedback);
          last_response_times_[index] = update_start;
        } else {
          rpm_feedback_valid_[index] = false;
        }
      } catch (const std::exception & error) {
        rpm_feedback_valid_[index] = false;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000, "%s motor ID %d communication failed: %s",
          channel.name.c_str(), channel.motor_id, error.what());
      }
    }

    online_ids_.clear();
    for (const auto & channel : channels_) {
      const auto index = static_cast<std::size_t>(channel.motor_id - 1);
      if (update_start - last_response_times_[index] <= online_timeout_) {
        online_ids_.push_back(channel.motor_id);
      }
    }
    std::sort(online_ids_.begin(), online_ids_.end());
    if (online_ids_.size() == channels_.size()) {
      all_motors_offline_since_.reset();
    } else if (!all_motors_offline_since_) {
      all_motors_offline_since_ = update_start;
    } else if (update_start - *all_motors_offline_since_ > reconnect_restart_timeout_) {
      RCLCPP_ERROR(
        get_logger(), "One or more motors offline for %.3f s; exiting to reopen USB serial devices",
        reconnect_restart_timeout_.count());
      rclcpp::shutdown();
      return;
    }
    publish_feedback(update_start);
  }

  void publish_feedback(const std::chrono::steady_clock::time_point & publish_time)
  {
    std_msgs::msg::Int16MultiArray rpm_message;
    rpm_message.data = rpm_feedback_;
    for (std::size_t index = 0; index < rpm_message.data.size(); ++index) {
      if (!rpm_feedback_valid_[index]) {
        rpm_message.data[index] = invalid_rpm;
      }
    }
    rpm_publisher_->publish(rpm_message);

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
  }

  std::vector<MotorChannel> channels_;
  std::vector<int> online_ids_;
  std::vector<std::optional<std::int16_t>> rpm_commands_;
  std::vector<std::int16_t> rpm_feedback_;
  std::vector<bool> rpm_feedback_valid_;
  std::vector<std::int8_t> temperature_feedback_;
  std::vector<float> current_feedback_;
  std::vector<std::int8_t> errors_;
  bool brake_enabled_{false};
  bool freewheel_enabled_{false};
  int maximum_motor_id_{0};
  std::chrono::duration<double> command_timeout_{0.5};
  std::chrono::duration<double> motor_update_period_{0.05};
  std::chrono::duration<double> online_timeout_{0.5};
  std::chrono::duration<double> reconnect_restart_timeout_{2.0};
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
  std::vector<std::chrono::steady_clock::time_point> last_response_times_;
  std::optional<std::chrono::steady_clock::time_point> all_motors_offline_since_;
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
