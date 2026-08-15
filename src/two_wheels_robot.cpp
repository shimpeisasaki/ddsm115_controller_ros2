#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace ddsm115_controller
{

class TwoWheelsRobot : public rclcpp::Node
{
public:
  TwoWheelsRobot()
  : Node("two_wheels_robot_node"),
    wheel_base_(declare_parameter("wheel_base", 0.207)),
    wheel_radius_(declare_parameter("R_wheel", 0.051)),
    publish_tf_(declare_parameter("pub_tf", false)),
    joystick_timeout_(declare_parameter("joystick_timeout", 2.0)),
    feedback_timeout_(declare_parameter("feedback_timeout", 0.2)),
    enable_joystick_(declare_parameter("enable_joystick", true)),
    left_motor_id_(declare_parameter("left_motor_id", 1)),
    right_motor_id_(declare_parameter("right_motor_id", 2)),
    left_direction_(declare_parameter("left_direction", 1)),
    right_direction_(declare_parameter("right_direction", -1)),
    transform_broadcaster_(*this)
  {
    if (wheel_base_ <= 0.0) {
      throw std::invalid_argument("wheel_base must be greater than zero");
    }
    if (wheel_radius_ <= 0.0) {
      throw std::invalid_argument("R_wheel must be greater than zero");
    }
    if (joystick_timeout_ <= 0.0) {
      throw std::invalid_argument("joystick_timeout must be greater than zero");
    }
    if (feedback_timeout_ <= 0.0) {
      throw std::invalid_argument("feedback_timeout must be greater than zero");
    }
    if (left_motor_id_ < 1 || right_motor_id_ < 1 || left_motor_id_ == right_motor_id_) {
      throw std::invalid_argument("left_motor_id and right_motor_id must be distinct positive IDs");
    }
    if (std::abs(left_direction_) != 1 || std::abs(right_direction_) != 1) {
      throw std::invalid_argument("motor directions must be 1 or -1");
    }
    RCLCPP_INFO(get_logger(), "Start two_wheels_robot_node");
    RCLCPP_INFO(get_logger(), "wheel_base: %.3f", wheel_base_);
    RCLCPP_INFO(get_logger(), "R_wheel: %.3f", wheel_radius_);
    RCLCPP_INFO(get_logger(), "pub_tf: %s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "joystick_timeout: %.3f s", joystick_timeout_);
    RCLCPP_INFO(get_logger(), "feedback_timeout: %.3f s", feedback_timeout_);
    RCLCPP_INFO(get_logger(), "enable_joystick: %s", enable_joystick_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(), "left motor: ID %d, direction %d", left_motor_id_, left_direction_);
    RCLCPP_INFO(
      get_logger(), "right motor: ID %d, direction %d", right_motor_id_, right_direction_);

    auto motor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    rpm_publisher_ =
      create_publisher<std_msgs::msg::Int16MultiArray>("/ddsm115/rpm_cmd", motor_qos);
    brake_publisher_ = create_publisher<std_msgs::msg::Bool>("/ddsm115/brake", motor_qos);
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    freewheel_client_ = create_client<std_srvs::srv::SetBool>("/ddsm115/set_freewheel");
    rpm_subscription_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "/ddsm115/rpm_fb", motor_qos,
      [this](const std_msgs::msg::Int16MultiArray & message) {
        const auto required_size = static_cast<std::size_t>(
          std::max(left_motor_id_, right_motor_id_));
        if (message.data.size() >= required_size) {
          left_rpm_ = message.data[left_motor_id_ - 1] * left_direction_;
          right_rpm_ = message.data[right_motor_id_ - 1] * right_direction_;
          last_feedback_time_ = std::chrono::steady_clock::now();
        }
      });
    velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&TwoWheelsRobot::velocity_command, this, std::placeholders::_1));
    if (enable_joystick_) {
      joystick_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&TwoWheelsRobot::joystick_command, this, std::placeholders::_1));
    }
    timer_ = create_wall_timer(10ms, std::bind(&TwoWheelsRobot::update, this));
  }

private:
  double linear_to_rpm(double velocity) const
  {
    return (60.0 / (2.0 * M_PI)) * (velocity / wheel_radius_);
  }

  double rpm_to_linear(double rpm) const
  {
    return rpm * ((2.0 * M_PI) / 60.0) * wheel_radius_;
  }

  static std::pair<double, double> mix_axes(double x, double y)
  {
    double left = y + x;
    double right = y - x;
    const double maximum = std::max({1.0, std::abs(left), std::abs(right)});
    if (maximum > 1.0) {
      left /= maximum;
      right /= maximum;
    }
    return {left, right};
  }

  void publish_rpm(double left, double right)
  {
    constexpr double maximum_rpm = 330.0;
    const double scale = maximum_rpm / std::max({maximum_rpm, std::abs(left), std::abs(right)});
    const auto limited_left = static_cast<std::int16_t>(std::lround(left * scale));
    const auto limited_right = static_cast<std::int16_t>(std::lround(right * scale));
    std_msgs::msg::Int16MultiArray message;
    message.data.resize(std::max(left_motor_id_, right_motor_id_), 0);
    message.data[left_motor_id_ - 1] = limited_left * left_direction_;
    message.data[right_motor_id_ - 1] = limited_right * right_direction_;
    rpm_publisher_->publish(message);
  }

  void publish_brake(bool enabled)
  {
    std_msgs::msg::Bool message;
    message.data = enabled;
    brake_publisher_->publish(message);
  }

  void request_freewheel(bool enabled, bool enable_drive_after_response)
  {
    if (!freewheel_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Freewheel service is not available");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    freewheel_client_->async_send_request(
      request,
      [this, enabled, enable_drive_after_response](
        rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!response->success) {
            RCLCPP_ERROR(get_logger(), "Motor state change failed: %s", response->message.c_str());
            return;
          }
          if (enable_drive_after_response) {
            publish_brake(false);
            cart_mode_ = 1;
          }
          RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Motor state change failed: %s", error.what());
        }
      });
  }

  void velocity_command(const geometry_msgs::msg::Twist & message)
  {
    if (cart_mode_ != 2) {
      return;
    }
    const double linear = message.linear.x;
    const double angular = message.angular.z;
    if (!std::isfinite(linear) || !std::isfinite(angular)) {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite cmd_vel");
      publish_rpm(0.0, 0.0);
      return;
    }
    const double left_velocity = linear - angular * wheel_base_ / 2.0;
    const double right_velocity = linear + angular * wheel_base_ / 2.0;
    publish_rpm(
      linear_to_rpm(left_velocity),
      linear_to_rpm(right_velocity));
  }

  void joystick_command(const sensor_msgs::msg::Joy & message)
  {
    last_joystick_time_ = std::chrono::steady_clock::now();
    double throttle = message.axes.size() > 4 ? message.axes[4] : 0.0;
    double steering = message.axes.size() > 3 ? -message.axes[3] : 0.0;
    if (std::abs(throttle) < 0.05) {
      throttle = 0.0;
    }
    if (std::abs(steering) < 0.05) {
      steering = 0.0;
    }
    if (throttle == 0.0 && steering == 0.0) {
      throttle = message.axes.size() > 7 ? message.axes[7] : 0.0;
      steering = message.axes.size() > 6 ? -message.axes[6] : 0.0;
    }
    throttle_ = throttle * 100.0;
    steering_ = steering * 100.0;

    const auto pressed = [&message, this](std::size_t index) {
        return message.buttons.size() > index && message.buttons[index] == 1 &&
               (previous_buttons_.size() <= index || previous_buttons_[index] == 0);
      };
    if (pressed(0)) {  // X: Free
      cart_mode_ = 0;
      publish_rpm(0.0, 0.0);
      publish_brake(false);
      request_freewheel(true, false);
    } else if (pressed(1)) {  // B: Brake
      cart_mode_ = 0;
      publish_rpm(0.0, 0.0);
      publish_brake(true);
    } else if (pressed(2)) {  // A: Drive
      cart_mode_ = 0;
      publish_rpm(0.0, 0.0);
      request_freewheel(false, true);
    }
    previous_buttons_ = message.buttons;
  }

  void update()
  {
    const auto update_time = std::chrono::steady_clock::now();
    const double period_seconds =
      std::chrono::duration<double>(update_time - last_update_time_).count();
    last_update_time_ = update_time;

    if (cart_mode_ == 1) {
      double left = 0.0;
      double right = 0.0;
      const bool joystick_online =
        std::chrono::duration<double>(update_time - last_joystick_time_).count() <=
        joystick_timeout_;
      if (joystick_online && (std::abs(throttle_) > 5.0 || std::abs(steering_) > 5.0)) {
        const auto mixed = mix_axes(steering_ / 100.0, throttle_ / 100.0);
        left = mixed.first * 150.0;
        right = mixed.second * 150.0;
      }
      publish_rpm(left, right);
    }

    const bool feedback_fresh =
      std::chrono::duration<double>(update_time - last_feedback_time_).count() <=
      feedback_timeout_;
    const double left_velocity = feedback_fresh ? rpm_to_linear(left_rpm_) : 0.0;
    const double right_velocity = feedback_fresh ? rpm_to_linear(right_rpm_) : 0.0;
    const double linear = (left_velocity + right_velocity) / 2.0;
    const double angular = (right_velocity - left_velocity) / wheel_base_;
    const double delta_yaw = angular * period_seconds;

    if (std::abs(angular) < 1.0e-9) {
      x_ += linear * std::cos(yaw_) * period_seconds;
      y_ += linear * std::sin(yaw_) * period_seconds;
    } else {
      const double radius = linear / angular;
      x_ += radius * (std::sin(yaw_ + delta_yaw) - std::sin(yaw_));
      y_ -= radius * (std::cos(yaw_ + delta_yaw) - std::cos(yaw_));
    }
    yaw_ += delta_yaw;
    yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_);
    const auto stamp = now();
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.pose.pose.orientation.w = orientation.w();
    odometry.pose.covariance[0] = 0.0001;
    odometry.pose.covariance[7] = 0.0001;
    odometry.pose.covariance[14] = 0.000001;
    odometry.pose.covariance[21] = 0.000001;
    odometry.pose.covariance[28] = 0.000001;
    odometry.pose.covariance[35] = 0.0001;
    odometry.twist.twist.linear.x = linear;
    odometry.twist.twist.angular.z = angular;
    odometry_publisher_->publish(odometry);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = "odom";
      transform.child_frame_id = "base_link";
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.rotation = odometry.pose.pose.orientation;
      transform_broadcaster_.sendTransform(transform);
    }
  }

  double wheel_base_;
  double wheel_radius_;
  bool publish_tf_;
  double joystick_timeout_;
  double feedback_timeout_;
  bool enable_joystick_;
  int left_motor_id_;
  int right_motor_id_;
  int left_direction_;
  int right_direction_;
  double left_rpm_{0.0};
  double right_rpm_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double throttle_{0.0};
  double steering_{0.0};
  int cart_mode_{2};
  std::vector<int32_t> previous_buttons_;
  std::chrono::steady_clock::time_point last_joystick_time_{};
  std::chrono::steady_clock::time_point last_feedback_time_{};
  std::chrono::steady_clock::time_point last_update_time_{std::chrono::steady_clock::now()};
  tf2_ros::TransformBroadcaster transform_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr rpm_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rpm_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr freewheel_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ddsm115_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ddsm115_controller::TwoWheelsRobot>());
  rclcpp::shutdown();
  return 0;
}
