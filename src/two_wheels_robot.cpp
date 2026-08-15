#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
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
    RCLCPP_INFO(get_logger(), "Start two_wheels_robot_node");
    RCLCPP_INFO(get_logger(), "wheel_base: %.3f", wheel_base_);
    RCLCPP_INFO(get_logger(), "R_wheel: %.3f", wheel_radius_);
    RCLCPP_INFO(get_logger(), "pub_tf: %s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "joystick_timeout: %.3f s", joystick_timeout_);

    auto motor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    rpm_publisher_ =
      create_publisher<std_msgs::msg::Int16MultiArray>("/ddsm115/rpm_cmd", motor_qos);
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    rpm_subscription_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "/ddsm115/rpm_fb", motor_qos,
      [this](const std_msgs::msg::Int16MultiArray & message) {
        if (message.data.size() >= 2) {
          left_rpm_ = message.data[0];
          right_rpm_ = -message.data[1];
        }
      });
    velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&TwoWheelsRobot::velocity_command, this, std::placeholders::_1));
    joystick_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&TwoWheelsRobot::joystick_command, this, std::placeholders::_1));
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

  static double map_with_limit(
    double value, double in_min, double in_max, double out_min,
    double out_max)
  {
    const double output = (out_max - out_min) / (in_max - in_min) * (value - in_min) + out_min;
    return std::clamp(output, std::min(out_min, out_max), std::max(out_min, out_max));
  }

  std::pair<double, double> mix_axes(double x, double y)
  {
    double left = y + x;
    double right = y - x;
    const double difference = std::abs(x) - std::abs(y);
    left += std::copysign(std::abs(difference), left);
    right += std::copysign(std::abs(difference), right);
    if (previous_y_ < 0.0) {
      std::swap(left, right);
    }
    previous_y_ = y;
    return {left, right};
  }

  void publish_rpm(std::int16_t left, std::int16_t right)
  {
    std_msgs::msg::Int16MultiArray message;
    message.data = {left, right};
    rpm_publisher_->publish(message);
  }

  void velocity_command(const geometry_msgs::msg::Twist & message)
  {
    const double linear = message.linear.x;
    const double angular = message.angular.z;
    double left_velocity = 0.0;
    double right_velocity = 0.0;

    if (linear != 0.0 && angular == 0.0) {
      left_velocity = linear;
      right_velocity = linear;
    } else if (linear == 0.0 && angular != 0.0) {
      left_velocity = -angular * wheel_base_ / 2.0;
      right_velocity = angular * wheel_base_ / 2.0;
    } else if (linear != 0.0 && angular != 0.0) {
      const double radius = std::abs(linear) / std::abs(angular);
      const double direction = std::copysign(1.0, linear);
      if (angular > 0.0) {
        left_velocity = direction * angular * (radius - wheel_base_ / 2.0);
        right_velocity = direction * angular * (radius + wheel_base_ / 2.0);
      } else {
        left_velocity = direction * std::abs(angular) * (radius + wheel_base_ / 2.0);
        right_velocity = direction * std::abs(angular) * (radius - wheel_base_ / 2.0);
      }
    }
    publish_rpm(
      static_cast<std::int16_t>(linear_to_rpm(left_velocity)),
      static_cast<std::int16_t>(-linear_to_rpm(right_velocity)));
  }

  void joystick_command(const sensor_msgs::msg::Joy & message)
  {
    last_joystick_time_ = std::chrono::steady_clock::now();
    if (message.axes.size() > 3) {
      throttle_ = message.axes[1] * 100.0;
      steering_ = -message.axes[3] * 100.0;
    }
    if (!message.buttons.empty() && message.buttons[0] == 1) {
      cart_mode_ = 2;
    } else if (message.buttons.size() > 2 && message.buttons[2] == 1) {
      cart_mode_ = 1;
    }
  }

  void update()
  {
    const auto update_time = std::chrono::steady_clock::now();
    const double period_seconds =
      std::chrono::duration<double>(update_time - last_update_time_).count();
    last_update_time_ = update_time;

    if (cart_mode_ == 1) {
      std::int16_t left = 0;
      std::int16_t right = 0;
      const bool joystick_online =
        std::chrono::duration<double>(update_time - last_joystick_time_).count() <=
        joystick_timeout_;
      if (joystick_online && (std::abs(throttle_) > 5.0 || std::abs(steering_) > 5.0)) {
        const auto mixed = mix_axes(steering_, throttle_);
        left = static_cast<std::int16_t>(map_with_limit(mixed.first, -200.0, 200.0, -150.0, 150.0));
        right =
          static_cast<std::int16_t>(map_with_limit(mixed.second, -200.0, 200.0, 150.0, -150.0));
      }
      publish_rpm(left, right);
    }

    const double left_velocity = std::round(rpm_to_linear(left_rpm_) * 1000.0) / 1000.0;
    const double right_velocity = std::round(rpm_to_linear(right_rpm_) * 1000.0) / 1000.0;
    double linear = (left_velocity + right_velocity) / 2.0;
    double angular = 0.0;

    if (left_velocity != right_velocity) {
      angular = (right_velocity - left_velocity) / wheel_base_;
      if ((left_velocity > 0.0 && right_velocity < 0.0) ||
        (right_velocity > 0.0 && left_velocity < 0.0))
      {
        linear = 0.0;
        yaw_ += angular * period_seconds;
      } else {
        const double radius = (wheel_base_ / 2.0) *
          ((left_velocity + right_velocity) / (right_velocity - left_velocity));
        x_ = x_ - radius * std::sin(yaw_) + radius * std::sin(yaw_ + angular * period_seconds);
        y_ = y_ + radius * std::cos(yaw_) - radius * std::cos(yaw_ + angular * period_seconds);
        yaw_ += angular * period_seconds;
      }
    } else {
      x_ += linear * std::cos(yaw_) * period_seconds;
      y_ += linear * std::sin(yaw_) * period_seconds;
    }

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
  double left_rpm_{0.0};
  double right_rpm_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double throttle_{0.0};
  double steering_{0.0};
  double previous_y_{0.0};
  int cart_mode_{2};
  std::chrono::steady_clock::time_point last_joystick_time_{};
  std::chrono::steady_clock::time_point last_update_time_{std::chrono::steady_clock::now()};
  tf2_ros::TransformBroadcaster transform_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr rpm_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rpm_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscription_;
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
