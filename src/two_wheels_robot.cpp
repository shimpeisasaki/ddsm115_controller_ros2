#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ddsm115_controller/differential_drive.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
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
    publish_tf_(declare_parameter("pub_tf", true)),
    update_period_(declare_parameter("update_period", 0.02)),
    joystick_timeout_(declare_parameter("joystick_timeout", 2.0)),
    feedback_timeout_(declare_parameter("feedback_timeout", 0.2)),
    enable_joystick_(declare_parameter("enable_joystick", false)),
    joystick_deadzone_(declare_parameter("joystick_deadzone", 0.10)),
    joystick_low_max_linear_speed_(declare_parameter("joystick_low_max_linear_speed", 1.111111)),
    joystick_high_max_linear_speed_(declare_parameter("joystick_high_max_linear_speed", 1.666667)),
    joystick_max_angular_speed_(declare_parameter("joystick_max_angular_speed", 1.2)),
    joystick_linear_acceleration_(declare_parameter("joystick_linear_acceleration", 0.6)),
    joystick_linear_deceleration_(declare_parameter("joystick_linear_deceleration", 1.0)),
    joystick_angular_acceleration_(declare_parameter("joystick_angular_acceleration", 1.5)),
    joystick_angular_deceleration_(declare_parameter("joystick_angular_deceleration", 2.5)),
    joystick_linear_response_exponent_(
      declare_parameter("joystick_linear_response_exponent", 1.8)),
    joystick_angular_response_exponent_(
      declare_parameter("joystick_angular_response_exponent", 1.8)),
    joystick_linear_jerk_(declare_parameter("joystick_linear_jerk", 0.8)),
    joystick_angular_jerk_(declare_parameter("joystick_angular_jerk", 2.0)),
    left_motor_id_(declare_parameter("left_motor_id", 1)),
    right_motor_id_(declare_parameter("right_motor_id", 2)),
    left_direction_(declare_parameter("left_direction", 1)),
    right_direction_(declare_parameter("right_direction", -1)),
    odom_frame_(declare_parameter("odom_frame", std::string("odom"))),
    base_frame_(declare_parameter("base_frame", std::string("base_link"))),
    pose_xy_covariance_(declare_parameter("pose_xy_covariance", 0.01)),
    pose_yaw_covariance_(declare_parameter("pose_yaw_covariance", 0.02)),
    twist_linear_covariance_(declare_parameter("twist_linear_covariance", 0.01)),
    twist_angular_covariance_(declare_parameter("twist_angular_covariance", 0.02)),
    transform_broadcaster_(*this)
  {
    if (wheel_base_ <= 0.0) {
      throw std::invalid_argument("wheel_base must be greater than zero");
    }
    if (wheel_radius_ <= 0.0) {
      throw std::invalid_argument("R_wheel must be greater than zero");
    }
    if (update_period_ <= 0.0) {
      throw std::invalid_argument("update_period must be greater than zero");
    }
    if (joystick_timeout_ <= 0.0) {
      throw std::invalid_argument("joystick_timeout must be greater than zero");
    }
    if (feedback_timeout_ <= 0.0) {
      throw std::invalid_argument("feedback_timeout must be greater than zero");
    }
    if (joystick_deadzone_ < 0.0 || joystick_deadzone_ >= 1.0) {
      throw std::invalid_argument("joystick_deadzone must be in [0, 1)");
    }
    if (joystick_low_max_linear_speed_ <= 0.0 || joystick_high_max_linear_speed_ <= 0.0 ||
      joystick_max_angular_speed_ <= 0.0 || joystick_linear_acceleration_ <= 0.0 ||
      joystick_linear_deceleration_ <= 0.0 || joystick_angular_acceleration_ <= 0.0 ||
      joystick_angular_deceleration_ <= 0.0 || joystick_linear_response_exponent_ < 1.0 ||
      joystick_angular_response_exponent_ < 1.0 || joystick_linear_jerk_ <= 0.0 ||
      joystick_angular_jerk_ <= 0.0)
    {
      throw std::invalid_argument(
              "joystick limits must be positive and response exponents must be at least one");
    }
    if (left_motor_id_ < 1 || right_motor_id_ < 1 || left_motor_id_ == right_motor_id_) {
      throw std::invalid_argument("left_motor_id and right_motor_id must be distinct positive IDs");
    }
    if (std::abs(left_direction_) != 1 || std::abs(right_direction_) != 1) {
      throw std::invalid_argument("motor directions must be 1 or -1");
    }
    if (odom_frame_.empty() || base_frame_.empty() || odom_frame_ == base_frame_) {
      throw std::invalid_argument("odom_frame and base_frame must be distinct non-empty names");
    }
    if (pose_xy_covariance_ < 0.0 || pose_yaw_covariance_ < 0.0 ||
      twist_linear_covariance_ < 0.0 || twist_angular_covariance_ < 0.0)
    {
      throw std::invalid_argument("odometry covariance parameters must not be negative");
    }
    RCLCPP_INFO(get_logger(), "Start two_wheels_robot_node");
    RCLCPP_INFO(get_logger(), "wheel_base: %.3f", wheel_base_);
    RCLCPP_INFO(get_logger(), "R_wheel: %.3f", wheel_radius_);
    RCLCPP_INFO(get_logger(), "pub_tf: %s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "update_period: %.3f s", update_period_);
    RCLCPP_INFO(get_logger(), "joystick_timeout: %.3f s", joystick_timeout_);
    RCLCPP_INFO(get_logger(), "feedback_timeout: %.3f s", feedback_timeout_);
    RCLCPP_INFO(get_logger(), "enable_joystick: %s", enable_joystick_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(), "joystick: right stick only, low %.3f m/s, high %.3f m/s",
      joystick_low_max_linear_speed_, joystick_high_max_linear_speed_);
    RCLCPP_INFO(
      get_logger(), "joystick response exponents: linear %.2f, angular %.2f",
      joystick_linear_response_exponent_, joystick_angular_response_exponent_);
    RCLCPP_INFO(
      get_logger(), "left motor: ID %d, direction %d", left_motor_id_, left_direction_);
    RCLCPP_INFO(
      get_logger(), "right motor: ID %d, direction %d", right_motor_id_, right_direction_);
    RCLCPP_INFO(
      get_logger(), "odometry frames: %s -> %s", odom_frame_.c_str(), base_frame_.c_str());

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
          constexpr auto invalid_rpm = std::numeric_limits<std::int16_t>::min();
          if (message.data[left_motor_id_ - 1] == invalid_rpm ||
          message.data[right_motor_id_ - 1] == invalid_rpm)
          {
            feedback_valid_ = false;
            return;
          }
          left_rpm_ = message.data[left_motor_id_ - 1] * left_direction_;
          right_rpm_ = message.data[right_motor_id_ - 1] * right_direction_;
          feedback_valid_ = true;
          last_feedback_time_ = std::chrono::steady_clock::now();
        }
      });
    online_id_subscription_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/ddsm115/online_id", motor_qos,
      [this](const std_msgs::msg::UInt8MultiArray & message) {
        const auto contains = [&message](int id) {
          return std::find(message.data.begin(), message.data.end(), id) != message.data.end();
        };
        wheels_online_ = contains(left_motor_id_) && contains(right_motor_id_);
      });
    velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&TwoWheelsRobot::velocity_command, this, std::placeholders::_1));
    if (enable_joystick_) {
      joystick_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&TwoWheelsRobot::joystick_command, this, std::placeholders::_1));
    }
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(update_period_)),
      std::bind(&TwoWheelsRobot::update, this));
  }

private:
  double rpm_to_linear(double rpm) const
  {
    return rpm * ((2.0 * M_PI) / 60.0) * wheel_radius_;
  }

  static double apply_deadzone(double value, double deadzone)
  {
    value = std::clamp(value, -1.0, 1.0);
    if (std::abs(value) <= deadzone) {
      return 0.0;
    }
    // Preserve full scale while avoiding a step at the edge of the dead zone.
    return std::copysign((std::abs(value) - deadzone) / (1.0 - deadzone), value);
  }

  static double approach(double current, double target, double maximum_change)
  {
    const double difference = target - current;
    return current + std::clamp(difference, -maximum_change, maximum_change);
  }

  static double apply_response_curve(double value, double exponent)
  {
    return std::copysign(std::pow(std::abs(value), exponent), value);
  }

  static std::pair<double, double> smooth_velocity(
    double current_velocity, double target_velocity, double current_acceleration,
    double maximum_acceleration, double maximum_deceleration, double maximum_jerk,
    double period_seconds)
  {
    const double difference = target_velocity - current_velocity;
    if (std::abs(difference) < 1e-9) {
      return {target_velocity, 0.0};
    }
    const bool decelerating =
      current_velocity * target_velocity < 0.0 ||
      std::abs(target_velocity) < std::abs(current_velocity);
    const double acceleration_limit = decelerating ? maximum_deceleration : maximum_acceleration;
    const double target_acceleration = std::copysign(acceleration_limit, difference);
    const double next_acceleration = approach(
      current_acceleration, target_acceleration, maximum_jerk * period_seconds);
    const double next_velocity = current_velocity + next_acceleration * period_seconds;
    if ((difference > 0.0 && next_velocity >= target_velocity) ||
      (difference < 0.0 && next_velocity <= target_velocity))
    {
      return {target_velocity, 0.0};
    }
    return {next_velocity, next_acceleration};
  }

  void reset_joystick_motion()
  {
    joystick_linear_input_ = 0.0;
    joystick_angular_input_ = 0.0;
    smoothed_linear_ = 0.0;
    smoothed_angular_ = 0.0;
    smoothed_linear_acceleration_ = 0.0;
    smoothed_angular_acceleration_ = 0.0;
  }

  void select_joystick_drive(bool high_speed)
  {
    // Do not wait for the service response to enable the joystick state machine.
    // The velocity controller ignores RPM while switching out of freewheel, and
    // the timer will keep publishing the ramped command once it is ready.
    cart_mode_ = 1;
    smoothed_linear_ = 0.0;
    smoothed_angular_ = 0.0;
    smoothed_linear_acceleration_ = 0.0;
    smoothed_angular_acceleration_ = 0.0;
    publish_rpm(0.0, 0.0);
    publish_brake(false);
    joystick_high_speed_mode_ = high_speed;
    RCLCPP_INFO(
      get_logger(), "Joystick %s drive mode selected (max %.3f m/s)",
      high_speed ? "high" : "normal",
      high_speed ? joystick_high_max_linear_speed_ : joystick_low_max_linear_speed_);
    request_freewheel(false, false);
  }

  void publish_rpm(double left, double right)
  {
    const auto limited = limit_wheel_rpm(left, right);
    const auto limited_left = static_cast<std::int16_t>(std::lround(limited.first));
    const auto limited_right = static_cast<std::int16_t>(std::lround(limited.second));
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
          (void)enable_drive_after_response;
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
    if (!wheels_online_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring cmd_vel until both wheel motors are online");
      publish_rpm(0.0, 0.0);
      return;
    }
    const double linear = message.linear.x;
    const double angular = message.angular.z;
    if (!std::isfinite(linear) || !std::isfinite(angular)) {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite cmd_vel");
      publish_rpm(0.0, 0.0);
      return;
    }
    const auto wheel_rpm = twist_to_wheel_rpm(linear, angular, wheel_base_, wheel_radius_);
    publish_rpm(wheel_rpm.first, wheel_rpm.second);
  }

  void joystick_command(const sensor_msgs::msg::Joy & message)
  {
    last_joystick_time_ = std::chrono::steady_clock::now();
    // This controller reports the analog right stick as axes 3 (horizontal)
    // and 4 (vertical). Axes 0/1 are the D-pad, so they are not used.
    const double right_horizontal = message.axes.size() > 3 ? message.axes[3] : 0.0;
    const double right_vertical = message.axes.size() > 4 ? message.axes[4] : 0.0;
    joystick_linear_input_ = apply_response_curve(
      apply_deadzone(right_vertical, joystick_deadzone_), joystick_linear_response_exponent_);
    joystick_angular_input_ = apply_response_curve(
      apply_deadzone(-right_horizontal, joystick_deadzone_), joystick_angular_response_exponent_);

    const auto pressed = [&message, this](std::size_t index) {
        return message.buttons.size() > index && message.buttons[index] == 1 &&
               (previous_buttons_.size() <= index || previous_buttons_[index] == 0);
      };
    if (pressed(3)) {  // Y: Free
      cart_mode_ = 0;
      reset_joystick_motion();
      publish_rpm(0.0, 0.0);
      publish_brake(false);
      request_freewheel(true, false);
    } else if (pressed(1)) {  // B: Brake
      cart_mode_ = 0;
      reset_joystick_motion();
      publish_rpm(0.0, 0.0);
      publish_brake(true);
    } else if (pressed(0)) {  // A: High (6 km/h) drive
      select_joystick_drive(true);
    } else if (pressed(2)) {  // X: Normal (4 km/h) drive
      select_joystick_drive(false);
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
      const bool joystick_online =
        std::chrono::duration<double>(update_time - last_joystick_time_).count() <=
        joystick_timeout_;
      const double maximum_linear = joystick_high_speed_mode_ ?
        joystick_high_max_linear_speed_ : joystick_low_max_linear_speed_;
      const double target_linear = joystick_online && wheels_online_ ?
        joystick_linear_input_ * maximum_linear : 0.0;
      const double target_angular = joystick_online && wheels_online_ ?
        joystick_angular_input_ * joystick_max_angular_speed_ : 0.0;
      const auto linear_state = smooth_velocity(
        smoothed_linear_, target_linear, smoothed_linear_acceleration_,
        joystick_linear_acceleration_, joystick_linear_deceleration_, joystick_linear_jerk_,
        period_seconds);
      smoothed_linear_ = linear_state.first;
      smoothed_linear_acceleration_ = linear_state.second;
      const auto angular_state = smooth_velocity(
        smoothed_angular_, target_angular, smoothed_angular_acceleration_,
        joystick_angular_acceleration_, joystick_angular_deceleration_, joystick_angular_jerk_,
        period_seconds);
      smoothed_angular_ = angular_state.first;
      smoothed_angular_acceleration_ = angular_state.second;
      const auto wheel_rpm = twist_to_wheel_rpm(
        smoothed_linear_, smoothed_angular_, wheel_base_, wheel_radius_);
      publish_rpm(wheel_rpm.first, wheel_rpm.second);
    }

    const bool feedback_fresh = feedback_valid_ && wheels_online_ &&
      std::chrono::duration<double>(update_time - last_feedback_time_).count() <=
      feedback_timeout_;
    const double left_velocity = feedback_fresh ? rpm_to_linear(left_rpm_) : 0.0;
    const double right_velocity = feedback_fresh ? rpm_to_linear(right_rpm_) : 0.0;
    const double linear = (left_velocity + right_velocity) / 2.0;
    const double angular = (right_velocity - left_velocity) / wheel_base_;
    const auto pose = integrate_odometry({x_, y_, yaw_}, linear, angular, period_seconds);
    x_ = pose.x;
    y_ = pose.y;
    yaw_ = pose.yaw;

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_);
    const auto stamp = now();
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.pose.pose.orientation.w = orientation.w();
    odometry.pose.covariance[0] = pose_xy_covariance_;
    odometry.pose.covariance[7] = pose_xy_covariance_;
    odometry.pose.covariance[14] = 1.0e6;
    odometry.pose.covariance[21] = 1.0e6;
    odometry.pose.covariance[28] = 1.0e6;
    odometry.pose.covariance[35] = pose_yaw_covariance_;
    odometry.twist.twist.linear.x = linear;
    odometry.twist.twist.angular.z = angular;
    odometry.twist.covariance[0] = twist_linear_covariance_;
    odometry.twist.covariance[7] = 1.0e6;
    odometry.twist.covariance[14] = 1.0e6;
    odometry.twist.covariance[21] = 1.0e6;
    odometry.twist.covariance[28] = 1.0e6;
    odometry.twist.covariance[35] = twist_angular_covariance_;
    odometry_publisher_->publish(odometry);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.rotation = odometry.pose.pose.orientation;
      transform_broadcaster_.sendTransform(transform);
    }
  }

  double wheel_base_;
  double wheel_radius_;
  bool publish_tf_;
  double update_period_;
  double joystick_timeout_;
  double feedback_timeout_;
  bool enable_joystick_;
  double joystick_deadzone_;
  double joystick_low_max_linear_speed_;
  double joystick_high_max_linear_speed_;
  double joystick_max_angular_speed_;
  double joystick_linear_acceleration_;
  double joystick_linear_deceleration_;
  double joystick_angular_acceleration_;
  double joystick_angular_deceleration_;
  double joystick_linear_response_exponent_;
  double joystick_angular_response_exponent_;
  double joystick_linear_jerk_;
  double joystick_angular_jerk_;
  int left_motor_id_;
  int right_motor_id_;
  int left_direction_;
  int right_direction_;
  std::string odom_frame_;
  std::string base_frame_;
  double pose_xy_covariance_;
  double pose_yaw_covariance_;
  double twist_linear_covariance_;
  double twist_angular_covariance_;
  bool wheels_online_{false};
  bool feedback_valid_{false};
  double left_rpm_{0.0};
  double right_rpm_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double joystick_linear_input_{0.0};
  double joystick_angular_input_{0.0};
  double smoothed_linear_{0.0};
  double smoothed_angular_{0.0};
  double smoothed_linear_acceleration_{0.0};
  double smoothed_angular_acceleration_{0.0};
  bool joystick_high_speed_mode_{false};
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
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr online_id_subscription_;
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
