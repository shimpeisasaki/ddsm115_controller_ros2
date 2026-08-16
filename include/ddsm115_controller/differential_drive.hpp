#ifndef DDSM115_CONTROLLER__DIFFERENTIAL_DRIVE_HPP_
#define DDSM115_CONTROLLER__DIFFERENTIAL_DRIVE_HPP_

#include <algorithm>
#include <cmath>
#include <utility>

namespace ddsm115_controller
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline std::pair<double, double> limit_wheel_rpm(
  double left_rpm, double right_rpm, double maximum_rpm = 330.0)
{
  const double scale = maximum_rpm /
    std::max({maximum_rpm, std::abs(left_rpm), std::abs(right_rpm)});
  return {left_rpm * scale, right_rpm * scale};
}

inline std::pair<double, double> twist_to_wheel_rpm(
  double linear, double angular, double wheel_base, double wheel_radius,
  double maximum_rpm = 330.0)
{
  constexpr double pi = 3.14159265358979323846;
  const double velocity_to_rpm = 60.0 / (2.0 * pi * wheel_radius);
  const double left_velocity = linear - angular * wheel_base / 2.0;
  const double right_velocity = linear + angular * wheel_base / 2.0;
  return limit_wheel_rpm(
    left_velocity * velocity_to_rpm, right_velocity * velocity_to_rpm, maximum_rpm);
}

inline Pose2D integrate_odometry(
  Pose2D pose, double linear, double angular, double period_seconds)
{
  const double delta_yaw = angular * period_seconds;
  if (std::abs(angular) < 1.0e-9) {
    pose.x += linear * std::cos(pose.yaw) * period_seconds;
    pose.y += linear * std::sin(pose.yaw) * period_seconds;
  } else {
    const double radius = linear / angular;
    pose.x += radius * (std::sin(pose.yaw + delta_yaw) - std::sin(pose.yaw));
    pose.y -= radius * (std::cos(pose.yaw + delta_yaw) - std::cos(pose.yaw));
  }
  pose.yaw += delta_yaw;
  pose.yaw = std::atan2(std::sin(pose.yaw), std::cos(pose.yaw));
  return pose;
}

}  // namespace ddsm115_controller

#endif  // DDSM115_CONTROLLER__DIFFERENTIAL_DRIVE_HPP_
