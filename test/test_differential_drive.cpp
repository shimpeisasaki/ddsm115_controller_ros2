#include <cmath>

#include "ddsm115_controller/differential_drive.hpp"
#include "gtest/gtest.h"

namespace ddsm115_controller
{

TEST(DifferentialDrive, ConvertsStraightAndTurningCommands)
{
  const auto straight = twist_to_wheel_rpm(0.5, 0.0, 0.207, 0.051);
  EXPECT_NEAR(straight.first, 93.62, 0.01);
  EXPECT_NEAR(straight.second, 93.62, 0.01);

  const auto reverse_turn = twist_to_wheel_rpm(-0.5, 1.0, 0.207, 0.051);
  EXPECT_LT(reverse_turn.first, reverse_turn.second);
  EXPECT_LT(reverse_turn.second, 0.0);

  const auto rotate = twist_to_wheel_rpm(0.0, 1.0, 0.207, 0.051);
  EXPECT_NEAR(rotate.first, -rotate.second, 1.0e-9);
}

TEST(DifferentialDrive, LimitsRpmProportionally)
{
  const auto limited = limit_wheel_rpm(620.0, 660.0);
  EXPECT_NEAR(limited.first, 310.0, 1.0e-9);
  EXPECT_NEAR(limited.second, 330.0, 1.0e-9);
}

TEST(DifferentialDrive, IntegratesStraightAndArcMotion)
{
  const auto straight = integrate_odometry({}, 1.0, 0.0, 1.0);
  EXPECT_NEAR(straight.x, 1.0, 1.0e-9);
  EXPECT_NEAR(straight.y, 0.0, 1.0e-9);
  EXPECT_NEAR(straight.yaw, 0.0, 1.0e-9);

  const auto arc = integrate_odometry({}, 1.0, 1.0, 1.0);
  EXPECT_NEAR(arc.x, std::sin(1.0), 1.0e-9);
  EXPECT_NEAR(arc.y, 1.0 - std::cos(1.0), 1.0e-9);
  EXPECT_NEAR(arc.yaw, 1.0, 1.0e-9);
}

}  // namespace ddsm115_controller
