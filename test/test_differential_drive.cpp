#include <cmath>

#include "ddsm115_controller/differential_drive.hpp"
#include "ddsm115_controller/feedback.hpp"
#include "gtest/gtest.h"

namespace ddsm115_controller
{

TEST(Feedback, DistinguishesStoppedWheelsFromMissingReplies)
{
  EXPECT_TRUE(wheel_feedback_valid({0, 0}, 1, 2));
  EXPECT_TRUE(wheel_feedback_valid({56, -57}, 1, 2));
  EXPECT_FALSE(wheel_feedback_valid({56, invalid_rpm}, 1, 2));
  EXPECT_FALSE(wheel_feedback_valid({invalid_rpm, -57}, 1, 2));
  EXPECT_FALSE(wheel_feedback_valid({56}, 1, 2));
  EXPECT_FALSE(wheel_feedback_valid({}, 1, 2));
  EXPECT_FALSE(wheel_feedback_valid({56, -57}, 0, 2));
}

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

TEST(DifferentialDrive, CorrectsSignedFeedbackWithoutMovingZero)
{
  EXPECT_DOUBLE_EQ(correct_rpm_feedback(0.0, 0.5), 0.0);
  EXPECT_DOUBLE_EQ(correct_rpm_feedback(89.5, 0.5), 90.0);
  EXPECT_DOUBLE_EQ(correct_rpm_feedback(-90.2, 0.5), -89.7);
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
