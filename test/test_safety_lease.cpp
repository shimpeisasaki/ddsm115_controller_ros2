#include <chrono>
#include "gtest/gtest.h"
#include "ddsm115_controller/safety_lease.hpp"

using ddsm115_controller::SafetyLease;
using namespace std::chrono_literals;

TEST(SafetyLease, StartupRequiresBrakeHandshake)
{
  SafetyLease lease(.3);
  const auto now = SafetyLease::Clock::now();
  lease.receive(2, now);
  EXPECT_EQ(lease.mode(now), 0);
  lease.receive(0, now);
  lease.receive(2, now);
  EXPECT_EQ(lease.mode(now), 2);
}

TEST(SafetyLease, TimeoutCannotAutomaticallyResume)
{
  SafetyLease lease(.3);
  const auto now = SafetyLease::Clock::now();
  lease.receive(0, now);
  lease.receive(2, now);
  EXPECT_EQ(lease.mode(now + 301ms), 0);
  lease.receive(2, now + 302ms);
  EXPECT_EQ(lease.mode(now + 303ms), 0);
  lease.receive(0, now + 304ms);
  lease.receive(2, now + 305ms);
  EXPECT_EQ(lease.mode(now + 306ms), 2);
}

TEST(SafetyLease, FreewheelExpiresAndBrakeTrips)
{
  SafetyLease lease(.3);
  const auto now = SafetyLease::Clock::now();
  lease.receive(1, now);
  EXPECT_EQ(lease.mode(now), 1);
  EXPECT_EQ(lease.mode(now + 301ms), 0);
  lease.receive(0, now + 302ms);
  lease.receive(2, now + 303ms);
  lease.trip();
  lease.receive(2, now + 304ms);
  EXPECT_EQ(lease.mode(now + 304ms), 0);
}

TEST(SafetyLease, InvalidModeBrakes)
{
  SafetyLease lease(.3);
  const auto now = SafetyLease::Clock::now();
  lease.receive(0, now);
  lease.receive(2, now);
  lease.receive(255, now);
  EXPECT_EQ(lease.mode(now), 0);
}

TEST(SafetyLease, LateHeartbeatDoesNotHideExpiredLease)
{
  SafetyLease lease(.3);
  const auto now = SafetyLease::Clock::now();
  lease.receive(0, now);
  lease.receive(2, now);
  lease.receive(2, now + 500ms);
  EXPECT_EQ(lease.mode(now + 500ms), 0);
}
