#ifndef DDSM115_CONTROLLER__SAFETY_LEASE_HPP_
#define DDSM115_CONTROLLER__SAFETY_LEASE_HPP_

#include <chrono>
#include <cstdint>

namespace ddsm115_controller
{
// 0=brake, 1=freewheel, 2=drive. A timeout requires a non-drive handshake.
class SafetyLease
{
public:
  using Clock = std::chrono::steady_clock;
  explicit SafetyLease(double timeout) : timeout_(timeout) {}
  void trip() {mode_ = 0; acknowledged_ = false;}
  void receive(std::uint8_t mode, Clock::time_point now)
  {
    if (now - last_ > timeout_) {trip();}
    if (mode > 2) {trip(); return;}
    if (mode < 2) {acknowledged_ = true;}
    if (mode < 2 || acknowledged_) {mode_ = mode;}
    last_ = now;
  }
  std::uint8_t mode(Clock::time_point now)
  {
    if (now - last_ > timeout_) {trip();}
    return mode_;
  }

private:
  std::uint8_t mode_{0};
  bool acknowledged_{false};
  Clock::time_point last_{};
  std::chrono::duration<double> timeout_;
};
}  // namespace ddsm115_controller
#endif
