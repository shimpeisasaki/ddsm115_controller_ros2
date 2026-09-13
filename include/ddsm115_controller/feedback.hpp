#ifndef DDSM115_CONTROLLER__FEEDBACK_HPP_
#define DDSM115_CONTROLLER__FEEDBACK_HPP_

#include <cstdint>
#include <limits>
#include <vector>

namespace ddsm115_controller
{

// Wire contract for /ddsm115/rpm_fb: missing replies are not zero speed.
constexpr std::int16_t invalid_rpm = std::numeric_limits<std::int16_t>::min();

inline bool wheel_feedback_valid(
  const std::vector<std::int16_t> & rpm, int left_id, int right_id)
{
  return left_id > 0 && right_id > 0 &&
         static_cast<std::size_t>(left_id) <= rpm.size() &&
         static_cast<std::size_t>(right_id) <= rpm.size() &&
         rpm[left_id - 1] != invalid_rpm && rpm[right_id - 1] != invalid_rpm;
}

}  // namespace ddsm115_controller

#endif  // DDSM115_CONTROLLER__FEEDBACK_HPP_
