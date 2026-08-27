#ifndef CAR_NAVIGATION__SCAN_PATTERN_HPP_
#define CAR_NAVIGATION__SCAN_PATTERN_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace car_navigation
{

struct ClearanceCandidate
{
  bool clear{false};
  double score{0.0};
  double offset{0.0};
};

inline double normalize_angle(double value)
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double two_pi = 2.0 * pi;
  while (value > pi) {
    value -= two_pi;
  }
  while (value < -pi) {
    value += two_pi;
  }
  return value;
}

inline std::vector<double> build_scan_ranges(double step, double limit)
{
  if (!std::isfinite(step) || !std::isfinite(limit) || step <= 0.0 || limit <= 0.0) {
    throw std::invalid_argument("scan step and limit must be positive finite values");
  }
  std::vector<double> ranges;
  for (double value = step; value < limit - 1.0e-9; value += step) {
    ranges.push_back(value);
  }
  ranges.push_back(limit);
  return ranges;
}

inline std::vector<double> build_scan_offsets(double step, double limit)
{
  std::vector<double> offsets;
  for (const double range : build_scan_ranges(step, limit)) {
    offsets.push_back(range);
    offsets.push_back(-range);
  }
  return offsets;
}

inline double relative_spin_for_target(
  double encounter_yaw, double current_yaw, double target_offset)
{
  const double current_offset = normalize_angle(current_yaw - encounter_yaw);
  return normalize_angle(target_offset - current_offset);
}

inline std::optional<double> choose_clear_offset(
  const ClearanceCandidate & left, const ClearanceCandidate & right,
  double tie_distance)
{
  if (!left.clear && !right.clear) {
    return std::nullopt;
  }
  if (left.clear && !right.clear) {
    return left.offset;
  }
  if (!left.clear && right.clear) {
    return right.offset;
  }
  if (std::isinf(left.score) && !std::isinf(right.score)) {
    return left.offset;
  }
  if (!std::isinf(left.score) && std::isinf(right.score)) {
    return right.offset;
  }
  if (left.score > right.score + std::max(tie_distance, 0.0)) {
    return left.offset;
  }
  // 两侧相近时保留扫描结束时所在的右侧，避免一次无意义的回转。
  return right.offset;
}

}  // namespace car_navigation

#endif  // CAR_NAVIGATION__SCAN_PATTERN_HPP_
