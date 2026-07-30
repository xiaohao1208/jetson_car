// Copyright 2026 hao

#include "car_mapping/scan_geometry_normalizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace car_mapping
{

void ScanGeometryNormalizer::lock(
  float angle_min, float angle_max, std::size_t point_count)
{
  if (!std::isfinite(angle_min) || !std::isfinite(angle_max) ||
    angle_max <= angle_min || point_count < 2U)
  {
    throw std::invalid_argument("雷达固定角度网格参数无效");
  }
  angle_min_ = angle_min;
  angle_max_ = angle_max;
  point_count_ = point_count;
}

bool ScanGeometryNormalizer::locked() const
{
  return point_count_ >= 2U;
}

std::size_t ScanGeometryNormalizer::point_count() const
{
  return point_count_;
}

bool ScanGeometryNormalizer::valid(
  const sensor_msgs::msg::LaserScan & input)
{
  return input.ranges.size() >= 2U &&
         std::isfinite(input.angle_min) &&
         std::isfinite(input.angle_max) &&
         std::isfinite(input.angle_increment) &&
         input.angle_max > input.angle_min &&
         input.angle_increment > 0.0F;
}

std::size_t ScanGeometryNormalizer::expected_point_count(
  const sensor_msgs::msg::LaserScan & input)
{
  if (!valid(input)) {
    return 0U;
  }
  const double span =
    static_cast<double>(input.angle_max) -
    static_cast<double>(input.angle_min);
  const double increment = static_cast<double>(input.angle_increment);
  return static_cast<std::size_t>(std::llround(span / increment)) + 1U;
}

sensor_msgs::msg::LaserScan ScanGeometryNormalizer::normalize(
  const sensor_msgs::msg::LaserScan & input) const
{
  if (!locked()) {
    throw std::logic_error("雷达固定角度网格尚未锁定");
  }
  if (!valid(input)) {
    throw std::invalid_argument("输入雷达帧无效");
  }

  sensor_msgs::msg::LaserScan output = input;
  output.angle_min = angle_min_;
  output.angle_increment =
    (angle_max_ - angle_min_) /
    static_cast<float>(point_count_ - 1U);
  output.angle_max =
    output.angle_min +
    output.angle_increment * static_cast<float>(point_count_ - 1U);
  output.ranges.assign(
    point_count_, std::numeric_limits<float>::infinity());
  if (input.intensities.empty()) {
    output.intensities.clear();
  } else {
    output.intensities.assign(point_count_, 0.0F);
  }

  for (std::size_t index = 0U; index < point_count_; ++index) {
    const double angle =
      static_cast<double>(output.angle_min) +
      static_cast<double>(output.angle_increment) *
      static_cast<double>(index);
    const double source_position =
      (angle - static_cast<double>(input.angle_min)) /
      static_cast<double>(input.angle_increment);
    const auto source_index =
      static_cast<std::int64_t>(std::llround(source_position));
    if (source_index < 0 ||
      source_index >= static_cast<std::int64_t>(input.ranges.size()))
    {
      continue;
    }
    const auto source = static_cast<std::size_t>(source_index);
    output.ranges[index] = input.ranges[source];
    if (!output.intensities.empty() && source < input.intensities.size()) {
      output.intensities[index] = input.intensities[source];
    }
  }

  if (output.scan_time > 0.0F) {
    output.time_increment =
      output.scan_time / static_cast<float>(point_count_);
  }
  return output;
}

}  // namespace car_mapping
