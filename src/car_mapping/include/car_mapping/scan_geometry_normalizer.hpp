// Copyright 2026 hao

#ifndef CAR_MAPPING__SCAN_GEOMETRY_NORMALIZER_HPP_
#define CAR_MAPPING__SCAN_GEOMETRY_NORMALIZER_HPP_

#include <cstddef>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace car_mapping
{

/**
 * @brief 将变化的雷达帧重采样到固定角度网格
 */
class ScanGeometryNormalizer
{
public:
  void lock(float angle_min, float angle_max, std::size_t point_count);
  bool locked() const;
  std::size_t point_count() const;
  sensor_msgs::msg::LaserScan normalize(
    const sensor_msgs::msg::LaserScan & input) const;

  static bool valid(const sensor_msgs::msg::LaserScan & input);
  static std::size_t expected_point_count(
    const sensor_msgs::msg::LaserScan & input);

private:
  float angle_min_{0.0F};
  float angle_max_{0.0F};
  std::size_t point_count_{0U};
};

}  // namespace car_mapping

#endif  // CAR_MAPPING__SCAN_GEOMETRY_NORMALIZER_HPP_
