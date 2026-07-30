#include "car_base/odometry_calibration.hpp"

#include <cmath>
#include <stdexcept>

namespace car_base
{

OdometryCalibrationResult calculate_odometry_calibration(
  const OdometryCalibrationInput & input)
{
  const double values[] = {
    input.actual_distance,
    input.odom_distance,
    input.actual_angle,
    input.odom_angle,
    input.wheel_distance,
    input.left_wheel_per_tick,
    input.right_wheel_per_tick,
  };
  for (const double value : values) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("里程计标定输入必须为有限正数");
    }
  }

  OdometryCalibrationResult result;
  result.distance_scale = input.actual_distance / input.odom_distance;
  result.left_wheel_per_tick =
    input.left_wheel_per_tick * result.distance_scale;
  result.right_wheel_per_tick =
    input.right_wheel_per_tick * result.distance_scale;
  result.wheel_distance =
    input.wheel_distance * result.distance_scale *
    input.odom_angle / input.actual_angle;
  return result;
}

}  // namespace car_base
