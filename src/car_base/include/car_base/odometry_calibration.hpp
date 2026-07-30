#ifndef CAR_BASE__ODOMETRY_CALIBRATION_HPP_
#define CAR_BASE__ODOMETRY_CALIBRATION_HPP_

namespace car_base
{

struct OdometryCalibrationInput
{
  double actual_distance{1.0};
  double odom_distance{1.0};
  double actual_angle{1.0};
  double odom_angle{1.0};
  double wheel_distance{0.175};
  double left_wheel_per_tick{0.000105805};
  double right_wheel_per_tick{0.000105805};
};

struct OdometryCalibrationResult
{
  double distance_scale{1.0};
  double left_wheel_per_tick{0.0};
  double right_wheel_per_tick{0.0};
  double wheel_distance{0.0};
};

OdometryCalibrationResult calculate_odometry_calibration(
  const OdometryCalibrationInput & input);

}  // namespace car_base

#endif  // CAR_BASE__ODOMETRY_CALIBRATION_HPP_
