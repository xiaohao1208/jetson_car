#include "car_move/motion_arbitrator.hpp"

#include <algorithm>
#include <cmath>

namespace car_move
{

ArbitrationResult MotionArbitrator::select(const ArbitrationInput & input)
{
  ArbitrationResult result;
  const auto requests_motion = [](const PlanarCommand & command) {
      return std::abs(command.linear) >= 0.001 ||
             std::abs(command.angular) >= 0.001;
    };
  const bool have_fresh_motion_command =
    (input.calibration_fresh && requests_motion(input.calibration)) ||
    (input.manual_fresh && requests_motion(input.manual)) ||
    (input.navigation_fresh && requests_motion(input.navigation));
  if (!input.mcu_online || input.emergency_stop) {
    // 新鲜零命令本来就要求停车，不应产生“运动命令被拦截”的误告警。
    result.blocked_by_safety = have_fresh_motion_command;
    return result;
  }

  if (input.calibration_active) {
    if (!input.calibration_fresh) {
      result.blocked_by_safety = true;
      return result;
    }
    result.command = input.calibration;
  } else if (input.observation_status_stale) {
    result.blocked_by_safety = true;
    return result;
  } else if (input.observation_active) {
    if (!input.navigation_fresh) {
      result.blocked_by_safety = true;
      return result;
    }
    result.command = input.navigation;
  } else if (input.manual_fresh) {
    result.command = input.manual;
  } else if (input.navigation_fresh) {
    result.command = input.navigation;
  }

  const bool distance_blocks =
    std::isfinite(input.obstacle_distance) &&
    input.obstacle_distance > 0.0 &&
    input.obstacle_distance <= input.obstacle_stop_distance;
  // ESP32 会在单次超声波无回波时保留已确认的障碍物锁存。锁存不依赖
  // 当前测距是否有效；只有直接使用距离值时才要求 ultrasonic_valid。
  result.obstacle_detected =
    input.obstacle || (input.ultrasonic_valid && distance_blocks);
  if (result.command.linear > 0.0 && result.obstacle_detected)
  {
    result.command = PlanarCommand();
    result.blocked_by_obstacle = true;
  }
  return result;
}

double MotionArbitrator::limit_angular_speed(
  double linear_speed, double angular_speed,
  double in_place_linear_threshold,
  double minimum_in_place, double maximum)
{
  if (!std::isfinite(angular_speed) || angular_speed == 0.0) {
    return 0.0;
  }
  const double limited = std::clamp(angular_speed, -maximum, maximum);
  const bool in_place = !std::isfinite(linear_speed) ||
    std::abs(linear_speed) <= std::max(0.0, in_place_linear_threshold);
  if (!in_place || std::abs(limited) >= minimum_in_place) {
    return limited;
  }
  return std::copysign(minimum_in_place, limited);
}

}  // namespace car_move
