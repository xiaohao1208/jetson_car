#include "car_move/motion_arbitrator.hpp"

#include <algorithm>
#include <cmath>

namespace car_move {

ArbitrationResult MotionArbitrator::select(const ArbitrationInput & input)
{
  ArbitrationResult result;
  const bool have_fresh_command =
    input.manual_fresh || input.navigation_fresh;
  if (!input.mcu_online || input.emergency_stop) {
    result.blocked_by_safety = have_fresh_command;
    return result;
  }

  if (input.manual_fresh) {
    result.command = input.manual;
  } else if (input.navigation_fresh) {
    result.command = input.navigation;
  }

  const bool distance_blocks =
    std::isfinite(input.obstacle_distance) &&
    input.obstacle_distance > 0.0 &&
    input.obstacle_distance <= input.obstacle_stop_distance;
  if (result.command.linear > 0.0 && input.ultrasonic_valid &&
    (input.obstacle || distance_blocks))
  {
    result.command = PlanarCommand();
    result.blocked_by_obstacle = true;
  }
  return result;
}

double MotionArbitrator::limit_angular_speed(
  double value, double minimum, double maximum)
{
  if (!std::isfinite(value) || value == 0.0) {
    return 0.0;
  }
  const double limited = std::clamp(value, -maximum, maximum);
  if (std::abs(limited) >= minimum) {
    return limited;
  }
  return std::copysign(minimum, limited);
}

}  // namespace car_move
