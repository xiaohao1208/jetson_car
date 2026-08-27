#ifndef CAR_MOVE__MOTION_ARBITRATOR_HPP_
#define CAR_MOVE__MOTION_ARBITRATOR_HPP_

namespace car_move
{

/** @brief 与 ROS 消息无关的二维速度，用于独立测试仲裁规则 */
struct PlanarCommand
{
  double linear{0.0};
  double angular{0.0};
};

/** @brief 一次仲裁所需的来源新鲜度和安全状态 */
struct ArbitrationInput
{
  PlanarCommand calibration;
  PlanarCommand manual;
  PlanarCommand navigation;
  bool calibration_active{false};
  bool calibration_fresh{false};
  bool observation_active{false};
  bool observation_status_stale{false};
  bool manual_fresh{false};
  bool navigation_fresh{false};
  bool mcu_online{false};
  bool emergency_stop{false};
  bool ultrasonic_valid{false};
  bool obstacle{false};
  double obstacle_distance{0.0};
  double obstacle_stop_distance{0.10};
};

/** @brief 仲裁输出以及被哪一层安全逻辑拦截 */
struct ArbitrationResult
{
  PlanarCommand command;
  bool blocked_by_safety{false};
  bool obstacle_detected{false};
  bool blocked_by_obstacle{false};
};

/**
 * @brief 实现标定/观测导航/手动优先级、故障闭锁和仅拦截前进的近障规则
 *
 * 该类不依赖 rclcpp，规则修改后可以在没有真实小车时快速回归测试
 */
class MotionArbitrator
{
public:
  static ArbitrationResult select(const ArbitrationInput & input);
  static double limit_angular_speed(
    double linear_speed, double angular_speed,
    double in_place_linear_threshold,
    double minimum_in_place, double maximum);
};

}  // namespace car_move

#endif  // CAR_MOVE__MOTION_ARBITRATOR_HPP_
