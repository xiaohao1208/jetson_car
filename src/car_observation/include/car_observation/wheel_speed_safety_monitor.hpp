#ifndef CAR_OBSERVATION__WHEEL_SPEED_SAFETY_MONITOR_HPP_
#define CAR_OBSERVATION__WHEEL_SPEED_SAFETY_MONITOR_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace car_observation
{

enum class WheelSpeedSafetyState
{
  ok,
  pending,
  sustained_overspeed,
  immediate_overspeed,
  invalid_feedback,
};

struct WheelSpeedSafetyResult
{
  WheelSpeedSafetyState state{WheelSpeedSafetyState::ok};
  double maximum_speed_mps{0.0};
  double overspeed_duration_sec{0.0};
};

/**
 * @brief 对按采样时间排列的左右轮反馈执行持续超速和严重超速判定。
 *
 * 相同时间戳表示同一帧 MCU 数据，重复检查不会让持续时间增长。
 */
class WheelSpeedSafetyMonitor
{
public:
  void configure(double sustained_limit_mps, double hold_sec, double immediate_limit_mps)
  {
    sustained_limit_mps_ = sustained_limit_mps;
    hold_sec_ = hold_sec;
    immediate_limit_mps_ = immediate_limit_mps;
    reset();
  }

  void reset()
  {
    last_sample_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    overspeed_started_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_result_ = WheelSpeedSafetyResult();
  }

  WheelSpeedSafetyResult update(
    double left_speed_mps, double right_speed_mps, double sample_time_sec)
  {
    if (std::isfinite(last_sample_time_sec_) && sample_time_sec == last_sample_time_sec_) {
      return last_result_;
    }
    if (!std::isfinite(sample_time_sec) ||
      (std::isfinite(last_sample_time_sec_) && sample_time_sec < last_sample_time_sec_))
    {
      reset();
    }
    last_sample_time_sec_ = sample_time_sec;

    if (!std::isfinite(left_speed_mps) || !std::isfinite(right_speed_mps)) {
      overspeed_started_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_result_ = {
        WheelSpeedSafetyState::invalid_feedback,
        std::numeric_limits<double>::quiet_NaN(), 0.0};
      return last_result_;
    }

    const double maximum_speed =
      std::max(std::abs(left_speed_mps), std::abs(right_speed_mps));
    if (maximum_speed >= immediate_limit_mps_) {
      overspeed_started_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_result_ = {
        WheelSpeedSafetyState::immediate_overspeed, maximum_speed, 0.0};
      return last_result_;
    }
    if (maximum_speed <= sustained_limit_mps_) {
      overspeed_started_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_result_ = {WheelSpeedSafetyState::ok, maximum_speed, 0.0};
      return last_result_;
    }

    if (!std::isfinite(overspeed_started_sec_)) {
      overspeed_started_sec_ = sample_time_sec;
    }
    const double duration = std::max(0.0, sample_time_sec - overspeed_started_sec_);
    last_result_ = {
      duration + 1.0e-9 >= hold_sec_ ? WheelSpeedSafetyState::sustained_overspeed :
      WheelSpeedSafetyState::pending,
      maximum_speed, duration};
    return last_result_;
  }

private:
  double sustained_limit_mps_{0.20};
  double hold_sec_{0.20};
  double immediate_limit_mps_{0.30};
  double last_sample_time_sec_{std::numeric_limits<double>::quiet_NaN()};
  double overspeed_started_sec_{std::numeric_limits<double>::quiet_NaN()};
  WheelSpeedSafetyResult last_result_;
};

}  // namespace car_observation

#endif  // CAR_OBSERVATION__WHEEL_SPEED_SAFETY_MONITOR_HPP_
