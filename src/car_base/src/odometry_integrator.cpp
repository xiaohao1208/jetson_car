#include "car_base/odometry_integrator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace car_base {

OdometryIntegrator::OdometryIntegrator(const OdometryConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.wheel_distance) ||
    config_.wheel_distance <= 0.0 ||
    !std::isfinite(config_.left_wheel_per_tick) ||
    config_.left_wheel_per_tick <= 0.0 ||
    !std::isfinite(config_.right_wheel_per_tick) ||
    config_.right_wheel_per_tick <= 0.0)
  {
    throw std::invalid_argument("差速里程计几何参数必须为有限正数");
  }
  config_.imu_yaw_weight = std::clamp(config_.imu_yaw_weight, 0.0, 1.0);
  config_.imu_timeout = std::max(config_.imu_timeout, config_.min_dt);
  config_.imu_gyro_deadband = std::max(0.0, config_.imu_gyro_deadband);
  config_.max_tick_delta = std::max<int64_t>(1, config_.max_tick_delta);
}

int64_t OdometryIntegrator::tick_delta(
  int32_t current, int32_t previous)
{
  // 使用 2^32 模数处理累计 int32 tick 的自然回绕
  constexpr int64_t modulo = static_cast<int64_t>(1ULL << 32U);
  constexpr int64_t half_range = modulo / 2;
  int64_t delta =
    static_cast<int64_t>(current) - static_cast<int64_t>(previous);
  if (delta > half_range) {
    delta -= modulo;
  } else if (delta < -half_range) {
    delta += modulo;
  }
  return delta;
}

double OdometryIntegrator::normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void OdometryIntegrator::add_imu(
  double angular_z, double time_seconds)
{
  if (!std::isfinite(angular_z) || !std::isfinite(time_seconds)) {
    have_imu_ = false;
    pending_imu_yaw_ = 0.0;
    return;
  }
  double rate = angular_z * config_.imu_yaw_sign;
  if (std::abs(rate) < config_.imu_gyro_deadband) {
    rate = 0.0;
  }
  if (have_imu_) {
    const double dt = time_seconds - previous_imu_time_;
    if (dt > 0.0 && dt <= config_.imu_timeout) {
      pending_imu_yaw_ +=
        0.5 * (previous_imu_rate_ + rate) * dt;
    } else {
      pending_imu_yaw_ = 0.0;
    }
  }
  previous_imu_rate_ = rate;
  previous_imu_time_ = time_seconds;
  have_imu_ = true;
}

std::optional<OdometryUpdate> OdometryIntegrator::update(
  int32_t left_ticks, int32_t right_ticks, bool encoder_ok,
  double time_seconds)
{
  if (!std::isfinite(time_seconds)) {
    reset_measurement_baseline();
    return std::nullopt;
  }
  if (!have_ticks_ || !encoder_ok) {
    previous_left_ticks_ = left_ticks;
    previous_right_ticks_ = right_ticks;
    previous_tick_time_ = time_seconds;
    have_ticks_ = encoder_ok;
    pending_imu_yaw_ = 0.0;
    return std::nullopt;
  }

  const double dt = time_seconds - previous_tick_time_;
  const int64_t left_delta = tick_delta(left_ticks, previous_left_ticks_);
  const int64_t right_delta = tick_delta(right_ticks, previous_right_ticks_);
  previous_left_ticks_ = left_ticks;
  previous_right_ticks_ = right_ticks;
  previous_tick_time_ = time_seconds;

  if (dt < config_.min_dt || dt > config_.max_dt ||
    std::abs(left_delta) > config_.max_tick_delta ||
    std::abs(right_delta) > config_.max_tick_delta)
  {
    pending_imu_yaw_ = 0.0;
    return std::nullopt;
  }

  const double left_distance =
    static_cast<double>(left_delta) * config_.left_wheel_per_tick;
  const double right_distance =
    static_cast<double>(right_delta) * config_.right_wheel_per_tick;
  const double distance = 0.5 * (left_distance + right_distance);
  const double encoder_yaw =
    (right_distance - left_distance) / config_.wheel_distance;
  const bool wheel_motion = left_delta != 0 || right_delta != 0;
  const bool imu_recent =
    have_imu_ && time_seconds >= previous_imu_time_ &&
    time_seconds - previous_imu_time_ <= config_.imu_timeout;

  double delta_yaw = encoder_yaw;
  bool imu_used = false;
  if (wheel_motion && imu_recent && std::isfinite(pending_imu_yaw_)) {
    delta_yaw =
      (1.0 - config_.imu_yaw_weight) * encoder_yaw +
      config_.imu_yaw_weight * pending_imu_yaw_;
    imu_used = config_.imu_yaw_weight > 0.0;
  }
  pending_imu_yaw_ = 0.0;

  // 中点法比直接使用更新前偏航更适合弧线运动
  const double midpoint_yaw = yaw_ + delta_yaw * 0.5;
  x_ += distance * std::cos(midpoint_yaw);
  y_ += distance * std::sin(midpoint_yaw);
  yaw_ = normalize_angle(yaw_ + delta_yaw);

  OdometryUpdate result;
  result.x = x_;
  result.y = y_;
  result.yaw = yaw_;
  result.linear_velocity = distance / dt;
  result.angular_velocity = delta_yaw / dt;
  result.imu_used = imu_used;
  return result;
}

void OdometryIntegrator::reset_measurement_baseline()
{
  have_ticks_ = false;
  have_imu_ = false;
  pending_imu_yaw_ = 0.0;
}

}  // namespace car_base
