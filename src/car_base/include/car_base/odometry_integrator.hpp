#ifndef CAR_BASE__ODOMETRY_INTEGRATOR_HPP_
#define CAR_BASE__ODOMETRY_INTEGRATOR_HPP_

#include <cstdint>
#include <optional>

namespace car_base {

/** @brief 不依赖 ROS 的差速里程计参数 */
struct OdometryConfig {
  double wheel_distance{0.175};
  double left_wheel_per_tick{0.0001039203};
  double right_wheel_per_tick{0.0001033942};
  double imu_yaw_weight{0.7};
  double imu_timeout{0.25};
  double imu_gyro_deadband{0.02};
  double imu_yaw_sign{1.0};
  double min_dt{0.001};
  double max_dt{1.0};
  int64_t max_tick_delta{20000};
};

/** @brief 一次有效编码器更新产生的完整二维里程计状态 */
struct OdometryUpdate {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  bool imu_used{false};
};

/**
 * @brief 将左右轮累计 tick 与陀螺仪角速度融合为连续二维位姿
 *
 * IMU 只修正编码器已经表明发生运动的帧，静止时不会单独积分陀螺零偏
 */
class OdometryIntegrator {
public:
  explicit OdometryIntegrator(const OdometryConfig & config);

  /** @brief 加入一帧 Z 轴角速度，时间单位为秒 */
  void add_imu(double angular_z, double time_seconds);

  /**
   * @brief 消费累计编码器 tick，首次或异常帧只重建基线
   * @return 有效时间间隔内的里程计结果，否则为空
   */
  std::optional<OdometryUpdate> update(
    int32_t left_ticks, int32_t right_ticks, bool encoder_ok,
    double time_seconds);

  /** @brief 清除 tick/IMU 时间基线，但保留已经累计的位姿 */
  void reset_measurement_baseline();

private:
  static int64_t tick_delta(int32_t current, int32_t previous);
  static double normalize_angle(double angle);

  OdometryConfig config_;
  bool have_ticks_{false};
  bool have_imu_{false};
  int32_t previous_left_ticks_{0};
  int32_t previous_right_ticks_{0};
  double previous_tick_time_{0.0};
  double previous_imu_time_{0.0};
  double previous_imu_rate_{0.0};
  double pending_imu_yaw_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
};

}  // namespace car_base

#endif  // CAR_BASE__ODOMETRY_INTEGRATOR_HPP_
