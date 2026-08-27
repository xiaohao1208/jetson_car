#ifndef CAR_BASE__BASE_NODE_HPP_
#define CAR_BASE__BASE_NODE_HPP_

#include <memory>
#include <string>

#include "car_base/odometry_integrator.hpp"
#include "car_interfaces/msg/car_mcu_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace car_base {

/** @brief 订阅 ESP32 传感器并发布标准 /odom 与动态 TF */
class CarBaseNode : public rclcpp::Node {
public:
  explicit CarBaseNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  OdometryConfig load_odometry_config();
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_mcu_status(
    const car_interfaces::msg::CarMcuStatus::SharedPtr msg);
  void publish_odometry(
    const OdometryUpdate & update, const rclcpp::Time & stamp);

  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  double pose_xy_variance_;
  double pose_yaw_variance_;
  double twist_linear_variance_;
  double twist_angular_variance_;

  std::unique_ptr<OdometryIntegrator> integrator_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<car_interfaces::msg::CarMcuStatus>::SharedPtr
    mcu_status_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace car_base

#endif  // CAR_BASE__BASE_NODE_HPP_
