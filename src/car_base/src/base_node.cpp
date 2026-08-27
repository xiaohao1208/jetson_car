#include "car_base/base_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace car_base {

CarBaseNode::CarBaseNode(const rclcpp::NodeOptions & options)
: Node("car_base", options),
  publish_tf_(true),
  pose_xy_variance_(0.02),
  pose_yaw_variance_(0.04),
  twist_linear_variance_(0.04),
  twist_angular_variance_(0.06)
{
  declare_parameters();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  publish_tf_ = get_parameter("publish_tf").as_bool();
  pose_xy_variance_ = get_parameter("pose_xy_variance").as_double();
  pose_yaw_variance_ = get_parameter("pose_yaw_variance").as_double();
  twist_linear_variance_ =
    get_parameter("twist_linear_variance").as_double();
  twist_angular_variance_ =
    get_parameter("twist_angular_variance").as_double();
  integrator_ =
    std::make_unique<OdometryIntegrator>(load_odometry_config());

  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SystemDefaultsQoS());
  tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&CarBaseNode::on_imu, this, std::placeholders::_1));
  mcu_status_subscription_ =
    create_subscription<car_interfaces::msg::CarMcuStatus>(
    "/car/mcu_status", rclcpp::SensorDataQoS(),
    std::bind(&CarBaseNode::on_mcu_status, this, std::placeholders::_1));
}

void CarBaseNode::declare_parameters()
{
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_footprint");
  declare_parameter<bool>("publish_tf", true);
  declare_parameter<double>("wheel_distance", 0.175);
  declare_parameter<double>("left_wheel_per_tick", 0.0001039203);
  declare_parameter<double>("right_wheel_per_tick", 0.0001033942);
  declare_parameter<double>("imu_yaw_weight", 0.7);
  declare_parameter<double>("imu_timeout_sec", 0.25);
  declare_parameter<double>("imu_gyro_deadband_rps", 0.02);
  declare_parameter<double>("imu_yaw_sign", 1.0);
  declare_parameter<double>("min_odom_dt_sec", 0.001);
  declare_parameter<double>("max_odom_dt_sec", 1.0);
  declare_parameter<int>("max_tick_delta", 20000);
  declare_parameter<double>("pose_xy_variance", 0.02);
  declare_parameter<double>("pose_yaw_variance", 0.04);
  declare_parameter<double>("twist_linear_variance", 0.04);
  declare_parameter<double>("twist_angular_variance", 0.06);
}

OdometryConfig CarBaseNode::load_odometry_config()
{
  OdometryConfig config;
  config.wheel_distance = get_parameter("wheel_distance").as_double();
  config.left_wheel_per_tick =
    get_parameter("left_wheel_per_tick").as_double();
  config.right_wheel_per_tick =
    get_parameter("right_wheel_per_tick").as_double();
  config.imu_yaw_weight =
    get_parameter("imu_yaw_weight").as_double();
  config.imu_timeout =
    get_parameter("imu_timeout_sec").as_double();
  config.imu_gyro_deadband =
    get_parameter("imu_gyro_deadband_rps").as_double();
  config.imu_yaw_sign = get_parameter("imu_yaw_sign").as_double();
  config.min_dt = get_parameter("min_odom_dt_sec").as_double();
  config.max_dt = get_parameter("max_odom_dt_sec").as_double();
  config.max_tick_delta = get_parameter("max_tick_delta").as_int();
  return config;
}

void CarBaseNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  integrator_->add_imu(msg->angular_velocity.z, now().seconds());
}

void CarBaseNode::on_mcu_status(
  const car_interfaces::msg::CarMcuStatus::SharedPtr msg)
{
  const rclcpp::Time receive_time = now();
  const auto update = integrator_->update(
    msg->left_encoder_ticks, msg->right_encoder_ticks,
    msg->encoder_ok, receive_time.seconds());
  if (update.has_value()) {
    publish_odometry(*update, receive_time);
  }
}

void CarBaseNode::publish_odometry(
  const OdometryUpdate & update, const rclcpp::Time & stamp)
{
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, update.yaw);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = update.x;
  odom.pose.pose.position.y = update.y;
  odom.pose.pose.orientation.x = orientation.x();
  odom.pose.pose.orientation.y = orientation.y();
  odom.pose.pose.orientation.z = orientation.z();
  odom.pose.pose.orientation.w = orientation.w();
  odom.twist.twist.linear.x = update.linear_velocity;
  odom.twist.twist.angular.z = update.angular_velocity;
  odom.pose.covariance[0] = pose_xy_variance_;
  odom.pose.covariance[7] = pose_xy_variance_;
  odom.pose.covariance[35] = pose_yaw_variance_;
  odom.twist.covariance[0] = twist_linear_variance_;
  odom.twist.covariance[35] = twist_angular_variance_;
  odom_publisher_->publish(odom);

  if (!publish_tf_) {
    return;
  }
  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom.header;
  transform.child_frame_id = base_frame_;
  transform.transform.translation.x = update.x;
  transform.transform.translation.y = update.y;
  transform.transform.rotation = odom.pose.pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace car_base
