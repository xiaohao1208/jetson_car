#ifndef CAR_MOVE__MOVE_NODE_HPP_
#define CAR_MOVE__MOVE_NODE_HPP_

#include <string>

#include "car_interfaces/msg/car_mcu_status.hpp"
#include "car_interfaces/msg/car_status.hpp"
#include "car_interfaces/msg/calibration_status.hpp"
#include "car_interfaces/msg/observation_collection_status.hpp"
#include "car_interfaces/srv/emergency_stop.hpp"
#include "car_move/motion_arbitrator.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace car_move
{

/**
 * @brief 小车速度仲裁与基础安全节点
 *
 * Web、键盘和 Nav2 都不能直接向 ESP32 下发速度。本节点统一完成来源优先级、
 * 超时停车、限速、急停和前向近障拦截，再通过 /cmd_vel_move 输出
 */
class CarMoveNode : public rclcpp::Node
{
public:
  explicit CarMoveNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void load_parameters();
  void create_interfaces();

  void on_manual_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_calibration_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_calibration_status(
    const car_interfaces::msg::CalibrationStatus::SharedPtr msg);
  void on_observation_status(
    const car_interfaces::msg::ObservationCollectionStatus::SharedPtr msg);
  void on_nav_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_mcu_status(
    const car_interfaces::msg::CarMcuStatus::SharedPtr msg);
  void on_emergency_stop(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<car_interfaces::srv::EmergencyStop::Request> request,
    std::shared_ptr<car_interfaces::srv::EmergencyStop::Response> response);
  void on_timer();

  void publish_move_command();
  void publish_status();
  void publish_ultrasonic();

  geometry_msgs::msg::Twist clamp_twist(
    const geometry_msgs::msg::Twist & input) const;
  bool is_recent(const rclcpp::Time & stamp, double timeout) const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    manual_cmd_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    calibration_cmd_subscription_;
  rclcpp::Subscription<car_interfaces::msg::CalibrationStatus>::SharedPtr
    calibration_status_subscription_;
  rclcpp::Subscription<car_interfaces::msg::ObservationCollectionStatus>::SharedPtr
    observation_status_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    nav_cmd_subscription_;
  rclcpp::Subscription<car_interfaces::msg::CarMcuStatus>::SharedPtr
    mcu_status_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    move_cmd_publisher_;
  rclcpp::Publisher<car_interfaces::msg::CarStatus>::SharedPtr
    status_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
    ultrasonic_scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr
    ultrasonic_range_publisher_;
  rclcpp::Service<car_interfaces::srv::EmergencyStop>::SharedPtr
    emergency_stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  double manual_cmd_timeout_;
  double calibration_cmd_timeout_;
  double nav_cmd_timeout_;
  double observation_status_timeout_;
  double mcu_status_timeout_;
  double move_publish_rate_;
  double max_linear_speed_;
  double in_place_linear_threshold_;
  double min_angular_speed_;
  double max_angular_speed_;
  double obstacle_stop_distance_;
  double ultrasonic_min_range_;
  double ultrasonic_max_range_;
  double ultrasonic_fov_;
  std::string ultrasonic_frame_id_;

  bool e_stop_active_;
  std::string e_stop_source_;
  std::string e_stop_reason_;
  bool calibration_active_;
  bool observation_active_;
  bool observation_monitor_armed_;
  bool observation_status_stale_latched_;
  bool observation_fresh_inactive_;
  bool have_mcu_status_;
  bool obstacle_blocking_active_;
  std::string active_command_source_;
  geometry_msgs::msg::Twist latest_manual_cmd_;
  geometry_msgs::msg::Twist latest_calibration_cmd_;
  geometry_msgs::msg::Twist latest_nav_cmd_;
  car_interfaces::msg::CarMcuStatus latest_mcu_status_;
  rclcpp::Time last_manual_cmd_time_;
  rclcpp::Time last_calibration_cmd_time_;
  rclcpp::Time last_observation_status_time_;
  rclcpp::Time last_nav_cmd_time_;
  rclcpp::Time last_mcu_status_time_;
  rclcpp::Time motion_stall_started_;
};

}  // namespace car_move

#endif  // CAR_MOVE__MOVE_NODE_HPP_
