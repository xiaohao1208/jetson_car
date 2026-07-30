#include "car_move/move_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace car_move {

CarMoveNode::CarMoveNode(const rclcpp::NodeOptions & options)
: Node("car_move", options),
  manual_cmd_timeout_(0.75),
  nav_cmd_timeout_(0.75),
  mcu_status_timeout_(3.0),
  move_publish_rate_(15.0),
  max_linear_speed_(0.4),
  min_angular_speed_(0.523598776),
  max_angular_speed_(2.094395102),
  obstacle_stop_distance_(0.25),
  ultrasonic_min_range_(0.03),
  ultrasonic_max_range_(1.0),
  ultrasonic_fov_(0.35),
  ultrasonic_frame_id_("ultrasonic_link"),
  e_stop_active_(false),
  have_mcu_status_(false),
  obstacle_blocking_active_(false),
  active_command_source_("初始化")
{
  declare_parameters();
  load_parameters();
  create_interfaces();
}

void CarMoveNode::declare_parameters()
{
  declare_parameter<double>("manual_cmd_timeout_sec", 0.75);
  declare_parameter<double>("nav_cmd_timeout_sec", 0.75);
  declare_parameter<double>("mcu_status_timeout_sec", 3.0);
  declare_parameter<double>("move_cmd_publish_hz", 15.0);
  declare_parameter<double>("max_linear_speed", 0.4);
  declare_parameter<double>("min_angular_speed", 0.523598776);
  declare_parameter<double>("max_angular_speed", 2.094395102);
  declare_parameter<double>("obstacle_stop_distance_m", 0.25);
  declare_parameter<double>("ultrasonic_min_range_m", 0.03);
  declare_parameter<double>("ultrasonic_max_range_m", 1.0);
  declare_parameter<double>("ultrasonic_fov_rad", 0.35);
  declare_parameter<std::string>(
    "ultrasonic_frame_id", "ultrasonic_link");
}

void CarMoveNode::load_parameters()
{
  const auto positive = [](double value, double fallback) {
      return std::isfinite(value) && value > 0.0 ? value : fallback;
    };
  const auto nonnegative = [](double value, double fallback) {
      return std::isfinite(value) && value >= 0.0 ? value : fallback;
    };

  manual_cmd_timeout_ = positive(
    get_parameter("manual_cmd_timeout_sec").as_double(), 0.75);
  nav_cmd_timeout_ = positive(
    get_parameter("nav_cmd_timeout_sec").as_double(), 0.75);
  mcu_status_timeout_ = positive(
    get_parameter("mcu_status_timeout_sec").as_double(), 3.0);
  move_publish_rate_ = positive(
    get_parameter("move_cmd_publish_hz").as_double(), 15.0);
  max_linear_speed_ = positive(
    get_parameter("max_linear_speed").as_double(), 0.4);
  max_angular_speed_ = positive(
    get_parameter("max_angular_speed").as_double(), 2.094395102);
  min_angular_speed_ = std::min(
    positive(
      get_parameter("min_angular_speed").as_double(), 0.523598776),
    max_angular_speed_);
  obstacle_stop_distance_ = nonnegative(
    get_parameter("obstacle_stop_distance_m").as_double(), 0.25);
  ultrasonic_min_range_ = nonnegative(
    get_parameter("ultrasonic_min_range_m").as_double(), 0.03);
  ultrasonic_max_range_ = std::max(
    ultrasonic_min_range_,
    nonnegative(
      get_parameter("ultrasonic_max_range_m").as_double(), 1.0));
  ultrasonic_fov_ = nonnegative(
    get_parameter("ultrasonic_fov_rad").as_double(), 0.35);
  ultrasonic_frame_id_ =
    get_parameter("ultrasonic_frame_id").as_string();
  if (ultrasonic_frame_id_.empty()) {
    ultrasonic_frame_id_ = "ultrasonic_link";
  }
}

void CarMoveNode::create_interfaces()
{
  // 手动入口供 Web 和 teleop 使用，新鲜的零速也会短暂压住导航残留速度
  manual_cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_manual", rclcpp::SystemDefaultsQoS(),
    std::bind(&CarMoveNode::on_manual_cmd, this, std::placeholders::_1));
  // Nav2 velocity_smoother 的最终输出固定为 /cmd_vel
  nav_cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&CarMoveNode::on_nav_cmd, this, std::placeholders::_1));
  mcu_status_subscription_ =
    create_subscription<car_interfaces::msg::CarMcuStatus>(
    "/car/mcu_status", rclcpp::SensorDataQoS(),
    std::bind(&CarMoveNode::on_mcu_status, this, std::placeholders::_1));

  move_cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_move", rclcpp::SensorDataQoS());
  status_publisher_ = create_publisher<car_interfaces::msg::CarStatus>(
    "/car/status", rclcpp::SystemDefaultsQoS());
  ultrasonic_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
    "/ultrasonic_scan", rclcpp::SensorDataQoS());
  ultrasonic_range_publisher_ = create_publisher<sensor_msgs::msg::Range>(
    "/ultrasonic_range", rclcpp::SensorDataQoS());
  emergency_stop_service_ =
    create_service<car_interfaces::srv::EmergencyStop>(
    "/car/e_stop",
    std::bind(
      &CarMoveNode::on_emergency_stop, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / move_publish_rate_),
    std::bind(&CarMoveNode::on_timer, this));
}

geometry_msgs::msg::Twist CarMoveNode::clamp_twist(
  const geometry_msgs::msg::Twist & input) const
{
  geometry_msgs::msg::Twist output;
  const double linear =
    std::isfinite(input.linear.x) ? input.linear.x : 0.0;
  const double angular =
    std::isfinite(input.angular.z) ? input.angular.z : 0.0;
  output.linear.x =
    std::clamp(linear, -max_linear_speed_, max_linear_speed_);
  output.angular.z = MotionArbitrator::limit_angular_speed(
    angular, min_angular_speed_, max_angular_speed_);
  return output;
}

bool CarMoveNode::is_recent(
  const rclcpp::Time & stamp, double timeout) const
{
  if (stamp.nanoseconds() == 0) {
    return false;
  }
  const double age = (now() - stamp).seconds();
  return age >= 0.0 && age <= timeout;
}

void CarMoveNode::on_manual_cmd(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_manual_cmd_ = clamp_twist(*msg);
  last_manual_cmd_time_ = now();
}

void CarMoveNode::on_nav_cmd(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_nav_cmd_ = clamp_twist(*msg);
  last_nav_cmd_time_ = now();
}

void CarMoveNode::on_mcu_status(
  const car_interfaces::msg::CarMcuStatus::SharedPtr msg)
{
  latest_mcu_status_ = *msg;
  have_mcu_status_ = true;
  last_mcu_status_time_ = now();
}

void CarMoveNode::on_emergency_stop(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<car_interfaces::srv::EmergencyStop::Request> request,
  std::shared_ptr<car_interfaces::srv::EmergencyStop::Response> response)
{
  e_stop_active_ = request->stop;
  if (e_stop_active_) {
    geometry_msgs::msg::Twist stop_command;
    move_cmd_publisher_->publish(stop_command);
  }
  response->success = true;
  response->message = e_stop_active_ ? "急停已触发" : "急停已解除";
  RCLCPP_WARN(
    get_logger(), "%s，来源=%s，原因=%s", response->message.c_str(),
    request->source.c_str(), request->reason.c_str());
}

void CarMoveNode::publish_move_command()
{
  const bool mcu_online =
    have_mcu_status_ && is_recent(last_mcu_status_time_, mcu_status_timeout_);
  const bool manual_fresh =
    is_recent(last_manual_cmd_time_, manual_cmd_timeout_);
  const bool nav_fresh = is_recent(last_nav_cmd_time_, nav_cmd_timeout_);
  ArbitrationInput input;
  input.manual = {
    latest_manual_cmd_.linear.x, latest_manual_cmd_.angular.z};
  input.navigation = {
    latest_nav_cmd_.linear.x, latest_nav_cmd_.angular.z};
  input.manual_fresh = manual_fresh;
  input.navigation_fresh = nav_fresh;
  input.mcu_online = mcu_online;
  input.emergency_stop = e_stop_active_;
  input.ultrasonic_valid =
    have_mcu_status_ && latest_mcu_status_.ultrasonic_ok;
  input.obstacle = latest_mcu_status_.obstacle_ok;
  input.obstacle_distance = latest_mcu_status_.obstacle_distance;
  input.obstacle_stop_distance = obstacle_stop_distance_;
  const auto selection = MotionArbitrator::select(input);

  geometry_msgs::msg::Twist output;
  output.linear.x = selection.command.linear;
  output.angular.z = selection.command.angular;
  obstacle_blocking_active_ = selection.blocked_by_obstacle;
  std::string source = "空闲";
  if (!mcu_online) {
    source = "底盘离线";
  } else if (e_stop_active_) {
    source = "急停";
  } else if (selection.blocked_by_obstacle) {
    source = "超声波近障";
  } else if (manual_fresh) {
    source = "手动";
  } else if (nav_fresh) {
    source = "导航";
  }
  const bool command_is_zero =
    std::abs(output.linear.x) < 0.001 &&
    std::abs(output.angular.z) < 0.001;
  if (
    command_is_zero &&
    (source == "手动" || source == "导航"))
  {
    source = "空闲";
  }
  if (source != active_command_source_) {
    active_command_source_ = source;
    if (source != "空闲") {
      RCLCPP_INFO(
        get_logger(),
        "速度来源切换为%s，输出线速度=%.3f m/s，角速度=%.3f rad/s",
        source.c_str(), output.linear.x, output.angular.z);
    }
  }
  if (selection.blocked_by_obstacle) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "前进命令已拦截：前方障碍距离 %.3f m",
      latest_mcu_status_.obstacle_distance);
  }
  if (selection.blocked_by_safety) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s",
      e_stop_active_ ? "运动命令已拦截：急停已触发" :
      "运动命令已拦截：MCU 状态缺失或过期");
  }
  const bool motion_requested =
    std::abs(output.linear.x) >= 0.03 ||
    std::abs(output.angular.z) >= min_angular_speed_;
  const bool wheels_stopped =
    have_mcu_status_ &&
    std::abs(latest_mcu_status_.left_wheel_speed) < 0.01 &&
    std::abs(latest_mcu_status_.right_wheel_speed) < 0.01;
  if (motion_requested && wheels_stopped) {
    if (motion_stall_started_.nanoseconds() == 0) {
      motion_stall_started_ = now();
    } else if ((now() - motion_stall_started_).seconds() >= 1.5) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "%s命令持续存在但车轮未转动，线速度=%.3f m/s，角速度=%.3f rad/s，"
        "左轮=%.3f m/s，右轮=%.3f m/s",
        manual_fresh ? "手动" : "导航",
        output.linear.x, output.angular.z,
        latest_mcu_status_.left_wheel_speed,
        latest_mcu_status_.right_wheel_speed);
    }
  } else {
    motion_stall_started_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }
  move_cmd_publisher_->publish(output);
}

void CarMoveNode::publish_status()
{
  car_interfaces::msg::CarStatus status;
  status.stamp = now();
  // 接口字段沿用现有命名，true 表示当前急停处于触发状态
  status.e_stop_ok = e_stop_active_;
  status.mcu_ok =
    have_mcu_status_ && is_recent(last_mcu_status_time_, mcu_status_timeout_);
  if (status.mcu_ok) {
    status.wifi_connect_ok = latest_mcu_status_.wifi_connect_ok;
    status.agent_connect_ok = latest_mcu_status_.agent_connect_ok;
    status.imu_ok = latest_mcu_status_.imu_ok;
    status.encoder_ok = latest_mcu_status_.encoder_ok;
    status.motor_driver_ok = latest_mcu_status_.motor_driver_ok;
    status.wheel_speed = static_cast<float>(
      (latest_mcu_status_.left_wheel_speed +
      latest_mcu_status_.right_wheel_speed) * 0.5);
    status.ultrasonic_ok = latest_mcu_status_.ultrasonic_ok;
    status.obstacle_ok =
      latest_mcu_status_.obstacle_ok || obstacle_blocking_active_;
    status.obstacle_distance = latest_mcu_status_.obstacle_distance;
    status.fault_bits = latest_mcu_status_.fault_bits;
  }
  if (e_stop_active_) {
    status.fault_bits |= (1U << 18U);
  }
  if (!status.mcu_ok) {
    status.fault_bits |= (1U << 19U);
  }
  if (obstacle_blocking_active_) {
    status.fault_bits |= (1U << 20U);
  }
  status_publisher_->publish(status);
}

void CarMoveNode::publish_ultrasonic()
{
  const auto stamp = now();
  const bool reading_valid =
    have_mcu_status_ &&
    is_recent(last_mcu_status_time_, mcu_status_timeout_) &&
    latest_mcu_status_.ultrasonic_ok &&
    std::isfinite(latest_mcu_status_.obstacle_distance) &&
    latest_mcu_status_.obstacle_distance >= ultrasonic_min_range_ &&
    latest_mcu_status_.obstacle_distance <= ultrasonic_max_range_;
  const float range = reading_valid ?
    latest_mcu_status_.obstacle_distance :
    std::numeric_limits<float>::infinity();

  sensor_msgs::msg::Range range_message;
  range_message.header.stamp = stamp;
  range_message.header.frame_id = ultrasonic_frame_id_;
  range_message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  range_message.field_of_view = static_cast<float>(ultrasonic_fov_);
  range_message.min_range = static_cast<float>(ultrasonic_min_range_);
  range_message.max_range = static_cast<float>(ultrasonic_max_range_);
  range_message.range = range;
  ultrasonic_range_publisher_->publish(range_message);

  // 兼容旧调试工具，Nav2不再使用该话题
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = ultrasonic_frame_id_;
  scan.angle_min = static_cast<float>(-ultrasonic_fov_ * 0.5);
  scan.angle_max = static_cast<float>(ultrasonic_fov_ * 0.5);
  scan.angle_increment = static_cast<float>(ultrasonic_fov_ * 0.5);
  scan.scan_time = static_cast<float>(1.0 / move_publish_rate_);
  scan.range_min = static_cast<float>(ultrasonic_min_range_);
  scan.range_max = static_cast<float>(ultrasonic_max_range_);
  scan.ranges.assign(3U, std::numeric_limits<float>::infinity());

  if (reading_valid) {
    std::fill(scan.ranges.begin(), scan.ranges.end(), range);
  }
  ultrasonic_scan_publisher_->publish(scan);
}

void CarMoveNode::on_timer()
{
  publish_move_command();
  publish_ultrasonic();
  publish_status();
}

}  // namespace car_move
