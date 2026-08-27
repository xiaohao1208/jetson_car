#include "car_move/move_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace car_move
{

CarMoveNode::CarMoveNode(const rclcpp::NodeOptions & options)
: Node("car_move", options),
  manual_cmd_timeout_(0.75),
  calibration_cmd_timeout_(0.25),
  nav_cmd_timeout_(0.75),
  observation_status_timeout_(1.5),
  mcu_status_timeout_(0.5),
  move_publish_rate_(15.0),
  max_linear_speed_(0.4),
  in_place_linear_threshold_(0.005),
  min_angular_speed_(0.523598776),
  max_angular_speed_(2.094395102),
  obstacle_stop_distance_(0.10),
  ultrasonic_min_range_(0.03),
  ultrasonic_max_range_(1.0),
  ultrasonic_fov_(0.35),
  ultrasonic_frame_id_("ultrasonic_link"),
  e_stop_active_(false),
  calibration_active_(false),
  observation_active_(false),
  observation_monitor_armed_(false),
  observation_status_stale_latched_(false),
  observation_fresh_inactive_(false),
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
  declare_parameter<double>("calibration_cmd_timeout_sec", 0.25);
  declare_parameter<double>("nav_cmd_timeout_sec", 0.75);
  declare_parameter<double>("observation_status_timeout_sec", 1.5);
  declare_parameter<double>("mcu_status_timeout_sec", 0.5);
  declare_parameter<double>("move_cmd_publish_hz", 15.0);
  declare_parameter<double>("max_linear_speed", 0.4);
  declare_parameter<double>("in_place_linear_threshold_mps", 0.005);
  declare_parameter<double>("min_angular_speed", 0.523598776);
  declare_parameter<double>("max_angular_speed", 2.094395102);
  declare_parameter<double>("obstacle_stop_distance_m", 0.10);
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
  calibration_cmd_timeout_ = positive(
    get_parameter("calibration_cmd_timeout_sec").as_double(), 0.25);
  nav_cmd_timeout_ = positive(
    get_parameter("nav_cmd_timeout_sec").as_double(), 0.75);
  observation_status_timeout_ = positive(
    get_parameter("observation_status_timeout_sec").as_double(), 1.5);
  mcu_status_timeout_ = positive(
    get_parameter("mcu_status_timeout_sec").as_double(), 0.5);
  move_publish_rate_ = positive(
    get_parameter("move_cmd_publish_hz").as_double(), 15.0);
  max_linear_speed_ = positive(
    get_parameter("max_linear_speed").as_double(), 0.4);
  in_place_linear_threshold_ = nonnegative(
    get_parameter("in_place_linear_threshold_mps").as_double(), 0.005);
  max_angular_speed_ = positive(
    get_parameter("max_angular_speed").as_double(), 2.094395102);
  min_angular_speed_ = std::min(
    positive(
      get_parameter("min_angular_speed").as_double(), 0.523598776),
    max_angular_speed_);
  obstacle_stop_distance_ = nonnegative(
    get_parameter("obstacle_stop_distance_m").as_double(), 0.10);
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
  calibration_cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_calibration", rclcpp::SystemDefaultsQoS(),
    std::bind(&CarMoveNode::on_calibration_cmd, this, std::placeholders::_1));
  calibration_status_subscription_ =
    create_subscription<car_interfaces::msg::CalibrationStatus>(
    "/car/calibration_status",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(
      &CarMoveNode::on_calibration_status, this, std::placeholders::_1));
  observation_status_subscription_ =
    create_subscription<car_interfaces::msg::ObservationCollectionStatus>(
    "/car/observation_collection_status",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(
      &CarMoveNode::on_observation_status, this, std::placeholders::_1));
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
    output.linear.x, angular, in_place_linear_threshold_,
    min_angular_speed_, max_angular_speed_);
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

void CarMoveNode::on_calibration_cmd(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_calibration_cmd_ = clamp_twist(*msg);
  last_calibration_cmd_time_ = now();
}

void CarMoveNode::on_calibration_status(
  const car_interfaces::msg::CalibrationStatus::SharedPtr msg)
{
  calibration_active_ = msg->active;
  if (!calibration_active_) {
    latest_calibration_cmd_ = geometry_msgs::msg::Twist();
    last_calibration_cmd_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }
}

void CarMoveNode::on_observation_status(
  const car_interfaces::msg::ObservationCollectionStatus::SharedPtr msg)
{
  observation_active_ = msg->active;
  last_observation_status_time_ = now();
  if (observation_active_) {
    observation_monitor_armed_ = true;
    observation_fresh_inactive_ = false;
  } else if (observation_status_stale_latched_) {
    observation_fresh_inactive_ = true;
  }
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
  const bool calibration_request = request->source == "car_calibrate";
  const bool observation_request = request->source == "car_observation_collect";
  const bool program_request = calibration_request || observation_request;
  const bool real_estop_already_active =
    e_stop_active_ && !e_stop_source_.empty() &&
    e_stop_source_ != "car_calibrate" &&
    e_stop_source_ != "car_observation_collect";
  if (program_request && real_estop_already_active) {
    response->success = false;
    response->message =
      "真实急停已触发，程序安全锁不得覆盖或解除";
    RCLCPP_WARN(
      get_logger(), "%s，当前来源=%s，当前原因=%s",
      response->message.c_str(), e_stop_source_.c_str(), e_stop_reason_.c_str());
    return;
  }
  if (
    program_request && e_stop_active_ && !e_stop_source_.empty() &&
    e_stop_source_ != request->source)
  {
    response->success = false;
    response->message = "另一个程序已持有安全锁，拒绝覆盖或解除";
    return;
  }
  if (!request->stop && e_stop_active_ && e_stop_source_ != request->source) {
    response->success = false;
    response->message = "急停解除来源与当前锁所有者不一致";
    return;
  }
  if (!request->stop && observation_status_stale_latched_) {
    if (!observation_fresh_inactive_) {
      response->success = false;
      response->message = "观测状态失联闭锁尚未收到fresh inactive状态";
      return;
    }
    observation_status_stale_latched_ = false;
    observation_monitor_armed_ = false;
    observation_fresh_inactive_ = false;
  }
  e_stop_active_ = request->stop;
  if (e_stop_active_) {
    e_stop_source_ = request->source;
    e_stop_reason_ = request->reason;
    geometry_msgs::msg::Twist stop_command;
    move_cmd_publisher_->publish(stop_command);
  } else {
    e_stop_source_.clear();
    e_stop_reason_.clear();
  }
  const bool calibration_lock = calibration_request;
  const bool observation_lock = observation_request;
  response->success = true;
  if (calibration_lock) {
    response->message = e_stop_active_ ?
      "标定安全锁已启用" : "标定安全锁已解除";
  } else if (observation_lock) {
    response->message = e_stop_active_ ?
      "观测采集安全锁已启用" : "观测采集安全锁已解除";
  } else {
    response->message = e_stop_active_ ? "真实急停已触发" : "真实急停已解除";
  }
  if (e_stop_active_ && !calibration_lock) {
    RCLCPP_WARN(
      get_logger(), "%s，来源=%s，原因=%s", response->message.c_str(),
      request->source.c_str(), request->reason.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "%s，来源=%s，原因=%s", response->message.c_str(),
      request->source.c_str(), request->reason.c_str());
  }
}

void CarMoveNode::publish_move_command()
{
  const bool mcu_online =
    have_mcu_status_ && is_recent(last_mcu_status_time_, mcu_status_timeout_);
  const bool manual_fresh =
    is_recent(last_manual_cmd_time_, manual_cmd_timeout_);
  const bool calibration_fresh =
    is_recent(last_calibration_cmd_time_, calibration_cmd_timeout_);
  const bool nav_fresh = is_recent(last_nav_cmd_time_, nav_cmd_timeout_);
  const bool observation_status_fresh =
    is_recent(last_observation_status_time_, observation_status_timeout_);
  if (
    observation_monitor_armed_ && !observation_status_fresh &&
    !observation_status_stale_latched_)
  {
    observation_status_stale_latched_ = true;
    observation_fresh_inactive_ = false;
    RCLCPP_ERROR(
      get_logger(),
      "观测采集状态超过%.2fs未刷新，闭锁为零并等待fresh inactive和人工解除",
      observation_status_timeout_);
  }
  ArbitrationInput input;
  input.calibration = {
    latest_calibration_cmd_.linear.x, latest_calibration_cmd_.angular.z};
  input.manual = {
    latest_manual_cmd_.linear.x, latest_manual_cmd_.angular.z};
  input.navigation = {
    latest_nav_cmd_.linear.x, latest_nav_cmd_.angular.z};
  input.calibration_active = calibration_active_;
  input.calibration_fresh = calibration_fresh;
  input.observation_active = observation_active_;
  input.observation_status_stale = observation_status_stale_latched_;
  input.manual_fresh = manual_fresh;
  input.navigation_fresh = nav_fresh;
  input.mcu_online = mcu_online;
  input.emergency_stop = e_stop_active_ || observation_status_stale_latched_;
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
    source = e_stop_source_ == "car_calibrate" ? "标定安全锁" :
      (e_stop_source_ == "car_observation_collect" ? "观测采集安全锁" : "急停");
  } else if (observation_status_stale_latched_) {
    source = "观测采集状态失联闭锁";
  } else if (calibration_active_ && !calibration_fresh) {
    source = "标定命令过期";
  } else if (selection.blocked_by_obstacle) {
    source = "超声波近障";
  } else if (calibration_active_) {
    source = "标定";
  } else if (observation_active_) {
    source = nav_fresh ? "观测导航" : "观测导航命令过期";
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
    (source == "标定" || source == "手动" || source == "导航"))
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
    const char * safety_message = e_stop_active_ ?
      (e_stop_source_ == "car_calibrate" ?
      "非零运动命令已拦截：标定安全锁已启用" :
      (e_stop_source_ == "car_observation_collect" ?
      "非零运动命令已拦截：观测采集安全锁已启用" :
      "非零运动命令已拦截：真实急停已触发")) :
      (observation_status_stale_latched_ ?
      "运动命令已拦截：观测采集状态失联闭锁" :
      (calibration_active_ && !calibration_fresh ?
      "运动命令已拦截：标定命令缺失或过期" :
      (observation_active_ && !nav_fresh ?
      "运动命令已拦截：观测期间导航命令缺失或过期" :
      "运动命令已拦截：MCU 状态缺失或过期")));
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s",
      safety_message);
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
        calibration_active_ ? "标定" :
        (observation_active_ ? "观测导航" : (manual_fresh ? "手动" : "导航")),
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
  status.e_stop_ok = e_stop_active_ || observation_status_stale_latched_;
  status.e_stop_source = observation_status_stale_latched_ && !e_stop_active_ ?
    "observation_collection_status_stale" : e_stop_source_;
  status.e_stop_reason = observation_status_stale_latched_ && !e_stop_active_ ?
    "观测采集状态失联，等待fresh inactive状态和人工解除" : e_stop_reason_;
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
    // 障碍状态必须与当前命令解耦。观测程序取消前进并开始左转后仍需知道
    // 前方是否真正恢复安全，不能因为“当前没有前进命令”就把障碍清掉。
    const bool distance_blocks =
      latest_mcu_status_.ultrasonic_ok &&
      std::isfinite(latest_mcu_status_.obstacle_distance) &&
      latest_mcu_status_.obstacle_distance > 0.0F &&
      latest_mcu_status_.obstacle_distance <= obstacle_stop_distance_;
    status.obstacle_ok = latest_mcu_status_.obstacle_ok || distance_blocks;
    status.obstacle_distance = latest_mcu_status_.obstacle_distance;
    status.fault_bits = latest_mcu_status_.fault_bits;
  }
  if (status.e_stop_ok && status.e_stop_source != "car_calibrate" &&
    status.e_stop_source != "car_observation_collect")
  {
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
