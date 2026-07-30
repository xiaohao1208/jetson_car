#include "car_rl/controller.hpp"

#include <nav2_core/exceptions.hpp>
#include <nav2_util/node_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace car_rl
{

// 配置插件参数、模型契约、已验证引擎和雷达输入
void Controller::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("强化学习控制器无法锁定生命周期节点");
  }
  logger_ = node_->get_logger();
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);

  // 当前插件实例的 Nav2 参数前缀
  const std::string prefix = plugin_name_ + ".";
  nav2_util::declare_parameter_if_not_declared(
    node_, prefix + "bundle_path", rclcpp::ParameterValue(std::string{}));
  nav2_util::declare_parameter_if_not_declared(
    node_, prefix + "scan_topic", rclcpp::ParameterValue(std::string("/scan")));
  nav2_util::declare_parameter_if_not_declared(
    node_, prefix + "scan_stale_timeout_sec", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, prefix + "input_ready_timeout_sec", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, prefix + "transform_tolerance_sec", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node_, prefix + "max_linear_speed", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node_, prefix + "max_angular_speed", rclcpp::ParameterValue(1.047197551));
  nav2_util::declare_parameter_if_not_declared(
    node_, prefix + "max_consecutive_failures", rclcpp::ParameterValue(2));

  // 用户配置的模型 bundle 路径，空值表示使用安装目录默认路径
  std::string bundle_path;
  node_->get_parameter(prefix + "bundle_path", bundle_path);
  node_->get_parameter(prefix + "scan_topic", scan_topic_);
  node_->get_parameter(prefix + "scan_stale_timeout_sec", scan_stale_timeout_);
  node_->get_parameter(prefix + "input_ready_timeout_sec", input_ready_timeout_);
  node_->get_parameter(prefix + "transform_tolerance_sec", transform_tolerance_);
  node_->get_parameter(prefix + "max_linear_speed", max_linear_speed_);
  node_->get_parameter(prefix + "max_angular_speed", max_angular_speed_);
  // 从 ROS 参数读取的连续失败阈值临时值
  int max_failures = 2;
  node_->get_parameter(prefix + "max_consecutive_failures", max_failures);
  max_consecutive_failures_ = static_cast<unsigned int>(std::max(1, max_failures));
  if (scan_topic_.empty()) {
    throw std::runtime_error("强化学习控制器雷达话题不能为空");
  }
  if (scan_stale_timeout_ <= 0.0 || input_ready_timeout_ <= 0.0 ||
    transform_tolerance_ < 0.0 || max_linear_speed_ <= 0.0 ||
    max_angular_speed_ <= 0.0)
  {
    throw std::runtime_error("强化学习控制器超时和速度参数必须为有效正数");
  }

  // 最终使用的模型 bundle 根目录
  const std::filesystem::path root = bundle_path.empty() ?
    default_bundle_path() : std::filesystem::path(bundle_path);
  bundle_ = load_model_bundle(root);
  session_ = std::make_unique<ModelSession>();
  session_->load(bundle_.controller);
  scan_subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&Controller::on_scan, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_, "强化学习控制器已配置，模型版本=%s，推理后端=%s",
    bundle_.controller.model_version.c_str(), session_->backend_name().c_str());
}

// 释放插件持有的全部运行资源并清空共享输入
void Controller::cleanup()
{
  {
    // 先在数据锁内停止控制并清空输入，避免清理期间读取半释放状态
    std::lock_guard<std::mutex> lock(data_mutex_);
    active_ = false;
    has_scan_ = false;
    global_plan_.poses.clear();
    previous_action_ = {0.0F, 0.0F};
    consecutive_failures_ = 0U;
  }
  scan_subscription_.reset();
  session_.reset();
  node_.reset();
  tf_.reset();
}

// 预热后端并开始新的控制运行周期
void Controller::activate()
{
  if (!session_ || !session_->available()) {
    throw std::runtime_error("强化学习控制器推理后端不可用");
  }
  session_->warmup();
  // 激活状态与观测历史需要作为一个共享状态整体更新
  std::lock_guard<std::mutex> lock(data_mutex_);
  active_ = true;
  activated_at_ = node_->now();
  consecutive_failures_ = 0U;
  previous_action_ = {0.0F, 0.0F};
}

// 停止控制器并重置运行期状态
void Controller::deactivate()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  active_ = false;
  consecutive_failures_ = 0U;
  previous_action_ = {0.0F, 0.0F};
}

// 保存新的全局路径并开始一段独立动作历史
void Controller::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("强化学习控制器收到空路径");
  }
  // 更新全局路径共享状态时持有的数据锁
  std::lock_guard<std::mutex> lock(data_mutex_);
  global_plan_ = path;
  previous_action_ = {0.0F, 0.0F};
  consecutive_failures_ = 0U;
}

// 执行一个 Nav2 控制周期
geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker *)
{
  // 本轮控制使用的雷达扫描副本
  sensor_msgs::msg::LaserScan scan;
  // 本轮控制使用的全局路径副本
  nav_msgs::msg::Path plan;
  // 本轮扫描实际到达控制器的 ROS 时间
  rclcpp::Time scan_received(0, 0, node_->get_clock()->get_clock_type());
  // 本轮观测使用的上一动作副本
  ControllerAction previous_action{};
  // 本轮读取的控制器激活状态
  bool active = false;
  // 首帧输入等待开始时间
  rclcpp::Time activated_at(0, 0, node_->get_clock()->get_clock_type());
  {
    // 复制扫描和路径共享状态时持有的数据锁
    std::lock_guard<std::mutex> lock(data_mutex_);
    active = active_;
    activated_at = activated_at_;
    if (has_scan_) {
      scan = latest_scan_;
      scan_received = latest_scan_received_;
    }
    plan = global_plan_;
    previous_action = previous_action_;
  }
  if (!active) {
    return zero_command(pose, "强化学习控制器尚未激活");
  }
  if (scan.ranges.empty() || plan.poses.empty()) {
    // 初始输入短暂未就绪时停车等待，不消耗运行期失败次数
    const double waiting_age = (node_->now() - activated_at).seconds();
    return zero_command(
      pose, "等待雷达或全局路径", waiting_age > input_ready_timeout_);
  }

  // 同时检查本机接收时间和消息时间，任一过期都拒绝用于控制
  const rclcpp::Time now = node_->now();
  const double receive_age = (now - scan_received).seconds();
  double message_age = receive_age;
  if (scan.header.stamp.sec != 0 || scan.header.stamp.nanosec != 0U) {
    message_age = (
      now - rclcpp::Time(scan.header.stamp, now.get_clock_type())).seconds();
  }
  const double scan_age = std::max(receive_age, message_age);
  if (scan_age < 0.0 || scan_age > scan_stale_timeout_) {
    return zero_command(pose, "雷达数据过期");
  }

  try {
    plan = path_in_frame(plan, pose.header.frame_id);
    // 严格按模型契约构造的控制器观测
    const ControllerObservation observation = build_controller_observation(
      scan, plan, pose, velocity, previous_action);
    // 推理后端写入的归一化线速度和角速度动作
    ControllerAction action{};
    // 推理后端报告的执行耗时，与模型契约中的毫秒上限比较
    double elapsed = 0.0;
    // 推理失败时由后端填写的原因
    std::string error;
    if (!session_->run(
        observation.data(), observation.size(), action.data(), action.size(),
        elapsed, error))
    {
      return zero_command(pose, "推理失败: " + error);
    }
    if (elapsed > bundle_.controller.max_inference_ms) {
      return zero_command(pose, "推理超时");
    }
    if (!std::isfinite(action[0]) || !std::isfinite(action[1])) {
      return zero_command(pose, "推理输出包含NaN/Inf");
    }

    {
      // 推理成功后原子更新下一轮观测和失败计数
      std::lock_guard<std::mutex> lock(data_mutex_);
      previous_action_ = action;
      consecutive_failures_ = 0U;
    }
    // 本轮使用的动态限速比例副本
    double speed_limit_factor = 1.0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      speed_limit_factor = speed_limit_factor_;
    }
    // 应用当前 Nav2 限速后的候选速度命令
    const auto command = action_to_twist(
      action,
      max_linear_speed_ * speed_limit_factor,
      max_angular_speed_ * speed_limit_factor);
    // 返回给 controller_server 的带时间戳速度命令
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = node_->now();
    stamped.header.frame_id = pose.header.frame_id;
    stamped.twist = command;
    return stamped;
  } catch (const nav2_core::PlannerException &) {
    throw;
  } catch (const std::exception & exception) {
    return zero_command(pose, exception.what());
  }
}

// 更新 Nav2 动态限速比例，不改变配置中的物理速度上限
void Controller::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (percentage) {
    speed_limit_factor_ = std::clamp(speed_limit / 100.0, 0.0, 1.0);
  } else if (speed_limit <= 0.0) {
    speed_limit_factor_ = 1.0;
  } else {
    speed_limit_factor_ = std::clamp(speed_limit / max_linear_speed_, 0.0, 1.0);
  }
}

// 接收雷达扫描并记录本机到达时间
void Controller::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  // 更新最近扫描共享状态时持有的数据锁
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_scan_ = *message;
  latest_scan_received_ = node_->now();
  has_scan_ = true;
}

// 将路径转换到当前机器人位姿使用的坐标系
nav_msgs::msg::Path Controller::path_in_frame(
  const nav_msgs::msg::Path & path,
  const std::string & target_frame) const
{
  if (path.header.frame_id.empty() || path.header.frame_id == target_frame) {
    return path;
  }

  // 转换到目标坐标系后的路径
  nav_msgs::msg::Path transformed;
  transformed.header = path.header;
  transformed.header.frame_id = target_frame;
  transformed.poses.reserve(path.poses.size());
  // 当前需要转换的路径位姿
  for (const auto & path_pose : path.poses) {
    transformed.poses.push_back(
      tf_->transform(
        path_pose, target_frame, tf2::durationFromSec(transform_tolerance_)));
  }
  return transformed;
}

// 构造零速度命令并在运行故障累计达到阈值时终止本次 Nav2 控制
geometry_msgs::msg::TwistStamped Controller::zero_command(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & reason,
  bool count_failure)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    previous_action_ = {0.0F, 0.0F};
    if (count_failure) {
      ++consecutive_failures_;
      if (consecutive_failures_ >= max_consecutive_failures_) {
        throw nav2_core::PlannerException("强化学习控制器停止：" + reason);
      }
    }
  }

  RCLCPP_WARN(logger_, "强化学习控制器输出零速度：%s", reason.c_str());
  // 返回给 controller_server 的零速度命令
  geometry_msgs::msg::TwistStamped command;
  if (node_) {
    command.header.stamp = node_->now();
  } else {
    command.header.stamp = pose.header.stamp;
  }
  command.header.frame_id = pose.header.frame_id;
  return command;
}

}  // namespace car_rl
