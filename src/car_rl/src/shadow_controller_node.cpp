#include "car_rl/model_contract.hpp"
#include "car_rl/model_session.hpp"
#include "car_rl/observation.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace car_rl
{

class ShadowControllerNode : public rclcpp::Node
{
public:
  // 加载控制器模型并创建只读输入订阅和影子输出话题
  ShadowControllerNode()
  : Node("car_rl_shadow_controller"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 用户配置的模型 bundle 路径参数
    const std::string bundle_parameter = declare_parameter<std::string>("bundle_path", "");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    // 影子控制器订阅的雷达扫描话题
    const std::string scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    // 影子控制器订阅的全局路径话题
    const std::string plan_topic = declare_parameter<std::string>("plan_topic", "/plan");
    // 影子控制器订阅的正式里程计话题
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    scan_stale_timeout_ =
      declare_parameter<double>("scan_stale_timeout_sec", 0.5);
    max_linear_speed_ =
      declare_parameter<double>("max_linear_speed", 0.10);
    max_angular_speed_ =
      declare_parameter<double>("max_angular_speed", 1.047197551);
    if (scan_stale_timeout_ <= 0.0 ||
      max_linear_speed_ <= 0.0 || max_angular_speed_ <= 0.0)
    {
      throw std::runtime_error("影子控制器超时和速度参数必须大于0");
    }
    // 最终使用的模型 bundle 根目录
    const std::filesystem::path root = bundle_parameter.empty() ?
      default_bundle_path() : std::filesystem::path(bundle_parameter);
    bundle_ = load_model_bundle(root);
    session_ = std::make_unique<ModelSession>();
    session_->load(bundle_.controller);
    session_->warmup();

    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/car_rl/shadow_cmd_vel", 10);
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
        // 更新扫描共享状态时持有的数据锁
        std::lock_guard<std::mutex> lock(mutex_);
        scan_ = *message;
        scan_received_ = now();
        has_scan_ = true;
      });
    plan_subscription_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic, 10,
      [this](const nav_msgs::msg::Path::SharedPtr message) {
        // 更新路径共享状态时持有的数据锁
        std::lock_guard<std::mutex> lock(mutex_);
        plan_ = *message;
      });
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        // 更新速度共享状态时持有的数据锁
        std::lock_guard<std::mutex> lock(mutex_);
        velocity_ = message->twist.twist;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {run_once();});
    RCLCPP_INFO(get_logger(), "强化学习影子控制器已启动，不会发布底盘控制话题");
  }

private:
  // 在输入完整且雷达新鲜时执行一次不会进入底盘控制链的推理
  void run_once()
  {
    // 本轮影子推理使用的扫描副本
    sensor_msgs::msg::LaserScan scan;
    // 本轮影子推理使用的全局路径副本
    nav_msgs::msg::Path plan;
    // 本轮影子推理使用的正式里程计速度副本
    geometry_msgs::msg::Twist velocity;
    // 本轮扫描到达本机的 ROS 时间
    rclcpp::Time scan_received(0, 0, get_clock()->get_clock_type());
    {
      // 一次性复制全部输入共享状态时持有的数据锁
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_scan_ || plan_.poses.empty()) {
        return;
      }
      scan = scan_;
      plan = plan_;
      velocity = velocity_;
      scan_received = scan_received_;
    }
    const double scan_age = (now() - scan_received).seconds();
    if (scan_age < 0.0 || scan_age > scan_stale_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "影子控制等待新鲜雷达数据");
      return;
    }

    try {
      // base 坐标系在当前全局路径坐标系中的变换
      const auto transform = tf_buffer_.lookupTransform(
        plan.header.frame_id, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.1));
      // 从 TF 构造的当前机器人位姿
      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      // 严格按模型契约构造的控制器观测
      const auto observation = build_controller_observation(
        scan, plan, pose, velocity, previous_action_);
      // 推理后端写入的归一化动作
      ControllerAction action{};
      // 推理后端报告的执行耗时，与模型契约中的毫秒上限比较
      double elapsed = 0.0;
      // 推理失败时由后端填写的原因
      std::string error;
      if (!session_->run(
          observation.data(), observation.size(), action.data(), action.size(),
          elapsed, error) || elapsed > bundle_.controller.max_inference_ms)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "影子推理失败：%s", error.c_str());
        return;
      }
      previous_action_ = action;
      // 仅发布到影子诊断话题的候选速度，不进入底盘控制链
      geometry_msgs::msg::TwistStamped command;
      command.header.stamp = now();
      command.header.frame_id = base_frame_;
      command.twist = action_to_twist(
        action, max_linear_speed_, max_angular_speed_);
      publisher_->publish(command);
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "影子控制等待输入：%s", exception.what());
    }
  }

  // 保护扫描、路径和速度输入快照的互斥量
  std::mutex mutex_;
  // 最近一次雷达扫描
  sensor_msgs::msg::LaserScan scan_;
  // 最近一次全局路径
  nav_msgs::msg::Path plan_;
  // 最近一次正式里程计速度
  geometry_msgs::msg::Twist velocity_;
  // 是否已经收到至少一帧雷达扫描
  bool has_scan_{false};
  // 最近一帧雷达到达本机的 ROS 时间
  rclcpp::Time scan_received_{0, 0, RCL_ROS_TIME};
  // 上一轮影子动作，属于下一轮模型观测
  ControllerAction previous_action_{0.0F, 0.0F};
  // 当前机器人底盘坐标系
  std::string base_frame_;
  // 雷达扫描允许的最大本机接收年龄，单位秒
  double scan_stale_timeout_{0.5};
  // 影子候选命令最大线速度，单位米每秒
  double max_linear_speed_{0.10};
  // 影子候选命令最大角速度，单位弧度每秒
  double max_angular_speed_{1.047197551};
  // 已严格校验的控制器模型bundle
  ModelBundle bundle_;
  // 统一管理验证、加载和推理的控制器模型会话
  std::unique_ptr<ModelSession> session_;

  // 影子节点自己的 TF 缓存
  tf2_ros::Buffer tf_buffer_;
  // 向 TF 缓存填充全局变换的监听器
  tf2_ros::TransformListener tf_listener_;
  // 影子候选速度发布器，不连接 car_move
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  // 雷达扫描订阅器
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  // 全局路径订阅器
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
  // 正式里程计订阅器
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  // 固定周期执行影子推理的定时器
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace car_rl

int main(int argc, char ** argv)
{
  // 初始化 ROS 后运行影子节点，启动异常通过非零退出码交给进程管理器
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<car_rl::ShadowControllerNode>());
  } catch (const std::exception & exception) {
    fprintf(stderr, "强化学习影子控制器启动失败：%s\n", exception.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
