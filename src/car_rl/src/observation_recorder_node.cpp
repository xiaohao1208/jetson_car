#include "car_rl/model_contract.hpp"
#include "car_rl/observation.hpp"
#include "car_rl/observation_recording.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace car_rl
{

class ObservationRecorderNode : public rclcpp::Node
{
public:
  ObservationRecorderNode()
  : Node("car_rl_observation_recorder"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    const std::string output_prefix =
      declare_parameter<std::string>("output_prefix", "");
    target_samples_ = declare_parameter<int>("target_samples", 5000);
    sample_hz_ = declare_parameter<double>("sample_hz", 15.0);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.5);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    plan_topic_ = declare_parameter<std::string>("plan_topic", "/plan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    command_topic_ = declare_parameter<std::string>("command_topic", "/cmd_vel_nav");
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.10);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 1.047197551);
    if (output_prefix.empty() || target_samples_ <= 0 || sample_hz_ <= 0.0 ||
      stale_timeout_sec_ <= 0.0 || base_frame_.empty() ||
      max_linear_speed_ <= 0.0 || max_angular_speed_ <= 0.0)
    {
      throw std::runtime_error("观测记录器路径、数量、频率、超时和速度参数无效");
    }

    const auto prefix = std::filesystem::absolute(output_prefix);
    csv_path_ = prefix.string() + ".csv";
    metadata_path_ = prefix.string() + ".metadata.yaml";
    pending_csv_path_ = csv_path_.string() + ".pending";
    pending_metadata_path_ = metadata_path_.string() + ".pending";
    if (std::filesystem::exists(csv_path_) || std::filesystem::exists(metadata_path_) ||
      std::filesystem::exists(pending_csv_path_) ||
      std::filesystem::exists(pending_metadata_path_))
    {
      throw std::runtime_error("观测输出或pending文件已存在，拒绝覆盖");
    }
    std::filesystem::create_directories(csv_path_.parent_path());
    csv_stream_.open(pending_csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_stream_) {
      throw std::runtime_error("无法创建观测CSV pending文件");
    }
    write_observation_csv_header(csv_stream_);

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        scan_ = *message;
        scan_received_ = now();
        has_scan_ = true;
      });
    plan_subscription_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, 10,
      [this](const nav_msgs::msg::Path::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        plan_ = *message;
      });
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_ = *message;
        velocity_ = message->twist.twist;
        odom_received_ = now();
        has_odom_ = true;
      });
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_command_action_[0] = static_cast<float>(std::clamp(
          2.0 * message->linear.x / max_linear_speed_ - 1.0, -1.0, 1.0));
        latest_command_action_[1] = static_cast<float>(std::clamp(
          message->angular.z / max_angular_speed_, -1.0, 1.0));
        command_received_ = now();
        has_command_ = true;
      });
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / sample_hz_));
    timer_ = create_wall_timer(period, [this]() {record_once();});
    RCLCPP_INFO(
      get_logger(), "开始采集实车控制器观测，目标=%d，输出=%s",
      target_samples_, csv_path_.c_str());
  }

  ~ObservationRecorderNode() override
  {
    if (csv_stream_.is_open()) {
      csv_stream_.close();
    }
    if (!completed_) {
      std::error_code ignored;
      std::filesystem::remove(pending_csv_path_, ignored);
      std::filesystem::remove(pending_metadata_path_, ignored);
    }
  }

private:
  void record_once()
  {
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Path plan;
    geometry_msgs::msg::Twist velocity;
    ControllerAction previous_action{};
    ControllerAction latest_command_action{};
    rclcpp::Time scan_received(0, 0, get_clock()->get_clock_type());
    rclcpp::Time scan_stamp(0, 0, get_clock()->get_clock_type());
    rclcpp::Time odom_received(0, 0, get_clock()->get_clock_type());
    rclcpp::Time odom_stamp(0, 0, get_clock()->get_clock_type());
    rclcpp::Time command_received(0, 0, get_clock()->get_clock_type());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_scan_ || !has_odom_ || !has_command_ || plan_.poses.empty()) {
        return;
      }
      scan = scan_;
      plan = plan_;
      velocity = velocity_;
      previous_action = previous_action_;
      latest_command_action = latest_command_action_;
      scan_received = scan_received_;
      scan_stamp = rclcpp::Time(scan.header.stamp, get_clock()->get_clock_type());
      odom_received = odom_received_;
      odom_stamp = rclcpp::Time(odom_.header.stamp, get_clock()->get_clock_type());
      command_received = command_received_;
    }
    const auto current = now();
    const auto fresh = [this, &current](const rclcpp::Time & stamp) {
        const double age = (current - stamp).seconds();
        return age >= 0.0 && age <= stale_timeout_sec_;
      };
    if (!fresh(scan_received) || !fresh(scan_stamp) || !fresh(odom_received) ||
      !fresh(odom_stamp) || !fresh(command_received))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "等待新鲜scan/odom/cmd_vel_nav数据");
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        plan.header.frame_id, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.1));
      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      const auto observation = build_controller_observation(
        scan, plan, pose, velocity, previous_action);
      write_observation_csv_row(csv_stream_, observation);
      if (!csv_stream_) {
        throw std::runtime_error("写入观测CSV失败");
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        previous_action_ = latest_command_action;
      }
      ++sample_count_;
      if (sample_count_ % 250 == 0) {
        RCLCPP_INFO(get_logger(), "实车观测采集进度=%d/%d", sample_count_, target_samples_);
      }
      if (sample_count_ >= target_samples_) {
        finish();
      }
    } catch (const std::exception & exception) {
      if (sample_count_ >= target_samples_) {
        RCLCPP_FATAL(get_logger(), "发布实车观测失败：%s", exception.what());
        rclcpp::shutdown();
        return;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "观测采集等待有效输入：%s", exception.what());
    }
  }

  void finish()
  {
    timer_->cancel();
    csv_stream_.flush();
    csv_stream_.close();
    if (!csv_stream_) {
      throw std::runtime_error("关闭观测CSV前刷新失败");
    }
    const std::string csv_digest = sha256_file(pending_csv_path_);
    YAML::Emitter yaml;
    yaml << YAML::BeginMap
         << YAML::Key << "schema_version" << YAML::Value << 1
         << YAML::Key << "source" << YAML::Value << "real_car_raw"
         << YAML::Key << "completed" << YAML::Value << true
         << YAML::Key << "observation_contract_version" << YAML::Value << 1
         << YAML::Key << "observation_size" << YAML::Value << 86
         << YAML::Key << "count" << YAML::Value << sample_count_
         << YAML::Key << "sample_hz" << YAML::Value << sample_hz_
         << YAML::Key << "csv_sha256" << YAML::Value << csv_digest
         << YAML::Key << "scan_topic" << YAML::Value << scan_topic_
         << YAML::Key << "plan_topic" << YAML::Value << plan_topic_
         << YAML::Key << "odom_topic" << YAML::Value << odom_topic_
         << YAML::Key << "command_topic" << YAML::Value << command_topic_
         << YAML::EndMap;
    std::ofstream metadata(pending_metadata_path_, std::ios::out | std::ios::trunc);
    metadata << yaml.c_str() << '\n';
    metadata.close();
    if (!metadata) {
      throw std::runtime_error("写入观测元数据失败");
    }
    try {
      std::filesystem::rename(pending_csv_path_, csv_path_);
      std::filesystem::rename(pending_metadata_path_, metadata_path_);
    } catch (...) {
      std::error_code ignored;
      if (!std::filesystem::exists(metadata_path_)) {
        std::filesystem::remove(csv_path_, ignored);
      }
      throw;
    }
    completed_ = true;
    RCLCPP_INFO(
      get_logger(), "实车观测采集完成，数量=%d，CSV=%s，元数据=%s",
      sample_count_, csv_path_.c_str(), metadata_path_.c_str());
    rclcpp::shutdown();
  }

  std::mutex mutex_;
  sensor_msgs::msg::LaserScan scan_;
  nav_msgs::msg::Path plan_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::Twist velocity_;
  ControllerAction previous_action_{0.0F, 0.0F};
  ControllerAction latest_command_action_{0.0F, 0.0F};
  bool has_scan_{false};
  bool has_odom_{false};
  bool has_command_{false};
  bool completed_{false};
  rclcpp::Time scan_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time command_received_{0, 0, RCL_ROS_TIME};
  int target_samples_{5000};
  int sample_count_{0};
  double sample_hz_{15.0};
  double stale_timeout_sec_{0.5};
  double max_linear_speed_{0.10};
  double max_angular_speed_{1.047197551};
  std::string base_frame_;
  std::string scan_topic_;
  std::string plan_topic_;
  std::string odom_topic_;
  std::string command_topic_;
  std::filesystem::path csv_path_;
  std::filesystem::path metadata_path_;
  std::filesystem::path pending_csv_path_;
  std::filesystem::path pending_metadata_path_;
  std::ofstream csv_stream_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace car_rl

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<car_rl::ObservationRecorderNode>());
  } catch (const std::exception & exception) {
    fprintf(stderr, "实车观测记录器失败：%s\n", exception.what());
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
