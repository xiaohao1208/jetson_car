// Copyright 2026 ROS 2 Car Contributors

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "car_mapping/scan_geometry_normalizer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace car_mapping
{

struct GeometrySample
{
  std::size_t point_count;
  float angle_min;
  float angle_max;
};

class ScanWarmupRelay : public rclcpp::Node
{
public:
  ScanWarmupRelay()
  : Node("scan_warmup_relay"),
    started_at_(std::chrono::steady_clock::now())
  {
    declare_parameter<std::string>("input_topic", "/scan");
    declare_parameter<std::string>("output_topic", "/mapping_scan");
    declare_parameter<double>("warmup_sec", 1.0);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    warmup_sec_ = std::max(0.0, get_parameter("warmup_sec").as_double());

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanWarmupRelay::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "建图扫描暖机 %.1fs，正在统计稳定角度网格",
      warmup_sec_);
  }

private:
  void collect_geometry(const sensor_msgs::msg::LaserScan & message)
  {
    geometry_samples_.push_back(
      {message.ranges.size(), message.angle_min, message.angle_max});
  }

  void lock_geometry(const sensor_msgs::msg::LaserScan & fallback)
  {
    if (geometry_samples_.empty()) {
      collect_geometry(fallback);
    }
    std::map<std::size_t, std::size_t> counts;
    for (const auto & sample : geometry_samples_) {
      ++counts[sample.point_count];
    }
    const auto selected = std::max_element(
      counts.begin(), counts.end(),
      [](const auto & left, const auto & right) {
        if (left.second != right.second) {
          return left.second < right.second;
        }
        return left.first < right.first;
      });
    const std::size_t point_count = selected->first;
    double angle_min_sum = 0.0;
    double angle_max_sum = 0.0;
    std::size_t matching = 0U;
    for (const auto & sample : geometry_samples_) {
      if (sample.point_count != point_count) {
        continue;
      }
      angle_min_sum += sample.angle_min;
      angle_max_sum += sample.angle_max;
      ++matching;
    }
    normalizer_.lock(
      static_cast<float>(angle_min_sum / static_cast<double>(matching)),
      static_cast<float>(angle_max_sum / static_cast<double>(matching)),
      point_count);
    geometry_samples_.clear();
    RCLCPP_INFO(
      get_logger(),
      "建图扫描暖机完成，固定输出点数=%zu，开始转发 /mapping_scan",
      point_count);
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    if (!ScanGeometryNormalizer::valid(*message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "丢弃无效建图扫描，点数=%zu，角度增量=%.9f",
        message->ranges.size(), message->angle_increment);
      return;
    }

    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started_at_).count();
    if (!normalizer_.locked()) {
      collect_geometry(*message);
      if (elapsed < warmup_sec_) {
        return;
      }
      lock_geometry(*message);
    }

    const auto normalized = normalizer_.normalize(*message);
    if (
      message->ranges.size() != normalized.ranges.size() ||
      ScanGeometryNormalizer::expected_point_count(*message) !=
      message->ranges.size())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "已归一化雷达扫描，输入点数=%zu，输出点数=%zu",
        message->ranges.size(), normalized.ranges.size());
    }
    publisher_->publish(normalized);
  }

  std::string input_topic_;
  std::string output_topic_;
  double warmup_sec_;
  std::chrono::steady_clock::time_point started_at_;
  std::vector<GeometrySample> geometry_samples_;
  ScanGeometryNormalizer normalizer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

}  // namespace car_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<car_mapping::ScanWarmupRelay>());
  rclcpp::shutdown();
  return 0;
}
