#include "car_rl/observation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{

// 黄金向量文件格式版本
constexpr int kGoldenVectorSchemaVersion = 1;
// 模型交付包格式版本
constexpr int kBundleSchemaVersion = 2;
// 观测排列契约版本
constexpr int kObservationContractVersion = 1;
// 导出用例数量
constexpr std::size_t kCaseCount = 100U;
// 导出动作映射使用的最大线速度
constexpr double kMaxLinearSpeed = 0.10;
// 导出动作映射使用的最大角速度
constexpr double kMaxAngularSpeed = 1.047197551;
// 平面角度计算使用的圆周率
constexpr double kPi = 3.14159265358979323846;

void write_number(std::ostream & output, double value)
{
  if (!std::isfinite(value)) {
    throw std::runtime_error("黄金向量包含非有限数值");
  }
  output << std::setprecision(10) << value;
}

template<typename ContainerT>
void write_array(std::ostream & output, const ContainerT & values)
{
  output << '[';
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index > 0U) {
      output << ',';
    }
    write_number(output, static_cast<double>(values[index]));
  }
  output << ']';
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion result;
  result.x = quaternion.x();
  result.y = quaternion.y();
  result.z = quaternion.z();
  result.w = quaternion.w();
  return result;
}

sensor_msgs::msg::LaserScan make_scan(std::size_t case_index)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = static_cast<float>(-kPi);
  scan.angle_increment = static_cast<float>(2.0 * kPi / 359.0);
  scan.angle_max = scan.angle_min + 359.0F * scan.angle_increment;
  scan.range_min = 0.05F;
  scan.range_max = 6.0F;
  scan.ranges.resize(360U);

  for (std::size_t index = 0U; index < scan.ranges.size(); ++index) {
    const double wave =
      2.4 +
      1.2 * std::sin(0.031 * static_cast<double>(index + case_index)) +
      0.6 * std::cos(0.017 * static_cast<double>(index * 3U + case_index));
    scan.ranges[index] = static_cast<float>(std::max(0.05, std::min(6.0, wave)));
  }
  return scan;
}

geometry_msgs::msg::PoseStamped make_robot_pose(std::size_t case_index)
{
  geometry_msgs::msg::PoseStamped pose;
  const double phase = 0.11 * static_cast<double>(case_index);
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.8 * std::cos(phase);
  pose.pose.position.y = 0.6 * std::sin(phase);
  pose.pose.orientation = quaternion_from_yaw(
    -0.8 + 1.6 * static_cast<double>(case_index) /
    static_cast<double>(kCaseCount - 1U));
  return pose;
}

nav_msgs::msg::Path make_path(
  std::size_t case_index,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const double curve = 0.08 * std::sin(0.19 * static_cast<double>(case_index));
  const double robot_yaw = std::atan2(
    2.0 * robot_pose.pose.orientation.w * robot_pose.pose.orientation.z,
    1.0 - 2.0 * robot_pose.pose.orientation.z * robot_pose.pose.orientation.z);

  for (std::size_t index = 0U; index < 12U; ++index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    const double distance = 0.12 * static_cast<double>(index);
    const double local_x = distance;
    const double local_y = curve * static_cast<double>(index * index) / 11.0;
    pose.pose.position.x =
      robot_pose.pose.position.x +
      std::cos(robot_yaw) * local_x -
      std::sin(robot_yaw) * local_y;
    pose.pose.position.y =
      robot_pose.pose.position.y +
      std::sin(robot_yaw) * local_x +
      std::cos(robot_yaw) * local_y;
    pose.pose.orientation = quaternion_from_yaw(robot_yaw + std::atan2(curve, 0.12));
    path.poses.push_back(pose);
  }
  return path;
}

geometry_msgs::msg::Twist make_velocity(std::size_t case_index)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x =
    0.20 * static_cast<double>(case_index % 11U) / 10.0 - 0.04;
  velocity.angular.z =
    1.6 * static_cast<double>(case_index % 13U) / 12.0 - 0.8;
  return velocity;
}

car_rl::ControllerAction make_previous_action(std::size_t case_index)
{
  return {
    static_cast<float>(
      -1.0 + 2.0 * static_cast<double>(case_index % 17U) / 16.0),
    static_cast<float>(
      1.0 - 2.0 * static_cast<double>(case_index % 19U) / 18.0)};
}

car_rl::ControllerAction make_test_action(std::size_t case_index)
{
  return {
    static_cast<float>(
      -1.2 + 2.4 * static_cast<double>(case_index % 23U) / 22.0),
    static_cast<float>(
      1.2 - 2.4 * static_cast<double>(case_index % 29U) / 28.0)};
}

void write_pose(
  std::ostream & output,
  const geometry_msgs::msg::PoseStamped & pose)
{
  output << "{\"x\":";
  write_number(output, pose.pose.position.x);
  output << ",\"y\":";
  write_number(output, pose.pose.position.y);
  output << ",\"qx\":";
  write_number(output, pose.pose.orientation.x);
  output << ",\"qy\":";
  write_number(output, pose.pose.orientation.y);
  output << ",\"qz\":";
  write_number(output, pose.pose.orientation.z);
  output << ",\"qw\":";
  write_number(output, pose.pose.orientation.w);
  output << '}';
}

void write_path(std::ostream & output, const nav_msgs::msg::Path & path)
{
  output << '[';
  for (std::size_t index = 0U; index < path.poses.size(); ++index) {
    if (index > 0U) {
      output << ',';
    }
    write_pose(output, path.poses[index]);
  }
  output << ']';
}

void export_vectors(const std::filesystem::path & output_path)
{
  const auto parent = output_path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }

  std::ofstream output(output_path);
  if (!output.is_open()) {
    throw std::runtime_error("无法创建黄金向量文件");
  }

  output << "{\n";
  output << "  \"schema_version\":" << kGoldenVectorSchemaVersion << ",\n";
  output << "  \"bundle_schema_version\":" << kBundleSchemaVersion << ",\n";
  output << "  \"observation_contract_version\":" << kObservationContractVersion << ",\n";
  output << "  \"observation_size\":" << car_rl::kControllerObservationSize << ",\n";
  output << "  \"action_size\":" << car_rl::kControllerActionSize << ",\n";
  output << "  \"max_linear_speed\":";
  write_number(output, kMaxLinearSpeed);
  output << ",\n  \"max_angular_speed\":";
  write_number(output, kMaxAngularSpeed);
  output << ",\n  \"cases\":[\n";

  for (std::size_t case_index = 0U; case_index < kCaseCount; ++case_index) {
    const auto scan = make_scan(case_index);
    const auto robot_pose = make_robot_pose(case_index);
    const auto path = make_path(case_index, robot_pose);
    const auto velocity = make_velocity(case_index);
    const auto previous_action = make_previous_action(case_index);
    const auto test_action = make_test_action(case_index);
    const auto observation = car_rl::build_controller_observation(
      scan, path, robot_pose, velocity, previous_action);
    const auto command = car_rl::action_to_twist(
      test_action, kMaxLinearSpeed, kMaxAngularSpeed);

    if (case_index > 0U) {
      output << ",\n";
    }
    output << "    {\"id\":" << case_index;
    output << ",\"scan\":{\"angle_min\":";
    write_number(output, scan.angle_min);
    output << ",\"angle_increment\":";
    write_number(output, scan.angle_increment);
    output << ",\"range_min\":";
    write_number(output, scan.range_min);
    output << ",\"range_max\":";
    write_number(output, scan.range_max);
    output << ",\"ranges\":";
    write_array(output, scan.ranges);
    output << "},\"path\":";
    write_path(output, path);
    output << ",\"robot_pose\":";
    write_pose(output, robot_pose);
    output << ",\"velocity\":{\"linear_x\":";
    write_number(output, velocity.linear.x);
    output << ",\"angular_z\":";
    write_number(output, velocity.angular.z);
    output << "},\"previous_action\":";
    write_array(output, previous_action);
    output << ",\"test_action\":";
    write_array(output, test_action);
    output << ",\"expected_observation\":";
    write_array(output, observation);
    output << ",\"expected_twist\":{\"linear_x\":";
    write_number(output, command.linear.x);
    output << ",\"angular_z\":";
    write_number(output, command.angular.z);
    output << "}}";
  }
  output << "\n  ]\n}\n";

  if (!output.good()) {
    throw std::runtime_error("写入黄金向量文件失败");
  }
}

void print_usage()
{
  std::cout << "用法：contract_tool export --output <文件路径>\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc != 4 || std::string(argv[1]) != "export" ||
      std::string(argv[2]) != "--output")
    {
      print_usage();
      return 2;
    }

    const std::filesystem::path output_path(argv[3]);
    export_vectors(output_path);
    std::cout << "契约黄金向量已导出，文件=" << output_path.string() << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "契约黄金向量导出失败，原因=" << error.what() << '\n';
    return 1;
  }
}
