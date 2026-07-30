#include "car_rl/observation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace car_rl
{
namespace
{

// 平面角度归一化使用的圆周率
constexpr double kPi = 3.14159265358979323846;
// 控制器沿全局路径采样的三个前视距离
constexpr std::array<double, 3> kLookaheadDistances{0.3, 0.6, 1.0};

double quaternion_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

double clamp(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(maximum, value));
}

double distance(double ax, double ay, double bx, double by)
{
  return std::hypot(bx - ax, by - ay);
}

std::array<double, 2> world_to_base(
  double world_x, double world_y,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // 机器人在世界坐标系中的平面朝向
  const double yaw = quaternion_yaw(robot_pose.pose.orientation);
  // 世界点相对机器人的 X 方向差值
  const double dx = world_x - robot_pose.pose.position.x;
  // 世界点相对机器人的 Y 方向差值
  const double dy = world_y - robot_pose.pose.position.y;
  // 机器人朝向的余弦值
  const double cosine = std::cos(yaw);
  // 机器人朝向的正弦值
  const double sine = std::sin(yaw);
  return {cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

}  // namespace

std::array<float, kScanRayCount> normalize_scan(
  const sensor_msgs::msg::LaserScan & scan,
  float range_min,
  float range_max)
{
  if (scan.ranges.empty() || scan.angle_increment == 0.0F || range_max <= range_min) {
    throw std::invalid_argument("LaserScan参数无效");
  }

  // 固定 72 束的归一化雷达结果
  std::array<float, kScanRayCount> result{};
  // 当前输出雷达束索引
  for (std::size_t ray = 0U; ray < result.size(); ++ray) {
    // 当前输出束在机器人坐标系中的目标角度
    const double target_angle = -kPi +
      (2.0 * kPi * static_cast<double>(ray) / static_cast<double>(result.size()));
    // 目标角度对应的原始 LaserScan 浮点索引
    const double source_index =
      (target_angle - static_cast<double>(scan.angle_min)) /
      static_cast<double>(scan.angle_increment);

    // 当前输出束的有效距离，默认使用最大量程代替无效值
    float range = range_max;
    if (source_index >= 0.0 &&
      source_index <= static_cast<double>(scan.ranges.size() - 1U))
    {
      // 浮点索引下方的原始扫描索引
      const std::size_t lower = static_cast<std::size_t>(std::floor(source_index));
      // 浮点索引上方的原始扫描索引
      const std::size_t upper = std::min(lower + 1U, scan.ranges.size() - 1U);
      // 上方扫描值参与线性插值的权重
      const double weight = source_index - static_cast<double>(lower);
      // 下方原始扫描距离
      const float lower_value = scan.ranges[lower];
      // 上方原始扫描距离
      const float upper_value = scan.ranges[upper];
      // 下方距离是否为可用有限值
      const bool lower_valid = std::isfinite(lower_value) && lower_value >= range_min;
      // 上方距离是否为可用有限值
      const bool upper_valid = std::isfinite(upper_value) && upper_value >= range_min;
      if (lower_valid && upper_valid) {
        range = static_cast<float>(
          static_cast<double>(lower_value) * (1.0 - weight) +
          static_cast<double>(upper_value) * weight);
      } else if (lower_valid) {
        range = lower_value;
      } else if (upper_valid) {
        range = upper_value;
      }
    }

    range = static_cast<float>(clamp(range, range_min, range_max));
    result[ray] = (range - range_min) / (range_max - range_min);
  }
  return result;
}

std::array<float, 6> sample_path_points(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (path.poses.empty()) {
    throw std::invalid_argument("全局路径为空");
  }

  // 先求机器人到整条折线的最近投影，而不是选最近离散路径点
  // 这样训练端和实车端不会因全局路径采样密度不同而产生观测跳变
  // 机器人最近投影所在的路径段索引
  std::size_t nearest_segment = 0U;
  // 最近投影在路径段内的比例
  double nearest_ratio = 0.0;
  // 机器人到当前最近路径投影的距离
  double nearest_distance = std::numeric_limits<double>::infinity();
  if (path.poses.size() > 1U) {
    // 当前检查的路径段起点索引
    for (std::size_t index = 0U; index + 1U < path.poses.size(); ++index) {
      // 当前路径段起点
      const auto & first = path.poses[index].pose.position;
      // 当前路径段终点
      const auto & second = path.poses[index + 1U].pose.position;
      // 当前路径段 X 方向增量
      const double dx = second.x - first.x;
      // 当前路径段 Y 方向增量
      const double dy = second.y - first.y;
      // 当前路径段长度平方
      const double length_squared = dx * dx + dy * dy;
      // 机器人正交投影在当前路径段内的比例
      const double ratio = length_squared <= 1.0e-12 ? 0.0 : clamp(
        ((robot_pose.pose.position.x - first.x) * dx +
        (robot_pose.pose.position.y - first.y) * dy) / length_squared,
        0.0, 1.0);
      // 当前路径段投影点的 X 坐标
      const double projection_x = first.x + ratio * dx;
      // 当前路径段投影点的 Y 坐标
      const double projection_y = first.y + ratio * dy;
      // 机器人到当前投影点的距离
      const double candidate = distance(
        robot_pose.pose.position.x, robot_pose.pose.position.y,
        projection_x, projection_y);
      if (candidate < nearest_distance) {
        nearest_distance = candidate;
        nearest_segment = index;
        nearest_ratio = ratio;
      }
    }
  }

  // 三个前视点在 base 坐标系中的归一化 X/Y 结果
  std::array<float, 6> sampled{};
  // 当前前视距离索引
  for (std::size_t target = 0U; target < kLookaheadDistances.size(); ++target) {
    // 沿路径还需前进的剩余弧长
    double remaining = kLookaheadDistances[target];
    // 当前前视采样点 X 坐标，默认退化为路径终点
    double sample_x = path.poses.back().pose.position.x;
    // 当前前视采样点 Y 坐标，默认退化为路径终点
    double sample_y = path.poses.back().pose.position.y;
    // 当前沿路径累计弧长的路径段索引
    for (std::size_t index = nearest_segment; index + 1U < path.poses.size(); ++index) {
      // 当前路径段起点
      const auto & first = path.poses[index].pose.position;
      // 当前路径段终点
      const auto & second = path.poses[index + 1U].pose.position;
      // 当前路径段长度
      const double segment_length = distance(first.x, first.y, second.x, second.y);
      if (segment_length <= 1.0e-9) {
        continue;
      }
      // 第一段从最近投影开始，后续段从段首开始
      // 当前路径段开始累计弧长的位置比例
      const double start_ratio = index == nearest_segment ? nearest_ratio : 0.0;
      // 当前路径段从开始比例到段尾的可用弧长
      const double available_length = segment_length * (1.0 - start_ratio);
      if (remaining <= available_length) {
        // 前视点在当前路径段内的比例
        const double ratio = start_ratio + remaining / segment_length;
        sample_x = first.x + ratio * (second.x - first.x);
        sample_y = first.y + ratio * (second.y - first.y);
        break;
      }
      remaining -= available_length;
    }

    // 当前前视点转换到机器人 base 坐标系后的 X/Y
    const auto relative = world_to_base(sample_x, sample_y, robot_pose);
    sampled[target * 2U] = static_cast<float>(clamp(relative[0], -1.5, 1.5) / 1.5);
    sampled[target * 2U + 1U] = static_cast<float>(clamp(relative[1], -1.5, 1.5) / 1.5);
  }
  return sampled;
}

ControllerObservation build_controller_observation(
  const sensor_msgs::msg::LaserScan & scan,
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & velocity,
  const ControllerAction & previous_action)
{
  // 观测前 72 维使用的归一化雷达束
  const auto scan_values = normalize_scan(scan);
  // 观测第 72 至 77 维使用的三个路径前视点
  const auto path_values = sample_path_points(path, robot_pose);
  // 当前全局路径终点位姿
  const auto & goal_pose = path.poses.back().pose;
  // 路径终点在机器人 base 坐标系中的位置
  const auto relative_goal = world_to_base(
    goal_pose.position.x, goal_pose.position.y, robot_pose);
  // 路径终点与机器人之间归一化后的朝向误差
  const double yaw_error = std::atan2(
    std::sin(
      quaternion_yaw(goal_pose.orientation) -
      quaternion_yaw(robot_pose.pose.orientation)),
    std::cos(
      quaternion_yaw(goal_pose.orientation) -
      quaternion_yaw(robot_pose.pose.orientation)));

  // 严格按 contract v1 排列的 86 维控制器观测
  ControllerObservation observation{};
  std::copy(scan_values.begin(), scan_values.end(), observation.begin());
  std::copy(path_values.begin(), path_values.end(), observation.begin() + 72);
  observation[78] = static_cast<float>(clamp(relative_goal[0], -3.0, 3.0) / 3.0);
  observation[79] = static_cast<float>(clamp(relative_goal[1], -3.0, 3.0) / 3.0);
  observation[80] = static_cast<float>(std::sin(yaw_error));
  observation[81] = static_cast<float>(std::cos(yaw_error));
  observation[82] = static_cast<float>(
    clamp(velocity.linear.x, -0.25, 0.25) / 0.25);
  observation[83] = static_cast<float>(clamp(velocity.angular.z, -1.0, 1.0));
  observation[84] = static_cast<float>(clamp(previous_action[0], -1.0, 1.0));
  observation[85] = static_cast<float>(clamp(previous_action[1], -1.0, 1.0));

  // 当前检查的观测元素
  for (const float value : observation) {
    if (!std::isfinite(value)) {
      throw std::runtime_error("控制器观测包含非有限值");
    }
  }
  return observation;
}

geometry_msgs::msg::Twist action_to_twist(
  const ControllerAction & action,
  double max_linear_speed,
  double max_angular_speed)
{
  // 映射到 Nav2 速度范围后的命令
  geometry_msgs::msg::Twist command;
  // 限制在模型动作范围内的线速度动作
  const double normalized_linear = clamp(action[0], -1.0, 1.0);
  // 限制在模型动作范围内的角速度动作
  const double normalized_angular = clamp(action[1], -1.0, 1.0);
  command.linear.x = 0.5 * max_linear_speed * (normalized_linear + 1.0);
  command.angular.z = max_angular_speed * normalized_angular;
  return command;
}

}  // namespace car_rl
