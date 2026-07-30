#ifndef CAR_RL__OBSERVATION_HPP_
#define CAR_RL__OBSERVATION_HPP_

#include <array>
#include <cstddef>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace car_rl
{

// 控制器观测固定使用的雷达束数量
constexpr std::size_t kScanRayCount = 72;
// controller contract v1 的输入元素数量
constexpr std::size_t kControllerObservationSize = 86;
// controller contract v1 的输出动作元素数量
constexpr std::size_t kControllerActionSize = 2;

// 固定 86 维控制器观测数组类型
using ControllerObservation = std::array<float, kControllerObservationSize>;
// 固定 2 维控制器动作数组类型
using ControllerAction = std::array<float, kControllerActionSize>;

// 把LaserScan按固定角度重采样为72束，并完成无效值替换和归一化
std::array<float, kScanRayCount> normalize_scan(
  const sensor_msgs::msg::LaserScan & scan,
  float range_min = 0.05F,
  float range_max = 6.0F);

// 按0.3/0.6/1.0m弧长采样路径，输出base坐标系中的三个x/y点
std::array<float, 6> sample_path_points(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose);

// 严格按contract v1顺序构造86维控制器输入
ControllerObservation build_controller_observation(
  const sensor_msgs::msg::LaserScan & scan,
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & velocity,
  const ControllerAction & previous_action);

// 把[-1,1]动作映射到当前Nav2速度范围
geometry_msgs::msg::Twist action_to_twist(
  const ControllerAction & action,
  double max_linear_speed,
  double max_angular_speed);

}  // namespace car_rl

#endif  // CAR_RL__OBSERVATION_HPP_
