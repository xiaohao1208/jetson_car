#include <gtest/gtest.h>

#include <cmath>

#include "car_rl/observation.hpp"

namespace
{

geometry_msgs::msg::Quaternion yaw_quaternion(double yaw)
{
  // 仅包含平面偏航角的测试四元数
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

}  // namespace

TEST(Observation, ResamplesFrontRayAtIndex36)
{
  // 包含 360 束且正前方有近障的测试扫描
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -static_cast<float>(M_PI);
  scan.angle_increment = static_cast<float>(2.0 * M_PI / 360.0);
  scan.ranges.assign(360U, 6.0F);
  scan.ranges[180] = 0.05F;

  // 重采样为固定 72 束后的归一化结果
  const auto normalized = car_rl::normalize_scan(scan);

  EXPECT_NEAR(normalized[36], 0.0F, 1.0e-4F);
  EXPECT_NEAR(normalized[0], 1.0F, 1.0e-4F);
}

TEST(Observation, BuildsStableEightySixElementContract)
{
  // 所有束距离一致的控制器测试扫描
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -static_cast<float>(M_PI);
  scan.angle_increment = static_cast<float>(2.0 * M_PI / 360.0);
  scan.ranges.assign(360U, 2.0F);

  // 沿地图 X 轴延伸的测试全局路径
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  // 构造测试路径时使用的位姿序号
  for (int index = 0; index <= 10; ++index) {
    // 当前路径采样点
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 0.1 * index;
    pose.pose.orientation = yaw_quaternion(0.0);
    path.poses.push_back(pose);
  }
  // 位于路径起点且朝向 X 轴的机器人位姿
  geometry_msgs::msg::PoseStamped robot;
  robot.header.frame_id = "map";
  robot.pose.orientation = yaw_quaternion(0.0);
  // 默认零值的当前机器人速度
  geometry_msgs::msg::Twist velocity;
  // 默认零值的上一控制周期动作
  const car_rl::ControllerAction previous{0.0F, 0.0F};

  // 按固定顺序拼接后的 86 维控制器观测
  const auto observation = car_rl::build_controller_observation(
    scan, path, robot, velocity, previous);

  EXPECT_EQ(observation.size(), 86U);
  EXPECT_NEAR(observation[72], 0.2F, 1.0e-4F);
  EXPECT_NEAR(observation[74], 0.4F, 1.0e-4F);
  EXPECT_NEAR(observation[76], 2.0F / 3.0F, 1.0e-4F);
  EXPECT_NEAR(observation[81], 1.0F, 1.0e-4F);
}

TEST(Observation, ActionMappingDoesNotAllowReverse)
{
  // 网络最小线速度动作映射出的停车命令
  const auto stopped = car_rl::action_to_twist({-1.0F, 0.0F}, 0.25, 1.0);
  // 网络最大动作映射出的前进和转向命令
  const auto forward = car_rl::action_to_twist({1.0F, 1.0F}, 0.25, 1.0);

  EXPECT_DOUBLE_EQ(stopped.linear.x, 0.0);
  EXPECT_NEAR(forward.linear.x, 0.25, 1.0e-9);
  EXPECT_NEAR(forward.angular.z, 1.0, 1.0e-9);
}

TEST(Observation, SamplesFromNearestSegmentProjection)
{
  // 沿 X 轴延伸、供弧长采样的测试路径
  nav_msgs::msg::Path path;
  // 构造测试路径时使用的位姿序号
  for (int index = 0; index <= 10; ++index) {
    // 当前路径采样点
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.1 * index;
    pose.pose.orientation = yaw_quaternion(0.0);
    path.poses.push_back(pose);
  }
  // 位于路径侧方的机器人测试位姿
  geometry_msgs::msg::PoseStamped robot;
  robot.pose.position.x = 0.45;
  robot.pose.position.y = 0.20;
  robot.pose.orientation = yaw_quaternion(0.0);

  // 从最近路径投影点向前采样得到的局部路径点
  const auto sampled = car_rl::sample_path_points(path, robot);

  // 最近投影是(0.45, 0)，向前0.3m后为(0.75, 0)
  EXPECT_NEAR(sampled[0], 0.30 / 1.5, 1.0e-5);
  EXPECT_NEAR(sampled[1], -0.20 / 1.5, 1.0e-5);
}
