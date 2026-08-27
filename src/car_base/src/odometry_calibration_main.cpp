#include "car_base/odometry_calibration.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace
{

void print_usage()
{
  std::cout <<
    "用法: ros2 run car_base odometry_calibration "
    "--actual-distance 米 --odom-distance 米 "
    "--actual-yaw-rad 弧度 --odom-yaw-rad 弧度 "
    "[--wheel-distance 米] [--left-wheel-per-tick 米] "
    "[--right-wheel-per-tick 米]\n";
}

double read_value(
  const std::unordered_map<std::string, std::string> & arguments,
  const std::string & name, double fallback, bool required)
{
  const auto iterator = arguments.find(name);
  if (iterator == arguments.end()) {
    if (required) {
      throw std::invalid_argument("缺少参数 " + name);
    }
    return fallback;
  }
  std::size_t used = 0U;
  const double value = std::stod(iterator->second, &used);
  if (used != iterator->second.size()) {
    throw std::invalid_argument("参数不是有效数字 " + name);
  }
  return value;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    std::unordered_map<std::string, std::string> arguments;
    for (int index = 1; index < argc; index += 2) {
      const std::string name = argv[index];
      if (name == "--help" || name == "-h") {
        print_usage();
        return 0;
      }
      if (index + 1 >= argc || name.rfind("--", 0U) != 0U) {
        throw std::invalid_argument("参数必须使用 --名称 数值 的形式");
      }
      arguments[name] = argv[index + 1];
    }

    car_base::OdometryCalibrationInput input;
    input.actual_distance = read_value(
      arguments, "--actual-distance", 0.0, true);
    input.odom_distance = read_value(
      arguments, "--odom-distance", 0.0, true);
    input.actual_angle = read_value(
      arguments, "--actual-yaw-rad", 0.0, true);
    input.odom_angle = read_value(
      arguments, "--odom-yaw-rad", 0.0, true);
    input.wheel_distance = read_value(
      arguments, "--wheel-distance", 0.175, false);
    input.left_wheel_per_tick = read_value(
      arguments, "--left-wheel-per-tick", 0.0001039203, false);
    input.right_wheel_per_tick = read_value(
      arguments, "--right-wheel-per-tick", 0.0001033942, false);

    const auto result =
      car_base::calculate_odometry_calibration(input);
    std::cout << std::fixed << std::setprecision(9)
              << "建议参数\n"
              << "wheel_distance: " << result.wheel_distance << "\n"
              << "left_wheel_per_tick: "
              << result.left_wheel_per_tick << "\n"
              << "right_wheel_per_tick: "
              << result.right_wheel_per_tick << "\n"
              << "直线比例修正: " << result.distance_scale << "\n"
              << "请同步修改 car_base、ESP32 固件和 URDF 轮距\n";
  } catch (const std::exception & error) {
    std::cerr << "标定计算失败：" << error.what() << "\n";
    print_usage();
    return 1;
  }
  return 0;
}
