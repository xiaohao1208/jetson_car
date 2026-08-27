#include "car_observation/collection_support.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>

#include "ament_index_cpp/get_package_prefix.hpp"

namespace car_observation
{

CollectionError::CollectionError(std::string code, const std::string & message)
: std::runtime_error(message), code_(std::move(code)) {}

const std::string & CollectionError::code() const noexcept
{
  return code_;
}

double pose_distance(
  const geometry_msgs::msg::PoseStamped & first,
  const geometry_msgs::msg::PoseStamped & second)
{
  return std::hypot(
    second.pose.position.x - first.pose.position.x,
    second.pose.position.y - first.pose.position.y);
}

bool finite_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return pose.header.frame_id == "map" &&
         std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w) &&
         norm > 1.0e-12 && std::abs(norm - 1.0) <= 1.0e-3;
}

std::filesystem::path resolve_project_path(const std::filesystem::path & path)
{
  if (path.is_absolute()) {
    return path.lexically_normal();
  }
  if (const char * configured = std::getenv("JETSON_CAR_ROOT")) {
    if (*configured != '\0') {
      return (std::filesystem::absolute(configured) / path).lexically_normal();
    }
  }

  auto current = std::filesystem::weakly_canonical(
    ament_index_cpp::get_package_prefix("car_observation"));
  while (current.has_parent_path()) {
    if (current.filename() == "install") {
      return (current.parent_path() / path).lexically_normal();
    }
    const auto parent = current.parent_path();
    if (parent == current) {
      break;
    }
    current = parent;
  }
  throw std::runtime_error("无法确定观测数据对应的 Jetson 项目目录");
}

std::string local_timestamp(bool compact)
{
  const auto now = std::chrono::system_clock::now();
  const auto value = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&value, &local);
  std::ostringstream stream;
  stream << std::put_time(&local, compact ? "%Y%m%dT%H%M%S%z" : "%FT%T%z");
  return stream.str();
}

void fsync_path(const std::filesystem::path & path, bool directory)
{
  const int flags = directory ? O_RDONLY | O_DIRECTORY : O_RDONLY;
  const int descriptor = ::open(path.c_str(), flags);
  if (descriptor < 0) {
    throw std::runtime_error("无法打开待同步路径: " + path.string());
  }
  const int result = ::fsync(descriptor);
  ::close(descriptor);
  if (result != 0) {
    throw std::runtime_error("fsync失败: " + path.string());
  }
}

nlohmann::json counts_json(const car_rl::ObservationCoverageCounts & counts)
{
  return {
    {"straight", counts.straight},
    {"left_turn", counts.left_turn},
    {"right_turn", counts.right_turn},
    {"near_goal", counts.near_goal},
    {"front_near", counts.front_near},
    {"left_near", counts.left_near},
    {"right_near", counts.right_near},
    {"bilateral_near", counts.bilateral_near},
  };
}

}  // namespace car_observation
