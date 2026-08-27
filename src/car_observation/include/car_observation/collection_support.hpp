#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "car_rl/observation_recording.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace car_observation
{

enum class NavigationOutcome
{
  reached,
  failed,
  quality_ready,
};

class CollectionError : public std::runtime_error
{
public:
  CollectionError(std::string code, const std::string & message);
  const std::string & code() const noexcept;

private:
  std::string code_;
};

double pose_distance(
  const geometry_msgs::msg::PoseStamped & first,
  const geometry_msgs::msg::PoseStamped & second);
bool finite_pose(const geometry_msgs::msg::PoseStamped & pose);
std::filesystem::path resolve_project_path(const std::filesystem::path & path);
std::string local_timestamp(bool compact);
void fsync_path(const std::filesystem::path & path, bool directory = false);
nlohmann::json counts_json(const car_rl::ObservationCoverageCounts & counts);

}  // namespace car_observation
