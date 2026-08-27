#ifndef CAR_RL__OBSERVATION_RECORDING_HPP_
#define CAR_RL__OBSERVATION_RECORDING_HPP_

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

#include "car_rl/observation.hpp"

namespace car_rl
{

struct ObservationCoverage
{
  bool straight{false};
  bool left_turn{false};
  bool right_turn{false};
  bool near_goal{false};
  bool front_near{false};
  bool left_near{false};
  bool right_near{false};
  bool bilateral_near{false};
};

struct ObservationCoverageThresholds
{
  double near_goal_distance_m{0.35};
  double straight_min_linear_mps{0.02};
  double straight_max_angular_radps{0.10};
  double turn_min_angular_radps{0.15};
  double near_obstacle_distance_m{0.60};
  double bilateral_obstacle_distance_m{0.80};
};

struct ObservationCoverageCounts
{
  std::size_t straight{0U};
  std::size_t left_turn{0U};
  std::size_t right_turn{0U};
  std::size_t near_goal{0U};
  std::size_t front_near{0U};
  std::size_t left_near{0U};
  std::size_t right_near{0U};
  std::size_t bilateral_near{0U};
};

struct ObservationSelectionRequirements
{
  std::size_t target_samples{5000U};
  std::size_t validation_prefix_samples{4000U};
  std::size_t minimum_unique_samples{100U};
  std::size_t minimum_straight_samples{500U};
  std::size_t minimum_left_turn_samples{200U};
  std::size_t minimum_right_turn_samples{200U};
  std::size_t minimum_near_goal_samples{200U};
};

struct RecordedObservation
{
  ControllerObservation values{};
  ObservationCoverage coverage{};
  std::size_t sequence{0U};
};

ObservationCoverage classify_observation(
  const ControllerObservation & observation,
  const ObservationCoverageThresholds & thresholds = {});

ObservationCoverageCounts count_coverage(
  const std::vector<RecordedObservation> & samples,
  const std::vector<std::size_t> * indices = nullptr);

std::size_t count_exact_unique(
  const std::vector<RecordedObservation> & samples,
  const std::vector<std::size_t> * indices = nullptr);

std::size_t count_quantized_unique(
  const std::vector<RecordedObservation> & samples,
  const std::vector<std::size_t> * indices = nullptr,
  double quantum = 1.0e-3);

bool coverage_requirements_met(
  const ObservationCoverageCounts & counts,
  std::size_t unique_samples,
  const ObservationSelectionRequirements & requirements);

std::vector<std::size_t> select_coverage_balanced(
  const std::vector<RecordedObservation> & samples,
  const ObservationSelectionRequirements & requirements);

void write_observation_csv_header(std::ostream & stream);
void write_observation_csv_row(
  std::ostream & stream, const ControllerObservation & observation);

}  // namespace car_rl

#endif  // CAR_RL__OBSERVATION_RECORDING_HPP_
