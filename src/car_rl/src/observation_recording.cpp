#include "car_rl/observation_recording.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <ostream>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_set>

namespace car_rl
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

using ExactKey = std::array<std::uint32_t, kControllerObservationSize>;
using QuantizedKey = std::array<std::int64_t, kControllerObservationSize>;

struct ExactKeyLess
{
  bool operator()(const ExactKey & first, const ExactKey & second) const
  {
    return std::lexicographical_compare(
      first.begin(), first.end(), second.begin(), second.end());
  }
};

struct QuantizedKeyLess
{
  bool operator()(const QuantizedKey & first, const QuantizedKey & second) const
  {
    return std::lexicographical_compare(
      first.begin(), first.end(), second.begin(), second.end());
  }
};

ExactKey exact_key(const ControllerObservation & observation)
{
  ExactKey key{};
  for (std::size_t index = 0U; index < observation.size(); ++index) {
    static_assert(sizeof(float) == sizeof(std::uint32_t), "float32 contract required");
    std::memcpy(&key[index], &observation[index], sizeof(float));
  }
  return key;
}

template<typename Predicate>
std::vector<std::size_t> matching_indices(
  const std::vector<RecordedObservation> & samples,
  const std::vector<bool> & selected,
  Predicate predicate)
{
  std::vector<std::size_t> matches;
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    if (!selected[index] && predicate(samples[index])) {
      matches.push_back(index);
    }
  }
  return matches;
}

void append_evenly(
  const std::vector<std::size_t> & candidates,
  std::size_t count,
  std::vector<bool> & selected,
  std::vector<std::size_t> & output)
{
  if (count == 0U) {
    return;
  }
  if (candidates.size() < count) {
    throw std::runtime_error("coverage_balanced_v1候选样本不足");
  }
  std::vector<std::size_t> remaining = candidates;
  for (std::size_t slot = 0U; slot < count; ++slot) {
    const std::size_t remaining_slots = count - slot;
    const std::size_t position = std::min(
      remaining.size() - 1U,
      (remaining.size() - 1U) / remaining_slots);
    const std::size_t index = remaining[position];
    if (!selected[index]) {
      selected[index] = true;
      output.push_back(index);
    }
    remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(position));
  }
}

ObservationCoverageCounts counts_for_selected(
  const std::vector<RecordedObservation> & samples,
  const std::vector<std::size_t> & selected)
{
  return count_coverage(samples, &selected);
}

}  // namespace

ObservationCoverage classify_observation(
  const ControllerObservation & observation,
  const ObservationCoverageThresholds & thresholds)
{
  for (const float value : observation) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("观测包含NaN或Inf");
    }
  }
  const double linear = static_cast<double>(observation[82]) * 0.25;
  const double angular = static_cast<double>(observation[83]);
  const double goal_distance = std::hypot(
    static_cast<double>(observation[78]),
    static_cast<double>(observation[79])) * 3.0;
  double front = std::numeric_limits<double>::infinity();
  double left = std::numeric_limits<double>::infinity();
  double right = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < kScanRayCount; ++index) {
    const double angle = -kPi + 2.0 * kPi * static_cast<double>(index) /
      static_cast<double>(kScanRayCount);
    const double degrees = angle * 180.0 / kPi;
    const double distance = 0.05 + static_cast<double>(observation[index]) * 5.95;
    if (std::abs(degrees) <= 30.0 + 1.0e-9) {
      front = std::min(front, distance);
    }
    if (degrees >= 30.0 - 1.0e-9 && degrees <= 100.0 + 1.0e-9) {
      left = std::min(left, distance);
    }
    if (degrees >= -100.0 - 1.0e-9 && degrees <= -30.0 + 1.0e-9) {
      right = std::min(right, distance);
    }
  }
  ObservationCoverage result;
  constexpr double boundary_epsilon = 1.0e-6;
  result.straight = linear + boundary_epsilon >= thresholds.straight_min_linear_mps &&
    std::abs(angular) <= thresholds.straight_max_angular_radps + boundary_epsilon;
  result.left_turn = angular + boundary_epsilon >= thresholds.turn_min_angular_radps;
  result.right_turn = angular - boundary_epsilon <= -thresholds.turn_min_angular_radps;
  result.near_goal = goal_distance <= thresholds.near_goal_distance_m + boundary_epsilon;
  result.front_near = front < thresholds.near_obstacle_distance_m;
  result.left_near = left < thresholds.near_obstacle_distance_m;
  result.right_near = right < thresholds.near_obstacle_distance_m;
  result.bilateral_near = left < thresholds.bilateral_obstacle_distance_m &&
    right < thresholds.bilateral_obstacle_distance_m;
  return result;
}

ObservationCoverageCounts count_coverage(
  const std::vector<RecordedObservation> & samples,
  const std::vector<std::size_t> * indices)
{
  ObservationCoverageCounts counts;
  const auto add = [&counts](const ObservationCoverage & value) {
      counts.straight += value.straight ? 1U : 0U;
      counts.left_turn += value.left_turn ? 1U : 0U;
      counts.right_turn += value.right_turn ? 1U : 0U;
      counts.near_goal += value.near_goal ? 1U : 0U;
      counts.front_near += value.front_near ? 1U : 0U;
      counts.left_near += value.left_near ? 1U : 0U;
      counts.right_near += value.right_near ? 1U : 0U;
      counts.bilateral_near += value.bilateral_near ? 1U : 0U;
    };
  if (indices == nullptr) {
    for (const auto & sample : samples) {
      add(sample.coverage);
    }
  } else {
    for (const auto index : *indices) {
      if (index >= samples.size()) {
        throw std::out_of_range("观测选择索引越界");
      }
      add(samples[index].coverage);
    }
  }
  return counts;
}

std::size_t count_exact_unique(
  const std::vector<RecordedObservation> & samples,
  const std::vector<std::size_t> * indices)
{
  std::set<ExactKey, ExactKeyLess> unique;
  if (indices == nullptr) {
    for (const auto & sample : samples) {
      unique.insert(exact_key(sample.values));
    }
  } else {
    for (const auto index : *indices) {
      if (index >= samples.size()) {
        throw std::out_of_range("观测唯一性索引越界");
      }
      unique.insert(exact_key(samples[index].values));
    }
  }
  return unique.size();
}

std::size_t count_quantized_unique(
  const std::vector<RecordedObservation> & samples,
  const std::vector<std::size_t> * indices,
  double quantum)
{
  if (!std::isfinite(quantum) || quantum <= 0.0) {
    throw std::invalid_argument("观测量化步长必须为有限正数");
  }
  std::set<QuantizedKey, QuantizedKeyLess> unique;
  const auto insert = [&unique, quantum](const ControllerObservation & values) {
      QuantizedKey key{};
      for (std::size_t index = 0U; index < values.size(); ++index) {
        key[index] = std::llround(static_cast<double>(values[index]) / quantum);
      }
      unique.insert(key);
    };
  if (indices == nullptr) {
    for (const auto & sample : samples) {
      insert(sample.values);
    }
  } else {
    for (const auto index : *indices) {
      if (index >= samples.size()) {
        throw std::out_of_range("观测量化索引越界");
      }
      insert(samples[index].values);
    }
  }
  return unique.size();
}

bool coverage_requirements_met(
  const ObservationCoverageCounts & counts,
  std::size_t unique_samples,
  const ObservationSelectionRequirements & requirements)
{
  return unique_samples >= requirements.minimum_unique_samples &&
         counts.straight >= requirements.minimum_straight_samples &&
         counts.left_turn >= requirements.minimum_left_turn_samples &&
         counts.right_turn >= requirements.minimum_right_turn_samples &&
         counts.near_goal >= requirements.minimum_near_goal_samples;
}

std::vector<std::size_t> select_coverage_balanced(
  const std::vector<RecordedObservation> & samples,
  const ObservationSelectionRequirements & requirements)
{
  if (requirements.validation_prefix_samples > requirements.target_samples ||
    requirements.target_samples > samples.size())
  {
    throw std::invalid_argument("coverage_balanced_v1样本数配置无效");
  }
  const auto raw_counts = count_coverage(samples);
  if (!coverage_requirements_met(
      raw_counts, count_exact_unique(samples), requirements))
  {
    throw std::runtime_error("原始观测没有满足核心覆盖门禁");
  }

  std::vector<bool> selected(samples.size(), false);
  std::vector<std::size_t> output;
  output.reserve(requirements.target_samples);

  // 先等距选择不同完整float32位模式，保证验证前缀的唯一性硬门禁。
  std::set<ExactKey, ExactKeyLess> seen_keys;
  std::vector<std::size_t> unique_candidates;
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    if (seen_keys.insert(exact_key(samples[index].values)).second) {
      unique_candidates.push_back(index);
    }
  }
  append_evenly(
    unique_candidates, requirements.minimum_unique_samples, selected, output);

  struct Category
  {
    std::size_t raw;
    std::size_t required;
    int order;
    bool ObservationCoverage::* member;
  };
  std::array<Category, 4> categories{{
    {raw_counts.straight, requirements.minimum_straight_samples, 0,
      &ObservationCoverage::straight},
    {raw_counts.left_turn, requirements.minimum_left_turn_samples, 1,
      &ObservationCoverage::left_turn},
    {raw_counts.right_turn, requirements.minimum_right_turn_samples, 2,
      &ObservationCoverage::right_turn},
    {raw_counts.near_goal, requirements.minimum_near_goal_samples, 3,
      &ObservationCoverage::near_goal},
  }};
  std::stable_sort(
    categories.begin(), categories.end(),
    [](const Category & first, const Category & second) {
      const double first_ratio = first.required == 0U ?
      std::numeric_limits<double>::infinity() :
      static_cast<double>(first.raw) / static_cast<double>(first.required);
      const double second_ratio = second.required == 0U ?
      std::numeric_limits<double>::infinity() :
      static_cast<double>(second.raw) / static_cast<double>(second.required);
      return std::tie(first_ratio, first.order) < std::tie(second_ratio, second.order);
    });
  for (const auto & category : categories) {
    const auto current = counts_for_selected(samples, output);
    std::size_t have = 0U;
    if (category.member == &ObservationCoverage::straight) {
      have = current.straight;
    } else if (category.member == &ObservationCoverage::left_turn) {
      have = current.left_turn;
    } else if (category.member == &ObservationCoverage::right_turn) {
      have = current.right_turn;
    } else {
      have = current.near_goal;
    }
    if (have >= category.required) {
      continue;
    }
    const auto candidates = matching_indices(
      samples, selected,
      [&category](const RecordedObservation & sample) {
        return sample.coverage.*(category.member);
      });
    append_evenly(candidates, category.required - have, selected, output);
  }

  const auto fill_to = [&](std::size_t target) {
      if (output.size() > target) {
        throw std::runtime_error("核心样本超过验证前缀容量");
      }
      const auto candidates = matching_indices(
        samples, selected, [](const RecordedObservation &) {return true;});
      append_evenly(candidates, target - output.size(), selected, output);
    };
  fill_to(requirements.validation_prefix_samples);
  std::vector<std::size_t> prefix(output.begin(), output.end());
  if (!coverage_requirements_met(
      count_coverage(samples, &prefix), count_exact_unique(samples, &prefix), requirements))
  {
    throw std::runtime_error("coverage_balanced_v1前4000条未满足核心门禁");
  }
  fill_to(requirements.target_samples);
  return output;
}

void write_observation_csv_header(std::ostream & stream)
{
  for (std::size_t index = 0U; index < kControllerObservationSize; ++index) {
    if (index > 0U) {
      stream << ',';
    }
    stream << "observation_" << index;
  }
  stream << '\n';
}

void write_observation_csv_row(
  std::ostream & stream, const ControllerObservation & observation)
{
  stream << std::setprecision(std::numeric_limits<float>::max_digits10);
  for (std::size_t index = 0U; index < observation.size(); ++index) {
    if (!std::isfinite(observation[index])) {
      throw std::invalid_argument("观测CSV包含NaN或Inf");
    }
    if (index > 0U) {
      stream << ',';
    }
    stream << observation[index];
  }
  stream << '\n';
}

}  // namespace car_rl
