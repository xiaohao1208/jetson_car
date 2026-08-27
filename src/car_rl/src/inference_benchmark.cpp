#include "car_rl/inference_benchmark.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace car_rl
{
namespace
{

double nearest_rank(const std::vector<double> & sorted, const double quantile)
{
  const auto rank = static_cast<std::size_t>(
    std::ceil(quantile * static_cast<double>(sorted.size())));
  return sorted[std::max<std::size_t>(1U, rank) - 1U];
}

} // namespace

InferenceBenchmarkStatistics
summarize_inference_times(const std::vector<double> & elapsed_ms)
{
  if (elapsed_ms.empty()) {
    throw std::invalid_argument("推理耗时样本不能为空");
  }
  if (!std::all_of(
      elapsed_ms.begin(), elapsed_ms.end(),
      [](const double value) {
        return std::isfinite(value) && value >= 0.0;
      }))
  {
    throw std::invalid_argument("推理耗时样本必须是非负有限数");
  }

  std::vector<double> sorted = elapsed_ms;
  std::sort(sorted.begin(), sorted.end());
  InferenceBenchmarkStatistics result;
  result.sample_count = sorted.size();
  result.mean_ms = std::accumulate(sorted.begin(), sorted.end(), 0.0) /
    static_cast<double>(sorted.size());
  result.p50_ms = nearest_rank(sorted, 0.50);
  result.p95_ms = nearest_rank(sorted, 0.95);
  result.p99_ms = nearest_rank(sorted, 0.99);
  result.max_ms = sorted.back();
  return result;
}

} // namespace car_rl
