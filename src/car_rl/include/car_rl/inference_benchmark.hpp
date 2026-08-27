#ifndef CAR_RL__INFERENCE_BENCHMARK_HPP_
#define CAR_RL__INFERENCE_BENCHMARK_HPP_

#include <cstddef>
#include <vector>

namespace car_rl
{

// 一组同步推理耗时的稳定摘要，单位均为毫秒。
struct InferenceBenchmarkStatistics
{
  std::size_t sample_count{0U};
  double mean_ms{0.0};
  double p50_ms{0.0};
  double p95_ms{0.0};
  double p99_ms{0.0};
  double max_ms{0.0};
};

// 使用 nearest-rank 分位数计算可复现的推理耗时摘要。
InferenceBenchmarkStatistics
summarize_inference_times(const std::vector<double> & elapsed_ms);

} // namespace car_rl

#endif // CAR_RL__INFERENCE_BENCHMARK_HPP_
