#ifndef CAR_RL__MODEL_SESSION_HPP_
#define CAR_RL__MODEL_SESSION_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "car_rl/inference_backend.hpp"
#include "car_rl/model_contract.hpp"

namespace car_rl
{

// 统一插件和影子节点的引擎验证、加载、预热与推理入口
class ModelSession
{
public:
  // 创建尚未加载模型的会话
  ModelSession();

  // 校验并加载指定模型当前精度的推理引擎
  void load(
    const ModelMetadata & metadata,
    const std::string & precision = "fp16");

  // 预热已经加载的推理后端
  void warmup();

  // 执行一次同步推理并透传耗时和错误原因
  bool run(
    const float * input,
    std::size_t input_size,
    float * output,
    std::size_t output_size,
    double & elapsed_ms,
    std::string & error);

  // 返回当前会话是否已经加载可用引擎
  bool available() const;

  // 返回具体推理后端名称
  std::string backend_name() const;

private:
  // 当前平台创建的 TensorRT 或不可用后端
  std::unique_ptr<InferenceBackend> backend_;
};

}  // namespace car_rl

#endif  // CAR_RL__MODEL_SESSION_HPP_
