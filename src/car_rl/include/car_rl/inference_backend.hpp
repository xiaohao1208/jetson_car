#ifndef CAR_RL__INFERENCE_BACKEND_HPP_
#define CAR_RL__INFERENCE_BACKEND_HPP_

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>

#include "car_rl/model_contract.hpp"

namespace car_rl
{

// 插件只依赖这个窄接口，TensorRT和测试后端不会污染控制/规划逻辑
class InferenceBackend
{
public:
  // 允许通过基类安全释放具体推理后端
  virtual ~InferenceBackend() = default;

  // 加载并校验与模型元数据对应的序列化推理引擎
  virtual void load(
    const std::filesystem::path & engine_path,
    const ModelMetadata & metadata) = 0;
  // 使用固定形状的零输入预热推理后端
  virtual void warmup() = 0;
  // 执行一次同步推理并返回耗时和中文错误原因
  virtual bool run(
    const float * input,
    std::size_t input_size,
    float * output,
    std::size_t output_size,
    double & elapsed_ms,
    std::string & error) = 0;
  // 返回当前后端是否已经成功加载推理引擎
  virtual bool available() const = 0;
  // 返回用于日志和状态接口的后端名称
  virtual std::string name() const = 0;
  // 返回与引擎兼容性相关的 TensorRT、CUDA 和 GPU 环境指纹
  virtual std::string runtime_fingerprint() const = 0;
};

// 当前平台有TensorRT时返回真实后端，否则返回明确不可用的后端
std::unique_ptr<InferenceBackend> make_inference_backend();

}  // namespace car_rl

#endif  // CAR_RL__INFERENCE_BACKEND_HPP_
