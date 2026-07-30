#include "car_rl/inference_backend.hpp"

#include <stdexcept>

namespace car_rl
{
namespace
{

class UnavailableBackend final : public InferenceBackend
{
public:
  // 不可用后端在加载阶段给出明确原因，避免延迟到导航运行时失败
  void load(const std::filesystem::path &, const ModelMetadata &) override
  {
    throw std::runtime_error(
            "当前构建未检测到推理环境，经典导航可用，强化学习模式不可用");
  }

  // 不可用后端没有需要预热的设备资源
  void warmup() override {}

  // 所有推理请求都返回后端不可用
  bool run(
    const float *, std::size_t, float *, std::size_t,
    double & elapsed_ms, std::string & error) override
  {
    elapsed_ms = 0.0;
    error = "推理后端不可用";
    return false;
  }

  // 返回当前构建不包含真实推理后端
  bool available() const override {return false;}
  // 返回中文后端名称供状态和日志显示
  std::string name() const override {return "不可用";}
  // 不可用构建使用稳定指纹，验证清单不会把它误认为 Jetson 后端
  std::string runtime_fingerprint() const override {return "unavailable";}
};

}  // namespace

// 为没有 TensorRT 的开发机创建明确不可用的兼容后端
std::unique_ptr<InferenceBackend> make_inference_backend()
{
  return std::make_unique<UnavailableBackend>();
}

}  // namespace car_rl
