#include "car_rl/inference_backend.hpp"

#include <onnxruntime_cxx_api.h>

#include <sys/utsname.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace car_rl
{
namespace
{

void validate_tensor(
  const Ort::TypeInfo & type_info,
  const TensorSpec & expected,
  const std::string & description)
{
  const auto tensor = type_info.GetTensorTypeAndShapeInfo();
  if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
    throw std::runtime_error(description + "必须是float32");
  }
  if (tensor.GetShape() != expected.shape) {
    throw std::runtime_error(description + "固定shape与模型元数据不一致");
  }
}

class OnnxRuntimeBackend final : public InferenceBackend
{
public:
  OnnxRuntimeBackend()
  : environment_(ORT_LOGGING_LEVEL_WARNING, "car_rl")
  {
  }

  void load(
    const std::filesystem::path & model_path,
    const ModelMetadata & metadata) override
  {
    if (model_path != metadata.onnx_path ||
      !std::filesystem::is_regular_file(model_path))
    {
      throw std::runtime_error("ONNX Runtime只能加载元数据中已校验的模型");
    }
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetInterOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    auto candidate = std::make_unique<Ort::Session>(
      environment_, model_path.c_str(), options);
    if (candidate->GetInputCount() != 1U || candidate->GetOutputCount() != 1U) {
      throw std::runtime_error("控制器ONNX必须恰好包含一个输入和一个输出");
    }
    Ort::AllocatorWithDefaultOptions allocator;
    const auto input_name = candidate->GetInputNameAllocated(0U, allocator);
    const auto output_name = candidate->GetOutputNameAllocated(0U, allocator);
    if (std::string(input_name.get()) != metadata.input.name ||
      std::string(output_name.get()) != metadata.output.name)
    {
      throw std::runtime_error("控制器ONNX张量名称与元数据不一致");
    }
    validate_tensor(candidate->GetInputTypeInfo(0U), metadata.input, "控制器输入");
    validate_tensor(candidate->GetOutputTypeInfo(0U), metadata.output, "控制器输出");
    input_name_ = input_name.get();
    output_name_ = output_name.get();
    input_shape_ = metadata.input.shape;
    output_shape_ = metadata.output.shape;
    session_ = std::move(candidate);
  }

  void warmup() override
  {
    if (!session_) {
      throw std::runtime_error("ONNX Runtime模型尚未加载");
    }
    std::vector<float> input(86U, 0.0F);
    std::vector<float> output(2U, 0.0F);
    for (int index = 0; index < 5; ++index) {
      double elapsed = 0.0;
      std::string error;
      if (!run(
          input.data(), input.size(), output.data(), output.size(), elapsed, error))
      {
        throw std::runtime_error("ONNX Runtime预热失败：" + error);
      }
    }
  }

  bool run(
    const float * input,
    std::size_t input_size,
    float * output,
    std::size_t output_size,
    double & elapsed_ms,
    std::string & error) override
  {
    elapsed_ms = 0.0;
    error.clear();
    if (!session_) {
      error = "ONNX Runtime模型尚未加载";
      return false;
    }
    if (input == nullptr || output == nullptr || input_size != 86U || output_size != 2U) {
      error = "ONNX Runtime输入输出缓冲区不符合固定契约";
      return false;
    }
    try {
      const auto memory = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator, OrtMemTypeDefault);
      auto input_tensor = Ort::Value::CreateTensor<float>(
        memory, const_cast<float *>(input), input_size,
        input_shape_.data(), input_shape_.size());
      auto output_tensor = Ort::Value::CreateTensor<float>(
        memory, output, output_size, output_shape_.data(), output_shape_.size());
      const char * input_names[] = {input_name_.c_str()};
      const char * output_names[] = {output_name_.c_str()};
      const auto started = std::chrono::steady_clock::now();
      session_->Run(
        Ort::RunOptions{nullptr}, input_names, &input_tensor, 1U,
        output_names, &output_tensor, 1U);
      elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      if (!std::all_of(
          output, output + output_size, [](float value) {
            return std::isfinite(value);
          }))
      {
        error = "ONNX Runtime输出包含NaN或Inf";
        return false;
      }
      return true;
    } catch (const Ort::Exception & exception) {
      error = exception.what();
      return false;
    } catch (const std::exception & exception) {
      error = exception.what();
      return false;
    }
  }

  bool available() const override {return session_ != nullptr;}
  std::string name() const override {return "onnxruntime";}
  BackendArtifact artifact_type() const override
  {
    return BackendArtifact::kOnnxModel;
  }
  std::string runtime_fingerprint() const override
  {
    struct utsname platform {};
    if (uname(&platform) != 0) {
      throw std::runtime_error("无法读取ONNX Runtime所在平台信息");
    }
    return std::string("onnxruntime=") + Ort::GetVersionString() +
           ";provider=cpu;sysname=" + platform.sysname +
           ";release=" + platform.release + ";machine=" + platform.machine;
  }

private:
  Ort::Env environment_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
};

}  // namespace

std::unique_ptr<InferenceBackend> make_inference_backend()
{
  return std::make_unique<OnnxRuntimeBackend>();
}

}  // namespace car_rl
