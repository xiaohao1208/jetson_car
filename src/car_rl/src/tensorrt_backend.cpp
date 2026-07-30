#include "car_rl/inference_backend.hpp"

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace car_rl
{
namespace
{

class TensorRtLogger final : public nvinfer1::ILogger
{
public:
  // 只输出警告及更严重的 TensorRT 内部消息，避免持续刷屏
  void log(Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      fprintf(stderr, "[car_rl TensorRT] %s\n", message);
    }
  }
};

// 把 CUDA 错误码转换为带操作名称的 C++ 异常
void check_cuda(cudaError_t result, const std::string & operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(operation + "：" + cudaGetErrorString(result));
  }
}

// 严格比较 TensorRT 固定维度及其顺序与模型元数据是否一致
bool dims_match(const nvinfer1::Dims & dims, const std::vector<int64_t> & expected)
{
  if (dims.nbDims != static_cast<int>(expected.size())) {
    return false;
  }
  // 当前参与乘积的 TensorRT 维度索引
  for (int index = 0; index < dims.nbDims; ++index) {
    if (dims.d[index] <= 0 ||
      static_cast<int64_t>(dims.d[index]) != expected[static_cast<std::size_t>(index)])
    {
      return false;
    }
  }
  return true;
}

// TensorRT 后端拥有一个引擎、执行上下文、CUDA stream 和输入输出显存
class TensorRtBackend final : public InferenceBackend
{
public:
  // 释放 CUDA 和 TensorRT 资源
  ~TensorRtBackend() override
  {
    release();
  }

  // 反序列化引擎并严格验证输入输出名称、类型和维度
  void load(
    const std::filesystem::path & engine_path,
    const ModelMetadata & metadata) override
  {
    release();
    // 从磁盘读取序列化 engine 的二进制文件流
    std::ifstream stream(engine_path, std::ios::binary | std::ios::ate);
    if (!stream) {
      throw std::runtime_error("无法读取推理引擎：" + engine_path.string());
    }
    // 序列化 TensorRT engine 文件大小
    const std::streamsize size = stream.tellg();
    if (size <= 0) {
      throw std::runtime_error("推理引擎为空");
    }
    stream.seekg(0, std::ios::beg);
    // 序列化 TensorRT engine 文件内容
    std::vector<char> bytes(static_cast<std::size_t>(size));
    if (!stream.read(bytes.data(), size)) {
      throw std::runtime_error("读取推理引擎失败");
    }

    runtime_ = nvinfer1::createInferRuntime(logger_);
    if (runtime_ == nullptr) {
      throw std::runtime_error("创建推理运行环境失败");
    }
    engine_ = runtime_->deserializeCudaEngine(bytes.data(), bytes.size());
    if (engine_ == nullptr) {
      throw std::runtime_error("解析推理引擎失败");
    }
    context_ = engine_->createExecutionContext();
    if (context_ == nullptr) {
      throw std::runtime_error("创建推理执行上下文失败");
    }

    input_name_ = metadata.input.name;
    output_name_ = metadata.output.name;
    input_size_ = metadata.input.element_count();
    output_size_ = metadata.output.element_count();
    if (engine_->getNbIOTensors() != 2) {
      throw std::runtime_error("推理引擎必须且只能包含一个输入和一个输出");
    }
    if (engine_->getTensorIOMode(input_name_.c_str()) != nvinfer1::TensorIOMode::kINPUT ||
      engine_->getTensorIOMode(output_name_.c_str()) != nvinfer1::TensorIOMode::kOUTPUT)
    {
      throw std::runtime_error("推理引擎绑定名称与元数据不一致");
    }
    if (engine_->getTensorDataType(input_name_.c_str()) != nvinfer1::DataType::kFLOAT ||
      engine_->getTensorDataType(output_name_.c_str()) != nvinfer1::DataType::kFLOAT)
    {
      throw std::runtime_error("输入输出必须保持单精度，内部层可使用半精度");
    }
    if (!dims_match(engine_->getTensorShape(input_name_.c_str()), metadata.input.shape) ||
      !dims_match(engine_->getTensorShape(output_name_.c_str()), metadata.output.shape))
    {
      throw std::runtime_error("推理引擎维度与元数据不一致");
    }

    check_cuda(cudaStreamCreate(&stream_), "创建CUDA stream失败");
    check_cuda(
      cudaMalloc(&device_input_, input_size_ * sizeof(float)),
      "分配TensorRT输入显存失败");
    check_cuda(
      cudaMalloc(&device_output_, output_size_ * sizeof(float)),
      "分配TensorRT输出显存失败");
    if (!context_->setTensorAddress(input_name_.c_str(), device_input_) ||
      !context_->setTensorAddress(output_name_.c_str(), device_output_))
    {
      throw std::runtime_error("设置推理张量地址失败");
    }
    loaded_ = true;
  }

  // 连续执行五次零输入推理以初始化 CUDA 和 TensorRT 内部缓存
  void warmup() override
  {
    if (!loaded_) {
      throw std::runtime_error("推理后端尚未加载");
    }
    // warmup 使用的全零输入张量
    std::vector<float> input(input_size_, 0.0F);
    // warmup 使用的输出张量缓冲区
    std::vector<float> output(output_size_, 0.0F);
    // 当前 warmup 轮次
    for (int index = 0; index < 5; ++index) {
      // 推理后端报告的 warmup 执行耗时
      double elapsed = 0.0;
      // warmup 失败时由 run 填写的原因
      std::string error;
      if (!run(
          input.data(), input.size(), output.data(), output.size(),
          elapsed, error))
      {
        throw std::runtime_error("推理预热失败：" + error);
      }
    }
  }

  // 完成主机到设备拷贝、同步推理和设备到主机拷贝
  bool run(
    const float * input,
    std::size_t input_size,
    float * output,
    std::size_t output_size,
    double & elapsed_ms,
    std::string & error) override
  {
    if (!loaded_ || input == nullptr || output == nullptr ||
      input_size != input_size_ || output_size != output_size_)
    {
      error = "推理输入输出未加载或维度错误";
      elapsed_ms = 0.0;
      return false;
    }

    elapsed_ms = 0.0;
    // 本轮推理开始的单调时间
    const auto started = std::chrono::steady_clock::now();
    // 当前 CUDA 异步操作结果
    cudaError_t result = cudaMemcpyAsync(
      device_input_, input, input_size_ * sizeof(float),
      cudaMemcpyHostToDevice, stream_);
    if (result != cudaSuccess) {
      error = cudaGetErrorString(result);
      return false;
    }
    if (!context_->enqueueV3(stream_)) {
      error = "TensorRT enqueueV3失败";
      return false;
    }
    result = cudaMemcpyAsync(
      output, device_output_, output_size_ * sizeof(float),
      cudaMemcpyDeviceToHost, stream_);
    if (result != cudaSuccess) {
      error = cudaGetErrorString(result);
      return false;
    }
    result = cudaStreamSynchronize(stream_);
    if (result != cudaSuccess) {
      error = cudaGetErrorString(result);
      return false;
    }
    elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    error.clear();
    return true;
  }

  // 返回引擎和全部运行资源是否已经创建
  bool available() const override {return loaded_;}
  // 返回稳定的后端名称
  std::string name() const override {return "tensorrt";}
  // 生成用于拒绝跨平台旧引擎的运行环境指纹
  std::string runtime_fingerprint() const override
  {
    int runtime_version = 0;
    int driver_version = 0;
    int device = 0;
    check_cuda(cudaRuntimeGetVersion(&runtime_version), "读取CUDA运行时版本失败");
    check_cuda(cudaDriverGetVersion(&driver_version), "读取CUDA驱动版本失败");
    check_cuda(cudaGetDevice(&device), "读取CUDA设备编号失败");
    // 当前 CUDA 设备的名称和计算能力
    cudaDeviceProp properties{};
    check_cuda(cudaGetDeviceProperties(&properties, device), "读取CUDA设备属性失败");
    // 指纹只包含影响 TensorRT engine 兼容性的稳定字段
    std::ostringstream output;
    output << "tensorrt=" << NV_TENSORRT_MAJOR << "." << NV_TENSORRT_MINOR << "."
           << NV_TENSORRT_PATCH
           << ",cuda_runtime=" << runtime_version
           << ",cuda_driver=" << driver_version
           << ",compute=" << properties.major << "." << properties.minor
           << ",device=" << properties.name;
    return output.str();
  }

private:
  // 按显存、stream、上下文、engine 和 runtime 的逆序释放资源
  void release()
  {
    loaded_ = false;
    if (device_input_ != nullptr) {
      cudaFree(device_input_);
      device_input_ = nullptr;
    }
    if (device_output_ != nullptr) {
      cudaFree(device_output_);
      device_output_ = nullptr;
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
    delete context_;
    context_ = nullptr;
    delete engine_;
    engine_ = nullptr;
    delete runtime_;
    runtime_ = nullptr;
  }

  // 过滤 TensorRT 内部日志的日志器
  TensorRtLogger logger_;
  // 反序列化 engine 使用的 TensorRT runtime
  nvinfer1::IRuntime * runtime_{nullptr};
  // 当前加载的 TensorRT engine
  nvinfer1::ICudaEngine * engine_{nullptr};
  // 当前 engine 的推理执行上下文
  nvinfer1::IExecutionContext * context_{nullptr};
  // 主机与设备异步拷贝和推理共用的 CUDA stream
  cudaStream_t stream_{nullptr};
  // 输入张量设备显存
  void * device_input_{nullptr};
  // 输出张量设备显存
  void * device_output_{nullptr};
  // engine 输入张量名称
  std::string input_name_;
  // engine 输出张量名称
  std::string output_name_;
  // 输入张量元素数量
  std::size_t input_size_{0U};
  // 输出张量元素数量
  std::size_t output_size_{0U};
  // engine、执行上下文、CUDA stream 和显存是否全部就绪
  bool loaded_{false};
};

}  // namespace

std::unique_ptr<InferenceBackend> make_inference_backend()
{
  return std::make_unique<TensorRtBackend>();
}

}  // namespace car_rl
