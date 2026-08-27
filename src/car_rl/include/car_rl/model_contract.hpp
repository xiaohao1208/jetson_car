#ifndef CAR_RL__MODEL_CONTRACT_HPP_
#define CAR_RL__MODEL_CONTRACT_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace car_rl
{

// ONNX 输入或输出张量的稳定契约。首版模型全部使用固定 batch=1
struct TensorSpec
{
  // ONNX/TensorRT 张量名称
  std::string name;
  // 包含 batch 维的固定张量形状
  std::vector<int64_t> shape;

  // 计算固定形状张量包含的元素总数并检查整数溢出
  std::size_t element_count() const;
};

// 单个模型的元数据。训练工程导出，car_rl 在加载和转换前再次校验
struct ModelMetadata
{
  // 模型元数据格式版本
  int schema_version{0};
  // 模型角色，只允许controller
  std::string role;
  // 训练工程导出的模型版本
  std::string model_version;
  // ONNX 导出使用的算子集版本
  int onnx_opset{0};
  // ONNX 文件的 SHA-256 摘要
  std::string onnx_sha256;
  // 当前元数据文件路径
  std::filesystem::path metadata_path;
  // 当前元数据引用的 ONNX 文件路径
  std::filesystem::path onnx_path;
  // 模型输入张量契约
  TensorSpec input;
  // 模型输出张量契约
  TensorSpec output;
  // 模型契约允许的单次最大推理毫秒数，字段名由训练部署契约固定
  double max_inference_ms{0.0};
};

// 一个bundle只携带强化学习局部控制器模型
struct ModelBundle
{
  // bundle文件格式版本
  int schema_version{0};
  // 控制器模型发布版本
  std::string bundle_version;
  // bundle根目录
  std::filesystem::path root;
  // 控制器模型元数据
  ModelMetadata controller;
};

// 读取并严格校验controller-only bundle，任何字段、shape或hash错误都会抛出异常
ModelBundle load_model_bundle(const std::filesystem::path & bundle_root);

// 只检查bundle是否存在并满足契约，不要求TensorRT engine已经生成
bool model_bundle_available(
  const std::filesystem::path & bundle_root,
  std::string * reason = nullptr);

// 计算文件SHA-256，返回64字符小写十六进制字符串
std::string sha256_file(const std::filesystem::path & path);

// 默认从CAR_RL_BUNDLE_PATH或源码包models/controller目录读取
std::filesystem::path default_bundle_path();

// 基准测试结果固定保存在源码包benchmark目录
std::filesystem::path benchmark_output_root();

// ONNX Runtime等直接模型后端的验证清单路径
std::filesystem::path runtime_validation_path(
  const ModelMetadata & metadata,
  const std::string & backend_name);

// 写入直接模型后端实际加载、预热和基准测试后的验证清单
void write_runtime_validation(
  const ModelMetadata & metadata,
  const std::string & backend_name,
  const std::string & runtime_fingerprint,
  double p99_inference_ms);

// 校验直接模型后端清单是否与当前模型和运行环境一致
bool runtime_validation_available(
  const ModelMetadata & metadata,
  const std::string & backend_name,
  const std::string & runtime_fingerprint,
  std::string * reason = nullptr,
  double * p99_inference_ms = nullptr);

// TensorRT engine与ONNX hash绑定，避免升级模型后误用旧engine
std::filesystem::path engine_cache_path(
  const ModelMetadata & metadata,
  const std::string & precision = "fp16");

// 返回与指定推理引擎相邻的验证清单路径
std::filesystem::path engine_validation_path(
  const ModelMetadata & metadata,
  const std::string & precision = "fp16");

// 写入经过实际加载和预热的引擎验证清单
void write_engine_validation(
  const ModelMetadata & metadata,
  const std::filesystem::path & engine_path,
  const std::string & precision,
  const std::string & runtime_fingerprint,
  double p99_inference_ms = 0.0);

// 校验引擎、模型摘要、运行环境和验证清单是否完全一致
bool engine_validation_available(
  const ModelMetadata & metadata,
  const std::string & precision,
  const std::string & runtime_fingerprint,
  std::string * reason = nullptr,
  double * p99_inference_ms = nullptr);

}  // namespace car_rl

#endif  // CAR_RL__MODEL_CONTRACT_HPP_
