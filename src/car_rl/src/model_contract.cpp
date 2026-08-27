#include "car_rl/model_contract.hpp"

#include <openssl/evp.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace car_rl
{
namespace
{

// 解析 YAML 中的固定正维度数组
std::vector<int64_t> parse_shape(const YAML::Node & node, const std::string & field)
{
  if (!node || !node.IsSequence() || node.size() == 0) {
    throw std::runtime_error(field + " 必须是非空维度数组");
  }

  // 解析并验证后的固定正维度列表
  std::vector<int64_t> shape;
  shape.reserve(node.size());
  // 当前 YAML shape 数组元素
  for (const auto & value : node) {
    // 当前张量维度值
    const int64_t dimension = value.as<int64_t>();
    if (dimension <= 0) {
      throw std::runtime_error(field + " 只允许固定正维度");
    }
    shape.push_back(dimension);
  }
  return shape;
}

// 解析单个输入或输出张量定义
TensorSpec parse_tensor(const YAML::Node & node, const std::string & field)
{
  if (!node || !node.IsMap()) {
    throw std::runtime_error(field + " 缺失");
  }

  // 当前 YAML 节点解析出的张量契约
  TensorSpec tensor;
  tensor.name = node["name"].as<std::string>();
  tensor.shape = parse_shape(node["shape"], field + ".shape");
  if (tensor.name.empty()) {
    throw std::runtime_error(field + ".name 不能为空");
  }
  return tensor;
}

// 判断规范化后的文件是否位于模型包根目录中
bool path_is_within(
  const std::filesystem::path & root,
  const std::filesystem::path & candidate)
{
  const auto relative = candidate.lexically_relative(root);
  if (relative.empty() || relative.is_absolute()) {
    return false;
  }
  for (const auto & part : relative) {
    if (part == "..") {
      return false;
    }
  }
  return true;
}

// 校验部署审计文件存在且非空，不在部署端解释训练工程私有字段
void require_nonempty_file(
  const std::filesystem::path & root,
  const std::filesystem::path & path,
  const std::string & description)
{
  const auto canonical = std::filesystem::weakly_canonical(path);
  if (!path_is_within(root, canonical) ||
    !std::filesystem::is_regular_file(canonical) ||
    std::filesystem::file_size(canonical) == 0U)
  {
    throw std::runtime_error(description + "不存在、为空或超出模型包目录");
  }
}

// 解析单个模型元数据并验证角色、摘要和固定张量契约
ModelMetadata load_metadata(
  const std::filesystem::path & metadata_path,
  const std::filesystem::path & bundle_root,
  const std::string & expected_role)
{
  const auto canonical_metadata = std::filesystem::weakly_canonical(metadata_path);
  if (!path_is_within(bundle_root, canonical_metadata)) {
    throw std::runtime_error("模型元数据路径超出模型包目录");
  }
  if (!std::filesystem::is_regular_file(canonical_metadata)) {
    throw std::runtime_error("模型元数据不存在：" + canonical_metadata.string());
  }

  // 模型元数据 YAML 根节点
  const YAML::Node root = YAML::LoadFile(canonical_metadata.string());
  // 逐字段填充并校验的模型元数据
  ModelMetadata metadata;
  metadata.schema_version = root["schema_version"].as<int>();
  metadata.role = root["role"].as<std::string>();
  metadata.model_version = root["model_version"].as<std::string>();
  metadata.onnx_opset = root["onnx_opset"].as<int>();
  metadata.onnx_sha256 = root["onnx_sha256"].as<std::string>();
  metadata.metadata_path = canonical_metadata;
  metadata.onnx_path = std::filesystem::weakly_canonical(
    canonical_metadata.parent_path() / root["model_file"].as<std::string>());
  metadata.input = parse_tensor(root["input"], "input");
  metadata.output = parse_tensor(root["output"], "output");
  metadata.max_inference_ms = root["max_inference_ms"].as<double>();

  if (metadata.schema_version != 2) {
    throw std::runtime_error("不支持的模型格式版本");
  }
  if (metadata.role != expected_role) {
    throw std::runtime_error("模型角色错误：期望" + expected_role + "，实际" + metadata.role);
  }
  if (metadata.model_version.empty()) {
    throw std::runtime_error("模型版本不能为空");
  }
  if (metadata.onnx_opset != 18) {
    throw std::runtime_error("当前模型约定固定要求ONNX算子集18");
  }
  if (metadata.onnx_sha256.size() != 64U) {
    throw std::runtime_error("onnx_sha256 必须是64字符");
  }
  if (!std::all_of(
      metadata.onnx_sha256.begin(), metadata.onnx_sha256.end(),
      [](unsigned char character) {
        return (character >= '0' && character <= '9') ||
        (character >= 'a' && character <= 'f');
      }))
  {
    throw std::runtime_error("onnx_sha256 必须是64字符小写十六进制摘要");
  }
  if (!path_is_within(bundle_root, metadata.onnx_path)) {
    throw std::runtime_error("ONNX路径超出模型包目录");
  }
  if (!std::filesystem::is_regular_file(metadata.onnx_path)) {
    throw std::runtime_error("ONNX文件不存在: " + metadata.onnx_path.string());
  }
  if (sha256_file(metadata.onnx_path) != metadata.onnx_sha256) {
    throw std::runtime_error("ONNX SHA-256与元数据不一致");
  }
  if (metadata.max_inference_ms <= 0.0) {
    throw std::runtime_error("最大推理时间必须大于0");
  }

  if (metadata.input.name != "observation" ||
    metadata.input.shape != std::vector<int64_t>({1, 86}) ||
    metadata.output.name != "action" ||
    metadata.output.shape != std::vector<int64_t>({1, 2}))
  {
    throw std::runtime_error("控制器张量不符合当前模型约定");
  }

  return metadata;
}

// 从显式环境或 colcon 安装位置确定 Jetson 工作区根目录
std::filesystem::path project_root()
{
  if (const char * configured = std::getenv("JETSON_CAR_ROOT")) {
    if (*configured != '\0') {
      const auto root = std::filesystem::weakly_canonical(
        std::filesystem::absolute(configured));
      if (!std::filesystem::is_directory(root)) {
        throw std::runtime_error("JETSON_CAR_ROOT不是有效目录");
      }
      return root;
    }
  }

  auto current = std::filesystem::weakly_canonical(
    ament_index_cpp::get_package_share_directory("car_rl"));
  while (current.has_parent_path()) {
    if (current.filename() == "install") {
      return current.parent_path();
    }
    const auto parent = current.parent_path();
    if (parent == current) {
      break;
    }
    current = parent;
  }
  throw std::runtime_error("无法从 car_rl 安装位置确定 Jetson 项目目录");
}

// 返回源码包内按模型种类划分的控制器 bundle 目录
std::filesystem::path controller_bundle_root()
{
  return project_root() / "src" / "car_rl" / "models" / "controller";
}

void validate_backend_name(const std::string & backend_name)
{
  if (backend_name.empty() || !std::all_of(
      backend_name.begin(), backend_name.end(), [](unsigned char character) {
        return (character >= 'a' && character <= 'z') ||
        (character >= '0' && character <= '9') || character == '_';
      }))
  {
    throw std::runtime_error("推理后端名称只能包含小写字母、数字和下划线");
  }
}

}  // namespace

std::size_t TensorSpec::element_count() const
{
  // 各维度相乘得到的张量元素总数
  std::size_t count = 1U;
  // 当前参与乘积的张量维度
  for (const int64_t dimension : shape) {
    if (dimension <= 0 ||
      static_cast<uint64_t>(dimension) >
      static_cast<uint64_t>(std::numeric_limits<std::size_t>::max() / count))
    {
      throw std::overflow_error("张量元素数量溢出");
    }
    count *= static_cast<std::size_t>(dimension);
  }
  return count;
}

ModelBundle load_model_bundle(const std::filesystem::path & bundle_root)
{
  // 归一化后的模型 bundle 绝对根目录
  const std::filesystem::path absolute_root =
    std::filesystem::weakly_canonical(std::filesystem::absolute(bundle_root));
  // bundle 清单文件路径
  const std::filesystem::path manifest_path = absolute_root / "bundle.yaml";
  if (!std::filesystem::is_regular_file(manifest_path)) {
    throw std::runtime_error("模型包缺少bundle.yaml：" + absolute_root.string());
  }

  // bundle 清单 YAML 根节点
  const YAML::Node root = YAML::LoadFile(manifest_path.string());
  // 逐字段填充并校验的模型 bundle
  ModelBundle bundle;
  bundle.schema_version = root["schema_version"].as<int>();
  bundle.bundle_version = root["bundle_version"].as<std::string>();
  bundle.root = absolute_root;
  if (bundle.schema_version == 1) {
    throw std::runtime_error("旧版双模型包已停用，请导出schema_version 2控制器模型包");
  }
  if (bundle.schema_version != 2 || bundle.bundle_version.empty()) {
    throw std::runtime_error("模型包版本字段无效，当前只支持schema_version 2");
  }

  // 控制器元数据文件路径；既支持训练包子目录，也支持部署后的扁平目录
  const auto controller_metadata =
    absolute_root / root["controller_metadata"].as<std::string>();
  bundle.controller = load_metadata(controller_metadata, absolute_root, "controller");
  if (bundle.controller.model_version != bundle.bundle_version) {
    throw std::runtime_error("控制器模型版本必须与模型包版本一致");
  }
  require_nonempty_file(
    absolute_root, controller_metadata.parent_path() / "evaluation.json",
    "控制器评估文件");
  require_nonempty_file(
    absolute_root, absolute_root / "environment-lock.json",
    "训练环境锁定文件");
  return bundle;
}

bool model_bundle_available(
  const std::filesystem::path & bundle_root,
  std::string * reason)
{
  try {
    (void)load_model_bundle(bundle_root);
    if (reason != nullptr) {
      reason->clear();
    }
    return true;
  } catch (const std::exception & exception) {
    if (reason != nullptr) {
      *reason = exception.what();
    }
    return false;
  }
}

std::string sha256_file(const std::filesystem::path & path)
{
  // 待计算摘要的二进制文件流
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("无法读取文件计算SHA-256：" + path.string());
  }

  // OpenSSL SHA-256 增量摘要上下文
  EVP_MD_CTX * context = EVP_MD_CTX_new();
  if (context == nullptr || EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) {
    if (context != nullptr) {
      EVP_MD_CTX_free(context);
    }
    throw std::runtime_error("初始化SHA-256失败");
  }

  // 分块读取模型文件的固定缓冲区
  std::array<char, 64 * 1024> buffer{};
  while (stream.good()) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    // 本轮实际读取的文件字节数
    const std::streamsize read_count = stream.gcount();
    if (read_count > 0 &&
      EVP_DigestUpdate(context, buffer.data(), static_cast<std::size_t>(read_count)) != 1)
    {
      EVP_MD_CTX_free(context);
      throw std::runtime_error("更新SHA-256失败");
    }
  }
  if (!stream.eof()) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("读取文件计算SHA-256失败：" + path.string());
  }

  // OpenSSL 写入的二进制摘要缓冲区
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  // OpenSSL 返回的实际摘要长度
  unsigned int digest_size = 0U;
  if (EVP_DigestFinal_ex(context, digest.data(), &digest_size) != 1) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("完成SHA-256失败");
  }
  EVP_MD_CTX_free(context);

  // 把二进制摘要转换为小写十六进制的输出流
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  // 当前转换的摘要字节索引
  for (unsigned int index = 0U; index < digest_size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return output.str();
}

std::filesystem::path default_bundle_path()
{
  if (const char * configured = std::getenv("CAR_RL_BUNDLE_PATH")) {
    if (*configured != '\0') {
      return std::filesystem::path(configured);
    }
  }
  return controller_bundle_root();
}

std::filesystem::path benchmark_output_root()
{
  return project_root() / "src" / "car_rl" / "benchmark";
}

std::filesystem::path runtime_validation_path(
  const ModelMetadata & metadata,
  const std::string & backend_name)
{
  validate_backend_name(backend_name);
  const std::string short_hash = metadata.onnx_sha256.substr(0U, 16U);
  return project_root() / "cache" / "car_rl" / short_hash /
         (metadata.role + "_" + backend_name + ".validated.yaml");
}

void write_runtime_validation(
  const ModelMetadata & metadata,
  const std::string & backend_name,
  const std::string & runtime_fingerprint,
  double p99_inference_ms)
{
  validate_backend_name(backend_name);
  if (runtime_fingerprint.empty() || !std::isfinite(p99_inference_ms) ||
    p99_inference_ms < 0.0)
  {
    throw std::runtime_error("推理运行时验证参数无效");
  }
  const auto manifest = runtime_validation_path(metadata, backend_name);
  const auto temporary = manifest.string() + ".tmp";
  std::filesystem::create_directories(manifest.parent_path());
  YAML::Emitter yaml;
  yaml << YAML::BeginMap
       << YAML::Key << "schema_version" << YAML::Value << 1
       << YAML::Key << "role" << YAML::Value << metadata.role
       << YAML::Key << "backend_name" << YAML::Value << backend_name
       << YAML::Key << "onnx_sha256" << YAML::Value << metadata.onnx_sha256
       << YAML::Key << "runtime_fingerprint" << YAML::Value << runtime_fingerprint
       << YAML::Key << "input_name" << YAML::Value << metadata.input.name
       << YAML::Key << "output_name" << YAML::Value << metadata.output.name
       << YAML::Key << "p99_inference_ms" << YAML::Value << p99_inference_ms
       << YAML::EndMap;
  if (!yaml.good()) {
    throw std::runtime_error("生成推理运行时验证清单失败");
  }
  std::ofstream stream(temporary, std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("无法写入推理运行时验证清单");
  }
  stream << yaml.c_str() << '\n';
  stream.close();
  if (!stream) {
    std::filesystem::remove(temporary);
    throw std::runtime_error("写入推理运行时验证清单失败");
  }
  std::filesystem::rename(temporary, manifest);
}

bool runtime_validation_available(
  const ModelMetadata & metadata,
  const std::string & backend_name,
  const std::string & runtime_fingerprint,
  std::string * reason,
  double * p99_inference_ms)
{
  try {
    const auto manifest = runtime_validation_path(metadata, backend_name);
    if (!std::filesystem::is_regular_file(manifest)) {
      throw std::runtime_error(metadata.role + "的" + backend_name + "运行时尚未准备");
    }
    const YAML::Node root = YAML::LoadFile(manifest.string());
    if (root["schema_version"].as<int>() != 1 ||
      root["role"].as<std::string>() != metadata.role ||
      root["backend_name"].as<std::string>() != backend_name ||
      root["onnx_sha256"].as<std::string>() != metadata.onnx_sha256 ||
      root["runtime_fingerprint"].as<std::string>() != runtime_fingerprint ||
      root["input_name"].as<std::string>() != metadata.input.name ||
      root["output_name"].as<std::string>() != metadata.output.name)
    {
      throw std::runtime_error(metadata.role + "推理运行时清单与当前环境不一致");
    }
    const double p99 = root["p99_inference_ms"].as<double>();
    if (!std::isfinite(p99) || p99 < 0.0 || p99 > metadata.max_inference_ms) {
      throw std::runtime_error(metadata.role + "推理运行时p99超过模型时限");
    }
    if (p99_inference_ms != nullptr) {
      *p99_inference_ms = p99;
    }
    if (reason != nullptr) {
      reason->clear();
    }
    return true;
  } catch (const std::exception & exception) {
    if (reason != nullptr) {
      *reason = exception.what();
    }
    return false;
  }
}

std::filesystem::path engine_cache_path(
  const ModelMetadata & metadata,
  const std::string & precision)
{
  // engine 缓存目录使用的 ONNX 摘要短前缀
  const std::string short_hash = metadata.onnx_sha256.substr(0U, 16U);
  return project_root() / "cache" / "car_rl" / short_hash /
         (metadata.role + "_" + precision + ".engine");
}

std::filesystem::path engine_validation_path(
  const ModelMetadata & metadata,
  const std::string & precision)
{
  return engine_cache_path(metadata, precision).string() + ".validated.yaml";
}

void write_engine_validation(
  const ModelMetadata & metadata,
  const std::filesystem::path & engine_path,
  const std::string & precision,
  const std::string & runtime_fingerprint,
  double p99_inference_ms)
{
  if (!std::filesystem::is_regular_file(engine_path)) {
    throw std::runtime_error("无法为不存在的推理引擎写入验证清单");
  }
  if (runtime_fingerprint.empty() || !std::isfinite(p99_inference_ms) ||
    p99_inference_ms < 0.0)
  {
    throw std::runtime_error("推理环境指纹或p99耗时无效");
  }

  const auto manifest = engine_validation_path(metadata, precision);
  const auto temporary = manifest.string() + ".tmp";
  std::filesystem::create_directories(manifest.parent_path());
  // 使用 YAML 发射器处理环境指纹中的特殊字符
  YAML::Emitter yaml;
  yaml << YAML::BeginMap
       << YAML::Key << "schema_version" << YAML::Value << 1
       << YAML::Key << "role" << YAML::Value << metadata.role
       << YAML::Key << "precision" << YAML::Value << precision
       << YAML::Key << "onnx_sha256" << YAML::Value << metadata.onnx_sha256
       << YAML::Key << "engine_sha256" << YAML::Value << sha256_file(engine_path)
       << YAML::Key << "runtime_fingerprint" << YAML::Value << runtime_fingerprint
       << YAML::Key << "p99_inference_ms" << YAML::Value << p99_inference_ms
       << YAML::Key << "input_name" << YAML::Value << metadata.input.name
       << YAML::Key << "output_name" << YAML::Value << metadata.output.name
       << YAML::EndMap;
  if (!yaml.good()) {
    throw std::runtime_error("生成推理引擎验证清单失败");
  }
  // 临时清单写完并关闭后再原子替换正式清单
  std::ofstream stream(temporary, std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("无法写入推理引擎验证清单");
  }
  stream << yaml.c_str() << '\n';
  stream.close();
  if (!stream) {
    std::filesystem::remove(temporary);
    throw std::runtime_error("写入推理引擎验证清单失败");
  }
  std::filesystem::rename(temporary, manifest);
}

bool engine_validation_available(
  const ModelMetadata & metadata,
  const std::string & precision,
  const std::string & runtime_fingerprint,
  std::string * reason,
  double * p99_inference_ms)
{
  try {
    const auto engine = engine_cache_path(metadata, precision);
    const auto manifest = engine_validation_path(metadata, precision);
    if (!std::filesystem::is_regular_file(engine)) {
      throw std::runtime_error(metadata.role + "推理引擎不存在");
    }
    if (!std::filesystem::is_regular_file(manifest)) {
      throw std::runtime_error(metadata.role + "推理引擎尚未经过加载和预热验证");
    }
    const YAML::Node root = YAML::LoadFile(manifest.string());
    if (root["schema_version"].as<int>() != 1 ||
      root["role"].as<std::string>() != metadata.role ||
      root["precision"].as<std::string>() != precision ||
      root["onnx_sha256"].as<std::string>() != metadata.onnx_sha256 ||
      root["runtime_fingerprint"].as<std::string>() != runtime_fingerprint ||
      root["input_name"].as<std::string>() != metadata.input.name ||
      root["output_name"].as<std::string>() != metadata.output.name)
    {
      throw std::runtime_error(metadata.role + "推理引擎验证清单与当前环境不一致");
    }
    if (root["engine_sha256"].as<std::string>() != sha256_file(engine)) {
      throw std::runtime_error(metadata.role + "推理引擎摘要与验证清单不一致");
    }
    const double p99 = root["p99_inference_ms"].as<double>();
    if (!std::isfinite(p99) || p99 < 0.0 || p99 > metadata.max_inference_ms) {
      throw std::runtime_error(metadata.role + "TensorRT推理p99超过模型时限");
    }
    if (p99_inference_ms != nullptr) {
      *p99_inference_ms = p99;
    }
    if (reason != nullptr) {
      reason->clear();
    }
    return true;
  } catch (const std::exception & exception) {
    if (reason != nullptr) {
      *reason = exception.what();
    }
    return false;
  }
}

}  // namespace car_rl
