#include "car_rl/inference_backend.hpp"
#include "car_rl/model_contract.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

// 转义状态原因中的特殊字符，保证标准输出始终是合法 JSON
std::string json_escape(const std::string & value)
{
  // 转义后可安全嵌入 JSON 字符串的结果
  std::string escaped;
  escaped.reserve(value.size());
  // 当前检查并按需转义的字符
  for (const char character : value) {
    switch (character) {
      case '\\': escaped += "\\\\"; break;
      case '"': escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped += character; break;
    }
  }
  return escaped;
}

// 按环境变量和 JetPack 常见目录查找 trtexec
std::filesystem::path find_trtexec()
{
  if (const char * configured = std::getenv("CAR_RL_TRTEXEC")) {
    if (*configured != '\0' && std::filesystem::is_regular_file(configured)) {
      return configured;
    }
  }
  // JetPack 和常见 Linux 安装中的 trtexec 候选路径
  const std::vector<std::filesystem::path> candidates = {
    "/usr/src/tensorrt/bin/trtexec",
    "/usr/local/bin/trtexec",
    "/usr/bin/trtexec"};
  // 当前检查的 trtexec 候选路径
  for (const auto & candidate : candidates) {
    if (std::filesystem::is_regular_file(candidate)) {
      return candidate;
    }
  }
  throw std::runtime_error(
          "未找到模型构建工具，Jetson Orin请安装TensorRT或配置工具路径");
}

// 使用参数数组执行外部进程，避免把模型路径交给 Shell 解析
int run_process(const std::vector<std::string> & arguments)
{
  // execv 要求的可写 C 字符串指针数组
  std::vector<char *> argv;
  argv.reserve(arguments.size() + 1U);
  // 当前转换为 execv 指针的命令参数
  for (const auto & argument : arguments) {
    argv.push_back(const_cast<char *>(argument.c_str()));
  }
  argv.push_back(nullptr);

  // fork 创建的 trtexec 子进程 ID
  const pid_t child = fork();
  if (child < 0) {
    throw std::runtime_error("创建模型构建进程失败");
  }
  if (child == 0) {
    execv(argv[0], argv.data());
    _exit(127);
  }

  // waitpid 返回的原始子进程退出状态
  int status = 0;
  if (waitpid(child, &status, 0) < 0) {
    throw std::runtime_error("等待模型构建进程失败");
  }
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  return 128;
}

// 构建临时 TensorRT engine，验证通过前不覆盖当前可用版本
std::filesystem::path build_temporary_engine(
  const car_rl::ModelMetadata & metadata,
  const std::filesystem::path & trtexec,
  const std::string & precision)
{
  if (precision != "fp16") {
    throw std::runtime_error("当前版本只允许经过验收的半精度模式");
  }
  const std::string role_name = "控制器";
  // 当前模型和精度对应的正式 engine 缓存路径
  const auto engine = car_rl::engine_cache_path(metadata, precision);
  std::filesystem::create_directories(engine.parent_path());
  // engine 完整构建前使用的临时文件路径
  const std::filesystem::path temporary = engine.string() + ".building";
  std::filesystem::remove(temporary);

  // 固定 FP16 构建 TensorRT engine 的 trtexec 参数
  const std::vector<std::string> command = {
    trtexec.string(),
    "--onnx=" + metadata.onnx_path.string(),
    "--saveEngine=" + temporary.string(),
    "--fp16",
    "--skipInference",
    "--builderOptimizationLevel=3",
    "--profilingVerbosity=layer_names_only"};
  std::cout << "正在构建" << role_name << "推理引擎" << std::endl;
  // trtexec 子进程退出码
  const int result = run_process(command);
  if (result != 0 || !std::filesystem::is_regular_file(temporary) ||
    std::filesystem::file_size(temporary) == 0U)
  {
    std::filesystem::remove(temporary);
    throw std::runtime_error("构建" + role_name + "推理引擎失败");
  }
  return temporary;
}

// 实际加载并预热临时引擎，确保张量和当前 Jetson 环境都可用
void validate_temporary_engine(
  const car_rl::ModelMetadata & metadata,
  const std::filesystem::path & engine)
{
  auto backend = car_rl::make_inference_backend();
  backend->load(engine, metadata);
  backend->warmup();
}

// 原子替换正式引擎并写入与当前运行环境绑定的验证清单
void publish_engine(
  const car_rl::ModelMetadata & metadata,
  const std::filesystem::path & temporary,
  const std::string & precision,
  const std::string & runtime_fingerprint)
{
  const auto engine = car_rl::engine_cache_path(metadata, precision);
  std::filesystem::rename(temporary, engine);
  car_rl::write_engine_validation(
    metadata, engine, precision, runtime_fingerprint);
  const std::string role_name = "控制器";
  std::cout << role_name << "推理引擎验证并发布成功：" << engine << std::endl;
}

// 输出模型工具支持的稳定命令行接口
void print_usage()
{
  std::cout <<
    "用法:\n"
    "  model_tool status [--bundle PATH] [--json]\n"
    "  model_tool verify [--bundle PATH]\n"
    "  model_tool build [--bundle PATH] [--precision fp16]\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    print_usage();
    return 2;
  }

  // 当前需要执行的状态、校验或构建动作
  const std::string action = argv[1];
  // 用户指定或默认的模型 bundle 路径
  std::filesystem::path bundle_path;
  // engine 构建精度，首版只接受 fp16
  std::string precision = "fp16";
  // status 是否输出供 Web 解析的 JSON
  bool json = false;
  // 当前解析的命令行参数索引
  for (int index = 2; index < argc; ++index) {
    // 当前命令行参数字符串
    const std::string argument = argv[index];
    if (argument == "--bundle" && index + 1 < argc) {
      bundle_path = argv[++index];
    } else if (argument == "--precision" && index + 1 < argc) {
      precision = argv[++index];
    } else if (argument == "--json") {
      json = true;
    } else {
      std::cerr << "未知参数：" << argument << std::endl;
      return 2;
    }
  }

  try {
    if (precision != "fp16") {
      throw std::runtime_error("当前版本只允许经过验收的半精度模式");
    }
    if (bundle_path.empty()) {
      bundle_path = car_rl::default_bundle_path();
    }
    if (action == "status") {
      // bundle 不可用时返回给用户或 JSON 的原因
      std::string reason;
      if (!car_rl::model_bundle_available(bundle_path, &reason)) {
        if (json) {
          std::cout << "{\"available\":false,\"bundle\":false,"
                    << "\"backend\":false,\"controller_engine\":false,"
                    << "\"controller_engine_valid\":false,"
                    << "\"controller_available\":false,"
                    << "\"reason\":\"" << json_escape(reason) << "\"}" << std::endl;
        } else {
          std::cout << "强化学习模型未安装：" << reason << std::endl;
        }
        return 0;
      }
      // 已严格校验的模型 bundle
      const auto bundle = car_rl::load_model_bundle(bundle_path);
      // 控制器 engine 是否已按当前 ONNX 哈希生成
      const bool controller_engine = std::filesystem::is_regular_file(
        car_rl::engine_cache_path(bundle.controller, precision));
#ifdef CAR_RL_HAS_TENSORRT
      // 当前构建是否包含真实 TensorRT 推理后端
      constexpr bool backend = true;
#else
      // 当前构建没有 TensorRT 时明确报告后端不可用
      constexpr bool backend = false;
#endif
      // 当前运行环境指纹用于拒绝其他 TensorRT、CUDA 或 GPU 生成的引擎
      const auto inference_backend = car_rl::make_inference_backend();
      const std::string runtime_fingerprint =
        inference_backend->runtime_fingerprint();
      // 控制器引擎是否通过当前模型和环境的构建后验证
      std::string controller_reason;
      const bool controller_engine_valid = backend &&
        car_rl::engine_validation_available(
        bundle.controller, precision, runtime_fingerprint, &controller_reason);
      // 启动强化学习局部控制器所需的部署条件
      const bool controller_available = backend && controller_engine_valid;
      // 模型工具总可用状态与控制器可用状态保持一致
      const bool available = controller_available;
      // 返回当前控制器不可用的最直接原因
      const std::string status_reason = !backend ? "当前构建没有TensorRT推理后端" :
        (!controller_engine_valid ? controller_reason : "");
      if (json) {
        std::cout << "{\"available\":" << (available ? "true" : "false")
                  << ",\"bundle\":true"
                  << ",\"backend\":" << (backend ? "true" : "false")
                  << ",\"controller_engine\":" << (controller_engine ? "true" : "false")
                  << ",\"controller_engine_valid\":"
                  << (controller_engine_valid ? "true" : "false")
                  << ",\"controller_available\":"
                  << (controller_available ? "true" : "false")
                  << ",\"bundle_version\":\"" << json_escape(bundle.bundle_version)
                  << "\",\"reason\":\""
                  << json_escape(status_reason)
                  << "\"}" << std::endl;
      } else {
        std::cout << "模型包版本=" << bundle.bundle_version
                  << " 推理后端=" << (backend ? "可用" : "不可用")
                  << " 控制器引擎=" << (controller_engine ? "是" : "否") << std::endl;
      }
      return 0;
    }

    // verify/build 共用的已严格校验模型 bundle
    const auto bundle = car_rl::load_model_bundle(bundle_path);
    if (action == "verify") {
      std::cout << "模型包校验通过：" << bundle.bundle_version << std::endl;
      return 0;
    }
    if (action == "build") {
      // 当前 Jetson 系统上的 trtexec 可执行文件路径
      const auto trtexec = find_trtexec();
      // 临时引擎通过实际加载和预热后才发布
      std::filesystem::path controller_temporary;
      try {
        controller_temporary =
          build_temporary_engine(bundle.controller, trtexec, precision);
        validate_temporary_engine(bundle.controller, controller_temporary);
        // 验证清单绑定当前平台运行环境指纹
        const auto backend = car_rl::make_inference_backend();
        const std::string runtime_fingerprint = backend->runtime_fingerprint();
        publish_engine(
          bundle.controller, controller_temporary, precision, runtime_fingerprint);
      } catch (...) {
        if (!controller_temporary.empty()) {
          std::filesystem::remove(controller_temporary);
        }
        throw;
      }
      return 0;
    }

    print_usage();
    return 2;
  } catch (const std::exception & exception) {
    std::cerr << "强化学习模型工具执行失败：" << exception.what() << std::endl;
    return 1;
  }
}
