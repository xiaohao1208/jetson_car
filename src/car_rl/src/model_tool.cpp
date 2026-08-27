#include "car_rl/inference_backend.hpp"
#include "car_rl/inference_benchmark.hpp"
#include "car_rl/model_contract.hpp"
#include "car_rl/model_session.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

constexpr std::size_t kDefaultBenchmarkWarmup = 500U;
constexpr std::size_t kDefaultBenchmarkRuns = 10000U;
constexpr std::size_t kMaximumBenchmarkCount = 10000000U;

struct BenchmarkTimestamp
{
  std::string json_value;
  std::string file_stem;
};

// 同一次基准测试共用一个带本地时区的测量时间和安全文件名。
BenchmarkTimestamp benchmark_timestamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  const std::time_t raw_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  if (localtime_r(&raw_time, &local_time) == nullptr) {
    throw std::runtime_error("读取基准测试时间失败");
  }

  std::ostringstream json_time;
  json_time << std::put_time(&local_time, "%Y-%m-%dT%H:%M:%S")
            << '.' << std::setfill('0') << std::setw(3)
            << milliseconds.count()
            << std::put_time(&local_time, "%z");
  std::ostringstream file_stem;
  file_stem << std::put_time(&local_time, "%Y%m%dT%H%M%S")
            << '_' << std::setfill('0') << std::setw(3)
            << milliseconds.count()
            << std::put_time(&local_time, "%z")
            << '_' << getpid();
  return {json_time.str(), file_stem.str()};
}

// 原子写入每次基准测试的JSON，写入失败时不留下半文件。
std::filesystem::path save_benchmark_result(
  const std::string & json_result, const std::string & file_stem)
{
  const auto output_root = car_rl::benchmark_output_root();
  std::filesystem::create_directories(output_root);
  const auto output = output_root / ("benchmark_" + file_stem + ".json");
  const auto temporary = output.string() + ".tmp";
  try {
    std::ofstream stream;
    stream.exceptions(std::ios::failbit | std::ios::badbit);
    stream.open(temporary, std::ios::out | std::ios::trunc);
    stream << json_result << '\n';
    stream.close();
    std::filesystem::rename(temporary, output);
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    throw;
  }
  return output;
}

// 严格解析基准测试次数，避免负数、尾随字符和过大的内存申请。
std::size_t parse_count(
  const std::string & value, const std::string & option,
  const bool allow_zero)
{
  std::size_t parsed_characters = 0U;
  unsigned long long parsed = 0U;
  try {
    parsed = std::stoull(value, &parsed_characters, 10);
  } catch (const std::exception &) {
    throw std::runtime_error(option + "必须是整数");
  }
  if (parsed_characters != value.size() || (!allow_zero && parsed == 0U)) {
    throw std::runtime_error(
            option +
            (allow_zero ? "必须是非负整数" : "必须是正整数"));
  }
  if (parsed > kMaximumBenchmarkCount ||
    parsed > static_cast<unsigned long long>(
      std::numeric_limits<std::size_t>::max()))
  {
    throw std::runtime_error(option + "不能超过10000000");
  }
  return static_cast<std::size_t>(parsed);
}

// 转义状态原因中的特殊字符，保证标准输出始终是合法 JSON
std::string json_escape(const std::string & value)
{
  // 转义后可安全嵌入 JSON 字符串的结果
  std::string escaped;
  escaped.reserve(value.size());
  // 当前检查并按需转义的字符
  for (const char character : value) {
    switch (character) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += character;
        break;
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
    "/usr/src/tensorrt/bin/trtexec", "/usr/local/bin/trtexec",
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
std::filesystem::path
build_temporary_engine(
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
double validate_temporary_engine(
  const car_rl::ModelMetadata & metadata,
  const std::filesystem::path & engine)
{
  auto backend = car_rl::make_inference_backend();
  backend->load(engine, metadata);
  backend->warmup();
  std::array<float, 86> input{};
  std::array<float, 2> output{};
  std::vector<double> elapsed_values;
  elapsed_values.reserve(1000U);
  for (std::size_t index = 0U; index < 1000U; ++index) {
    double elapsed = 0.0;
    std::string error;
    if (!backend->run(
        input.data(), input.size(), output.data(), output.size(),
        elapsed, error))
    {
      throw std::runtime_error("TensorRT基准推理失败：" + error);
    }
    elapsed_values.push_back(elapsed);
  }
  std::sort(elapsed_values.begin(), elapsed_values.end());
  const std::size_t p99_index =
    static_cast<std::size_t>(std::ceil(0.99 * elapsed_values.size())) - 1U;
  const double p99 = elapsed_values[p99_index];
  if (!std::isfinite(p99) || p99 > metadata.max_inference_ms) {
    throw std::runtime_error("TensorRT推理p99超过模型允许的毫秒上限");
  }
  return p99;
}

// 对直接加载ONNX的电脑后端进行预热和固定次数延迟基准测试
double prepare_onnx_runtime(const car_rl::ModelMetadata & metadata)
{
  auto backend = car_rl::make_inference_backend();
  if (backend->artifact_type() != car_rl::BackendArtifact::kOnnxModel) {
    throw std::runtime_error("当前构建不是ONNX Runtime后端，不能执行prepare");
  }
  backend->load(metadata.onnx_path, metadata);
  backend->warmup();
  std::array<float, 86> input{};
  std::array<float, 2> output{};
  std::vector<double> elapsed_values;
  elapsed_values.reserve(1000U);
  for (std::size_t index = 0U; index < 1000U; ++index) {
    double elapsed = 0.0;
    std::string error;
    if (!backend->run(
        input.data(), input.size(), output.data(), output.size(),
        elapsed, error))
    {
      throw std::runtime_error("ONNX Runtime基准推理失败：" + error);
    }
    elapsed_values.push_back(elapsed);
  }
  std::sort(elapsed_values.begin(), elapsed_values.end());
  const std::size_t p99_index =
    static_cast<std::size_t>(std::ceil(0.99 * elapsed_values.size())) - 1U;
  const double p99 = elapsed_values[p99_index];
  if (!std::isfinite(p99) || p99 > metadata.max_inference_ms) {
    throw std::runtime_error("ONNX Runtime推理p99超过模型允许的毫秒上限");
  }
  car_rl::write_runtime_validation(
    metadata, backend->name(),
    backend->runtime_fingerprint(), p99);
  return p99;
}

// 用控制器实际使用的ModelSession执行预热和固定次数同步推理。
car_rl::InferenceBenchmarkStatistics
run_benchmark(
  const car_rl::ModelMetadata & metadata,
  const std::string & precision, const std::size_t warmup_runs,
  const std::size_t measured_runs, std::string & backend_name,
  std::string & runtime_fingerprint)
{
  car_rl::ModelSession session;
  session.load(metadata, precision);
  // 后端自身先完成CUDA、TensorRT或ONNX Runtime的固定初始化预热。
  session.warmup();
  backend_name = session.backend_name();
  runtime_fingerprint = session.runtime_fingerprint();

  std::vector<float> input(metadata.input.element_count(), 0.0F);
  std::vector<float> output(metadata.output.element_count(), 0.0F);
  auto infer = [&](const std::size_t index) {
      // 输入保持在归一化范围内并逐轮变化，避免基准只覆盖恒定全零输入。
      input[0] =
        static_cast<float>(static_cast<int>(index % 201U) - 100) / 100.0F;
      double elapsed = 0.0;
      std::string error;
      if (!session.run(
          input.data(), input.size(), output.data(), output.size(),
          elapsed, error))
      {
        throw std::runtime_error("基准推理失败：" + error);
      }
      return elapsed;
    };

  for (std::size_t index = 0U; index < warmup_runs; ++index) {
    (void)infer(index);
  }
  std::vector<double> elapsed_values;
  elapsed_values.reserve(measured_runs);
  for (std::size_t index = 0U; index < measured_runs; ++index) {
    elapsed_values.push_back(infer(index + warmup_runs));
  }
  return car_rl::summarize_inference_times(elapsed_values);
}

// 原子替换正式引擎并写入与当前运行环境绑定的验证清单
void publish_engine(
  const car_rl::ModelMetadata & metadata,
  const std::filesystem::path & temporary,
  const std::string & precision,
  const std::string & runtime_fingerprint,
  double p99_inference_ms)
{
  const auto engine = car_rl::engine_cache_path(metadata, precision);
  std::filesystem::rename(temporary, engine);
  car_rl::write_engine_validation(
    metadata, engine, precision,
    runtime_fingerprint, p99_inference_ms);
  const std::string role_name = "控制器";
  std::cout << role_name << "推理引擎验证并发布成功：" << engine << std::endl;
}

// 输出模型工具支持的稳定命令行接口
void print_usage()
{
  std::cout << "用法:\n"
    "  model_tool status [--bundle PATH] [--json]\n"
    "  model_tool verify [--bundle PATH]\n"
    "  model_tool prepare [--bundle PATH]\n"
    "  model_tool build [--bundle PATH] [--precision fp16]\n"
    "  model_tool benchmark [--bundle PATH] [--precision fp16] "
    "[--warmup N] [--runs N] [--json]\n";
}

} // namespace

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
  // benchmark在正式计时前额外执行的预热次数
  std::size_t benchmark_warmup = kDefaultBenchmarkWarmup;
  // benchmark纳入统计的同步推理次数
  std::size_t benchmark_runs = kDefaultBenchmarkRuns;
  try {
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
      } else if (argument == "--warmup" && index + 1 < argc) {
        benchmark_warmup = parse_count(argv[++index], "--warmup", true);
      } else if (argument == "--runs" && index + 1 < argc) {
        benchmark_runs = parse_count(argv[++index], "--runs", false);
      } else {
        std::cerr << "未知参数：" << argument << std::endl;
        return 2;
      }
    }
    if (precision != "fp16") {
      throw std::runtime_error("当前版本只允许经过验收的半精度模式");
    }
    if (bundle_path.empty()) {
      bundle_path = car_rl::default_bundle_path();
    }
    if (action == "status") {
      const auto inference_backend = car_rl::make_inference_backend();
      const auto artifact = inference_backend->artifact_type();
      const bool backend = artifact != car_rl::BackendArtifact::kUnavailable;
      const std::string backend_name = inference_backend->name();
      // bundle 不可用时返回给用户或 JSON 的原因
      std::string reason;
      if (!car_rl::model_bundle_available(bundle_path, &reason)) {
        if (json) {
          std::cout << "{\"available\":false,\"bundle\":false,"
                    << "\"backend\":" << (backend ? "true" : "false")
                    << ",\"backend_name\":\"" << json_escape(backend_name)
                    << "\""
                    << ",\"controller_engine\":false,"
                    << "\"controller_engine_valid\":false,"
                    << "\"controller_runtime_ready\":false,"
                    << "\"runtime_p99_ms\":null,"
                    << "\"controller_available\":false,"
                    << "\"bundle_version\":\"\","
                    << "\"reason\":\"" << json_escape(reason) << "\"}"
                    << std::endl;
        } else {
          std::cout << "强化学习模型未安装，推理后端=" << backend_name << "："
                    << reason << std::endl;
        }
        return 0;
      }
      // 已严格校验的模型 bundle
      const auto bundle = car_rl::load_model_bundle(bundle_path);
      const std::string runtime_fingerprint =
        inference_backend->runtime_fingerprint();
      std::string controller_reason;
      double runtime_p99_ms = 0.0;
      const bool controller_engine =
        artifact == car_rl::BackendArtifact::kTensorRtEngine &&
        std::filesystem::is_regular_file(
        car_rl::engine_cache_path(bundle.controller, precision));
      const bool controller_engine_valid =
        artifact == car_rl::BackendArtifact::kTensorRtEngine &&
        car_rl::engine_validation_available(
        bundle.controller, precision, runtime_fingerprint,
        &controller_reason, &runtime_p99_ms);
      bool controller_runtime_ready = controller_engine_valid;
      if (artifact == car_rl::BackendArtifact::kOnnxModel) {
        controller_runtime_ready = car_rl::runtime_validation_available(
          bundle.controller, backend_name, runtime_fingerprint,
          &controller_reason, &runtime_p99_ms);
      }
      const bool controller_available = backend && controller_runtime_ready;
      // 模型工具总可用状态与控制器可用状态保持一致
      const bool available = controller_available;
      // 返回当前控制器不可用的最直接原因
      const std::string status_reason =
        !backend ? "当前构建没有可用推理后端" :
        (!controller_runtime_ready ? controller_reason : "");
      if (json) {
        std::cout << "{\"available\":" << (available ? "true" : "false")
                  << ",\"bundle\":true"
                  << ",\"backend\":" << (backend ? "true" : "false")
                  << ",\"backend_name\":\"" << json_escape(backend_name) << "\""
                  << ",\"controller_engine\":"
                  << (controller_engine ? "true" : "false")
                  << ",\"controller_engine_valid\":"
                  << (controller_engine_valid ? "true" : "false")
                  << ",\"controller_runtime_ready\":"
                  << (controller_runtime_ready ? "true" : "false")
                  << ",\"runtime_p99_ms\":";
        if (controller_runtime_ready) {
          std::cout << runtime_p99_ms;
        } else {
          std::cout << "null";
        }
        std::cout << ",\"controller_available\":"
                  << (controller_available ? "true" : "false")
                  << ",\"bundle_version\":\""
                  << json_escape(bundle.bundle_version) << "\",\"reason\":\""
                  << json_escape(status_reason) << "\"}" << std::endl;
      } else {
        std::cout << "模型包版本=" << bundle.bundle_version
                  << " 推理后端=" << backend_name
                  << " 运行时就绪=" << (controller_runtime_ready ? "是" : "否")
                  << std::endl;
      }
      return 0;
    }

    // verify/build 共用的已严格校验模型 bundle
    const auto bundle = car_rl::load_model_bundle(bundle_path);
    if (action == "verify") {
      std::cout << "模型包校验通过：" << bundle.bundle_version << std::endl;
      return 0;
    }
    if (action == "prepare") {
      const double p99 = prepare_onnx_runtime(bundle.controller);
      std::cout << "ONNX Runtime准备成功，p99=" << p99 << " ms" << std::endl;
      return 0;
    }
    if (action == "benchmark") {
      std::string backend_name;
      std::string runtime_fingerprint;
      const auto statistics =
        run_benchmark(
        bundle.controller, precision, benchmark_warmup,
        benchmark_runs, backend_name, runtime_fingerprint);
      const bool p99_within_limit =
        statistics.p99_ms <= bundle.controller.max_inference_ms;
      const std::string effective_precision =
        backend_name == "tensorrt" ? precision : "fp32";
      const auto measured_at = benchmark_timestamp();
      std::ostringstream json_stream;
      json_stream << std::setprecision(15)
                  << "{\"measured_at\":\""
                  << json_escape(measured_at.json_value)
                  << "\",\"bundle_version\":\""
                  << json_escape(bundle.bundle_version) << "\",\"backend\":\""
                  << json_escape(backend_name) << "\",\"precision\":\""
                  << json_escape(effective_precision)
                  << "\",\"runtime_fingerprint\":\""
                  << json_escape(runtime_fingerprint)
                  << "\",\"warmup_runs\":" << benchmark_warmup
                  << ",\"measured_runs\":" << statistics.sample_count
                  << ",\"mean_ms\":" << statistics.mean_ms
                  << ",\"p50_ms\":" << statistics.p50_ms
                  << ",\"p95_ms\":" << statistics.p95_ms
                  << ",\"p99_ms\":" << statistics.p99_ms
                  << ",\"max_ms\":" << statistics.max_ms
                  << ",\"contract_limit_ms\":"
                  << bundle.controller.max_inference_ms
                  << ",\"p99_within_contract_limit\":"
                  << (p99_within_limit ? "true" : "false") << '}';
      const std::string json_result = json_stream.str();
      const auto output = save_benchmark_result(
        json_result, measured_at.file_stem);
      std::cout << std::setprecision(15);
      if (json) {
        std::cout << json_result << std::endl;
      } else {
        std::cout << "模型包=" << bundle.bundle_version
                  << " 后端=" << backend_name
                  << " 统计次数=" << statistics.sample_count
                  << " 平均=" << statistics.mean_ms << " ms"
                  << " p50=" << statistics.p50_ms << " ms"
                  << " p95=" << statistics.p95_ms << " ms"
                  << " p99=" << statistics.p99_ms << " ms"
                  << " 最大=" << statistics.max_ms << " ms" << std::endl;
      }
      std::cerr << "基准结果已保存：src/car_rl/benchmark/"
                << output.filename().string() << std::endl;
      return p99_within_limit ? 0 : 1;
    }
    if (action == "build") {
      const auto backend = car_rl::make_inference_backend();
      if (backend->artifact_type() !=
        car_rl::BackendArtifact::kTensorRtEngine)
      {
        throw std::runtime_error(
                "当前构建不是TensorRT后端，电脑ONNX后端请使用prepare");
      }
      // 当前 Jetson 系统上的 trtexec 可执行文件路径
      const auto trtexec = find_trtexec();
      // 临时引擎通过实际加载和预热后才发布
      std::filesystem::path controller_temporary;
      try {
        controller_temporary =
          build_temporary_engine(bundle.controller, trtexec, precision);
        const double p99 =
          validate_temporary_engine(bundle.controller, controller_temporary);
        // 验证清单绑定当前平台运行环境指纹
        const auto backend = car_rl::make_inference_backend();
        const std::string runtime_fingerprint = backend->runtime_fingerprint();
        publish_engine(
          bundle.controller, controller_temporary, precision,
          runtime_fingerprint, p99);
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
