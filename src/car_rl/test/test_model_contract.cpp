#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "car_rl/model_contract.hpp"

namespace
{

// 写入模型契约测试所需的小型文本或二进制夹具
void write_file(const std::filesystem::path & path, const std::string & content)
{
  std::filesystem::create_directories(path.parent_path());
  // 当前测试夹具文件的二进制输出流
  std::ofstream stream(path, std::ios::binary);
  stream << content;
}

}  // namespace

TEST(ModelContract, LoadsValidBundleAndChecksShapes)
{
  // 本测试独占的临时模型 bundle 根目录
  const auto root = std::filesystem::temp_directory_path() / "car_rl_contract_test";
  std::filesystem::remove_all(root);
  write_file(root / "controller/model.onnx", "controller-fixture");
  write_file(root / "controller/evaluation.json", "{}");
  write_file(root / "environment-lock.json", "{}");
  // 控制器测试模型内容对应的摘要
  const std::string controller_hash = car_rl::sha256_file(root / "controller/model.onnx");
  write_file(
    root / "controller/metadata.yaml",
    "schema_version: 2\nrole: controller\nmodel_version: test\n"
    "model_file: model.onnx\nonnx_opset: 18\nonnx_sha256: \"" + controller_hash + "\"\n"
    "max_inference_ms: 10\ninput: {name: observation, shape: [1, 86]}\n"
    "output: {name: action, shape: [1, 2]}\n");
  write_file(
    root / "bundle.yaml",
    "schema_version: 2\nbundle_version: test\n"
    "controller_metadata: controller/metadata.yaml\n");

  // 经过完整契约校验后读取到的模型 bundle
  const auto bundle = car_rl::load_model_bundle(root);

  EXPECT_EQ(bundle.controller.input.element_count(), 86U);

  // 将本测试的引擎缓存隔离到临时目录
  const auto cache = root / "cache";
  setenv("XDG_CACHE_HOME", cache.c_str(), 1);
  const auto engine = car_rl::engine_cache_path(bundle.controller);
  write_file(engine, "validated-engine");
  car_rl::write_engine_validation(
    bundle.controller, engine, "fp16", "test-runtime");
  std::string reason;
  EXPECT_TRUE(
    car_rl::engine_validation_available(
      bundle.controller, "fp16", "test-runtime", &reason));
  // 引擎内容变化后摘要不再匹配验证清单
  write_file(engine, "modified-engine");
  EXPECT_FALSE(
    car_rl::engine_validation_available(
      bundle.controller, "fp16", "test-runtime", &reason));
  unsetenv("XDG_CACHE_HOME");
  std::filesystem::remove_all(root);
}

TEST(ModelContract, RejectsMissingBundle)
{
  // bundle 不存在时由校验函数填写的诊断原因
  std::string reason;
  EXPECT_FALSE(car_rl::model_bundle_available("/tmp/car_rl_missing_bundle", &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(ModelContract, RejectsMetadataOutsideBundle)
{
  // 模型清单不得通过相对路径引用模型包外部文件
  const auto root =
    std::filesystem::temp_directory_path() / "car_rl_path_boundary_test";
  std::filesystem::remove_all(root);
  write_file(
    root / "bundle.yaml",
    "schema_version: 2\nbundle_version: test\n"
    "controller_metadata: ../outside.yaml\n");
  EXPECT_THROW(car_rl::load_model_bundle(root), std::runtime_error);
  std::filesystem::remove_all(root);
}

TEST(ModelContract, RejectsLegacyDualModelBundle)
{
  // schema 1双模型包需要重新导出，避免部署端误读旧契约
  const auto root =
    std::filesystem::temp_directory_path() / "car_rl_legacy_bundle_test";
  std::filesystem::remove_all(root);
  write_file(
    root / "bundle.yaml",
    "schema_version: 1\nbundle_version: legacy\n"
    "controller_metadata: controller/metadata.yaml\n"
    "planner_metadata: planner/metadata.yaml\n");
  try {
    (void)car_rl::load_model_bundle(root);
    FAIL() << "旧版双模型包必须被拒绝";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("旧版双模型包"), std::string::npos);
  }
  std::filesystem::remove_all(root);
}
