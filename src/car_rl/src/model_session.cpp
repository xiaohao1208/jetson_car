#include "car_rl/model_session.hpp"

#include <stdexcept>

namespace car_rl
{

ModelSession::ModelSession()
: backend_(make_inference_backend())
{
}

void ModelSession::load(
  const ModelMetadata & metadata,
  const std::string & precision)
{
  if (!backend_) {
    throw std::runtime_error("强化学习推理后端未创建");
  }
  std::string reason;
  if (backend_->artifact_type() == BackendArtifact::kTensorRtEngine) {
    if (!engine_validation_available(
        metadata, precision, backend_->runtime_fingerprint(), &reason))
    {
      throw std::runtime_error("强化学习推理引擎不可用：" + reason);
    }
    backend_->load(engine_cache_path(metadata, precision), metadata);
    return;
  }
  if (backend_->artifact_type() == BackendArtifact::kOnnxModel) {
    if (!runtime_validation_available(
        metadata, backend_->name(), backend_->runtime_fingerprint(), &reason))
    {
      throw std::runtime_error("强化学习ONNX运行时尚未准备：" + reason);
    }
    backend_->load(metadata.onnx_path, metadata);
    return;
  }
  throw std::runtime_error("当前构建没有可用的强化学习推理后端");
}

void ModelSession::warmup()
{
  if (!backend_) {
    throw std::runtime_error("强化学习推理后端未创建");
  }
  backend_->warmup();
}

bool ModelSession::run(
  const float * input,
  std::size_t input_size,
  float * output,
  std::size_t output_size,
  double & elapsed_ms,
  std::string & error)
{
  if (!backend_) {
    elapsed_ms = 0.0;
    error = "强化学习推理后端未创建";
    return false;
  }
  return backend_->run(
    input, input_size, output, output_size, elapsed_ms, error);
}

bool ModelSession::available() const
{
  return backend_ && backend_->available();
}

std::string ModelSession::backend_name() const
{
  return backend_ ? backend_->name() : "未创建";
}

BackendArtifact ModelSession::artifact_type() const
{
  return backend_ ? backend_->artifact_type() : BackendArtifact::kUnavailable;
}

std::string ModelSession::runtime_fingerprint() const
{
  return backend_ ? backend_->runtime_fingerprint() : "unavailable";
}

}  // namespace car_rl
