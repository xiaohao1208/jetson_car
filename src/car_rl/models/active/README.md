# 活动模型目录

这里默认不包含模型。Windows 训练完成后，应优先使用工作区脚本校验并原子
导入 controller-only schema v2 模型包，而不是手工覆盖部分文件：

```bash
./scripts/import_rl_model_bundle.sh \
  --archive /path/to/car_rl_model_bundle.zip \
  --verify-only
./scripts/import_rl_model_bundle.sh \
  --archive /path/to/car_rl_model_bundle.zip
```

必须包含：

```text
bundle.yaml
controller/model.onnx
controller/metadata.yaml
controller/evaluation.json
environment-lock.json
```

不要把 TensorRT engine 放入本目录或提交到 Git。模型导入后，在 Jetson
Orin Nano Super 目标机执行：

```bash
source install/setup.bash
ros2 run car_rl model_tool status --json
ros2 run car_rl model_tool verify
ros2 run car_rl model_tool build --precision fp16
```

engine 缓存在用户目录中，并按 ONNX 摘要和运行环境校验；模型变化后不会
复用旧 engine。
