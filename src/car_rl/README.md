# car_rl

`car_rl` 是小车的强化学习局部控制器部署包。训练在独立 Windows 工程完成，本包只负责校验控制器 ONNX 模型包、在 Jetson Orin Nano Super 上生成 TensorRT FP16 engine，并把模型包装成 Nav2 局部控制器插件。

## 导航边界

```text
全局规划  NavFn
局部控制  DWB或car_rl::Controller
```

强化学习模式只替换 DWB，SLAM、AMCL、NavFn、Nav2 Action、速度平滑器和 `car_move` 安全链保持不变。模型不存在时本包仍可编译，经典 NavFn + DWB 导航不受影响，网页会将 RL 控制按钮置灰。

固定模型契约：

```text
Controller: observation [1,86] -> action [1,2]
dtype:      float32输入输出
ONNX:       opset 18，固定batch和固定shape
```

路径点从机器人到路径折线的最近投影开始，按 0.3、0.6、1.0 m 弧长采样。网络第一个动作映射到 0～0.10 m/s 前进速度，第二个动作映射到 ±1.047197551 rad/s 角速度，不允许模型直接倒车。

## controller-only模型包

模型包固定使用 schema v2：

```text
bundle.yaml
controller/model.onnx
controller/metadata.yaml
controller/evaluation.json
environment-lock.json
```

`bundle.yaml` 和 `controller/metadata.yaml` 可从 `models/templates` 复制。
旧 schema v1 同时包含控制器和规划器，已不再兼容；训练端必须重新导出
controller-only schema v2 包。

## 模型流程

1. Ubuntu 使用 `scripts/export_rl_windows_handoff.sh` 生成带 SHA-256 的训练输入 ZIP
2. Windows 使用交接包中的 URDF、配置和黄金向量完成训练
3. Windows 导出 controller-only schema v2 模型包 ZIP 和相邻 SHA-256
4. Orin 使用 `scripts/import_rl_model_bundle.sh` 先校验再原子安装
5. 只在 Orin 目标机生成 TensorRT engine

Ubuntu 导出示例：

```bash
./scripts/export_rl_windows_handoff.sh \
  --output /media/hao/USB/car_rl_windows_handoff.zip
```

Orin 导入示例：

```bash
./scripts/import_rl_model_bundle.sh \
  --archive /home/seeed/model_transfer/car_rl_model_bundle.zip \
  --verify-only
./scripts/import_rl_model_bundle.sh \
  --archive /home/seeed/model_transfer/car_rl_model_bundle.zip
```

在 Orin 目标机执行：

```bash
source install/setup.bash
ros2 run car_rl model_tool status --json
ros2 run car_rl model_tool verify
ros2 run car_rl model_tool build --precision fp16
```

`contract_tool` 直接调用部署端 C++ 观测和动作实现，导出供 Windows 复算的 100 组黄金向量：

```bash
ros2 run car_rl contract_tool export --output /tmp/golden_vectors.json
```

engine 缓存路径由 ONNX SHA-256 决定：

```text
~/.cache/car_rl/<onnx_sha256前16位>/controller_fp16.engine
```

模型升级后 hash 变化，不会误用旧 engine。Nav2 启动期间不会自动构建 engine。

构建命令先生成临时 engine，再使用当前 TensorRT、CUDA 和 GPU 实际加载并预热，通过后才发布正式文件和验证清单。模型摘要、engine 摘要或运行环境变化后，旧验证清单自动失效。

`status --json` 的稳定字段为：

```text
available                RL局部控制器是否可以启动
bundle                   schema v2模型包是否完整
backend                  当前构建是否包含TensorRT后端
controller_engine        控制器engine文件是否存在
controller_engine_valid  engine是否与模型和当前环境一致
controller_available     RL局部控制器是否可以启动
bundle_version           当前模型版本
reason                   不可用时的中文原因
```

`available` 与 `controller_available` 保持一致。

## 影子模式

经典 Nav2 运行时可以启动：

```bash
ros2 run car_rl shadow_controller
```

影子节点只发布 `/car_rl/shadow_cmd_vel`，不会发布 `/cmd_vel_nav`、`/cmd_vel` 或 `/cmd_vel_move`。

## 平台

- x86 开发机没有 TensorRT 时使用不可用后端，核心测试和经典导航仍可运行
- Jetson Orin Nano Super 8GB 使用 JetPack 6.x、CUDA 和 TensorRT 10.x 构建真实后端
- 当前只允许 FP16，INT8 需要单独校准和安全验收

## 测试

```bash
colcon test --packages-select car_rl
colcon test-result --verbose
```

测试覆盖 86 维观测、路径最近投影、动作映射、schema v2、摘要与路径边界、engine 验证清单和 pluginlib 实例化。TensorRT 后端必须在 Orin 目标机另行编译和运行验证。

训练端必须严格复现 86 维观测顺序、路径采样、动作范围和归一化方式，并使用
部署端导出的黄金向量逐项比对。导出包只有通过元数据、评估结果、环境锁文件、
ONNX 摘要和数值契约检查后才能安装到活动模型目录。
