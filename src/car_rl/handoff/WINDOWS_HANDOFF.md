# Windows 强化学习离线交接说明

## 1 三台机器的职责

- Ubuntu 开发机维护 ROS 2、ESP32、车体描述和部署契约源码
- Windows 11 训练机只负责 Isaac Lab 仿真、训练、评估和导出 ONNX
- Jetson Orin Nano Super 只负责模型包校验、TensorRT engine 构建和实车部署

三台机器之间不假设存在网络连接，所有交接都通过 ZIP 文件、相邻的 SHA-256 文件和移动存储设备完成

## 2 Windows 解压位置

把 Ubuntu 生成的交接包复制到 Windows 后执行

```powershell
Get-FileHash .\car_rl_windows_handoff.zip -Algorithm SHA256
Get-Content .\car_rl_windows_handoff.zip.sha256
Expand-Archive .\car_rl_windows_handoff.zip -DestinationPath D:\car_rl_handoff -Force
Set-Location D:\car_rl_handoff\car_rl_windows_handoff
.\verify_handoff.ps1
```

`Get-FileHash` 的结果必须与相邻 `.sha256` 文件第一列一致

Windows 上的 AI 和训练代码只能读取解压目录中的文件，不应请求访问 `/home/hao`、ROS 2 工作区、Jetson 或 ESP32 串口

## 3 文件权威级别

- `contract/golden_vectors.json` 是观测构造和动作映射的可执行权威数据
- `source/car_rl` 是模型元数据、模型包校验和部署接口的权威源码快照
- `assets/car_training.urdf` 是本次训练应导入的展开后机器人模型
- `source/car_description/urdf` 用于理解 URDF 来源，不要求 Windows 安装 xacro
- `parameters` 保存与训练和实车速度边界有关的配置快照
- `docs` 保存整体架构和参数说明
- `manifest.yaml` 记录机器职责、契约版本和源文件指纹

Markdown 中的参数表用于阅读，发生冲突时以交接包源码、配置和黄金向量为准

## 4 Windows 环境固定位置

- Isaac Lab 2.3 Git 源码仓库为 `D:\IsaacLab`
- Conda 环境为 `D:\ProgramData\anaconda3\envs\isaaclab`
- 外部训练工程为 `D:\car_rl_training`
- 本交接包解压目录为 `D:\car_rl_handoff\car_rl_windows_handoff`

先执行以下检查

```powershell
& D:\ProgramData\anaconda3\Scripts\activate.ps1 D:\ProgramData\anaconda3\envs\isaaclab
Set-Location D:\IsaacLab
.\isaaclab.bat -p -c "import isaaclab, torch; print(torch.cuda.get_device_name(0))"
```

如果 PowerShell 禁止激活脚本，可以在 Anaconda Prompt 中激活同一环境后执行后续命令

## 5 首个强制验收

在创建环境和开始训练前，先在 Windows 工程中实现与 C++ 完全相同的观测预处理和动作映射，并逐项复算 `contract/golden_vectors.json`

- 观测长度必须为 86
- 动作长度必须为 2
- 100 个用例必须全部通过
- float32 结果的推荐绝对误差不超过 `1e-5`
- 不得通过读取 `expected_observation` 直接作为策略输入来绕过复算

只有该验收通过后，训练结果才可能与 Jetson 上的 Nav2 控制器接口一致

## 6 Windows 最终输出

Windows 只输出便携模型包 ZIP，不输出 TensorRT engine

```text
car_rl_model_bundle/
├── bundle.yaml
├── environment-lock.json
└── controller/
    ├── metadata.yaml
    ├── model.onnx
    └── evaluation.json
```

模型包外再生成一个相邻的 SHA-256 文件

```powershell
Compress-Archive .\car_rl_model_bundle\* .\car_rl_model_bundle.zip -Force
Get-FileHash .\car_rl_model_bundle.zip -Algorithm SHA256 |
  ForEach-Object { "$($_.Hash.ToLower())  car_rl_model_bundle.zip" } |
  Set-Content .\car_rl_model_bundle.zip.sha256 -Encoding ascii
```

把 ZIP 和 `.sha256` 一起复制到 Orin，TensorRT engine 必须在目标 Orin 上重新构建
