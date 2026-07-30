# car_rl 局部控制器训练与部署实施计划

## 1. 文档目标

本文是 Windows 训练工程的完整实现规格。实施者不需要重新决定输入输出、动作含义、导航边界、算法基线、验收指标或部署格式。

项目使用三台彼此不互通的机器：

| 机器 | 固定目录 | 职责 |
| --- | --- | --- |
| Ubuntu x86_64 开发机 | `/home/hao/ros2_car_gpt` | 维护 ROS 2 和 ESP32 源码，生成 Windows 离线输入包 |
| Windows 11 训练机 | `D:\IsaacLab`、`D:\car_rl_training` | Isaac Lab 仿真、训练、评估和导出 ONNX |
| Jetson Orin Nano Super | `/home/seeed/ros2_car` | 校验模型包、构建 TensorRT engine 和实车部署 |

三台机器之间不得假定存在共享目录、SSH、局域网或相同操作系统环境。每次交接都使用 ZIP、相邻的 SHA-256 文件和移动存储设备完成。

最终系统保持以下边界：

```text
slam_toolbox               负责建图
AMCL                       负责地图定位
NavFn                      负责全局路径
car_rl::Controller         可选的强化学习局部控制器
velocity_smoother          负责速度变化约束
car_move                   负责速度仲裁、急停、超时和近障保护
ESP32                      负责双轮闭环和本地安全停车
```

强化学习只替换 DWB，不训练全局规划器，不生成地图，不接管 AMCL，不绕过 Nav2 或底盘安全链。

## 2. 完成定义

训练工程只有同时满足以下条件才算完成：

1. 能在 Windows、RTX 5080、Isaac Lab 2.3、Isaac Sim 5.1 上从零创建环境并训练 PPO
2. actor 只使用实车运行时可获得的 86 维观测
3. 导出的 ONNX 严格满足 `[1,86] -> [1,2]` controller-only schema v2
4. Python 与 C++ 预处理使用相同顺序、归一化和动作映射
5. 能生成 `model_tool verify` 接受的完整模型包
6. 模型在未见仿真地图达到本文定义的验收指标
7. 模型经过影子模式和分阶段实车安全测试
8. 没有模型或模型失效时，经典 NavFn + DWB 仍可使用

契约测试可以确保模型格式可用，不能单靠仿真绝对保证实车效果。实车可用性由最后的分阶段验收决定。

## 3. 固定软件和硬件环境

### 3.1 训练机

- 操作系统：Windows 11 x64
- GPU：GeForce RTX 5080 16 GB
- 系统内存：32 GB
- Isaac Sim：5.1.0
- Isaac Lab：v2.3.0
- 强化学习库：Isaac Lab v2.3.0 依赖的 RSL-RL 3.0.1
- 基线算法：PPO
- 模型框架：PyTorch
- 导出格式：ONNX opset 18

Isaac Sim 5.1 官方要求将 RTX 5080 列为 Good 配置，Windows 测试驱动为 580.88。开始实施前先运行 Compatibility Checker，驱动不得低于该版本：

- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html
- https://isaac-sim.github.io/IsaacLab/v2.3.0/source/setup/installation/index.html

### 3.2 Windows 安装

固定环境：

- Isaac Lab 2.3 Git 源码仓库：`D:\IsaacLab`
- Conda 环境：`D:\ProgramData\anaconda3\envs\isaaclab`
- 外部训练工程：`D:\car_rl_training`
- Ubuntu 交接包解压位置：`D:\car_rl_handoff\car_rl_windows_handoff`

不重复创建 Isaac Lab 仓库或 Conda 环境。每次开始工作先在 PowerShell 执行：

```powershell
& D:\ProgramData\anaconda3\Scripts\activate.ps1 D:\ProgramData\anaconda3\envs\isaaclab
Set-Location D:\IsaacLab
.\isaaclab.bat -p -c "import isaaclab, torch; print(torch.cuda.get_device_name(0))"
```

如果 PowerShell 执行策略禁止激活脚本，改用 Anaconda Prompt 激活同一路径的环境。不要因此新建第二个环境。

外部项目必须遵循 Isaac Lab 2.3 的扩展结构，首次创建可以使用：

```powershell
Set-Location D:\IsaacLab
.\isaaclab.bat --new
```

在交互选项中选择 external project，输出目录指定为 `D:\car_rl_training`，之后再按本文目录补充文件。

官方参考：

- [Isaac Lab 2.3 源码安装](https://isaac-sim.github.io/IsaacLab/v2.3.0/source/setup/installation/source_installation.html)
- [Isaac Lab 2.3 外部项目结构](https://isaac-sim.github.io/IsaacLab/v2.3.0/source/overview/own-project/project_structure.html)

### 3.3 目标机

- Jetson Orin Nano Super 8GB
- JetPack 6.x
- TensorRT 10.x
- 项目目录固定为 `/home/seeed/ros2_car`
- 构建精度固定为 FP16

TensorRT engine 只在目标机生成，不从 Windows 复制。

### 3.4 离线交接原则

Ubuntu 开发机生成 `car_rl_windows_handoff.zip` 和 `car_rl_windows_handoff.zip.sha256`。交接包包含：

- 完整 `car_rl` 源码、配置、模板和 C++ 测试
- `car_description` xacro 源码和已展开的 `car_training.urdf`
- Nav2、移动控制和 ESP32 关键参数快照
- 由实车端 C++ 实现生成的 100 组黄金向量
- 架构文档、参数文档、文件清单和源文件指纹

Windows 上的 AI 只能依赖解压后的交接包和本机 Isaac Lab，不得请求 Linux 路径、ROS 2 命令或在线访问 Ubuntu。实际文件应放入 ZIP，本文只保留便于审查的参数摘要。若摘要与交接包冲突，以交接包源码、配置和黄金向量为准。

Windows 收到文件后先验证外层 ZIP，再验证解压后的每个文件：

```powershell
$Zip = "E:\car_rl_windows_handoff.zip"
$Expected = (Get-Content "$Zip.sha256").Split()[0].ToLower()
$Actual = (Get-FileHash $Zip -Algorithm SHA256).Hash.ToLower()
if ($Actual -ne $Expected) { throw "交接包 SHA-256 不一致" }
Expand-Archive $Zip -DestinationPath D:\car_rl_handoff -Force
& D:\car_rl_handoff\car_rl_windows_handoff\verify_handoff.ps1
```

校验失败时不要训练，也不要只复制缺失文件修补旧目录，应由 Ubuntu 重新导出完整 ZIP。

## 4. 训练工程目录

不要把 Isaac Lab 训练依赖加入 ROS 2 工作区。

```text
D:\car_rl_training
├─ README.md
├─ pyproject.toml
├─ .gitignore
├─ configs/
│  ├─ robot.yaml
│  ├─ calibration.yaml
│  ├─ ppo.yaml
│  ├─ curriculum.yaml
│  └─ evaluation.yaml
├─ assets/
│  ├─ car_training.urdf
│  ├─ car.usd
│  └─ asset_manifest.json
├─ source/
│  └─ car_rl_training/
│     ├─ config/
│     │  └─ extension.toml
│     ├─ setup.py
│     └─ car_rl_training/
│        ├─ __init__.py
│        ├─ tasks/
│        │  └─ local_navigation/
│        │     ├─ __init__.py
│        │     ├─ env_cfg.py
│        │     ├─ scene_cfg.py
│        │     ├─ observations.py
│        │     ├─ actions.py
│        │     ├─ commands.py
│        │     ├─ rewards.py
│        │     ├─ terminations.py
│        │     ├─ curriculum.py
│        │     ├─ events.py
│        │     └─ agents/rsl_rl_ppo_cfg.py
│        ├─ models/
│        │  ├─ registry.py
│        │  ├─ flat_mlp.py
│        │  ├─ split_fusion.py
│        │  └─ actor_critic.py
│        ├─ contract/
│        │  ├─ observation.py
│        │  ├─ action.py
│        │  ├─ schema.py
│        │  └─ golden_vectors.py
│        ├─ maps/
│        │  ├─ generator.py
│        │  ├─ path_search.py
│        │  └─ path_smoothing.py
│        └─ export/
│           ├─ export_onnx.py
│           ├─ evaluate.py
│           ├─ make_bundle.py
│           └─ validate_bundle.py
├─ scripts/
│  ├─ benchmark_envs.py
│  ├─ train_all_seeds.py
│  ├─ play_checkpoint.py
│  ├─ export_best.py
│  └─ record_calibration.py
└─ tests/
   ├─ test_observation_contract.py
   ├─ test_action_contract.py
   ├─ test_path_sampling.py
   ├─ test_environment_reset.py
   ├─ test_reward_terms.py
   ├─ test_onnx_contract.py
   ├─ test_bundle_schema.py
   └─ golden/
```

安装外部项目：

```powershell
& D:\ProgramData\anaconda3\Scripts\activate.ps1 D:\ProgramData\anaconda3\envs\isaaclab
python -m pip install -e D:\car_rl_training\source\car_rl_training
```

Gymnasium 任务 ID 固定为：

```text
Car-LocalNavigation-v0
Car-LocalNavigation-Play-v0
```

`Play` 配置只减少环境数并关闭训练随机扰动，不改变 86 维观测或动作映射。

## 5. 实车契约

训练端不得自行修改本节。若以后需要 contract v3，必须先同时修改 C++、模板、测试和本文。

### 5.1 控制周期

- 物理仿真频率：120 Hz
- 仿真步长：`1 / 120 s`
- policy decimation：8
- policy 与 Nav2 控制频率：15 Hz
- 每个 episode 最大时长：30 s
- 最大 policy step：450
- 雷达刷新频率：5 Hz
- 同一雷达帧连续保持 3 个 policy step

### 5.2 固定物理和控制参数

参数来源：

- `D:\car_rl_handoff\car_rl_windows_handoff\source\car_description`
- `D:\car_rl_handoff\car_rl_windows_handoff\parameters\jetson_car\nav2_params.yaml`
- `D:\car_rl_handoff\car_rl_windows_handoff\parameters\jetson_car\move.yaml`
- `D:\car_rl_handoff\car_rl_windows_handoff\parameters\esp32_car\car_config.hpp`
- `D:\car_rl_handoff\car_rl_windows_handoff\contract\golden_vectors.json`

模型包 `schema_version: 2` 表示部署文件格式版本，86 维观测使用独立的 `observation_contract_version: 1`。两者不是同一版本号，训练工程必须分别记录。

初始固定值：

```yaml
geometry:
  wheel_radius_m: 0.032
  wheel_separation_m: 0.175
  footprint_m:
    - [0.10, 0.108]
    - [0.10, -0.108]
    - [-0.10, -0.108]
    - [-0.10, 0.108]
  footprint_padding_m: 0.01

controller:
  frequency_hz: 15.0
  max_linear_speed_mps: 0.10
  max_angular_speed_radps: 1.047197551
  min_nonzero_angular_speed_radps: 0.523598776
  max_linear_accel_mps2: 0.6
  max_linear_decel_mps2: 0.8
  max_angular_accel_radps2: 1.5
  max_angular_decel_radps2: 2.0

lidar:
  ray_count: 72
  min_range_m: 0.05
  max_range_m: 6.0
  update_rate_hz: 5.0

navigation:
  xy_goal_tolerance_m: 0.12
  yaw_goal_tolerance_rad: 0.174532925
  local_inflation_radius_m: 0.15
  progress_radius_m: 0.04
  progress_timeout_sec: 5.0
```

### 5.3 86 维观测

输入张量：

```text
name   observation
dtype  float32
shape  [1, 86]
```

严格顺序：

| 索引 | 内容 | 归一化 |
| --- | --- | --- |
| 0～71 | 72 束雷达 | `(clamp(range,0.05,6.0)-0.05)/5.95` |
| 72～77 | 0.3、0.6、1.0 m 三个路径前视点的 base x/y | 各轴 clamp 到 ±1.5 m 后除以 1.5 |
| 78 | 目标相对 x | clamp 到 ±3.0 m 后除以 3.0 |
| 79 | 目标相对 y | clamp 到 ±3.0 m 后除以 3.0 |
| 80 | 目标 yaw 误差 sin | `sin(yaw_error)` |
| 81 | 目标 yaw 误差 cos | `cos(yaw_error)` |
| 82 | 当前线速度 | clamp 到 ±0.25 m/s 后除以 0.25 |
| 83 | 当前角速度 | clamp 到 ±1.0 rad/s |
| 84 | 上一动作线速度分量 | clamp 到 ±1 |
| 85 | 上一动作角速度分量 | clamp 到 ±1 |

雷达目标角度：

\[
\theta_i=-\pi+\frac{2\pi i}{72},\quad i=0,\ldots,71
\]

索引 36 是车头方向。NaN、Inf、小于最小量程和扫描范围外数据按最大量程处理。

路径采样不能简单选择最近离散路径点。必须先找到机器人在整条路径折线上的最近投影，再从该投影沿路径弧长前进 0.3、0.6、1.0 m。路径不足时使用终点。

训练工程中的 `contract/observation.py` 必须逐语句对应：

```text
D:\car_rl_handoff\car_rl_windows_handoff\source\car_rl\src\observation.cpp
```

### 5.4 两维动作

输出张量：

```text
name   action
dtype  float32
shape  [1, 2]
range  [-1, 1]
```

运行时映射：

\[
v=0.5\times0.10\times(clamp(a_0,-1,1)+1)
\]

\[
\omega=1.047197551\times clamp(a_1,-1,1)
\]

因此：

- `a0=-1` 表示停车
- `a0=0` 表示 0.05 m/s
- `a0=1` 表示 0.10 m/s
- 不允许负线速度
- `a1=-1` 和 `a1=1` 分别表示最大左右旋转

训练环境也必须使用相同映射。不能在仿真中把 `a0=0` 当成停车。

## 6. 机器人仿真资产

### 6.1 生成展开后的 URDF

当前 xacro 用于 TF 和 RViz，质量均为占位值。Windows 不安装 ROS 2 或 xacro，也不自行展开机器人描述。

在 Ubuntu 开发机生成完整离线交接包：

```bash
cd /home/hao/ros2_car_gpt/jetson_car
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/export_rl_windows_handoff.sh \
  --output /media/hao/USB/car_rl_windows_handoff.zip
```

把 ZIP 和相邻 `.sha256` 一起复制到移动存储设备。Windows 解压后使用：

```text
D:\car_rl_handoff\car_rl_windows_handoff\assets\car_training.urdf
```

复制到训练工程时保留来源摘要：

```powershell
Copy-Item `
  D:\car_rl_handoff\car_rl_windows_handoff\assets\car_training.urdf `
  D:\car_rl_training\assets\car_training.urdf
Get-FileHash D:\car_rl_training\assets\car_training.urdf -Algorithm SHA256
```

该摘要写入 `asset_manifest.json`。更新车体源码后必须在 Ubuntu 重新生成整个交接包，不能只替换单个 URDF。

### 6.2 导入 USD

使用 Isaac Sim 5.1 URDF Importer：

1. 固定基座关闭
2. 合并固定关节关闭
3. 左右轮连续关节保留
4. 前后万向轮在首版保持固定几何
5. self collision 关闭
6. 将根节点设置为 articulation root
7. 保存为 `assets/car.usd`

导入后自动测试：

- 左右轮 joint 名分别为 `left_wheel_joint` 和 `right_wheel_joint`
- 轮轴方向正确
- 车头为 base x 正方向
- 雷达平面位于 `laser_link`
- footprint 与项目一致
- 给左右轮相同正速度时车辆向前
- 给左负右正速度时车辆逆时针旋转

### 6.3 质量与摩擦

URDF 中每个部件的 1.0 kg 只用于显示，不得直接作为物理真值。

训练分两阶段：

#### Bootstrap阶段

在没有完整实测参数前使用宽范围：

```yaml
bootstrap_randomization:
  total_mass_kg: [1.0, 4.0]
  wheel_static_friction: [0.5, 1.2]
  wheel_dynamic_friction: [0.4, 1.0]
  rolling_friction: [0.005, 0.03]
  motor_time_constant_sec: [0.05, 0.25]
  action_delay_sec: [0.0, 0.15]
  wheel_radius_scale: [0.97, 1.03]
  wheel_separation_scale: [0.97, 1.03]
  left_right_motor_gain_ratio: [0.90, 1.10]
```

这些是训练启动范围，不是实车测量结论。

#### Calibrated阶段

实车记录以下实验：

1. 称量整车质量
2. 5、10 cm/s 各做 10 次直线阶跃
3. 30、60 °/s 各做 10 次原地旋转阶跃
4. 记录命令时间、左右轮目标、左右轮反馈、编码器 tick 和 IMU yaw
5. 分别计算起步延迟、时间常数、最大加速度、制动时间和左右轮增益

结果写入 `configs/calibration.yaml`：

```yaml
schema_version: 1
measured_at: "YYYY-MM-DD"
total_mass_kg: null
motor_time_constant_sec: null
action_delay_sec: null
left_motor_gain: null
right_motor_gain: null
linear_accel_mps2: null
linear_decel_mps2: null
angular_accel_radps2: null
angular_decel_radps2: null
wheel_friction_center: null
source_log_sha256: null
```

生产模型导出脚本发现任一字段为 `null` 时必须拒绝生成 production bundle。校准后随机范围以实测中心为基准：

- 质量 ±15%
- 摩擦 ±25%
- 动作时延 ±1 个 policy step
- 左右电机增益 ±10%
- 轮径与轮距 ±3%

## 7. 场景和全局路径

### 7.1 程序化地图

每个环境生成独立二维房间：

- 地图分辨率 0.05 m
- 初始尺寸 6 m × 6 m
- 边界必须封闭
- 通道宽度 0.35～1.20 m
- 障碍物使用矩形、圆柱和 L 形组合
- 起点和终点最短路径长度 1～8 m
- 起终点距离障碍至少 0.20 m
- 训练、验证和测试使用不同随机种子

固定集合：

```text
train seeds       0～9999
validation seeds  10000～10999
test seeds        20000～20999
```

测试集合的地图参数和随机种子不得用于调奖励。

### 7.2 路径生成

Isaac Lab 环境内不启动 ROS 2。使用训练工程自己的确定性路径生成器模拟 NavFn 输出：

1. 根据 footprint 和 0.15 m 膨胀半径构造规划栅格
2. 使用八邻域 A*，禁止对角穿过障碍角点
3. 对路径做视线简化
4. 使用迭代平滑，保持路径点不穿过膨胀障碍
5. 按不大于 0.05 m 间隔重采样
6. 路径终点 orientation 使用目标 yaw

训练目的不是复刻 NavFn 内部搜索顺序，而是让局部控制器适应 Nav2 能产生的折线、转弯和重规划路径。

### 7.3 动态重规划

课程后期每 1～3 s 重新生成一次全局路径副本，加入以下变化：

- 路径点间隔改变
- 路径局部小幅横向变化
- 新动态障碍导致绕行
- 机器人已沿路径前进后删除身后路径

观测始终从机器人在最新折线上的最近投影开始，避免路径点密度变化导致输入跳变。

## 8. 传感器和执行器建模

### 8.1 雷达

使用 RayCaster 或等价 GPU 射线传感器，72 束角度直接匹配 contract。

基础噪声：

```yaml
lidar_randomization:
  gaussian_std_m: [0.0, 0.02]
  bias_m: [-0.01, 0.01]
  invalid_ray_probability: [0.0, 0.03]
  whole_frame_drop_probability: [0.0, 0.02]
  range_scale: [0.99, 1.01]
```

丢失单束时按 6.0 m 处理。整帧丢失时保持上一帧，超过 0.5 s 结束 episode 并记录输入超时。

### 8.2 定位与路径噪声

actor 不能读取仿真绝对真值作为额外输入。用于构造观测的机器人位姿加入：

```yaml
localization_randomization:
  position_std_m: [0.0, 0.03]
  yaw_std_rad: [0.0, 0.035]
  slow_position_drift_mps: [0.0, 0.005]
  slow_yaw_drift_radps: [0.0, 0.005]
```

碰撞、成功和评估指标使用真值，actor 观测使用带噪定位值。

### 8.3 轮速执行

动作先映射为 \(v,\omega\)，再转换为轮速：

\[
\dot q_l=\frac{v-\omega L/2}{r}, \qquad
\dot q_r=\frac{v+\omega L/2}{r}
\]

通过一阶响应、动作延迟、加速度限制和左右增益后送到轮关节 velocity target。不能直接瞬时设置 base twist，否则无法训练对电机滞后和打滑的鲁棒性。

## 9. ManagerBasedRLEnv设计

### 9.1 Scene

每个并行环境包含：

- 一台 `car.usd`
- 静态地面和墙体
- 0～20 个静态障碍物
- 课程后期 0～4 个动态障碍物
- 72 束雷达
- 接触传感器
- 独立地图、起点、终点和路径缓存

32 GB 内存默认 `num_envs=256`。运行 `scripts/benchmark_envs.py` 分别测试 128、256、512：

- GPU 峰值显存小于 85%
- 系统内存峰值小于 85%
- 仿真实际速度不低于实时 5 倍

只有三项都满足才允许使用更大环境数，最大不超过 512。

### 9.2 Observation

Policy group 只输出 86 维且禁止自动拼接其他项。Critic 基线也使用同一 86 维，首版不使用 privileged observation。

启动时断言：

```python
assert policy_observation.shape[-1] == 86
assert torch.isfinite(policy_observation).all()
assert policy_observation.dtype == torch.float32
```

### 9.3 Action

Action manager 接收两个归一化动作，先 clamp 到 `[-1,1]`，再执行本文固定映射。环境信息中同时记录：

- 原始网络输出
- clamp 后动作
- 映射后的 \(v,\omega\)
- 左右轮目标
- 左右轮反馈

这些字段只用于训练诊断，不加入 actor 观测。

### 9.4 Command

每个 episode 随机生成起点、目标 yaw 和全局路径。重置时保证：

- 起点和目标均可通行
- 存在有效全局路径
- 机器人 footprint 不与障碍重叠
- 初始雷达和路径观测均为有限值

若 100 次采样仍无法生成有效任务，抛出配置错误，不能无限循环。

## 10. 奖励、终止和课程

### 10.1 奖励变量

- \(\Delta s\)：当前 step 沿全局路径的前向进度，限制到 `[-0.05, 0.05] m`
- \(e_y\)：到路径最近投影的横向距离，限制到 `[0,1] m`
- \(e_\psi\)：机器人朝向与当前路径切线夹角
- \(d_{min}\)：最小雷达距离
- \(a_t\)：当前归一化动作
- \(g\)：到目标的欧氏距离

### 10.2 每步奖励

\[
\begin{aligned}
r_t={}&8.0\Delta s
-0.8e_y^2
-0.25(1-\cos e_\psi)\\
&-0.30\left[\max\left(0,\frac{0.35-d_{min}}{0.35}\right)\right]^2\\
&-0.03\lVert a_t-a_{t-1}\rVert_2^2
-0.01a_{t,1}^2
-0.02I(v<0.01\land g>0.25)
\end{aligned}
\]

终止奖励：

```text
到达目标       +12
发生碰撞       -12
离开路径超过1m -3
连续5秒进度不足 -5
30秒超时       -1
输入出现NaN     -12并计为环境错误
```

成功条件：

- 与目标距离不超过 0.12 m
- yaw 误差绝对值不超过 0.174532925 rad
- 连续 5 个 policy step 满足条件

停滞条件与实车 Nav2 一致：

- 5 s 窗口内移动距离小于 0.04 m
- 目标距离仍大于 0.12 m

### 10.3 课程阶段

| 阶段 | 场景 | 晋级条件 |
| --- | --- | --- |
| 0 | 无障碍直线，关闭噪声 | 3次评估成功率≥95% |
| 1 | 单弯、S弯和不同目标yaw | 3次评估成功率≥92% |
| 2 | 静态障碍和窄通道 | 成功率≥90%，碰撞率≤2% |
| 3 | 复杂地图和路径重规划 | 成功率≥90%，碰撞率≤2% |
| 4 | 雷达、定位、动作时延随机化 | 成功率≥90%，碰撞率≤2% |
| 5 | 动态障碍和完整物理随机化 | 成功率≥90%，碰撞率≤2% |

每次评估使用 500 个 validation episode。失败时保持当前阶段继续训练，不能自动降低指标。

## 11. PPO基线

### 11.1 固定超参数

```yaml
algorithm: PPO
runner: OnPolicyRunner
num_steps_per_env: 24
max_iterations: 3000
save_interval: 50
seed: 42

ppo:
  learning_rate: 0.0003
  schedule: adaptive
  gamma: 0.99
  lam: 0.95
  clip_param: 0.2
  entropy_coef: 0.01
  value_loss_coef: 1.0
  use_clipped_value_loss: true
  desired_kl: 0.01
  max_grad_norm: 1.0
  num_learning_epochs: 5
  num_mini_batches: 4

policy:
  actor_hidden_dims: [256, 256, 128]
  critic_hidden_dims: [256, 256, 128]
  activation: elu
  init_noise_std: 0.6
```

训练种子固定为 42、43、44。三次训练都要保留，不能只报告最好的一次。

### 11.2 基线网络

`FlatMlpPolicy`：

```text
observation 86
  -> Linear 256 + ELU
  -> Linear 256 + ELU
  -> Linear 128 + ELU
  -> Linear 2
  -> tanh
```

critic 使用独立同尺寸 MLP，输出一个 value。

### 11.3 自定义模型接口

所有模型必须注册到：

```python
MODEL_REGISTRY: dict[str, type[PolicyEncoder]]
```

接口：

```python
class PolicyEncoder(torch.nn.Module):
    output_dim: int

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        ...
```

actor head 始终是：

```python
action = torch.tanh(actor_head(encoder(observation)))
```

提供两个实现：

1. `flat_mlp`：作为固定对照组
2. `split_fusion`：
   - 雷达 72 维经过 `128 -> 64`
   - 路径、目标、速度和上一动作 14 维经过 `64 -> 64`
   - 拼接后经过 `128 -> 128`
   - actor 输出 2 维

新增自己的网络只允许替换 encoder 和 actor/critic 隐藏层，不能改变：

- 86 维顺序
- 两维动作含义
- 动作 tanh
- ONNX 名称与 shape
- 训练和测试集合
- 导出与验收门槛

比较自定义网络时使用相同三个种子、相同 curriculum、相同训练 step 和相同 test 集，报告均值与标准差。

## 12. 训练和评估命令

### 12.1 单次训练

先激活固定 Conda 环境并安装外部工程：

```powershell
& D:\ProgramData\anaconda3\Scripts\activate.ps1 D:\ProgramData\anaconda3\envs\isaaclab
python -m pip install -e D:\car_rl_training\source\car_rl_training
Set-Location D:\IsaacLab
python scripts\reinforcement_learning\rsl_rl\train.py `
  --task Car-LocalNavigation-v0 `
  --num_envs 256 `
  --headless `
  --seed 42 `
  --run_name flat_mlp_seed42
```

### 12.2 三种子训练

```powershell
python D:\car_rl_training\scripts\train_all_seeds.py `
  --model flat_mlp `
  --seeds 42 43 44 `
  --num_envs 256
```

脚本按顺序运行，避免 32 GB 内存同时启动多个 Isaac Sim。

### 12.3 可视化

```powershell
python D:\car_rl_training\scripts\play_checkpoint.py `
  --run D:\car_rl_training\logs\flat_mlp_seed42 `
  --num_envs 16
```

### 12.4 独立测试

每个候选 checkpoint 在 1000 个 test episode 上运行，测试随机种子固定为：

```text
20000～20999
```

输出至少包含：

- success_rate
- collision_rate
- timeout_rate
- stall_rate
- mean_cross_track_error_m
- p95_cross_track_error_m
- mean_goal_position_error_m
- mean_goal_yaw_error_rad
- mean_episode_time_sec
- mean_action_delta
- minimum_clearance_m

同一模型三种子取均值和标准差。选择规则按以下优先级：

1. collision_rate 最低
2. success_rate 最高
3. p95_cross_track_error 最低
4. mean_action_delta 最低

## 13. 导出与controller-only schema v2

### 13.1 ONNX导出

导出 wrapper 只保留 deterministic actor：

```python
class ExportedController(torch.nn.Module):
    def __init__(self, actor):
        super().__init__()
        self.actor = actor

    def forward(self, observation):
        return torch.clamp(self.actor(observation), -1.0, 1.0)
```

固定导出参数：

```python
torch.onnx.export(
    exported_controller,
    torch.zeros(1, 86, dtype=torch.float32),
    output_path,
    input_names=["observation"],
    output_names=["action"],
    opset_version=18,
    dynamic_axes=None,
    do_constant_folding=True,
)
```

导出前调用 `eval()` 并移动到 CPU。模型中不得包含随机采样、训练噪声或 critic。

### 13.2 数值一致性

生成 10,000 个观测：

- 4000 个真实 test episode 观测
- 3000 个边界值观测
- 3000 个随机合法观测

比较 PyTorch 与 ONNX Runtime：

```text
最大绝对误差 ≤ 1e-5
平均绝对误差 ≤ 1e-6
所有输出有限
所有输出在[-1,1]
```

不满足时禁止打包。

### 13.3 模型包

输出目录：

```text
car_rl_bundle_v2_<version>/
├─ bundle.yaml
├─ controller/
│  ├─ model.onnx
│  ├─ metadata.yaml
│  └─ evaluation.json
└─ environment-lock.json
```

`bundle.yaml`：

```yaml
schema_version: 2
bundle_version: "YYYYMMDD-model-seed"
controller_metadata: controller/metadata.yaml
```

`controller/metadata.yaml`：

```yaml
schema_version: 2
role: controller
model_version: "YYYYMMDD-model-seed"
model_file: model.onnx
onnx_opset: 18
onnx_sha256: "64字符小写SHA256"
max_inference_ms: 10.0
input:
  name: observation
  shape: [1, 86]
output:
  name: action
  shape: [1, 2]
```

`evaluation.json` 至少包含：

```json
{
  "schema_version": 1,
  "model_version": "YYYYMMDD-model-seed",
  "algorithm": "PPO",
  "network": "flat_mlp",
  "training_seeds": [42, 43, 44],
  "test_seed_range": [20000, 20999],
  "episodes": 1000,
  "success_rate": 0.0,
  "collision_rate": 0.0,
  "timeout_rate": 0.0,
  "stall_rate": 0.0,
  "mean_cross_track_error_m": 0.0,
  "p95_cross_track_error_m": 0.0,
  "pytorch_onnx_max_abs_error": 0.0,
  "checkpoint_sha256": "",
  "task_config_sha256": ""
}
```

`environment-lock.json` 至少包含：

- Windows 版本
- GPU 名称和驱动版本
- Isaac Sim 版本
- Isaac Lab commit
- RSL-RL、PyTorch、CUDA、ONNX、ONNX Runtime 版本
- 训练工程 commit
- Ubuntu 交接包 `manifest.yaml` 中的 `source_fingerprint_sha256`
- 机器人参数和校准文件 SHA-256
- curriculum、PPO 和场景配置 SHA-256
- 导出时间

## 14. 跨语言黄金测试

黄金输入由 Ubuntu 交接脚本调用当前 `car_rl` C++ 运行库生成，Windows 不反向生成期望值：

```text
原始LaserScan几何与ranges
全局Path
机器人Pose
当前Twist
上一动作
期望86维观测
期望Twist动作映射
```

固定文件为：

```text
D:\car_rl_handoff\car_rl_windows_handoff\contract\golden_vectors.json
```

Windows 必须根据 JSON 中的原始 Scan、Path、Pose、Twist、上一动作和测试动作自行复算，再与 `expected_observation` 和 `expected_twist` 比较。禁止直接把期望观测作为策略输入来绕过契约实现。

两个测试入口：

- Windows `D:\car_rl_training\tests\test_observation_contract.py`
- Ubuntu 交接包中的 `source\car_rl\test\test_observation.cpp`

允许误差：

```text
观测绝对误差 ≤ 1e-5
动作映射绝对误差 ≤ 1e-8
```

交接 JSON 固定提供 100 个正常且数值有限的跨语言复算用例。以下异常和边界继续由交接包中的 C++ 测试以及 Windows 对应单元测试覆盖：

- 72 束标准扫描
- 389、390、391、392 束源扫描
- NaN、Inf、低于最小量程、超过最大量程
- 路径只有一个点
- 路径重复点
- 机器人位于路径段中间
- 路径不足 1 m
- 目标位于车后
- yaw 跨越 ±π
- 动作超过 `[-1,1]`

## 15. 仿真验收门槛

生产候选模型必须同时满足：

```text
1000个未见episode成功率       ≥ 95%
碰撞率                       ≤ 1%
停滞率                       ≤ 2%
p95横向误差                  ≤ 0.10 m
平均终点位置误差             ≤ 0.08 m
平均终点yaw误差              ≤ 0.14 rad
PyTorch与ONNX最大绝对误差     ≤ 1e-5
Windows ONNX单次推理p99       ≤ 10 ms
```

任何指标未达到都只能作为实验模型，不能复制到 `models/active`。

## 16. Orin 离线导入与构建

### 16.1 Windows 输出

Windows 只生成 ONNX 模型包，不生成或复制 TensorRT engine：

```powershell
Set-Location D:\car_rl_training\exports
Compress-Archive .\car_rl_bundle_v2_<version>\* .\car_rl_model_bundle.zip -Force
Get-FileHash .\car_rl_model_bundle.zip -Algorithm SHA256 |
  ForEach-Object { "$($_.Hash.ToLower())  car_rl_model_bundle.zip" } |
  Set-Content .\car_rl_model_bundle.zip.sha256 -Encoding ascii
```

把 `car_rl_model_bundle.zip` 和 `car_rl_model_bundle.zip.sha256` 一起复制到移动存储设备。

### 16.2 Orin 校验和安装

先把两个文件复制到 Orin 本地目录，例如 `/home/seeed/model_transfer`，再执行：

```bash
cd /home/seeed/ros2_car
./jetson_car/scripts/import_rl_model_bundle.sh \
  --archive /home/seeed/model_transfer/car_rl_model_bundle.zip \
  --verify-only
./jetson_car/scripts/import_rl_model_bundle.sh \
  --archive /home/seeed/model_transfer/car_rl_model_bundle.zip
```

导入脚本依次检查 ZIP 摘要、危险路径、唯一 `bundle.yaml`、ONNX 摘要和固定张量契约。正式安装前使用 `--verify-only` 不会修改 `models/active`。安装时旧模型保留为时间戳备份，不能在 Windows 端伪造验证结果。

### 16.3 Orin 构建

```bash
cd /home/seeed/ros2_car/jetson_car
source /opt/ros/humble/setup.bash
colcon build --packages-select car_rl
source install/setup.bash
ros2 run car_rl model_tool status --bundle \
  /home/seeed/ros2_car/jetson_car/src/car_rl/models/active --json
ros2 run car_rl model_tool verify --bundle \
  /home/seeed/ros2_car/jetson_car/src/car_rl/models/active
ros2 run car_rl model_tool build --bundle \
  /home/seeed/ros2_car/jetson_car/src/car_rl/models/active --precision fp16
ros2 run car_rl model_tool status --bundle \
  /home/seeed/ros2_car/jetson_car/src/car_rl/models/active --json
```

构建后要求：

```text
bundle=true
backend=true
controller_engine=true
controller_engine_valid=true
controller_available=true
available=true
```

TensorRT FP16 与 ONNX 的 10,000 组观测误差要求：

```text
最大绝对误差 ≤ 1e-3
所有输出有限
所有输出经运行时clamp后位于[-1,1]
Orin推理p99 ≤ 10 ms
```

## 17. 实车分阶段验收

### 17.1 离线日志回放

用经典导航采集的 `/scan`、`/plan`、`/odom` 和 TF 回放模型，检查：

- 观测始终有限
- 没有雷达或 TF 超时误判
- 输出无高频正负旋转抖动
- 停车附近 `a0` 能接近 -1

### 17.2 影子模式

运行经典导航和：

```bash
ros2 run car_rl shadow_controller
```

确认影子节点只发布 `/car_rl/shadow_cmd_vel`，不会改变底盘运动。记录经典速度与影子速度，但不要求两者数值相同。

### 17.3 架空轮测试

车轮离地，切换 RL 控制：

- 急停立即让左右轮归零
- 雷达断开后控制器停止
- 模型异常后 Nav2 任务失败而不是保持最后速度
- 左右转方向正确
- 最大速度不超过项目限制

### 17.4 围栏低速测试

在无人员的封闭区域完成：

1. 1 m 直线路径 20 次
2. 左右 90° 转弯各 20 次
3. S 弯 20 次
4. 静态障碍绕行 20 次
5. 目标附近最终旋转 20 次

任一碰撞、失控或急停失败立即停止测试并回到经典模式。

### 17.5 地图导航测试

至少两张未用于校准的真实地图：

```text
每张地图单点导航        ≥ 30次
每张地图多点任务        ≥ 10组
每组目标点              ≥ 5个
总体成功率              ≥ 90%
接触碰撞                0次
急停失效                0次
最终位置平均误差        ≤ 0.12m
最终yaw平均误差         ≤ 0.174532925rad
```

多点任务继续使用现有 FollowWaypoints，单个不可达点由 Nav2 标红并继续后续点。

## 18. 回退和发布

发布前保留：

- 上一个已验收模型包
- 当前经典导航配置
- 本次训练、评估和实车日志
- calibration.yaml
- bundle SHA-256

出现以下任一情况立即回退经典模式：

- controller engine 验证失效
- p99 推理超过 10 ms
- 连续输入或推理失败
- 输出持续振荡
- 实车碰撞
- 急停或超时停车链异常

回退只切换导航模式，不删除地图、定位参数或目标点管理逻辑。

## 19. 自动化测试清单

Windows 每次提交运行：

```powershell
& D:\ProgramData\anaconda3\Scripts\activate.ps1 D:\ProgramData\anaconda3\envs\isaaclab
python -m pytest D:\car_rl_training\tests -q
python D:\car_rl_training\scripts\benchmark_envs.py --num_envs 128 256 512
python D:\car_rl_training\source\car_rl_training\car_rl_training\export\validate_bundle.py `
  --bundle D:\car_rl_training\exports\car_rl_bundle_v2_<version>
```

Ubuntu 修改契约后运行：

```bash
cd /home/hao/ros2_car_gpt/jetson_car
source /opt/ros/humble/setup.bash
colcon build --packages-select car_rl
colcon test --packages-select car_rl
colcon test-result --verbose
./scripts/export_rl_windows_handoff.sh \
  --output /tmp/car_rl_windows_handoff.zip --force
unzip -t /tmp/car_rl_windows_handoff.zip
```

Orin 导入候选模型后运行：

```bash
cd /home/seeed/ros2_car/jetson_car
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select car_rl
colcon test-result --verbose
ros2 run car_rl model_tool verify --bundle \
  /home/seeed/ros2_car/jetson_car/src/car_rl/models/active
```

最终 CI 或检查脚本还要断言：

- 仓库公开导航模式只有 `classic` 和 `rl_controller`
- Nav2 的 `GridBased.plugin` 固定为 `nav2_navfn_planner/NavfnPlanner`
- pluginlib 只导出 `car_rl::Controller`
- 模型包 schema 为 2
- 不存在规划器 engine 或第二个 ONNX 要求
- 无模型时经典模式仍可启动

## 20. 实施顺序

1. Ubuntu 构建 `car_rl` 并生成带 SHA-256 的 Windows 离线交接包
2. 通过移动存储设备把 ZIP 和校验文件复制到 Windows
3. Windows 校验并解压到 `D:\car_rl_handoff`
4. 固定 Windows 和 Isaac Lab 环境
5. 使用 `isaaclab.bat --new` 建立外部项目与自动测试
6. 根据交接包实现 C++ 观测和动作契约并通过 100 组黄金测试
7. 导入交接包 URDF 并验证左右轮方向
8. 实现无障碍环境和 PPO MLP 基线
9. 加入路径、静态障碍和课程
10. 加入雷达、定位、动作和物理随机化
11. 使用三个种子完成训练和未见地图评估
12. 离线采集实车低速日志并通过移动存储设备填写 calibration.yaml
13. 用校准范围重新训练和评估
14. Windows 导出 ONNX 和 schema v2 模型包 ZIP 及 SHA-256
15. 通过移动存储设备复制到 Orin，并用导入脚本先校验后安装
16. 只在 Orin 构建 TensorRT FP16 engine
17. 依次完成日志回放、影子、架空轮、围栏和完整地图测试
18. 达到全部指标后将模型标记为 production

任何步骤失败都在当前步骤修复，不跳过契约测试或安全验收。
