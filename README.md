# Jetson ROS 2 小车

## 第一部分：项目介绍

这个工作区运行在 Jetson 上，负责整车的上位机功能：micro-ROS Agent、速度安全仲裁、里程计、激光雷达、建图、定位、经典或强化学习导航、自动标定、观测数据采集和手机网页控制。ESP32 只接受 car_move 仲裁后的最终速度，网络中断、命令超时、急停或近障时会在底盘侧再次停车。

### 系统组成

| 包 | 职责 |
| --- | --- |
| car_interfaces | Jetson 与 ESP32 共用的消息、服务和 Action |
| car_description | URDF、关节和静态 TF |
| car_move | 手动、导航、标定与观测命令仲裁，限速、急停和近障拦截 |
| car_base | 编码器里程计和 IMU 偏航融合 |
| car_mapping | 激光扫描整形和 slam_toolbox 建图 |
| car_navigation | AMCL、Nav2、单点/多点导航和近障扫描行为树 |
| car_rl | 86 维观测契约、模型校验、TensorRT/ONNX Runtime 推理和 Nav2 插件 |
| car_observation | 使用经典 Nav2 循环航点并采集训练观测 |
| car_calibrate | 自动执行静止、直线和旋转标定 |
| car_web | FastAPI、ROS 桥接、地图显示和网页控制 |
| car_bringup | Agent、底盘、雷达、标定、观测和网页的统一启动 |

工作区还固定了四个上游包：micro-ROS-Agent、micro_ros_msgs、ros_serial2wifi 和 ydlidar_ros2。为保证普通克隆后可以直接构建，这四个包按固定版本完整保存在 `src` 中，不需要额外初始化子模块。

主要数据链路：

    网页/遥控 ──> /cmd_vel_manual ─┐
    Nav2 ───────> /cmd_vel_nav ────┼─> car_move ─> /cmd_vel_move ─> ESP32
    自动标定 ───> /cmd_vel_calibration

    ESP32 ─> /car/mcu_status ─> car_base ─> /odom
    ESP32 ─> /imu/data ───────────────┘
    雷达板 ─> TCP 8889 ─> /tmp/tty_laser ─> /scan

TF 主链为 map → odom → base_footprint → base_link → laser_link。建图时 map 由 slam_toolbox 提供；已有地图导航时由 AMCL 和 Nav2 提供。

### 固定参数与安全边界

默认网络：

| 项目 | 值 |
| --- | --- |
| NetworkManager 连接名 | jetson-car-hotspot |
| SSID / 密码 | jetson / 88888888 |
| Jetson 地址 | 192.168.4.1/24 |
| micro-ROS Agent | UDP 8888 |
| 雷达 TCP 服务 | TCP 8889 |
| Web | TCP 8000 |

机械参数必须与 ESP32 一致：

| 参数 | 值 |
| --- | --- |
| 左轮每 tick 距离 | 0.0001039203 m |
| 右轮每 tick 距离 | 0.0001033942 m |
| 驱动轮中心距 | 0.175 m |

所有运动输入都必须持续刷新。MCU 状态过期、真实急停、程序安全锁、底盘故障或前向近障都会阻止危险速度。近障状态允许后退脱困。自动标定和自动观测启动前都检查小车在线、车轮静止、传感器新鲜度和操作者的逐项安全确认。

网页对外接口保持稳定：

| 方法 | 路径 | 用途 |
| --- | --- | --- |
| GET | /、/api/status、/api/map | 页面、整车状态和地图 |
| POST | /api/move、/api/emergency-stop | 手动速度和急停 |
| POST | /api/calibration/start | 自动标定 |
| POST | /api/observation-collection/start、/cancel | 自动观测 |
| POST | /api/mapping/start、/save-stop | 建图 |
| POST | /api/navigation/start、/stop | 启停导航栈 |
| POST | /api/navigation/initial-pose、/goals | 设置位置和目标 |
| POST | /api/navigation/run、/resume、/cancel | 执行、继续和取消任务 |
| DELETE | /api/navigation/goals、/goals/last | 清理目标 |

## 第二部分：从零实现与部署

### 1. 准备系统和 ROS 2 Humble

目标系统为 Ubuntu 22.04。Jetson 使用 JetPack 6 时也按 Ubuntu 22.04 的方式安装 ROS 2 Humble。先设置 UTF-8、启用 Universe 软件源并加入 ROS 软件源：

    sudo apt update
    sudo apt install -y locales software-properties-common curl
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    sudo add-apt-repository universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |
      sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt update
    sudo apt install -y ros-humble-desktop ros-dev-tools

安装本项目使用的构建工具和运行库：

    sudo apt install -y \
      build-essential cmake git ninja-build pkg-config \
      python3-colcon-common-extensions python3-pip python3-rosdep \
      python3-vcstool python3-yaml python3-pil \
      libssl-dev libyaml-cpp-dev nlohmann-json3-dev libxml2-utils \
      network-manager openssh-server \
      ros-humble-xacro ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher ros-humble-tf2-ros \
      ros-humble-rviz2 ros-humble-slam-toolbox \
      ros-humble-navigation2 ros-humble-nav2-bringup \
      ros-humble-nav2-map-server ros-humble-nav2-amcl \
      ros-humble-laser-filters

Web 依赖优先使用系统软件包：

    sudo apt install -y python3-fastapi python3-uvicorn

如果系统源没有这两个包：

    python3 -m pip install --user fastapi "uvicorn[standard]" pillow pyyaml

初始化 rosdep。已经初始化过时，第一条命令会提示文件已存在，可以跳过：

    sudo rosdep init
    rosdep update

### 2. 从空工作区创建 ROS 2 包

下面是项目最初的包结构。复现源码结构时，在空目录中执行：

    mkdir -p jetson_car/src
    cd jetson_car/src

    ros2 pkg create --build-type ament_cmake --license Apache-2.0 car_interfaces
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 car_description
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 car_move
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 car_base
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 car_mapping
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 car_navigation
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 car_rl
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 car_observation
    ros2 pkg create --build-type ament_python --license Apache-2.0 car_calibrate
    ros2 pkg create --build-type ament_python --license Apache-2.0 car_web
    ros2 pkg create --build-type ament_python --license Apache-2.0 car_bringup

随后按本仓库对应目录补齐 msg、srv、action、include、src、launch、config、urdf、behavior_trees、templates 和 static。接口包先定义 CarStatus、CarMcuStatus、CalibrationStatus、ObservationCollectionStatus、ObstacleScanStatus、EmergencyStop、RunCalibration 和 RunObservationCollection，再由 rosidl_generate_interfaces 生成 C、C++ 和 Python 类型。其余包通过 package.xml 声明依赖，不把依赖路径写死在代码中。

### 3. 获取正式仓库和上游包

正常部署直接克隆完整仓库：

    git clone https://github.com/xiaohao1208/jetson_car.git
    cd jetson_car
    export JETSON_CAR_ROOT="$PWD"

后续项目目录都从 `JETSON_CAR_ROOT` 推导。地图位于 `maps`，观测数据位于 `src/car_observation/observations`，控制器模型位于 `src/car_rl/models/controller`，推理缓存位于 `cache/car_rl`。仓库保留现有实车观测数据用于复现实验；以后每次采集仍按日期创建独立目录，需要训练时由操作者手工复制到训练项目的 `observations` 目录。启动脚本会自动设置该变量，手动运行独立工具时可以先执行 `export JETSON_CAR_ROOT="$PWD"`。

仓库内保存的上游源码分别对应 micro-ROS-Agent `4f363a79ae96aed39dcc5269f53f73f6a70d3a2c`、micro_ros_msgs `100bf269e78da3fe0a58f6531ebe47d6991bd9ab`、ros_serial2wifi `bd677e5cea11632a875e5f98d9d54ddf209f87f3` 和 ydlidar_ros2 `180d5847450888789c2bbbb971be66055da41bfb`。这些目录是普通源码目录，克隆本仓库后无需再运行 `git submodule`。

安装 package.xml 中声明的其余依赖：

    source /opt/ros/humble/setup.bash
    rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

### 4. 实现顺序

从空包开始时，建议按以下顺序完成，后一个阶段只依赖已经稳定的接口：

1. 在 car_interfaces 固定跨端消息、急停服务和长任务 Action。
2. 在 car_description 建立 base_footprint、base_link、车轮、imu_link 和 laser_link。
3. 在 car_move 实现四路速度仲裁、输入超时、限速、急停和近障门禁。
4. 在 car_base 使用左右编码器增量计算平移和转向，并按配置融合 IMU 偏航。
5. 接入 ydlidar_ros2，在 car_mapping 统一 scan 几何并启动 slam_toolbox。
6. 在 car_navigation 配置 map_server、AMCL、规划器、控制器和行为树。
7. 在 car_web 将 ROS 状态做线程安全快照，提供固定 HTTP API 和本地静态页面。
8. 在 car_calibrate 实现静止检查、分段试验、质量门禁和原子结果保存。
9. 在 car_rl 固定 86 维观测和模型元数据契约，再实现 TensorRT/ONNX Runtime 后端。
10. 在 car_observation 用经典 Nav2 循环航点，完成采样、覆盖平衡和安全中止。
11. 在 car_bringup 按 Agent、底盘、雷达和网页的顺序组合启动。

### 5. 配置固定热点

第一次配置热点应通过本地终端、有线网络或第二块网卡操作，避免当前 Wi-Fi SSH 被切断。查找无线网卡：

    nmcli -f DEVICE,TYPE,STATE,CONNECTION device status

以下示例假设网卡为 wlan0，请按实际名称替换：

    sudo nmcli connection add type wifi ifname wlan0 \
      con-name jetson-car-hotspot ssid jetson
    sudo nmcli connection modify jetson-car-hotspot \
      connection.interface-name wlan0 \
      connection.autoconnect yes \
      connection.autoconnect-priority 100 \
      802-11-wireless.mode ap \
      802-11-wireless.band bg \
      802-11-wireless.channel 6 \
      ipv4.method shared \
      ipv4.addresses 192.168.4.1/24 \
      ipv6.method disabled \
      wifi-sec.key-mgmt wpa-psk \
      wifi-sec.psk 88888888
    sudo nmcli --wait 15 connection up id jetson-car-hotspot

确认连接和地址：

    nmcli connection show --active
    ip -4 address show wlan0
    sudo systemctl enable --now ssh

ROS launch 不创建或关闭热点。start_jetson_car.sh 只检查 bringup.yaml 中的 hotspot_connection 是否活动，这样停止 ROS 时不会断开 ESP32、雷达或 SSH。

### 6. 编译工作区

    cd jetson_car
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash

每个新终端都要加载环境。可以加入 shell 配置：

    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
    printf 'source "%s/install/setup.bash"\n' "$JETSON_CAR_ROOT" >> "$HOME/.bashrc"

### 7. 启动和检查

推荐通过脚本操作：

    ./scripts/start_jetson_car.sh
    tail -f log/robot_bringup.log
    ./scripts/restart_jetson_car.sh
    ./scripts/stop_jetson_car.sh

也可以直接启动：

    ros2 launch car_bringup robot_bringup.launch.py

只调试底盘：

    ros2 launch car_bringup robot_bringup.launch.py \
      start_lidar:=false start_web:=false

启动后检查：

    ros2 topic hz /car/mcu_status
    ros2 topic hz /imu/data
    ros2 topic hz /scan
    ros2 topic hz /odom
    ros2 run tf2_ros tf2_echo map base_footprint

网页地址为 http://192.168.4.1:8000。

### 8. 标定、建图和导航

第一次落地先架空车轮，确认左右电机和编码器方向，再在空旷场地完成自动标定。标定结果中的 wheel_distance、left_wheel_per_tick 和 right_wheel_per_tick 要同步到 car_base/config/base.yaml 与 ESP32 的 car_config.hpp。

单独建图：

    ros2 launch car_mapping mapping.launch.py use_rviz:=true

保存地图：

    mkdir -p maps
    ros2 run nav2_map_server map_saver_cli -f maps/map

经典导航：

    ros2 launch car_navigation navigation.launch.py \
      map:="$PWD/maps/map.yaml" navigation_mode:=classic use_rviz:=true

RL 模式先检查模型：

    ros2 run car_rl model_tool status --json

只有 controller_runtime_ready 和 controller_available 都为 true 时才启用 rl_controller。训练端输出包含 bundle.yaml、controller/metadata.yaml、controller/car_rl_model.onnx、controller/evaluation.json 和 environment-lock.json 的 ZIP，并生成相邻 SHA-256 文件。TensorRT engine 必须回到目标 Jetson 上构建，不能从训练电脑复制。模型部署使用：

    ros2 launch car_rl get_model class:=controller

`get_model` 的 `class` 默认值为 `controller`，所以也可以省略该参数。命令默认从与 `ros2_car_gpt` 同级的 `car_rl_train/exports/car_rl_model/` 读取 `car_rl_model.zip` 和 `car_rl_model.zip.sha256`，校验后复制到 `src/car_rl/car_rl_model/controller/`。如果训练目录不在本机，也可以把这两个文件手动放进该目录后执行同一条命令。指定其他位置时必须传入绝对路径：

    ros2 launch car_rl get_model \
      class:=controller \
      archive:=/absolute/path/car_rl_model.zip

该命令会检查 ZIP 和模型契约，把训练包中的控制器文件扁平化到 `src/car_rl/models/controller/`，再根据当前构建自动准备 ONNX Runtime 或构建 TensorRT FP16 引擎。全部成功后才替换原控制器模型，任何步骤失败都会保留原模型。当前只接受 `class:=controller`。

在 Jetson 导入训练结果：

    ros2 launch car_rl get_model class:=controller
    ros2 run car_rl model_tool status --json

模型就绪后可以直接测试推理延时：

    ros2 run car_rl model_tool benchmark

该命令默认使用 FP16，预热 500 次并统计 10000 次同步推理。每次测试都会在 `src/car_rl/benchmark` 中生成一个带日期的 JSON 文件，包含 mean、p50、p95、p99 和最大延时；需要临时调整次数时仍可使用 `--warmup` 和 `--runs`。

自动观测必须使用已有地图和经典导航，先完成 AMCL 定位并在网页设置至少三个安全航点，再按页面提示逐项确认。

### 9. 常见故障

- ESP32 不上线：核对 SSID、密码、192.168.4.1、UDP 8888 和 Agent 日志。
- 雷达没有 scan：检查 TCP 8889、/tmp/tty_laser、雷达供电和 ydlidar 参数。
- 有 odom 没有 map：建图时检查 slam_toolbox；导航时检查 map_server、AMCL 和初始位姿。
- 网页能打开但不能运动：查看 /api/status 中 car_online、e_stop、fault_bits 和输入新鲜度。
- 导航拒绝启动：确认地图存在、定位完成、没有建图进程或未结束的长任务。
- XML 校验器离线报错：先用 xmllint --noout 检查本地 XML；ament_xmllint 还会访问 ROS 在线 XSD。
