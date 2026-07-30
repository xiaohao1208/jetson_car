# Jetson ROS 2 小车

这是小车的 Jetson 上位机工作区，负责固定热点、micro-ROS Agent、速度安全
仲裁、里程计、雷达接入、建图、定位、单点/多点导航、Web 控制和可选的强化
学习局部控制器。ESP32 固件独立维护，两端通过 Wi-Fi UDP micro-ROS 通信。

当前软件链路已经具备离线构建和测试能力；首次实车运行前仍必须标定轮距、
左右轮每 tick 距离、IMU 方向、速度上限和安全距离。

## 系统组成

| 包 | 作用 |
| --- | --- |
| `car_interfaces` | Jetson 与 ESP32 共用的消息和急停服务 |
| `car_description` | 小车 URDF、外形和静态 TF |
| `car_move` | 手动/导航速度仲裁、超时停车、限速、急停和近障拦截 |
| `car_base` | 编码器平移与 IMU 偏航融合，发布里程计和 TF |
| `car_mapping` | 激光扫描整形、slam_toolbox 建图和 RViz |
| `car_navigation` | AMCL、Nav2、单点/多点导航及经典/RL 模式 |
| `car_rl` | RL 模型校验、TensorRT engine 构建和 Nav2 插件 |
| `car_web` | 移动、建图、地图、定位和导航网页 |
| `car_bringup` | 热点、Agent、底盘、雷达和 Web 的统一启动 |

四个上游依赖以 Git 子模块固定版本：`micro-ROS-Agent`、
`micro_ros_msgs`、`ros_serial2wifi` 和 `ydlidar_ros2`。

核心数据流如下：

```text
/cmd_vel_manual ─┐
                 ├─ car_move ─> /cmd_vel_move ─> ESP32
/cmd_vel         ┘

ESP32 ─> /car/mcu_status ─> car_base ─> /odom
ESP32 ─> /imu/data       ────────┘

雷达板 ─> TCP 8889 ─> /tmp/tty_laser ─> /scan
map ─> odom ─> base_footprint ─> base_link / laser_link / ...
```

`car_move` 仅在 MCU 状态新鲜且未急停时放行速度。手动命令优先于导航命令，
两类输入都必须持续刷新；前向近障只禁止继续前进，仍允许后退脱困。

## 环境准备

目标环境为 Ubuntu 22.04、ROS 2 Humble。Jetson Orin Nano Super 上的
TensorRT 功能还需要 JetPack 6.x、CUDA 和 TensorRT 10.x；经典导航不依赖
TensorRT。

首次获取仓库时必须同时拉取子模块：

```bash
git clone --recurse-submodules REPOSITORY_URL jetson_car
cd jetson_car
```

如果已经完成普通克隆：

```bash
git submodule update --init --recursive
```

安装 ROS 依赖与网页运行依赖：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  python3-fastapi python3-uvicorn python3-pil python3-yaml \
  libssl-dev libyaml-cpp-dev libxml2-utils
rosdep install --from-paths src --ignore-src --rosdistro humble \
  --skip-keys ament_python -y
```

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
./scripts/check_self_packages.sh
```

检查脚本会构建九个自写包、运行 `car_rl` 测试，并用本机 `xmllint`
检查包清单。每个新终端在运行节点前都需要加载 ROS 和工作区环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 跨端配置

默认热点名称为 `jetson`，密码为 `88888888`，Jetson 地址为
`192.168.4.1`。micro-ROS Agent 使用 UDP 8888，雷达转接板使用 TCP 8889。
这些值必须与 ESP32 固件配置一致。

修改机械或运动参数时同步检查：

- 轮距和左右轮每 tick 距离：ESP32 底盘配置、`car_base/config/base.yaml`
  以及 URDF 的轮子位置。
- 线速度和角速度范围：ESP32 速度限制、`car_move/config/move.yaml`
  和 Nav2 控制器参数。
- 热点、网关和 Agent 端口：ESP32 网络配置与
  `car_bringup/config/bringup.yaml`。

ROS 距离使用米，线速度使用米每秒，偏航和角速度使用弧度及弧度每秒。
网页目标角速度用度每秒显示，后端发送前转换为弧度每秒。

## 整车启动

在工作区根目录执行：

```bash
ros2 launch car_bringup robot_bringup.launch.py
```

启动程序先准备固定热点和 UDP Agent，再启动底盘节点。连续收到三帧正常
MCU 状态后才开放雷达 TCP 并启动雷达驱动。默认 Web 地址为
`http://192.168.4.1:8000`。

在已有网络中调试且不希望修改网卡配置时：

```bash
ros2 launch car_bringup robot_bringup.launch.py start_hotspot:=false
```

只调试底盘时可关闭雷达和 Web：

```bash
ros2 launch car_bringup robot_bringup.launch.py \
  start_lidar:=false start_web:=false
```

后台启动、停止和重启整车：

```bash
./scripts/start_jetson_car.sh
./scripts/stop_jetson_car.sh
./scripts/restart_jetson_car.sh
```

运行日志写入 `log/robot_bringup.log`。停止脚本会关闭项目热点；重启脚本保持
热点连续工作，减少 ESP32 和雷达重新联网时间。

## 建图与导航

单独启动建图：

```bash
ros2 launch car_mapping mapping.launch.py use_rviz:=true
```

保存地图：

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/map
```

地图和绑定位姿属于现场运行数据，默认不会提交到 Git。启动经典导航：

```bash
ros2 launch car_navigation navigation.launch.py \
  map:="$PWD/maps/map.yaml" navigation_mode:=classic use_rviz:=true
```

`classic` 使用 NavFn 全局规划器和 DWB 局部控制器，不需要模型。
`rl_controller` 保留 NavFn，只用 RL 插件替换 DWB。启用 RL 前检查模型、
TensorRT engine 和后端：

```bash
ros2 run car_rl model_tool status --json
```

单点导航使用 `/navigate_to_pose`，多点导航使用 `/follow_waypoints`。多点任务
会记录失败目标并继续后续目标，Web 中失败点显示为红色。

## 诊断

```bash
ros2 run car_bringup check_topics
ros2 topic hz /car/mcu_status
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

`check_topics` 会检查 MCU、雷达 TCP、Scan、Odometry、TF、全局路径、三级
速度链和左右轮反馈。实车测试时先架空驱动轮，确认急停、命令超时、近障停车
和轮子方向正确，再进行低速地面测试。
