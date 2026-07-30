# car_bringup

统一启动顺序为：固定热点、micro-ROS Agent、车体描述、速度仲裁、里程计、
底盘连接门控、雷达 TCP 桥/驱动监督和 Web。默认需要连续三帧
`/car/mcu_status` 同时报告 Wi-Fi 与 Agent 正常，随后等待 0.5 秒才开放
雷达 TCP 8889 并启动雷达驱动。

## 前台启动

```bash
ros2 launch car_bringup robot_bringup.launch.py
```

调试已有网络时使用 `start_hotspot:=false`，此时不会调用 `nmcli`。只调试
ROS 底盘时还可设置：

```bash
ros2 launch car_bringup robot_bringup.launch.py \
  start_hotspot:=false start_lidar:=false start_web:=false
```

单独调试雷达时可以显式设置 `lidar_wait_for_chassis:=false`。整车模式不建议
关闭该门控；默认等待超时为零，即小车未连接时雷达始终保持关闭。

默认链路：

```text
ESP32 -> UDP 8888 -> micro-ROS Agent
雷达板 -> TCP 8889 -> /tmp/tty_laser -> ydlidar_node -> /scan
Web -> http://192.168.4.1:8000
```

运行 `ros2 run car_bringup check_topics` 可在五秒窗口内检查 MCU、Scan、
Odometry、雷达 TCP、`odom -> base_footprint`、全局路径、三级速度链和
左右轮实际反馈。

默认网络、端口、伪串口和门控参数位于 `config/bringup.yaml`。热点参数必须
与 ESP32 固件一致；只更改 Jetson 一端会导致底盘无法连接。

## 后台启动控制

项目根目录下的三个脚本可以在后台启动、退出和重启整套小车节点：

```bash
/home/hao/ros2_car_gpt/jetson_car/scripts/start_jetson_car.sh
/home/hao/ros2_car_gpt/jetson_car/scripts/stop_jetson_car.sh
/home/hao/ros2_car_gpt/jetson_car/scripts/restart_jetson_car.sh
```

启动脚本可以直接加入 Jetson 开机启动项。开机任务需要调用脚本的绝对路径，
不应依赖只在交互式终端中加载的别名。

在 `~/.bashrc` 中加入以下三行后，重新打开终端即可使用三个简短命令：

```bash
alias car_start='/home/hao/ros2_car_gpt/jetson_car/scripts/start_jetson_car.sh'
alias car_stop='/home/hao/ros2_car_gpt/jetson_car/scripts/stop_jetson_car.sh'
alias car_restart='/home/hao/ros2_car_gpt/jetson_car/scripts/restart_jetson_car.sh'
```

运行输出保存在 `jetson_car/log/robot_bringup.log`。重复执行 `car_start` 不会
启动第二套节点，重复执行 `car_stop` 也不会报错。`car_stop` 会关闭项目热点
但保留 NetworkManager 配置，`car_restart` 不关闭热点，避免 ESP32 和雷达
重新连接。
