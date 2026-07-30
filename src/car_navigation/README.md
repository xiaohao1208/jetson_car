# car_navigation

该包统一启动 map_server、AMCL 和 Nav2。经典导航不依赖强化学习模型。

在 Jetson 工作区根目录执行：

```bash
ros2 launch car_navigation navigation.launch.py \
  map="$PWD/maps/map.yaml" navigation_mode:=classic use_rviz:=true
```

单点使用标准 `/navigate_to_pose` action，多点使用 `/follow_waypoints`。
小车连续 5 秒没有产生 0.04m 有效位移时判定当前目标失败。多点任务记录
失败点索引后继续剩余目标，单点任务直接结束。目标可达性由标准
FollowWaypoints 按顺序判断，不会在任务开始前预判后续目标。

定位栈先启动，导航栈默认延迟 3 秒启动，避免 AMCL 尚未建立
`map -> odom` 时全局代价地图提前激活。可通过
`navigation_start_delay_sec` 调整延迟。

经典和强化学习导航的最大线速度统一为 `0.10 m/s`，最大角速度统一为
`1.047197551 rad/s`。

经典模式按照 URDF 车体与轮子外廓使用矩形 footprint。全局代价地图使用
静态地图与二维雷达，局部代价地图同样只使用二维雷达。前向超声波不参与
路径规划，由 ESP32 和 `car_move` 在 0.25 m 内独立完成前进硬停车，
`/ultrasonic_range` 和 `/ultrasonic_scan` 继续保留用于调试。

经典模式明确使用标准 `NavfnPlanner` 全局规划器和 `DWBLocalPlanner`
局部控制器，但速度、加速度、采样、轨迹评分、目标容差、车体轮廓和代价地图
均使用本项目参数，不是 Nav2 原始默认配置。

经典模式对非零速度使用 `0.03 m/s` 平移死区和 `0.523598776 rad/s`
旋转死区，避免电机只有声音但无法克服静摩擦。DWB 的短时失败容忍为
1 秒。

可选模式为 `classic` 和 `rl_controller`。两种模式都使用 NavFn 全局规划器，
`rl_controller` 只替换 DWB 局部控制器。RL 模式启动前需先用
`ros2 run car_rl model_tool status` 确认控制器模型及 TensorRT engine 已就绪。

启动后可检查：

```bash
ros2 lifecycle nodes
ros2 action list
ros2 topic hz /plan
ros2 topic hz /cmd_vel
```
