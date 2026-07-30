# car_mapping

该包使用 slam_toolbox 读取 `/scan`、`/odom` 和标准 TF 建图。

在 Jetson 工作区根目录加载环境后启动：

```bash
ros2 launch car_mapping mapping.launch.py use_rviz:=true
```

建图完成后保存到默认地图目录：

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/map
```

成功后应同时得到 `maps/map.pgm` 和 `maps/map.yaml`。地图属于现场运行数据，
默认不提交到 Git。使用 Web 保存时会先验证临时地图，再原子替换正式文件，
保存失败不会覆盖上一份有效地图。

`map -> odom` 由 slam_toolbox 发布，`odom -> base_footprint` 由
`car_base` 发布，两者不能由其它节点重复广播。

`scan_warmup_relay` 会在暖机阶段选择稳定点数，并把后续变化帧重采样到
固定角度网格，保证点数、角度范围和角度增量一致。该处理位于自写包内，
不修改 YDLIDAR 驱动源码。

建图异常时依次检查：

```bash
ros2 topic hz /scan
ros2 topic hz /mapping_scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```
