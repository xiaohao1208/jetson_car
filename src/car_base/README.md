# car_base

`car_base` 将 ESP32 发布的累计编码器 tick 与 MPU6050 Z 轴角速度融合，
连续发布标准 `/odom` 和 `odom -> base_footprint`。

平移完全来自编码器；偏航在 IMU 新鲜且车轮确实发生变化时做互补融合。
IMU 过期或异常时自动使用纯编码器结果，静止时不单独积分陀螺仪，因此不会
因为零偏持续旋转。当前版本不处理轮胎空转、底盘卡住或“命令已发出但车体
没有真实移动”等问题。

## 启动与检查

```bash
ros2 launch car_base base.launch.py
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

主要输入为 `/car/mcu_status` 和 `/imu/data`，输出为 `/odom` 与 TF。
`config/base.yaml` 中的 `wheel_distance`、`left_wheel_per_tick` 和
`right_wheel_per_tick` 必须与 ESP32 一致。`imu_yaw_weight` 为 0 时只使用
编码器偏航，为 1 时在有效运动期间只使用 IMU 偏航增量。

`imu_timeout_sec` 和 `imu_gyro_deadband_rps` 控制 IMU 过期降级与静止噪声；
采样时间或 tick 增量超出限制时会拒绝异常帧，避免一次错误读数破坏里程计。

## 实车里程计标定

先让小车沿直线行驶约一米，记录实际距离和 `/odom` 的距离，再让小车原地
旋转一整圈，记录实际角度和 `/odom` 的角度。计算工具不会控制小车，也不会
自动改写配置：

```bash
ros2 run car_base odometry_calibration \
  --actual-distance 1.00 \
  --odom-distance 0.96 \
  --actual-yaw-rad 6.283185307 \
  --odom-yaw-rad 6.667157733
```

把输出的 `wheel_distance`、`left_wheel_per_tick` 和
`right_wheel_per_tick` 同步到 `car_base/config/base.yaml` 与 ESP32
的 `car_config.hpp`。轮距变化后也要同步 URDF 中左右轮中心位置。

标定前应确认左右编码器正方向正确、IMU Z 轴方向符合 ROS 右手系，并在低速
条件下完成多次往返测量后取稳定结果。
