#ifndef CAR_RL__CONTROLLER_HPP_
#define CAR_RL__CONTROLLER_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include "car_rl/model_contract.hpp"
#include "car_rl/model_session.hpp"
#include "car_rl/observation.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace car_rl
{

// Nav2局部控制器插件。只返回/cmd_vel_nav候选，不直接发布底盘速度
class Controller : public nav2_core::Controller
{
public:
  // 创建尚未配置的 Nav2 控制器插件
  Controller() = default;
  // 通过虚基类安全销毁控制器
  ~Controller() override = default;

  // 读取参数、校验模型和引擎并创建雷达订阅
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  // 释放订阅、推理后端、TF 和共享输入状态
  void cleanup() override;
  // 预热推理后端并开始接受控制周期
  void activate() override;
  // 停止输出并清理运行期失败状态
  void deactivate() override;
  // 保存 Nav2 下发的最新全局路径并重置上一动作
  void setPlan(const nav_msgs::msg::Path & path) override;
  // 根据雷达、路径、位姿和速度构造观测并计算候选速度
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  // 应用 Nav2 下发的百分比或绝对线速度限制
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // 保存最新雷达消息及其本机接收时间
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  // 把全局路径中的每个位姿转换到本轮控制坐标系
  nav_msgs::msg::Path path_in_frame(
    const nav_msgs::msg::Path & path,
    const std::string & target_frame) const;
  // 返回零速度并按需计入连续运行失败次数
  geometry_msgs::msg::TwistStamped zero_command(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & reason,
    bool count_failure = true);

  // Nav2 controller_server 提供的生命周期节点
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  // 控制器插件日志器
  rclcpp::Logger logger_{rclcpp::get_logger("car_rl.controller")};
  // 当前插件实例名称，用于构造参数前缀
  std::string plugin_name_;
  // Nav2 共享 TF 缓存
  std::shared_ptr<tf2_ros::Buffer> tf_;
  // 雷达扫描订阅器
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

  // 保护扫描、路径、上一动作、失败计数和动态限速的互斥量
  mutable std::mutex data_mutex_;
  // 最近一次雷达扫描消息
  sensor_msgs::msg::LaserScan latest_scan_;
  // 最近一次扫描到达控制器的 ROS 时间
  rclcpp::Time latest_scan_received_{0, 0, RCL_ROS_TIME};
  // 是否已经收到至少一帧扫描
  bool has_scan_{false};
  // Nav2 下发给控制器的当前全局路径
  nav_msgs::msg::Path global_plan_;
  // 当前路径终点是否已经进入Nav2位置容差，进入后只允许原地对齐
  bool goal_alignment_active_{false};
  // 是否已经保存可用于区分重规划与新任务的最终目标
  bool has_alignment_goal_{false};
  // 最近一次路径的最终目标；同一目标重规划不会解除对齐锁存
  geometry_msgs::msg::PoseStamped alignment_goal_;

  // 已校验的控制器模型bundle
  ModelBundle bundle_;
  // 统一管理验证、加载和推理的控制器模型会话
  std::unique_ptr<ModelSession> session_;
  // 上一控制周期动作，属于控制器观测的一部分
  ControllerAction previous_action_{0.0F, 0.0F};
  // 生命周期节点是否已激活该插件
  bool active_{false};
  // 连续推理或输入失败次数
  unsigned int consecutive_failures_{0U};
  // 超过该次数后持续返回零速度并报告错误
  unsigned int max_consecutive_failures_{2U};
  // 插件本次激活的 ROS 时间，用于初始输入等待
  rclcpp::Time activated_at_{0, 0, RCL_ROS_TIME};

  // 控制器订阅的雷达扫描话题
  std::string scan_topic_{"/scan"};
  // 雷达扫描允许的最大消息年龄
  double scan_stale_timeout_{0.5};
  // 首帧雷达和首条路径到达前允许等待的秒数
  double input_ready_timeout_{2.0};
  // 路径坐标转换允许的 TF 时间容差
  double transform_tolerance_{0.2};
  // 当前控制器允许的最大前进速度
  double max_linear_speed_{0.10};
  // 当前控制器允许的最大角速度
  double max_angular_speed_{1.047197551};
  // 终点原地对齐的比例角速度增益
  double goal_alignment_angular_gain_{1.5};
  // 允许原地旋转所需的全向最小雷达净空
  double goal_alignment_min_clearance_{0.20};
  // Nav2 动态限速施加到模型输出上的比例
  double speed_limit_factor_{1.0};
};

}  // namespace car_rl

#endif  // CAR_RL__CONTROLLER_HPP_
