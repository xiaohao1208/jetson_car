#include "car_navigation/scan_pattern.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "car_interfaces/msg/car_mcu_status.hpp"
#include "car_interfaces/msg/car_status.hpp"
#include "car_interfaces/msg/obstacle_scan_status.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace car_navigation
{

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;
using Spin = nav2_msgs::action::Spin;
using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

class ExpandingObstacleScan : public BT::ActionNodeBase
{
public:
  ExpandingObstacleScan(
    const std::string & name, const BT::NodeConfiguration & configuration)
  : BT::ActionNodeBase(name, configuration)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput("trigger_hold_sec", trigger_hold_sec_);
    getInput("clear_hold_sec", clear_hold_sec_);
    getInput("evaluation_timeout_sec", evaluation_timeout_sec_);
    getInput("scan_step_rad", scan_step_rad_);
    getInput("scan_limit_rad", scan_limit_rad_);
    getInput("retry_delay_sec", retry_delay_sec_);
    getInput("stop_wheel_speed_mps", stop_wheel_speed_mps_);
    getInput("stopped_hold_sec", stopped_hold_sec_);
    getInput("input_stale_timeout_sec", input_stale_timeout_sec_);
    getInput("minimum_spin_rate_radps", minimum_spin_rate_radps_);
    getInput("spin_time_margin_sec", spin_time_margin_sec_);
    getInput("clearance_tie_m", clearance_tie_m_);

    ranges_ = build_scan_ranges(scan_step_rad_, scan_limit_rad_);
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_executor_.add_callback_group(
      callback_group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    std::string status_topic;
    std::string mcu_topic;
    std::string odom_topic;
    std::string spin_action;
    getInput("status_topic", status_topic);
    getInput("mcu_status_topic", mcu_topic);
    getInput("odom_topic", odom_topic);
    getInput("spin_action", spin_action);
    car_status_sub_ = node_->create_subscription<car_interfaces::msg::CarStatus>(
      status_topic, rclcpp::QoS(10),
      [this](car_interfaces::msg::CarStatus::SharedPtr message) {
        car_status_ = *message;
        car_status_time_ = SteadyClock::now();
        have_car_status_ = true;
      }, options);
    mcu_status_sub_ = node_->create_subscription<car_interfaces::msg::CarMcuStatus>(
      mcu_topic, rclcpp::SensorDataQoS(),
      [this](car_interfaces::msg::CarMcuStatus::SharedPtr message) {
        mcu_status_ = *message;
        mcu_status_time_ = SteadyClock::now();
        have_mcu_status_ = true;
      }, options);
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr message) {
        odom_ = *message;
        odom_time_ = SteadyClock::now();
        have_odom_ = true;
      }, options);
    status_pub_ = node_->create_publisher<car_interfaces::msg::ObstacleScanStatus>(
      "/car/navigation_obstacle_scan_status",
      rclcpp::QoS(1).reliable().transient_local());
    spin_client_ = rclcpp_action::create_client<Spin>(node_, spin_action, callback_group_);
    publish_status(false, "idle", 0.0, 0.0, "近障扫描空闲");
  }

  static BT::PortsList providedPorts()
  {
    constexpr double degree = 3.14159265358979323846 / 180.0;
    return {
      BT::InputPort<std::string>("status_topic", "/car/status", "综合车辆状态话题"),
      BT::InputPort<std::string>("mcu_status_topic", "/car/mcu_status", "左右轮速状态话题"),
      BT::InputPort<std::string>("odom_topic", "/odom", "里程计话题"),
      BT::InputPort<std::string>("spin_action", "/spin", "Nav2旋转行为Action"),
      BT::InputPort<double>("trigger_hold_sec", 0.20, "近障触发保持时间"),
      BT::InputPort<double>("clear_hold_sec", 0.50, "方向净空保持时间"),
      BT::InputPort<double>("evaluation_timeout_sec", 0.75, "单方向评估最长时间"),
      BT::InputPort<double>("scan_step_rad", 20.0 * degree, "扫描范围递增步长"),
      BT::InputPort<double>("scan_limit_rad", 90.0 * degree, "相对遇障朝向最大范围"),
      BT::InputPort<double>("retry_delay_sec", 1.0, "整轮受阻后的停车时间"),
      BT::InputPort<double>("stop_wheel_speed_mps", 0.01, "停车轮速阈值"),
      BT::InputPort<double>("stopped_hold_sec", 0.50, "停车确认保持时间"),
      BT::InputPort<double>("input_stale_timeout_sec", 0.50, "输入新鲜度限制"),
      BT::InputPort<double>("minimum_spin_rate_radps", 0.25, "估算Spin超时的最低角速度"),
      BT::InputPort<double>("spin_time_margin_sec", 2.0, "Spin超时余量"),
      BT::InputPort<double>("clearance_tie_m", 0.05, "左右净空视为相同的距离差"),
    };
  }

  BT::NodeStatus tick() override
  {
    callback_executor_.spin_some();
    const auto now = SteadyClock::now();
    if (active_ && std::chrono::duration<double>(now - last_status_publish_).count() >= 0.25) {
      publish_status(
        last_status_active_, last_status_phase_, last_status_target_, last_status_range_,
        last_status_message_);
    }
    if (!active_) {
      return tick_idle(now);
    }
    if (!inputs_safe(now)) {
      cancel_spin();
      phase_ = Phase::waiting_inputs;
      publish_status(
        true, "waiting_inputs", target_offset_, current_range(),
        "近障扫描等待车辆状态、里程计或安全锁恢复");
      return BT::NodeStatus::RUNNING;
    }
    if (phase_ == Phase::waiting_inputs) {
      phase_ = Phase::waiting_stop;
      stopped_since_ = {};
    }

    switch (phase_) {
      case Phase::waiting_stop:
        return tick_waiting_stop(now);
      case Phase::waiting_goal:
        return tick_waiting_goal(now);
      case Phase::turning:
        return tick_turning(now);
      case Phase::evaluating:
        return tick_evaluating(now);
      case Phase::waiting_retry:
        return tick_waiting_retry(now);
      case Phase::waiting_inputs:
      case Phase::idle:
        break;
    }
    return BT::NodeStatus::RUNNING;
  }

  void halt() override
  {
    cancel_spin();
    reset(false);
    setStatus(BT::NodeStatus::IDLE);
  }

private:
  enum class Phase {idle, waiting_inputs, waiting_stop, waiting_goal, turning, evaluating,
    waiting_retry};

  using Candidate = ClearanceCandidate;

  bool recent(const SteadyClock::time_point & time, double timeout) const
  {
    return time != SteadyClock::time_point{} &&
           std::chrono::duration<double>(SteadyClock::now() - time).count() <= timeout;
  }

  static double yaw_of(const geometry_msgs::msg::Quaternion & value)
  {
    return std::atan2(
      2.0 * (value.w * value.z + value.x * value.y),
      1.0 - 2.0 * (value.y * value.y + value.z * value.z));
  }

  double current_range() const
  {
    return range_index_ < ranges_.size() ? ranges_[range_index_] : scan_limit_rad_;
  }

  bool inputs_safe(const SteadyClock::time_point &) const
  {
    return have_car_status_ && have_mcu_status_ && have_odom_ &&
           recent(car_status_time_, input_stale_timeout_sec_) &&
           recent(mcu_status_time_, input_stale_timeout_sec_) &&
           recent(odom_time_, input_stale_timeout_sec_) &&
           car_status_.mcu_ok && !car_status_.e_stop_ok;
  }

  BT::NodeStatus tick_idle(const SteadyClock::time_point & now)
  {
    if (!have_car_status_ || !have_odom_ ||
      !recent(car_status_time_, input_stale_timeout_sec_) ||
      !recent(odom_time_, input_stale_timeout_sec_) || !car_status_.obstacle_ok)
    {
      obstacle_since_ = {};
      return BT::NodeStatus::FAILURE;
    }
    if (obstacle_since_ == SteadyClock::time_point{}) {
      obstacle_since_ = now;
      return BT::NodeStatus::FAILURE;
    }
    if (std::chrono::duration<double>(now - obstacle_since_).count() < trigger_hold_sec_) {
      return BT::NodeStatus::FAILURE;
    }
    active_ = true;
    encounter_yaw_ = yaw_of(odom_.pose.pose.orientation);
    range_index_ = 0U;
    scanning_left_ = true;
    final_turn_ = false;
    left_ = Candidate{};
    right_ = Candidate{};
    phase_ = Phase::waiting_stop;
    stopped_since_ = {};
    publish_status(
      true, "waiting_stop", ranges_.front(), ranges_.front(),
      "检测到前方近障，正在停车准备左右扫描");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus tick_waiting_stop(const SteadyClock::time_point & now)
  {
    const bool stopped =
      std::abs(mcu_status_.left_wheel_speed) < stop_wheel_speed_mps_ &&
      std::abs(mcu_status_.right_wheel_speed) < stop_wheel_speed_mps_;
    if (!stopped) {
      stopped_since_ = {};
      return BT::NodeStatus::RUNNING;
    }
    if (stopped_since_ == SteadyClock::time_point{}) {
      stopped_since_ = now;
      return BT::NodeStatus::RUNNING;
    }
    if (std::chrono::duration<double>(now - stopped_since_).count() < stopped_hold_sec_) {
      return BT::NodeStatus::RUNNING;
    }
    if (!spin_client_->action_server_is_ready()) {
      publish_status(
        true, "waiting_spin_server", target_offset_, current_range(),
        "Nav2旋转行为尚未就绪，车辆保持停车");
      return BT::NodeStatus::RUNNING;
    }
    send_spin();
    return active_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }

  void send_spin()
  {
    if (!final_turn_) {
      target_offset_ = scanning_left_ ? current_range() : -current_range();
    }
    const double current_yaw = yaw_of(odom_.pose.pose.orientation);
    const double relative = relative_spin_for_target(encounter_yaw_, current_yaw, target_offset_);
    if (std::abs(relative) < 0.01) {
      if (final_turn_) {
        complete_recovery();
      } else {
        begin_evaluation();
      }
      return;
    }
    Spin::Goal goal;
    goal.target_yaw = static_cast<float>(relative);
    const double allowance =
      std::abs(relative) / std::max(minimum_spin_rate_radps_, 0.01) + spin_time_margin_sec_;
    const auto seconds = static_cast<std::int32_t>(std::floor(allowance));
    goal.time_allowance.sec = seconds;
    goal.time_allowance.nanosec = static_cast<std::uint32_t>(
      std::llround((allowance - static_cast<double>(seconds)) * 1.0e9));
    send_goal_future_ = spin_client_->async_send_goal(goal);
    phase_ = Phase::waiting_goal;
    publish_status(
      true, final_turn_ ? "selecting" : (scanning_left_ ? "scan_left" : "scan_right"),
      target_offset_, current_range(),
      final_turn_ ? "正在转向左右扫描后选出的安全方向" :
      (scanning_left_ ? "正在向左扫描前方净空" : "正在向右扫描前方净空"));
  }

  BT::NodeStatus tick_waiting_goal(const SteadyClock::time_point &)
  {
    if (!send_goal_future_.valid() ||
      send_goal_future_.wait_for(0ms) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    spin_goal_ = send_goal_future_.get();
    if (!spin_goal_) {
      handle_turn_failure();
      return BT::NodeStatus::RUNNING;
    }
    result_future_ = spin_client_->async_get_result(spin_goal_);
    phase_ = Phase::turning;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus tick_turning(const SteadyClock::time_point &)
  {
    if (!result_future_.valid() || result_future_.wait_for(0ms) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    const auto result = result_future_.get();
    spin_goal_.reset();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      handle_turn_failure();
      return BT::NodeStatus::RUNNING;
    }
    if (final_turn_) {
      complete_recovery();
      return BT::NodeStatus::FAILURE;
    }
    begin_evaluation();
    return BT::NodeStatus::RUNNING;
  }

  void begin_evaluation()
  {
    phase_ = Phase::evaluating;
    evaluation_started_ = SteadyClock::now();
    clear_since_ = {};
    evaluation_score_ = 0.0;
    no_echo_seen_ = false;
  }

  BT::NodeStatus tick_evaluating(const SteadyClock::time_point & now)
  {
    if (!car_status_.obstacle_ok) {
      if (clear_since_ == SteadyClock::time_point{}) {
        clear_since_ = now;
      }
      if (!car_status_.ultrasonic_ok) {
        no_echo_seen_ = true;
      } else if (std::isfinite(car_status_.obstacle_distance) &&
        car_status_.obstacle_distance > 0.0F)
      {
        evaluation_score_ = std::max(
          evaluation_score_, static_cast<double>(car_status_.obstacle_distance));
      }
    } else {
      clear_since_ = {};
      evaluation_score_ = 0.0;
      no_echo_seen_ = false;
    }
    const bool clear = clear_since_ != SteadyClock::time_point{} &&
    std::chrono::duration<double>(now - clear_since_).count() >= clear_hold_sec_;
    const bool timed_out =
      std::chrono::duration<double>(now - evaluation_started_).count() >=
      evaluation_timeout_sec_;
    if (!clear && !timed_out) {
      return BT::NodeStatus::RUNNING;
    }
    Candidate candidate;
    candidate.clear = clear;
    candidate.score = no_echo_seen_ ? std::numeric_limits<double>::infinity() : evaluation_score_;
    candidate.offset = target_offset_;
    if (scanning_left_) {
      left_ = candidate;
      scanning_left_ = false;
      phase_ = Phase::waiting_stop;
      stopped_since_ = {};
      return BT::NodeStatus::RUNNING;
    }
    right_ = candidate;
    return finish_pair();
  }

  BT::NodeStatus finish_pair()
  {
    const auto selected = choose_clear_offset(left_, right_, clearance_tie_m_);
    if (selected.has_value()) {
      target_offset_ = selected.value();
      final_turn_ = true;
      phase_ = Phase::waiting_stop;
      stopped_since_ = {};
      return BT::NodeStatus::RUNNING;
    }
    ++range_index_;
    left_ = Candidate{};
    right_ = Candidate{};
    scanning_left_ = true;
    if (range_index_ >= ranges_.size()) {
      range_index_ = ranges_.size() - 1U;
      phase_ = Phase::waiting_retry;
      retry_until_ = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(retry_delay_sec_));
      publish_status(
        true, "waiting_retry", target_offset_, current_range(),
        "左右90度均未找到安全方向，停车后重新扩大扫描");
    } else {
      phase_ = Phase::waiting_stop;
      stopped_since_ = {};
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus tick_waiting_retry(const SteadyClock::time_point & now)
  {
    if (now < retry_until_) {
      return BT::NodeStatus::RUNNING;
    }
    range_index_ = 0U;
    scanning_left_ = true;
    final_turn_ = false;
    left_ = Candidate{};
    right_ = Candidate{};
    phase_ = Phase::waiting_stop;
    stopped_since_ = {};
    return BT::NodeStatus::RUNNING;
  }

  void handle_turn_failure()
  {
    spin_goal_.reset();
    if (final_turn_) {
      final_turn_ = false;
      left_ = Candidate{};
      right_ = Candidate{};
      scanning_left_ = true;
      if (range_index_ + 1U < ranges_.size()) {
        ++range_index_;
        phase_ = Phase::waiting_stop;
      } else {
        phase_ = Phase::waiting_retry;
        retry_until_ = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
          std::chrono::duration<double>(retry_delay_sec_));
      }
      stopped_since_ = {};
      return;
    }
    Candidate blocked;
    blocked.offset = target_offset_;
    if (scanning_left_) {
      left_ = blocked;
      scanning_left_ = false;
      phase_ = Phase::waiting_stop;
      stopped_since_ = {};
    } else {
      right_ = blocked;
      (void)finish_pair();
    }
  }

  void complete_recovery()
  {
    publish_status(
      false, "idle", target_offset_, current_range(),
      "近障左右扫描完成，正在重新规划原导航目标");
    reset(true);
  }

  void cancel_spin()
  {
    if (!spin_goal_ && send_goal_future_.valid()) {
      if (callback_executor_.spin_until_future_complete(send_goal_future_, 500ms) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        spin_goal_ = send_goal_future_.get();
      }
    }
    if (spin_goal_) {
      auto cancel_future = spin_client_->async_cancel_goal(spin_goal_);
      (void)callback_executor_.spin_until_future_complete(cancel_future, 500ms);
      spin_goal_.reset();
    }
    send_goal_future_ = {};
    result_future_ = {};
  }

  void reset(bool preserve_completion_message)
  {
    active_ = false;
    phase_ = Phase::idle;
    obstacle_since_ = {};
    stopped_since_ = {};
    clear_since_ = {};
    final_turn_ = false;
    if (!preserve_completion_message) {
      publish_status(false, "idle", 0.0, 0.0, "近障扫描空闲");
    }
  }

  void publish_status(
    bool active, const std::string & phase, double target, double range,
    const std::string & message)
  {
    last_status_active_ = active;
    last_status_phase_ = phase;
    last_status_target_ = target;
    last_status_range_ = range;
    last_status_message_ = message;
    last_status_publish_ = SteadyClock::now();
    car_interfaces::msg::ObstacleScanStatus status;
    status.stamp = node_->now();
    status.active = active;
    status.phase = phase;
    status.target_offset_rad = static_cast<float>(target);
    status.scan_range_rad = static_cast<float>(range);
    status.message = message;
    status_pub_->publish(status);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;
  rclcpp::Subscription<car_interfaces::msg::CarStatus>::SharedPtr car_status_sub_;
  rclcpp::Subscription<car_interfaces::msg::CarMcuStatus>::SharedPtr mcu_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<car_interfaces::msg::ObstacleScanStatus>::SharedPtr status_pub_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  car_interfaces::msg::CarStatus car_status_;
  car_interfaces::msg::CarMcuStatus mcu_status_;
  nav_msgs::msg::Odometry odom_;
  SteadyClock::time_point car_status_time_;
  SteadyClock::time_point mcu_status_time_;
  SteadyClock::time_point odom_time_;
  bool have_car_status_{false};
  bool have_mcu_status_{false};
  bool have_odom_{false};

  bool active_{false};
  bool scanning_left_{true};
  bool final_turn_{false};
  Phase phase_{Phase::idle};
  std::vector<double> ranges_;
  std::size_t range_index_{0U};
  Candidate left_;
  Candidate right_;
  double encounter_yaw_{0.0};
  double target_offset_{0.0};
  double evaluation_score_{0.0};
  bool no_echo_seen_{false};
  SteadyClock::time_point obstacle_since_;
  SteadyClock::time_point stopped_since_;
  SteadyClock::time_point evaluation_started_;
  SteadyClock::time_point clear_since_;
  SteadyClock::time_point retry_until_;
  std::shared_future<GoalHandleSpin::SharedPtr> send_goal_future_;
  std::shared_future<GoalHandleSpin::WrappedResult> result_future_;
  GoalHandleSpin::SharedPtr spin_goal_;

  double trigger_hold_sec_{0.20};
  double clear_hold_sec_{0.50};
  double evaluation_timeout_sec_{0.75};
  double scan_step_rad_{0.3490658504};
  double scan_limit_rad_{1.5707963268};
  double retry_delay_sec_{1.0};
  double stop_wheel_speed_mps_{0.01};
  double stopped_hold_sec_{0.50};
  double input_stale_timeout_sec_{0.50};
  double minimum_spin_rate_radps_{0.25};
  double spin_time_margin_sec_{2.0};
  double clearance_tie_m_{0.05};
  bool last_status_active_{false};
  std::string last_status_phase_{"idle"};
  double last_status_target_{0.0};
  double last_status_range_{0.0};
  std::string last_status_message_{"近障扫描空闲"};
  SteadyClock::time_point last_status_publish_;
};

}  // namespace car_navigation

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<car_navigation::ExpandingObstacleScan>("ExpandingObstacleScan");
}
