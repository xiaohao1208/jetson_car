#include "car_rl/model_contract.hpp"
#include "car_rl/observation.hpp"
#include "car_rl/observation_recording.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "car_interfaces/action/run_observation_collection.hpp"
#include "car_interfaces/msg/calibration_status.hpp"
#include "car_interfaces/msg/car_mcu_status.hpp"
#include "car_interfaces/msg/car_status.hpp"
#include "car_interfaces/msg/observation_collection_status.hpp"
#include "car_interfaces/msg/obstacle_scan_status.hpp"
#include "car_interfaces/srv/emergency_stop.hpp"
#include "car_observation/collection_support.hpp"
#include "car_observation/wheel_speed_safety_monitor.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace car_observation
{

using RunObservationCollection =
  car_interfaces::action::RunObservationCollection;
using GoalHandleCollection =
  rclcpp_action::ServerGoalHandle<RunObservationCollection>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandlePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
using SteadyClock = std::chrono::steady_clock;

class ObservationCollector : public rclcpp::Node
{
public:
  ObservationCollector()
  : Node("car_observation_collect"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    declare_and_load_parameters();
    create_interfaces();
    state_ = "idle";
    phase_ = "idle";
    message_ = "等待自动观测采集请求";
    publish_status();
  }

  ~ObservationCollector() override
  {
    shutting_down_.store(true);
    recording_.store(false);
  }

private:
  void declare_and_load_parameters()
  {
    vehicle_id_ = declare_parameter<std::string>("vehicle_id", "car01");
    output_root_ = declare_parameter<std::string>(
      "output_root", "src/car_observation/observations");
    output_root_ = resolve_project_path(output_root_).string();
    sample_hz_ = declare_parameter<double>("sample_hz", 15.0);
    target_samples_ = declare_parameter<int>("target_samples", 5000);
    max_raw_samples_ = declare_parameter<int>("max_raw_samples", 6500);
    max_duration_sec_ = declare_parameter<double>("max_duration_sec", 900.0);
    countdown_sec_ = declare_parameter<double>("countdown_sec", 5.0);
    stale_timeout_sec_ = declare_parameter<double>("input_stale_timeout_sec", 0.5);
    input_failure_sec_ = declare_parameter<double>("continuous_input_failure_sec", 2.0);
    nav_retry_count_ = declare_parameter<int>("nav_goal_retry_count", 1);
    nav_retry_delay_sec_ = declare_parameter<double>("nav_goal_retry_delay_sec", 2.0);
    stop_wheel_speed_ = declare_parameter<double>("stop_wheel_speed_mps", 0.01);
    stopped_hold_sec_ = declare_parameter<double>("stopped_hold_sec", 0.5);
    minimum_waypoints_ = declare_parameter<int>("minimum_waypoints", 3);
    minimum_waypoint_distance_ = declare_parameter<double>(
      "minimum_adjacent_waypoint_distance_m", 0.30);
    minimum_route_length_ = declare_parameter<double>("minimum_route_length_m", 2.0);
    requirements_.target_samples = static_cast<std::size_t>(target_samples_);
    requirements_.validation_prefix_samples = 4000U;
    const int minimum_unique = declare_parameter<int>("minimum_unique_samples", 100);
    const int minimum_straight = declare_parameter<int>("minimum_straight_samples", 500);
    const int minimum_left_turn = declare_parameter<int>("minimum_left_turn_samples", 200);
    const int minimum_right_turn = declare_parameter<int>("minimum_right_turn_samples", 200);
    const int minimum_near_goal = declare_parameter<int>("minimum_near_goal_samples", 200);
    warning_front_ = declare_parameter<int>("warning_front_near_samples", 100);
    warning_left_ = declare_parameter<int>("warning_left_near_samples", 100);
    warning_right_ = declare_parameter<int>("warning_right_near_samples", 100);
    warning_bilateral_ = declare_parameter<int>("warning_bilateral_near_samples", 50);
    thresholds_.near_goal_distance_m = declare_parameter<double>(
      "near_goal_distance_m", 0.35);
    thresholds_.straight_min_linear_mps = declare_parameter<double>(
      "straight_min_linear_mps", 0.02);
    thresholds_.straight_max_angular_radps = declare_parameter<double>(
      "straight_max_angular_radps", 0.10);
    thresholds_.turn_min_angular_radps = declare_parameter<double>(
      "turn_min_angular_radps", 0.15);
    thresholds_.near_obstacle_distance_m = declare_parameter<double>(
      "near_obstacle_distance_m", 0.60);
    thresholds_.bilateral_obstacle_distance_m = declare_parameter<double>(
      "bilateral_obstacle_distance_m", 0.80);
    max_feedback_wheel_speed_ = declare_parameter<double>(
      "max_feedback_wheel_speed_mps", 0.20);
    overspeed_hold_sec_ = declare_parameter<double>("overspeed_hold_sec", 0.20);
    immediate_overspeed_mps_ = declare_parameter<double>(
      "immediate_overspeed_mps", 0.30);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed_mps", 0.10);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed_radps", 1.047197551);

    const std::vector<double> positive_values{
      sample_hz_, max_duration_sec_, countdown_sec_, stale_timeout_sec_,
      input_failure_sec_, nav_retry_delay_sec_, stop_wheel_speed_, stopped_hold_sec_,
      minimum_waypoint_distance_, minimum_route_length_,
      thresholds_.near_goal_distance_m, thresholds_.straight_min_linear_mps,
      thresholds_.straight_max_angular_radps, thresholds_.turn_min_angular_radps,
      thresholds_.near_obstacle_distance_m,
      thresholds_.bilateral_obstacle_distance_m, max_feedback_wheel_speed_,
      overspeed_hold_sec_, immediate_overspeed_mps_,
      max_linear_speed_, max_angular_speed_};
    if (vehicle_id_.empty() ||
      !std::all_of(
        positive_values.begin(), positive_values.end(), [](double value) {
          return std::isfinite(value) && value > 0.0;
        }) || target_samples_ < 4000 || max_raw_samples_ < target_samples_ ||
      minimum_waypoints_ < 3 || nav_retry_count_ < 0 ||
      immediate_overspeed_mps_ <= max_feedback_wheel_speed_ ||
      minimum_unique <= 0 || minimum_straight <= 0 || minimum_left_turn <= 0 ||
      minimum_right_turn <= 0 || minimum_near_goal <= 0 ||
      warning_front_ <= 0 || warning_left_ <= 0 || warning_right_ <= 0 ||
      warning_bilateral_ <= 0)
    {
      throw std::runtime_error("自动观测采集配置包含无效数量、频率、距离或超时");
    }
    wheel_speed_monitor_.configure(
      max_feedback_wheel_speed_, overspeed_hold_sec_, immediate_overspeed_mps_);
    requirements_.minimum_unique_samples = static_cast<std::size_t>(minimum_unique);
    requirements_.minimum_straight_samples = static_cast<std::size_t>(minimum_straight);
    requirements_.minimum_left_turn_samples = static_cast<std::size_t>(minimum_left_turn);
    requirements_.minimum_right_turn_samples = static_cast<std::size_t>(minimum_right_turn);
    requirements_.minimum_near_goal_samples = static_cast<std::size_t>(minimum_near_goal);
    const std::filesystem::path output(output_root_);
    if (!output.is_absolute() || output == output.root_path()) {
      throw std::runtime_error("观测输出根目录必须是非根绝对路径");
    }
  }

  void create_interfaces()
  {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", sensor_qos, [this](sensor_msgs::msg::LaserScan::SharedPtr value) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        scan_ = *value; scan_time_ = SteadyClock::now(); has_scan_ = true;
      });
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", 10, [this](nav_msgs::msg::Path::SharedPtr value) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        plan_ = *value; plan_time_ = SteadyClock::now();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", sensor_qos, [this](nav_msgs::msg::Odometry::SharedPtr value) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        odom_ = *value; odom_time_ = SteadyClock::now(); has_odom_ = true;
      });
    nav_command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav", 10, [this](geometry_msgs::msg::Twist::SharedPtr value) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        nav_command_ = *value; nav_command_time_ = SteadyClock::now(); has_nav_command_ = true;
      });
    move_command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_move", sensor_qos, [this](geometry_msgs::msg::Twist::SharedPtr value) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        move_command_ = *value; move_command_time_ = SteadyClock::now(); has_move_command_ = true;
      });
    mcu_sub_ = create_subscription<car_interfaces::msg::CarMcuStatus>(
      "/car/mcu_status", sensor_qos,
      [this](car_interfaces::msg::CarMcuStatus::SharedPtr value) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        mcu_ = *value; mcu_time_ = SteadyClock::now(); has_mcu_ = true;
      });
    car_status_sub_ = create_subscription<car_interfaces::msg::CarStatus>(
      "/car/status", 10, [this](car_interfaces::msg::CarStatus::SharedPtr value) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        car_status_ = *value; car_status_time_ = SteadyClock::now(); has_car_status_ = true;
      });
    obstacle_scan_sub_ = create_subscription<car_interfaces::msg::ObstacleScanStatus>(
      "/car/navigation_obstacle_scan_status",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](car_interfaces::msg::ObstacleScanStatus::SharedPtr value) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        obstacle_scan_status_ = *value;
        obstacle_scan_time_ = SteadyClock::now();
        has_obstacle_scan_status_ = true;
      });
    calibration_sub_ = create_subscription<car_interfaces::msg::CalibrationStatus>(
      "/car/calibration_status", rclcpp::QoS(1).reliable().transient_local(),
      [this](car_interfaces::msg::CalibrationStatus::SharedPtr value) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        calibration_active_ = value->active; calibration_time_ = SteadyClock::now();
      });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr value) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        map_ = *value; has_map_ = value->info.width > 0U && value->info.height > 0U &&
        !value->data.empty();
      });
    nav_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", 10,
      [this](action_msgs::msg::GoalStatusArray::SharedPtr value) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        other_nav_active_ = false;
        for (const auto & status : value->status_list) {
          if (status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
          status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          status.status == action_msgs::msg::GoalStatus::STATUS_CANCELING)
          {
            other_nav_active_ = true;
          }
        }
      });

    status_pub_ = create_publisher<car_interfaces::msg::ObservationCollectionStatus>(
      "/car/observation_collection_status",
      rclcpp::QoS(1).reliable().transient_local());
    estop_client_ = create_client<car_interfaces::srv::EmergencyStop>("/car/e_stop");
    controller_params_client_ = create_client<rcl_interfaces::srv::GetParameters>(
      "/controller_server/get_parameters");
    navigate_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    path_client_ = rclcpp_action::create_client<ComputePathToPose>(
      this, "/compute_path_to_pose");
    action_server_ = rclcpp_action::create_server<RunObservationCollection>(
      this, "/car/run_observation_collection",
      std::bind(&ObservationCollector::on_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ObservationCollector::on_cancel, this, std::placeholders::_1),
      std::bind(&ObservationCollector::on_accepted, this, std::placeholders::_1));
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), [this]() {
        publish_status();
      });
    const auto sample_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / sample_hz_));
    sample_timer_ = create_wall_timer(sample_period, [this]() {sample_once();});
  }

  rclcpp_action::GoalResponse on_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const RunObservationCollection::Goal> goal)
  {
    if (!goal->route_area_clear || !goal->physical_estop_ready ||
      !goal->supervised || !goal->accepts_maximum_duration ||
      goal->waypoints.size() < static_cast<std::size_t>(minimum_waypoints_) ||
      !std::all_of(goal->waypoints.begin(), goal->waypoints.end(), finite_pose))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!goal_reserved_.compare_exchange_strong(expected, true)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandleCollection>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accepted(const std::shared_ptr<GoalHandleCollection> goal_handle)
  {
    std::thread([this, goal_handle]() {execute(goal_handle);}).detach();
  }

  bool recent(const SteadyClock::time_point & value, double timeout) const
  {
    return value != SteadyClock::time_point{} &&
           std::chrono::duration<double>(SteadyClock::now() - value).count() <= timeout;
  }

  bool message_stamp_fresh(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0U) {
      return false;
    }
    const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return age >= 0.0 && age <= stale_timeout_sec_;
  }

  void note_sample_failure(const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    last_sample_failure_ = reason;
  }

  void reset_sample_watchdog()
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    last_valid_sample_ = SteadyClock::now();
    last_sample_failure_.clear();
  }

  bool set_sampling_paused(bool paused)
  {
    const bool changed = sampling_paused_.exchange(paused) != paused;
    if (changed && !paused) {
      reset_sample_watchdog();
    }
    return changed;
  }

  void sample_once()
  {
    if (!recording_.load() || sampling_paused_.load()) {
      return;
    }
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Path plan;
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::Twist command;
    SteadyClock::time_point scan_time;
    SteadyClock::time_point odom_time;
    SteadyClock::time_point command_time;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (!has_scan_ || !has_odom_ || !has_nav_command_ || plan_.poses.empty()) {
        if (!has_scan_) {
          note_sample_failure("尚未收到/scan");
        } else if (!has_odom_) {
          note_sample_failure("尚未收到/odom");
        } else if (!has_nav_command_) {
          note_sample_failure("尚未收到/cmd_vel_nav");
        } else {
          note_sample_failure("/plan为空");
        }
        return;
      }
      scan = scan_; plan = plan_; odom = odom_; command = nav_command_;
      scan_time = scan_time_; odom_time = odom_time_; command_time = nav_command_time_;
    }
    if (!recent(scan_time, stale_timeout_sec_) || !recent(odom_time, stale_timeout_sec_) ||
      !recent(command_time, stale_timeout_sec_) ||
      !message_stamp_fresh(scan.header.stamp) || !message_stamp_fresh(odom.header.stamp))
    {
      if (!recent(scan_time, stale_timeout_sec_)) {
        note_sample_failure("/scan接收时间过期");
      } else if (!message_stamp_fresh(scan.header.stamp)) {
        note_sample_failure("/scan消息时间戳过期");
      } else if (!recent(odom_time, stale_timeout_sec_)) {
        note_sample_failure("/odom接收时间过期");
      } else if (!message_stamp_fresh(odom.header.stamp)) {
        note_sample_failure("/odom消息时间戳过期");
      } else {
        note_sample_failure("/cmd_vel_nav接收时间过期");
      }
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        plan.header.frame_id, "base_footprint", tf2::TimePointZero,
        tf2::durationFromSec(0.1));
      geometry_msgs::msg::PoseStamped robot_pose;
      robot_pose.header = transform.header;
      robot_pose.pose.position.x = transform.transform.translation.x;
      robot_pose.pose.position.y = transform.transform.translation.y;
      robot_pose.pose.position.z = transform.transform.translation.z;
      robot_pose.pose.orientation = transform.transform.rotation;
      car_rl::ControllerAction previous;
      {
        std::lock_guard<std::mutex> lock(sample_mutex_);
        previous = previous_action_;
      }
      const auto observation = car_rl::build_controller_observation(
        scan, plan, robot_pose, odom.twist.twist, previous);
      car_rl::RecordedObservation recorded;
      recorded.values = observation;
      recorded.coverage = car_rl::classify_observation(observation, thresholds_);
      {
        std::lock_guard<std::mutex> lock(sample_mutex_);
        if (samples_.size() >= static_cast<std::size_t>(max_raw_samples_)) {
          return;
        }
        recorded.sequence = samples_.size();
        samples_.push_back(recorded);
        previous_action_[0] = static_cast<float>(std::clamp(
            2.0 * command.linear.x / max_linear_speed_ - 1.0, -1.0, 1.0));
        previous_action_[1] = static_cast<float>(std::clamp(
            command.angular.z / max_angular_speed_, -1.0, 1.0));
        last_valid_sample_ = SteadyClock::now();
        last_sample_failure_.clear();
      }
    } catch (const std::exception & error) {
      note_sample_failure(error.what());
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "观测帧暂时无效：%s", error.what());
    }
  }

  void set_state(
    const std::string & state, const std::string & phase,
    const std::string & message, bool active = true)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_ = state; phase_ = phase; message_ = message; active_ = active;
    }
    publish_status();
  }

  void publish_status()
  {
    car_interfaces::msg::ObservationCollectionStatus status;
    std::vector<car_rl::RecordedObservation> samples;
    {
      std::lock_guard<std::mutex> lock(sample_mutex_);
      samples = samples_;
    }
    const auto counts = car_rl::count_coverage(samples);
    const auto unique = car_rl::count_exact_unique(samples);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      status.active = active_; status.state = state_; status.phase = phase_;
      status.route_loops = route_loops_; status.current_waypoint = current_waypoint_;
      status.total_waypoints = total_waypoints_; status.run_id = run_id_;
      status.result_directory = result_directory_; status.message = message_;
    }
    status.stamp = now();
    status.collected_samples = static_cast<std::uint32_t>(samples.size());
    status.target_samples = static_cast<std::uint32_t>(target_samples_);
    status.unique_samples = static_cast<std::uint32_t>(unique);
    status.straight_samples = static_cast<std::uint32_t>(counts.straight);
    status.left_turn_samples = static_cast<std::uint32_t>(counts.left_turn);
    status.right_turn_samples = static_cast<std::uint32_t>(counts.right_turn);
    status.near_goal_samples = static_cast<std::uint32_t>(counts.near_goal);
    status.front_near_samples = static_cast<std::uint32_t>(counts.front_near);
    status.left_near_samples = static_cast<std::uint32_t>(counts.left_near);
    status.right_near_samples = static_cast<std::uint32_t>(counts.right_near);
    status.bilateral_near_samples = static_cast<std::uint32_t>(counts.bilateral_near);
    const auto ratio = [](std::size_t value, std::size_t target) {
        return target == 0U ? 1.0 : std::min(1.0, static_cast<double>(value) / target);
      };
    status.progress = static_cast<float>(std::min(
      {
        ratio(samples.size(), requirements_.target_samples),
        ratio(unique, requirements_.minimum_unique_samples),
        ratio(counts.straight, requirements_.minimum_straight_samples),
        ratio(counts.left_turn, requirements_.minimum_left_turn_samples),
        ratio(counts.right_turn, requirements_.minimum_right_turn_samples),
        ratio(counts.near_goal, requirements_.minimum_near_goal_samples)}));
    if (status.state == "succeeded") {
      status.progress = 1.0F;
    }
    // 可选近障覆盖不参与成败；样本量尚未达到目标时也不显示“场景不足”，
    // 避免预检失败或主动取消被误解为覆盖警告导致的停车。
    if (samples.size() >= requirements_.target_samples) {
      if (counts.front_near < static_cast<std::size_t>(warning_front_)) {
        status.warnings.push_back("正前近障场景不足（可选覆盖）");
      }
      if (counts.left_near < static_cast<std::size_t>(warning_left_)) {
        status.warnings.push_back("左侧近障场景不足（可选覆盖）");
      }
      if (counts.right_near < static_cast<std::size_t>(warning_right_)) {
        status.warnings.push_back("右侧近障场景不足（可选覆盖）");
      }
      if (counts.bilateral_near < static_cast<std::size_t>(warning_bilateral_)) {
        status.warnings.push_back("双侧近障场景不足（可选覆盖）");
      }
    }
    status_pub_->publish(status);
  }

  void publish_feedback(const std::shared_ptr<GoalHandleCollection> & goal_handle)
  {
    auto feedback = std::make_shared<RunObservationCollection::Feedback>();
    car_interfaces::msg::ObservationCollectionStatus status;
    std::vector<car_rl::RecordedObservation> samples;
    {
      std::lock_guard<std::mutex> lock(sample_mutex_); samples = samples_;
    }
    const auto unique = car_rl::count_exact_unique(samples);
    const auto counts = car_rl::count_coverage(samples);
    std::lock_guard<std::mutex> lock(state_mutex_);
    feedback->state = state_; feedback->phase = phase_;
    feedback->collected_samples = static_cast<std::uint32_t>(samples.size());
    feedback->target_samples = static_cast<std::uint32_t>(target_samples_);
    feedback->unique_samples = static_cast<std::uint32_t>(unique);
    feedback->route_loops = route_loops_; feedback->current_waypoint = current_waypoint_;
    feedback->total_waypoints = total_waypoints_;
    const auto ratio = [](std::size_t value, std::size_t target) {
        return target == 0U ? 1.0 : std::min(1.0, static_cast<double>(value) / target);
      };
    feedback->progress = static_cast<float>(std::min(
      {
        ratio(samples.size(), requirements_.target_samples),
        ratio(unique, requirements_.minimum_unique_samples),
        ratio(counts.straight, requirements_.minimum_straight_samples),
        ratio(counts.left_turn, requirements_.minimum_left_turn_samples),
        ratio(counts.right_turn, requirements_.minimum_right_turn_samples),
        ratio(counts.near_goal, requirements_.minimum_near_goal_samples)}));
    feedback->message = message_;
    goal_handle->publish_feedback(feedback);
  }

  void call_estop(bool stop, const std::string & reason)
  {
    if (!estop_client_->wait_for_service(std::chrono::seconds(2))) {
      throw CollectionError("estop_unavailable", "急停服务不可用");
    }
    auto request = std::make_shared<car_interfaces::srv::EmergencyStop::Request>();
    request->stop = stop; request->source = "car_observation_collect"; request->reason = reason;
    auto future = estop_client_->async_send_request(request);
    if (!future.valid() ||
      future.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
    {
      throw CollectionError("estop_failed", "观测采集安全锁操作失败");
    }
    const auto response = future.get();
    if (!response || !response->success) {
      throw CollectionError("estop_failed", "观测采集安全锁操作失败");
    }
    if (!stop) {
      // 服务响应会先于 /car/status 的下一帧到达。等待状态确认可避免把刚刚
      // 解除的本程序启动锁误判为外部急停。
      const auto deadline = SteadyClock::now() + std::chrono::seconds(1);
      while (SteadyClock::now() < deadline) {
        bool unlocked = false;
        bool other_estop = false;
        {
          std::lock_guard<std::mutex> lock(input_mutex_);
          unlocked = has_car_status_ && !car_status_.e_stop_ok;
          other_estop = has_car_status_ && car_status_.e_stop_ok &&
            car_status_.e_stop_source != "car_observation_collect";
        }
        if (unlocked) {
          return;
        }
        if (other_estop) {
          throw CollectionError("external_estop", "解除观测启动锁时检测到外部急停");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      throw CollectionError("estop_failed", "观测启动锁解除后状态未在1秒内确认");
    }
  }

  void validate_route(const std::vector<geometry_msgs::msg::PoseStamped> & waypoints)
  {
    double route_length = 0.0;
    for (std::size_t index = 0U; index < waypoints.size(); ++index) {
      if (!finite_pose(waypoints[index])) {
        throw CollectionError("invalid_waypoint", "航点必须是map坐标系有限归一化位姿");
      }
      if (index + 1U < waypoints.size()) {
        const double distance = pose_distance(waypoints[index], waypoints[index + 1U]);
        if (distance < minimum_waypoint_distance_) {
          throw CollectionError(
                  "waypoints_too_close", "相邻航点距离小于安全门限0.30米");
        }
        route_length += distance;
      }
    }
    route_length += pose_distance(waypoints.back(), waypoints.front());
    if (route_length < minimum_route_length_) {
      throw CollectionError("route_too_short", "循环路线总长度小于2米");
    }
    route_length_ = route_length;
  }

  void check_controller_plugin()
  {
    if (!controller_params_client_->wait_for_service(std::chrono::seconds(2))) {
      throw CollectionError("controller_unavailable", "controller_server参数服务不可用");
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("FollowPath.plugin");
    auto future = controller_params_client_->async_send_request(request);
    if (!future.valid() ||
      future.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
    {
      throw CollectionError(
              "controller_unavailable", "读取controller_server经典控制器参数超时");
    }
    // Humble 的服务 FutureAndRequestId 内部持有 std::future，get() 只能调用
    // 一次。缓存响应后再完成全部校验，避免 std::future_error(no_state)。
    const auto response = future.get();
    if (!response || response->values.size() != 1U ||
      response->values.front().string_value != "dwb_core::DWBLocalPlanner")
    {
      throw CollectionError(
              "not_classic_navigation", "观测采集必须使用经典DWB导航，不能使用RL控制器");
    }
  }

  void check_path(
    const geometry_msgs::msg::PoseStamped * start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & segment)
  {
    ComputePathToPose::Goal request;
    request.goal = goal;
    request.use_start = start != nullptr;
    if (start != nullptr) {
      request.start = *start;
    }
    auto handle_future = path_client_->async_send_goal(request);
    if (!handle_future.valid() ||
      handle_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
    {
      throw CollectionError("route_timeout", "路线航段" + segment + "规划请求超时");
    }
    const auto path_goal = handle_future.get();
    if (!path_goal) {
      throw CollectionError("route_unreachable", "路线航段" + segment + "规划请求被拒绝");
    }
    auto result_future = path_client_->async_get_result(path_goal);
    if (!result_future.valid() ||
      result_future.wait_for(std::chrono::seconds(8)) != std::future_status::ready)
    {
      path_client_->async_cancel_goal(path_goal);
      throw CollectionError("route_timeout", "路线航段" + segment + "规划超时");
    }
    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !result.result || result.result->path.poses.empty())
    {
      throw CollectionError("route_unreachable", "路线航段" + segment + "不可达");
    }
  }

  void preflight(const std::vector<geometry_msgs::msg::PoseStamped> & waypoints)
  {
    validate_route(waypoints);
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (has_car_status_ && car_status_.e_stop_ok) {
        throw CollectionError("preexisting_estop", "已有急停或程序安全锁，请先人工确认并解除");
      }
    }
    call_estop(true, "自动观测采集启动锁止");
    const auto deadline = SteadyClock::now() + std::chrono::seconds(8);
    bool inputs_ready = false;
    while (SteadyClock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(input_mutex_);
        inputs_ready = has_scan_ && has_odom_ && has_mcu_ && has_car_status_ &&
          has_move_command_ && has_map_ && recent(scan_time_, stale_timeout_sec_) &&
          recent(odom_time_, stale_timeout_sec_) && recent(mcu_time_, stale_timeout_sec_) &&
          recent(car_status_time_, stale_timeout_sec_) &&
          recent(move_command_time_, stale_timeout_sec_);
        if (calibration_active_) {
          throw CollectionError("calibration_active", "自动标定正在运行，不能开始观测");
        }
        if (other_nav_active_) {
          throw CollectionError("navigation_active", "已有导航目标正在运行");
        }
      }
      if (inputs_ready) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!inputs_ready) {
      throw CollectionError(
              "preflight_topics",
              "地图或观测所需scan/odom/底盘状态话题未就绪或已过期");
    }
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (!has_map_) {
        throw CollectionError("map_missing", "没有可用地图，请先建图并保存或加载地图");
      }
      if (!has_scan_ || !has_odom_ || !has_mcu_ || !has_car_status_ || !has_move_command_) {
        throw CollectionError("preflight_topics", "观测所需scan/odom/底盘状态话题未就绪");
      }
      if (!car_status_.mcu_ok || !mcu_.wifi_connect_ok || !mcu_.agent_connect_ok ||
        !mcu_.encoder_ok || !mcu_.motor_driver_ok)
      {
        throw CollectionError("health_gate", "底盘硬件或通信健康门禁未通过");
      }
    }
    wait_stopped();
    if (get_publishers_info_by_topic("/cmd_vel_nav").empty()) {
      throw CollectionError("nav_command_missing", "经典Nav2尚未提供/cmd_vel_nav发布者");
    }
    if (!navigate_client_->wait_for_action_server(std::chrono::seconds(3)) ||
      !path_client_->wait_for_action_server(std::chrono::seconds(3)))
    {
      throw CollectionError(
              "nav2_unavailable", "Nav2导航或路径规划Action尚未就绪");
    }
    try {
      (void)tf_buffer_.lookupTransform(
        "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.5));
    } catch (const std::exception & error) {
      throw CollectionError("localization_missing", std::string("地图定位TF不可用：") + error.what());
    }
    check_controller_plugin();
    check_path(nullptr, waypoints.front(), "当前位置→1");
    for (std::size_t index = 0U; index + 1U < waypoints.size(); ++index) {
      check_path(
        &waypoints[index], waypoints[index + 1U],
        std::to_string(index + 1U) + "→" + std::to_string(index + 2U));
    }
    check_path(
      &waypoints.back(), waypoints.front(),
      std::to_string(waypoints.size()) + "→1");
  }

  void check_runtime_safety(
    const std::shared_ptr<GoalHandleCollection> & goal_handle,
    bool enforce_sample_watchdog = true)
  {
    if (shutting_down_.load()) {
      throw CollectionError("shutdown", "观测节点正在退出");
    }
    if (goal_handle->is_canceling()) {
      throw CollectionError("canceled", "观测采集已被操作者取消");
    }
    const auto elapsed = std::chrono::duration<double>(SteadyClock::now() - run_started_).count();
    if (elapsed >= max_duration_sec_) {
      throw CollectionError("failed_quality", "达到15分钟上限仍未满足核心覆盖门禁");
    }
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (!recent(mcu_time_, stale_timeout_sec_) || !recent(car_status_time_, stale_timeout_sec_)) {
        throw CollectionError("status_timeout", "底盘安全状态连续过期");
      }
      if (!has_scan_ || !recent(scan_time_, input_failure_sec_)) {
        throw CollectionError("input_timeout", "连续2秒未收到新鲜/scan");
      }
      if (!has_odom_ || !recent(odom_time_, input_failure_sec_)) {
        throw CollectionError("input_timeout", "连续2秒未收到新鲜/odom");
      }
      if (car_status_.e_stop_ok) {
        throw CollectionError("external_estop", "检测到外部急停或其它程序安全锁");
      }
      if (!car_status_.mcu_ok || !mcu_.wifi_connect_ok || !mcu_.agent_connect_ok ||
        !mcu_.encoder_ok || !mcu_.motor_driver_ok)
      {
        throw CollectionError("health_lost", "底盘硬件或通信健康状态失效");
      }
      const auto speed_safety = wheel_speed_monitor_.update(
        mcu_.left_wheel_speed, mcu_.right_wheel_speed,
        std::chrono::duration<double>(mcu_time_.time_since_epoch()).count());
      if (speed_safety.state == WheelSpeedSafetyState::invalid_feedback) {
        throw CollectionError("overspeed", "车轮反馈速度不是有限数，观测采集已安全终止");
      }
      if (speed_safety.state == WheelSpeedSafetyState::immediate_overspeed ||
        speed_safety.state == WheelSpeedSafetyState::sustained_overspeed)
      {
        std::ostringstream message;
        message << std::fixed << std::setprecision(3)
                << "车轮反馈超速：左轮=" << mcu_.left_wheel_speed
                << " m/s，右轮=" << mcu_.right_wheel_speed
                << " m/s，持续=" << speed_safety.overspeed_duration_sec
                << " s；持续上限=" << max_feedback_wheel_speed_
                << " m/s，立即停止上限=" << immediate_overspeed_mps_ << " m/s";
        throw CollectionError("overspeed", message.str());
      }
      if (speed_safety.state == WheelSpeedSafetyState::pending) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "车轮反馈短时超过观测上限：左轮=%.3f m/s，右轮=%.3f m/s，持续=%.3f/%.3f s",
          mcu_.left_wheel_speed, mcu_.right_wheel_speed,
          speed_safety.overspeed_duration_sec, overspeed_hold_sec_);
      }
      if (calibration_active_) {
        throw CollectionError("calibration_active", "运行中检测到自动标定任务");
      }
    }
    std::lock_guard<std::mutex> lock(sample_mutex_);
    if (samples_.size() >= static_cast<std::size_t>(max_raw_samples_) && !quality_ready_locked()) {
      throw CollectionError("failed_quality", "达到6500条原始样本仍未满足核心覆盖门禁");
    }
    if (enforce_sample_watchdog &&
      std::chrono::duration<double>(SteadyClock::now() - last_valid_sample_).count() >=
      input_failure_sec_)
    {
      const std::string reason = last_sample_failure_.empty() ?
        "未知观测输入" : last_sample_failure_;
      throw CollectionError(
              "input_timeout", "连续2秒无法生成新鲜有效观测：" + reason);
    }
  }

  bool obstacle_scan_active(std::string * message = nullptr)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    const bool active = has_obstacle_scan_status_ &&
      recent(obstacle_scan_time_, std::max(stale_timeout_sec_, 1.0)) &&
      obstacle_scan_status_.active;
    if (active && message != nullptr) {
      *message = obstacle_scan_status_.message;
    }
    return active;
  }

  bool quality_ready_locked() const
  {
    if (samples_.size() < requirements_.target_samples) {
      return false;
    }
    return car_rl::coverage_requirements_met(
      car_rl::count_coverage(samples_), car_rl::count_exact_unique(samples_), requirements_);
  }

  bool quality_ready()
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    return quality_ready_locked();
  }

  NavigationOutcome navigate_once(
    const geometry_msgs::msg::PoseStamped & waypoint,
    const std::shared_ptr<GoalHandleCollection> & collection_goal)
  {
    NavigateToPose::Goal goal;
    goal.pose = waypoint;
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    const auto goal_sent_at = SteadyClock::now();
    auto future = navigate_client_->async_send_goal(goal, options);
    if (!future.valid() ||
      future.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
    {
      return NavigationOutcome::failed;
    }
    const auto navigation_goal = future.get();
    if (!navigation_goal) {return NavigationOutcome::failed;}
    {
      std::lock_guard<std::mutex> lock(active_nav_mutex_);
      active_nav_goal_ = navigation_goal;
    }
    auto result_future = navigate_client_->async_get_result(navigation_goal);
    if (!result_future.valid()) {
      throw CollectionError("navigation_future", "NavigateToPose结果Future无效");
    }
    bool command_seen = false;
    while (result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
      bool command_fresh = false;
      std::string scan_message;
      const bool scan_active = obstacle_scan_active(&scan_message);
      {
        std::lock_guard<std::mutex> lock(input_mutex_);
        command_fresh = has_nav_command_ && nav_command_time_ >= goal_sent_at &&
          recent(nav_command_time_, stale_timeout_sec_);
      }
      if (scan_active) {
        if (set_sampling_paused(true)) {
          reset_sample_watchdog();
        }
        set_state(
          "collecting", "obstacle_scanning",
          scan_message.empty() ? "Nav2正在进行近障左右扫描，观测采样已暂停" : scan_message);
      } else if (command_fresh) {
        command_seen = true;
        if (set_sampling_paused(false)) {
          set_state("collecting", "navigating", "经典Nav2正在循环路线并采集观测");
        }
      } else {
        if (!command_seen &&
          std::chrono::duration<double>(SteadyClock::now() - goal_sent_at).count() >=
          input_failure_sec_)
        {
          throw CollectionError(
                  "nav_command_timeout", "导航目标接受后2秒内未收到/cmd_vel_nav");
        }
        if (set_sampling_paused(true) && command_seen) {
          set_state(
            "collecting", "nav_recovering",
            "Nav2正在恢复或重新规划，观测采样已暂停");
        }
      }
      check_runtime_safety(collection_goal, command_fresh && !scan_active);
      publish_feedback(collection_goal);
      if (quality_ready()) {
        navigate_client_->async_cancel_goal(navigation_goal);
        set_sampling_paused(true);
        {
          std::lock_guard<std::mutex> lock(active_nav_mutex_);
          active_nav_goal_.reset();
        }
        return NavigationOutcome::quality_ready;
      }
    }
    {
      std::lock_guard<std::mutex> lock(active_nav_mutex_);
      active_nav_goal_.reset();
    }
    set_sampling_paused(true);
    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED ?
           NavigationOutcome::reached : NavigationOutcome::failed;
  }

  void cancel_active_motion()
  {
    std::shared_ptr<GoalHandleNavigate> navigation_goal;
    {
      std::lock_guard<std::mutex> lock(active_nav_mutex_);
      navigation_goal = active_nav_goal_;
    }
    if (navigation_goal) {
      auto future = navigate_client_->async_cancel_goal(navigation_goal);
      (void)future.wait_for(std::chrono::seconds(2));
    }
    {
      std::lock_guard<std::mutex> lock(active_nav_mutex_);
      active_nav_goal_.reset();
    }
  }

  void wait_stopped()
  {
    auto stable_since = SteadyClock::time_point{};
    const auto deadline = SteadyClock::now() + std::chrono::seconds(5);
    while (SteadyClock::now() < deadline) {
      bool stopped = false;
      {
        std::lock_guard<std::mutex> lock(input_mutex_);
        stopped = has_mcu_ && recent(mcu_time_, stale_timeout_sec_) &&
          std::max(std::abs(mcu_.left_wheel_speed), std::abs(mcu_.right_wheel_speed)) <
          stop_wheel_speed_;
      }
      if (stopped) {
        if (stable_since == SteadyClock::time_point{}) {stable_since = SteadyClock::now();}
        if (std::chrono::duration<double>(SteadyClock::now() - stable_since).count() >=
          stopped_hold_sec_)
        {
          return;
        }
      } else {
        stable_since = SteadyClock::time_point{};
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw CollectionError("failed_stop", "车轮未能在限定时间内稳定停止");
  }

  void safe_stop(const std::string & reason)
  {
    recording_.store(false);
    set_sampling_paused(true);
    cancel_active_motion();
    call_estop(true, reason);
    wait_stopped();
  }

  std::vector<std::string> warnings_for(
    const car_rl::ObservationCoverageCounts & counts) const
  {
    std::vector<std::string> warnings;
    if (counts.front_near < static_cast<std::size_t>(warning_front_)) {
      warnings.push_back("front_near coverage below optional target");
    }
    if (counts.left_near < static_cast<std::size_t>(warning_left_)) {
      warnings.push_back("left_near coverage below optional target");
    }
    if (counts.right_near < static_cast<std::size_t>(warning_right_)) {
      warnings.push_back("right_near coverage below optional target");
    }
    if (counts.bilateral_near < static_cast<std::size_t>(warning_bilateral_)) {
      warnings.push_back("bilateral_near coverage below optional target");
    }
    return warnings;
  }

  std::filesystem::path prepare_staging()
  {
    const std::filesystem::path root(output_root_);
    std::filesystem::create_directories(root);
    fsync_path(root, true);
    const auto pending = root / ("." + run_id_ + ".pending");
    const auto success = root / run_id_;
    if (std::filesystem::exists(pending) || std::filesystem::exists(success)) {
      throw CollectionError("output_exists", "观测run目录或pending目录已存在，拒绝覆盖");
    }
    const std::string failed_prefix = run_id_ + "_failed_";
    for (const auto & entry : std::filesystem::directory_iterator(root)) {
      if (entry.path().filename().string().rfind(failed_prefix, 0U) == 0U) {
        throw CollectionError("output_exists", "同一观测run的失败目录已存在，拒绝覆盖");
      }
    }
    std::filesystem::create_directory(pending);
    return pending;
  }

  void write_success(
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
    const std::vector<std::size_t> & selected)
  {
    std::vector<car_rl::RecordedObservation> samples;
    {
      std::lock_guard<std::mutex> lock(sample_mutex_); samples = samples_;
    }
    const auto pending = prepare_staging();
    const auto csv_pending = pending / "real_observations.csv.pending";
    const auto metadata_pending = pending / "real_observations.metadata.yaml.pending";
    const auto coverage_pending = pending / "coverage.json.pending";
    {
      std::ofstream csv(csv_pending, std::ios::out | std::ios::trunc);
      car_rl::write_observation_csv_header(csv);
      for (const auto index : selected) {
        car_rl::write_observation_csv_row(csv, samples.at(index).values);
      }
      csv.flush();
      if (!csv) {throw CollectionError("write_failed", "写入观测CSV失败");}
    }
    fsync_path(csv_pending);
    const std::string digest = car_rl::sha256_file(csv_pending);
    YAML::Emitter yaml;
    yaml << YAML::BeginMap
         << YAML::Key << "schema_version" << YAML::Value << 1
         << YAML::Key << "source" << YAML::Value << "real_car_raw"
         << YAML::Key << "completed" << YAML::Value << true
         << YAML::Key << "observation_contract_version" << YAML::Value << 1
         << YAML::Key << "observation_size" << YAML::Value << 86
         << YAML::Key << "count" << YAML::Value << selected.size()
         << YAML::Key << "sample_hz" << YAML::Value << sample_hz_
         << YAML::Key << "csv_sha256" << YAML::Value << digest
         << YAML::Key << "scan_topic" << YAML::Value << "/scan"
         << YAML::Key << "plan_topic" << YAML::Value << "/plan"
         << YAML::Key << "odom_topic" << YAML::Value << "/odom"
         << YAML::Key << "command_topic" << YAML::Value << "/cmd_vel_nav"
         << YAML::Key << "run_id" << YAML::Value << run_id_
         << YAML::Key << "selection_algorithm" << YAML::Value << "coverage_balanced_v1"
         << YAML::Key << "coverage_file" << YAML::Value << "coverage.json"
         << YAML::EndMap;
    {
      std::ofstream metadata(metadata_pending, std::ios::out | std::ios::trunc);
      metadata << yaml.c_str() << '\n'; metadata.flush();
      if (!metadata) {throw CollectionError("write_failed", "写入观测metadata失败");}
    }
    fsync_path(metadata_pending);

    std::vector<std::size_t> prefix(
      selected.begin(), selected.begin() + static_cast<std::ptrdiff_t>(
        requirements_.validation_prefix_samples));
    nlohmann::json waypoint_json = nlohmann::json::array();
    for (const auto & waypoint : waypoints) {
      waypoint_json.push_back(
        {
          {"frame_id", waypoint.header.frame_id},
          {"position", {waypoint.pose.position.x, waypoint.pose.position.y,
              waypoint.pose.position.z}},
          {"orientation", {waypoint.pose.orientation.x, waypoint.pose.orientation.y,
              waypoint.pose.orientation.z, waypoint.pose.orientation.w}},
        });
    }
    const auto raw_counts = car_rl::count_coverage(samples);
    const auto final_counts = car_rl::count_coverage(samples, &selected);
    const auto warning_values = warnings_for(raw_counts);
    nlohmann::json coverage{
      {"schema_version", 1},
      {"selection_algorithm", "coverage_balanced_v1"},
      {"run_id", run_id_},
      {"started_at", run_started_text_},
      {"finished_at", local_timestamp(false)},
      {"outcome", "succeeded"},
      {"route", {{"length_m", route_length_}, {"waypoints", waypoint_json}}},
      {"thresholds", {
          {"target_samples", requirements_.target_samples},
          {"minimum_unique_samples", requirements_.minimum_unique_samples},
          {"minimum_straight_samples", requirements_.minimum_straight_samples},
          {"minimum_left_turn_samples", requirements_.minimum_left_turn_samples},
          {"minimum_right_turn_samples", requirements_.minimum_right_turn_samples},
          {"minimum_near_goal_samples", requirements_.minimum_near_goal_samples}}},
      {"raw", {{"count", samples.size()},
          {"exact_unique", car_rl::count_exact_unique(samples)},
          {"quantized_unique_1e-3", car_rl::count_quantized_unique(samples)},
          {"coverage", counts_json(raw_counts)}}},
      {"first_4000", {{"count", prefix.size()},
          {"exact_unique", car_rl::count_exact_unique(samples, &prefix)},
          {"coverage", counts_json(car_rl::count_coverage(samples, &prefix))}}},
      {"final_5000", {{"count", selected.size()},
          {"exact_unique", car_rl::count_exact_unique(samples, &selected)},
          {"coverage", counts_json(final_counts)}}},
      {"warnings", warning_values},
    };
    {
      std::ofstream output(coverage_pending, std::ios::out | std::ios::trunc);
      output << coverage.dump(2) << '\n'; output.flush();
      if (!output) {throw CollectionError("write_failed", "写入coverage.json失败");}
    }
    fsync_path(coverage_pending);
    const auto csv = pending / "real_observations.csv";
    const auto metadata = pending / "real_observations.metadata.yaml";
    const auto coverage_file = pending / "coverage.json";
    std::filesystem::rename(csv_pending, csv);
    std::filesystem::rename(metadata_pending, metadata);
    std::filesystem::rename(coverage_pending, coverage_file);
    fsync_path(pending, true);
    const auto final = std::filesystem::path(output_root_) / run_id_;
    std::filesystem::rename(pending, final);
    fsync_path(std::filesystem::path(output_root_), true);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      result_directory_ = final.string();
      observations_csv_ = (final / "real_observations.csv").string();
      metadata_yaml_ = (final / "real_observations.metadata.yaml").string();
      coverage_json_ = (final / "coverage.json").string();
    }
  }

  void write_failure(const std::string & code, const std::string & message)
  {
    std::vector<car_rl::RecordedObservation> samples;
    {
      std::lock_guard<std::mutex> lock(sample_mutex_); samples = samples_;
    }
    if (samples.empty()) {return;}
    const auto pending = prepare_staging();
    const auto csv_pending = pending / "real_observations.csv.pending";
    {
      std::ofstream csv(csv_pending, std::ios::out | std::ios::trunc);
      car_rl::write_observation_csv_header(csv);
      for (const auto & sample : samples) {car_rl::write_observation_csv_row(csv, sample.values);}
      csv.flush();
      if (!csv) {throw CollectionError("write_failed", "写入失败审计CSV失败");}
    }
    fsync_path(csv_pending);
    const auto csv = pending / "real_observations.csv";
    std::filesystem::rename(csv_pending, csv);
    nlohmann::json failure{
      {"schema_version", 1}, {"run_id", run_id_}, {"completed", false},
      {"outcome", code}, {"message", message}, {"count", samples.size()},
      {"started_at", run_started_text_}, {"finished_at", local_timestamp(false)},
      {"coverage", counts_json(car_rl::count_coverage(samples))},
      {"exact_unique", car_rl::count_exact_unique(samples)},
    };
    const auto failure_path = pending / "failure.json";
    {
      std::ofstream output(failure_path, std::ios::out | std::ios::trunc);
      output << failure.dump(2) << '\n'; output.flush();
    }
    fsync_path(failure_path); fsync_path(pending, true);
    std::string safe_code = code;
    std::replace_if(
      safe_code.begin(), safe_code.end(), [](char value) {
        return !(std::isalnum(static_cast<unsigned char>(value)) || value == '_');
      }, '_');
    const auto final = std::filesystem::path(output_root_) /
      (run_id_ + "_failed_" + safe_code);
    if (std::filesystem::exists(final)) {
      throw CollectionError("output_exists", "失败审计目录已存在，拒绝覆盖");
    }
    std::filesystem::rename(pending, final);
    fsync_path(std::filesystem::path(output_root_), true);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      result_directory_ = final.string();
      observations_csv_ = (final / "real_observations.csv").string();
    }
  }

  void execute(const std::shared_ptr<GoalHandleCollection> goal_handle)
  {
    auto result = std::make_shared<RunObservationCollection::Result>();
    std::string outcome = "failed";
    std::string terminal_message;
    bool canceled = false;
    bool successful = false;
    try {
      const auto waypoints = goal_handle->get_goal()->waypoints;
      {
        std::lock_guard<std::mutex> lock(sample_mutex_);
        samples_.clear(); previous_action_ = {0.0F, 0.0F};
        last_valid_sample_ = SteadyClock::now();
        last_sample_failure_.clear();
      }
      {
        std::lock_guard<std::mutex> lock(input_mutex_);
        wheel_speed_monitor_.reset();
      }
      sampling_paused_.store(true);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        run_id_ = local_timestamp(true) + "_" + vehicle_id_;
        result_directory_.clear(); observations_csv_.clear(); metadata_yaml_.clear();
        coverage_json_.clear(); route_loops_ = 0U; current_waypoint_ = 0U;
        total_waypoints_ = static_cast<std::uint32_t>(waypoints.size());
      }
      run_started_ = SteadyClock::now(); run_started_text_ = local_timestamp(false);
      set_state("preflight", "preflight", "正在检查地图、定位、经典导航和路线");
      preflight(waypoints);
      set_state("countdown", "countdown", "安全检查通过，5秒后开始自动循环路线");
      const auto countdown_end = SteadyClock::now() +
        std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(countdown_sec_));
      while (SteadyClock::now() < countdown_end) {
        if (goal_handle->is_canceling()) {
          throw CollectionError("canceled", "倒计时期间已取消观测采集");
        }
        publish_feedback(goal_handle);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      call_estop(false, "自动观测采集开始，交由经典Nav2安全链控制");
      recording_.store(true);
      set_sampling_paused(true);
      reset_sample_watchdog();
      set_state("collecting", "navigating", "经典Nav2正在循环路线并采集观测");
      std::size_t waypoint = 0U;
      while (!quality_ready()) {
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          current_waypoint_ = static_cast<std::uint32_t>(waypoint);
        }
        bool reached = false;
        int failed_attempts = 0;
        while (!reached && !quality_ready()) {
          if (failed_attempts > 0) {
            set_state("collecting", "goal_retry", "导航失败，停车并重新规划后重试");
            set_sampling_paused(true);
            wait_stopped();
            const auto delay_end = SteadyClock::now() +
              std::chrono::duration_cast<SteadyClock::duration>(
              std::chrono::duration<double>(nav_retry_delay_sec_));
            while (SteadyClock::now() < delay_end) {
              check_runtime_safety(goal_handle, false);
              std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            check_path(nullptr, waypoints[waypoint], "重试→" + std::to_string(waypoint + 1U));
            set_state("collecting", "navigating", "重新规划成功，继续经典导航采集");
          }
          const auto navigation_outcome = navigate_once(
            waypoints[waypoint], goal_handle);
          if (navigation_outcome == NavigationOutcome::quality_ready || quality_ready()) {
            break;
          }
          if (navigation_outcome == NavigationOutcome::reached) {
            reached = true;
            break;
          }
          ++failed_attempts;
          if (failed_attempts > nav_retry_count_) {
            break;
          }
        }
        if (quality_ready()) {break;}
        if (!reached) {
          throw CollectionError("navigation_failed", "NavigateToPose连续两次失败");
        }
        if (waypoint + 1U == waypoints.size()) {
          std::lock_guard<std::mutex> lock(state_mutex_); ++route_loops_;
        }
        waypoint = (waypoint + 1U) % waypoints.size();
      }
      recording_.store(false);
      set_sampling_paused(true);
      cancel_active_motion();
      set_state("balancing", "balancing", "核心覆盖已满足，正在平衡选择5000条观测");
      std::vector<car_rl::RecordedObservation> samples;
      {
        std::lock_guard<std::mutex> lock(sample_mutex_); samples = samples_;
      }
      const auto selected = car_rl::select_coverage_balanced(samples, requirements_);
      set_state("stopping", "stopping", "正在取消导航并启用观测采集安全锁");
      safe_stop("观测采集完成，等待操作者检查并解除安全锁");
      set_state("finalizing", "finalizing", "车辆已停止，正在原子发布观测结果");
      write_success(waypoints, selected);
      successful = true; outcome = "succeeded";
      terminal_message = "观测采集完成；请检查结果和现场后人工解除观测采集安全锁";
      set_state("succeeded", "done", terminal_message, false);
    } catch (const CollectionError & error) {
      outcome = error.code(); terminal_message = error.what(); canceled = outcome == "canceled";
      try {
        {
          set_state("stopping", "stopping", terminal_message + "；正在安全停车");
          safe_stop("观测采集终止：" + terminal_message);
        }
      } catch (const std::exception & stop_error) {
        terminal_message += std::string("；停车确认失败：") + stop_error.what();
        outcome = "failed_stop";
      }
      try {
        write_failure(outcome, terminal_message);
      } catch (const std::exception & write_error) {
        terminal_message += std::string("；失败审计写盘失败：") + write_error.what();
      }
      const std::string state = outcome == "failed_quality" ? "failed_quality" :
        (canceled ? "aborted" : "failed");
      set_state(state, "done", terminal_message, false);
    } catch (const std::exception & error) {
      outcome = "internal_error"; terminal_message = error.what();
      try {safe_stop("观测采集内部异常");} catch (...) {}
      try {write_failure(outcome, terminal_message);} catch (...) {}
      set_state("failed", "done", terminal_message, false);
    }
    result->success = successful; result->outcome = outcome; result->run_id = run_id_;
    result->result_directory = result_directory_; result->observations_csv = observations_csv_;
    result->metadata_yaml = metadata_yaml_; result->coverage_json = coverage_json_;
    result->message = terminal_message;
    if (successful) {
      goal_handle->succeed(result);
    } else if (canceled) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    goal_reserved_.store(false);
  }

  std::string vehicle_id_;
  std::string output_root_;
  double sample_hz_{15.0};
  int target_samples_{5000};
  int max_raw_samples_{6500};
  double max_duration_sec_{900.0};
  double countdown_sec_{5.0};
  double stale_timeout_sec_{0.5};
  double input_failure_sec_{2.0};
  int nav_retry_count_{1};
  double nav_retry_delay_sec_{2.0};
  double stop_wheel_speed_{0.01};
  double stopped_hold_sec_{0.5};
  int minimum_waypoints_{3};
  double minimum_waypoint_distance_{0.30};
  double minimum_route_length_{2.0};
  int warning_front_{100};
  int warning_left_{100};
  int warning_right_{100};
  int warning_bilateral_{50};
  double max_feedback_wheel_speed_{0.20};
  double overspeed_hold_sec_{0.20};
  double immediate_overspeed_mps_{0.30};
  double max_linear_speed_{0.10};
  double max_angular_speed_{1.047197551};
  car_rl::ObservationCoverageThresholds thresholds_;
  car_rl::ObservationSelectionRequirements requirements_;

  std::mutex input_mutex_;
  sensor_msgs::msg::LaserScan scan_;
  nav_msgs::msg::Path plan_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::Twist nav_command_;
  geometry_msgs::msg::Twist move_command_;
  car_interfaces::msg::CarMcuStatus mcu_;
  car_interfaces::msg::CarStatus car_status_;
  car_interfaces::msg::ObstacleScanStatus obstacle_scan_status_;
  nav_msgs::msg::OccupancyGrid map_;
  bool has_scan_{false};
  bool has_odom_{false};
  bool has_nav_command_{false};
  bool has_move_command_{false};
  bool has_mcu_{false};
  bool has_car_status_{false};
  bool has_obstacle_scan_status_{false};
  bool has_map_{false};
  bool calibration_active_{false};
  bool other_nav_active_{false};
  SteadyClock::time_point scan_time_;
  SteadyClock::time_point plan_time_;
  SteadyClock::time_point odom_time_;
  SteadyClock::time_point nav_command_time_;
  SteadyClock::time_point move_command_time_;
  SteadyClock::time_point mcu_time_;
  SteadyClock::time_point car_status_time_;
  SteadyClock::time_point obstacle_scan_time_;
  SteadyClock::time_point calibration_time_;
  WheelSpeedSafetyMonitor wheel_speed_monitor_;

  std::mutex sample_mutex_;
  std::vector<car_rl::RecordedObservation> samples_;
  car_rl::ControllerAction previous_action_{0.0F, 0.0F};
  SteadyClock::time_point last_valid_sample_;
  std::string last_sample_failure_;
  std::atomic<bool> recording_{false};
  std::atomic<bool> sampling_paused_{true};
  std::atomic<bool> shutting_down_{false};
  std::atomic<bool> goal_reserved_{false};
  SteadyClock::time_point run_started_;
  std::string run_started_text_;
  double route_length_{0.0};

  std::mutex state_mutex_;
  bool active_{false};
  std::string state_;
  std::string phase_;
  std::string message_;
  std::string run_id_;
  std::string result_directory_;
  std::string observations_csv_;
  std::string metadata_yaml_;
  std::string coverage_json_;
  std::uint32_t route_loops_{0U};
  std::uint32_t current_waypoint_{0U};
  std::uint32_t total_waypoints_{0U};

  std::mutex active_nav_mutex_;
  std::shared_ptr<GoalHandleNavigate> active_nav_goal_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr move_command_sub_;
  rclcpp::Subscription<car_interfaces::msg::CarMcuStatus>::SharedPtr mcu_sub_;
  rclcpp::Subscription<car_interfaces::msg::CarStatus>::SharedPtr car_status_sub_;
  rclcpp::Subscription<car_interfaces::msg::ObstacleScanStatus>::SharedPtr obstacle_scan_sub_;
  rclcpp::Subscription<car_interfaces::msg::CalibrationStatus>::SharedPtr calibration_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
  rclcpp::Publisher<car_interfaces::msg::ObservationCollectionStatus>::SharedPtr status_pub_;
  rclcpp::Client<car_interfaces::srv::EmergencyStop>::SharedPtr estop_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr controller_params_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr path_client_;
  rclcpp_action::Server<RunObservationCollection>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
};

}  // namespace car_observation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<car_observation::ObservationCollector>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "自动观测采集节点失败：%s\n", error.what());
    result = 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}
