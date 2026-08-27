#include "car_navigation/localization_gate.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace car_navigation
{

class LocalizationGate : public rclcpp::Node
{
public:
  LocalizationGate()
  : Node("wait_for_localization")
  {
    timeout_sec_ = declare_parameter<double>("timeout_sec", 10.0);
    poll_period_sec_ = declare_parameter<double>("poll_period_sec", 0.10);
    request_timeout_sec_ = declare_parameter<double>("request_timeout_sec", 0.50);
    log_period_sec_ = declare_parameter<double>("log_period_sec", 1.0);
    if (!finite_positive(timeout_sec_) || !finite_positive(poll_period_sec_) ||
      !finite_positive(request_timeout_sec_) || !finite_positive(log_period_sec_))
    {
      throw std::invalid_argument("定位门禁的超时和轮询参数必须为有效正数");
    }

    map_state_client_ = create_client<lifecycle_msgs::srv::GetState>("/map_server/get_state");
    amcl_state_client_ = create_client<lifecycle_msgs::srv::GetState>("/amcl/get_state");
  }

  int wait_until_ready()
  {
    using SteadyClock = std::chrono::steady_clock;
    const auto started_at = SteadyClock::now();
    const auto deadline = started_at + seconds(timeout_sec_);
    auto next_log = started_at;

    while (rclcpp::ok() && SteadyClock::now() < deadline) {
      const auto map_state = request_state(map_state_client_, deadline);
      const auto amcl_state = request_state(amcl_state_client_, deadline);
      const auto gate_state = evaluate_localization_gate(
        map_state.has_value(), map_state.value_or(
          lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN),
        amcl_state.has_value(),
        amcl_state.value_or(lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN));
      if (gate_state == LocalizationGateState::ready) {
        RCLCPP_INFO(
          get_logger(), "map_server和AMCL均已激活，允许启动Nav2导航栈");
        return 0;
      }

      const auto now = SteadyClock::now();
      if (now >= next_log) {
        if (gate_state == LocalizationGateState::waiting_for_services) {
          RCLCPP_INFO(
            get_logger(), "等待map_server和AMCL生命周期服务响应");
        } else {
          RCLCPP_INFO(
            get_logger(), "等待定位节点激活：map_server=%s，AMCL=%s",
            state_label(map_state).c_str(), state_label(amcl_state).c_str());
        }
        next_log = now + seconds(log_period_sec_);
      }
      rclcpp::sleep_for(seconds(poll_period_sec_));
    }

    if (!rclcpp::ok()) {
      RCLCPP_WARN(get_logger(), "定位门禁因ROS退出请求而停止");
      return 130;
    }
    RCLCPP_ERROR(
      get_logger(),
      "定位栈在%.1f秒内未激活，取消启动planner和controller；请检查map_server与AMCL日志",
      timeout_sec_);
    return 1;
  }

private:
  using GetState = lifecycle_msgs::srv::GetState;
  using GetStateClient = rclcpp::Client<GetState>;

  static bool finite_positive(double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  static std::chrono::nanoseconds seconds(double value)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(value));
  }

  std::optional<std::uint8_t> request_state(
    const GetStateClient::SharedPtr & client,
    const std::chrono::steady_clock::time_point & deadline)
  {
    if (!client->service_is_ready()) {
      return std::nullopt;
    }
    const auto remaining = deadline - std::chrono::steady_clock::now();
    if (remaining <= std::chrono::steady_clock::duration::zero()) {
      return std::nullopt;
    }
    const auto request_timeout = std::min(
      seconds(request_timeout_sec_),
      std::chrono::duration_cast<std::chrono::nanoseconds>(remaining));
    auto future = client->async_send_request(std::make_shared<GetState::Request>());
    const auto result = rclcpp::spin_until_future_complete(
      shared_from_this(), future, request_timeout);
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      client->remove_pending_request(future);
      return std::nullopt;
    }
    return future.get()->current_state.id;
  }

  static std::string state_label(const std::optional<std::uint8_t> & state)
  {
    if (!state.has_value()) {
      return "无响应";
    }
    switch (state.value()) {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
        return "unconfigured";
      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        return "inactive";
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        return "active";
      case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
        return "finalized";
      default:
        return "unknown(" + std::to_string(state.value()) + ")";
    }
  }

  double timeout_sec_{10.0};
  double poll_period_sec_{0.10};
  double request_timeout_sec_{0.50};
  double log_period_sec_{1.0};
  GetStateClient::SharedPtr map_state_client_;
  GetStateClient::SharedPtr amcl_state_client_;
};

}  // namespace car_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 2;
  try {
    const auto gate = std::make_shared<car_navigation::LocalizationGate>();
    result = gate->wait_until_ready();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("wait_for_localization"), "%s", error.what());
  }
  rclcpp::shutdown();
  return result;
}
