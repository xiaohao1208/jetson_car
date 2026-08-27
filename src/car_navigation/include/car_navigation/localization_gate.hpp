#ifndef CAR_NAVIGATION__LOCALIZATION_GATE_HPP_
#define CAR_NAVIGATION__LOCALIZATION_GATE_HPP_

#include <cstdint>

#include "lifecycle_msgs/msg/state.hpp"

namespace car_navigation
{

enum class LocalizationGateState
{
  waiting_for_services,
  waiting_for_activation,
  ready,
};

inline LocalizationGateState evaluate_localization_gate(
  bool map_state_received,
  std::uint8_t map_state,
  bool amcl_state_received,
  std::uint8_t amcl_state)
{
  if (!map_state_received || !amcl_state_received) {
    return LocalizationGateState::waiting_for_services;
  }
  constexpr auto active = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  if (map_state != active || amcl_state != active) {
    return LocalizationGateState::waiting_for_activation;
  }
  return LocalizationGateState::ready;
}

}  // namespace car_navigation

#endif  // CAR_NAVIGATION__LOCALIZATION_GATE_HPP_
