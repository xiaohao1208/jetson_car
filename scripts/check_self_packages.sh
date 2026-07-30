#!/usr/bin/env bash

set -eo pipefail

source /opt/ros/humble/setup.bash

packages=(
  car_interfaces
  car_description
  car_move
  car_base
  car_mapping
  car_navigation
  car_rl
  car_web
  car_bringup
)

colcon build --packages-select "${packages[@]}"
source install/setup.bash
colcon test --packages-select car_rl --ctest-args -E xmllint

test_result="$(<"build/car_rl/colcon_test.rc")"
if [[ "${test_result}" != "0" ]]
then
  echo "car_rl测试失败，结果=${test_result}" >&2
  exit 1
fi

for package in "${packages[@]}"
do
  xmllint --noout "src/${package}/package.xml"
done

echo "Jetson自写包构建、RL测试和本地XML检查全部通过"
