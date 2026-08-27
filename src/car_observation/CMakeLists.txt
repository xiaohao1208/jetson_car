cmake_minimum_required(VERSION 3.8)
project(car_observation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(car_interfaces REQUIRED)
find_package(car_rl REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(observation_collector
  src/collection_support.cpp
  src/observation_collector.cpp)
target_include_directories(observation_collector PRIVATE include)
ament_target_dependencies(observation_collector
  action_msgs ament_index_cpp car_interfaces car_rl geometry_msgs nav2_msgs nav_msgs
  rcl_interfaces rclcpp rclcpp_action sensor_msgs tf2 tf2_ros)
target_link_libraries(observation_collector yaml-cpp nlohmann_json::nlohmann_json)

install(TARGETS observation_collector DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
ament_package()
