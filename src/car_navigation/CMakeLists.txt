cmake_minimum_required(VERSION 3.8)
project(car_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(car_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(car_expanding_obstacle_scan_bt_node SHARED
  src/expanding_obstacle_scan.cpp)
target_compile_definitions(car_expanding_obstacle_scan_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(car_expanding_obstacle_scan_bt_node PUBLIC include)
ament_target_dependencies(car_expanding_obstacle_scan_bt_node
  behaviortree_cpp_v3 car_interfaces nav2_msgs nav_msgs rclcpp rclcpp_action)

add_executable(wait_for_localization src/wait_for_localization.cpp)
target_include_directories(wait_for_localization PUBLIC include)
ament_target_dependencies(wait_for_localization lifecycle_msgs rclcpp)

install(
  DIRECTORY behavior_trees config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(TARGETS car_expanding_obstacle_scan_bt_node wait_for_localization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
