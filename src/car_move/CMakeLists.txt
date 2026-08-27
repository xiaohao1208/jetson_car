cmake_minimum_required(VERSION 3.8)
project(car_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 运动包依赖自定义状态接口、速度消息、传感器消息和 ROS 节点库
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(car_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(car_move_core src/motion_arbitrator.cpp)
target_include_directories(car_move_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(car_move_node
  src/move_node.cpp
  src/main.cpp
)

target_include_directories(car_move_node PUBLIC include)
target_link_libraries(car_move_node car_move_core)

ament_target_dependencies(car_move_node
  builtin_interfaces
  car_interfaces
  geometry_msgs
  rclcpp
  sensor_msgs
)

install(
  TARGETS car_move_core car_move_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
