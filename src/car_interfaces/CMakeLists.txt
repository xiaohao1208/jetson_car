cmake_minimum_required(VERSION 3.8)
project(car_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 接口包生成 C、C++ 和 Python 消息类型供 Jetson 与 ESP32 共用
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CarStatus.msg"
  "msg/CarMcuStatus.msg"
  "srv/EmergencyStop.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
