cmake_minimum_required(VERSION 3.8)
project(car_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(scan_geometry_normalizer src/scan_geometry_normalizer.cpp)
target_include_directories(scan_geometry_normalizer PUBLIC include)
ament_target_dependencies(scan_geometry_normalizer sensor_msgs)

add_executable(scan_warmup_relay src/scan_warmup_relay.cpp)
target_include_directories(scan_warmup_relay PUBLIC include)
target_link_libraries(scan_warmup_relay scan_geometry_normalizer)
ament_target_dependencies(scan_warmup_relay rclcpp sensor_msgs)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(
  TARGETS scan_geometry_normalizer scan_warmup_relay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
