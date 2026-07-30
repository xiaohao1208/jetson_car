cmake_minimum_required(VERSION 3.8)
project(car_mapping)

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

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(
  TARGETS scan_geometry_normalizer scan_warmup_relay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
