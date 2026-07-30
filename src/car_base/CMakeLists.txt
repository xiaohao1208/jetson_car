cmake_minimum_required(VERSION 3.8)
project(car_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(car_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(car_base_core
  src/odometry_calibration.cpp
  src/odometry_integrator.cpp)
target_include_directories(car_base_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(car_base_node src/base_node.cpp src/main.cpp)
target_include_directories(car_base_node PUBLIC include)
target_link_libraries(car_base_node car_base_core)
ament_target_dependencies(car_base_node
  car_interfaces geometry_msgs nav_msgs rclcpp sensor_msgs tf2 tf2_ros)

add_executable(
  odometry_calibration
  src/odometry_calibration_main.cpp)
target_include_directories(odometry_calibration PUBLIC include)
target_link_libraries(odometry_calibration car_base_core)

install(TARGETS car_base_core car_base_node odometry_calibration
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
