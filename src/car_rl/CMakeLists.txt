cmake_minimum_required(VERSION 3.8)
project(car_rl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# car_rl 使用 filesystem、clamp 等 C++17 标准库能力
set(CMAKE_CXX_STANDARD 17)
# 不允许编译器静默回退到较早的 C++ 标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(yaml-cpp REQUIRED)

# TensorRT 头文件位置，只在安装了 TensorRT 的目标机上有效
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include /usr/local/include
  PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu)
# TensorRT 推理运行库位置
find_library(TENSORRT_LIBRARY nvinfer
  PATHS /usr/lib /usr/local/lib
  PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu)
find_package(CUDAToolkit QUIET)

# 不依赖具体推理平台的运行库源文件列表
set(CAR_RL_RUNTIME_SOURCES
  src/model_contract.cpp
  src/model_session.cpp
  src/observation.cpp
)
if(TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY AND CUDAToolkit_FOUND)
  list(APPEND CAR_RL_RUNTIME_SOURCES src/tensorrt_backend.cpp)
  # 告知后续 target 配置当前构建包含 TensorRT 后端
  set(CAR_RL_HAS_TENSORRT TRUE)
  message(STATUS "car_rl: TensorRT后端已启用")
else()
  list(APPEND CAR_RL_RUNTIME_SOURCES src/unavailable_backend.cpp)
  # 开发机使用明确不可用后端，经典 Nav2 仍可正常构建和运行
  set(CAR_RL_HAS_TENSORRT FALSE)
  message(STATUS "car_rl: 未检测到TensorRT，构建经典导航兼容后端")
endif()

add_library(car_rl_runtime SHARED ${CAR_RL_RUNTIME_SOURCES})
target_include_directories(car_rl_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(car_rl_runtime
  ament_index_cpp
  geometry_msgs
  nav2_costmap_2d
  nav_msgs
  sensor_msgs
  tf2
)
target_link_libraries(car_rl_runtime OpenSSL::Crypto yaml-cpp)
if(CAR_RL_HAS_TENSORRT)
  target_include_directories(car_rl_runtime PRIVATE ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(car_rl_runtime ${TENSORRT_LIBRARY} CUDA::cudart)
  target_compile_definitions(car_rl_runtime PUBLIC CAR_RL_HAS_TENSORRT=1)
endif()

add_library(car_rl_plugins SHARED
  src/controller.cpp
  src/plugins.cpp
)
target_include_directories(car_rl_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(car_rl_plugins
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(car_rl_plugins car_rl_runtime)

add_executable(model_tool src/model_tool.cpp)
target_include_directories(model_tool PRIVATE include)
ament_target_dependencies(model_tool ament_index_cpp)
target_link_libraries(model_tool car_rl_runtime)

# 从实车端 C++ 实现导出训练端可复算的契约黄金向量
add_executable(contract_tool src/contract_tool.cpp)
target_include_directories(contract_tool PRIVATE include)
ament_target_dependencies(contract_tool geometry_msgs nav_msgs sensor_msgs tf2)
target_link_libraries(contract_tool car_rl_runtime)

add_executable(shadow_controller src/shadow_controller_node.cpp)
target_include_directories(shadow_controller PRIVATE include)
ament_target_dependencies(shadow_controller
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(shadow_controller car_rl_runtime)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS
  car_rl_runtime
  car_rl_plugins
  model_tool
  contract_tool
  shadow_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config models DESTINATION share/${PROJECT_NAME})
install(FILES plugins.xml README.md rl_plan.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY handoff DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(car_rl_runtime car_rl_plugins)
# 导出公开头文件直接使用的 ROS 依赖，方便后续训练或诊断包复用运行库
ament_export_dependencies(
  ament_index_cpp
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # 当前项目暂不要求每个教学文件携带版权头
  set(ament_cmake_copyright_FOUND TRUE)
  # 工作区不是独立标准 Git 包，跳过依赖 Git 元数据的 cpplint 包装检查
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_observation test/test_observation.cpp)
  target_link_libraries(test_observation car_rl_runtime)

  ament_add_gtest(test_model_contract test/test_model_contract.cpp)
  target_link_libraries(test_model_contract car_rl_runtime)

  ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp)
  ament_target_dependencies(test_plugin_loading nav2_core pluginlib)
endif()

ament_package()
