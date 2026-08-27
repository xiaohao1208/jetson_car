cmake_minimum_required(VERSION 3.8)
project(car_rl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# car_rl 使用 filesystem、clamp 等 C++17 标准库能力
set(CMAKE_CXX_STANDARD 17)
# 不允许编译器静默回退到较早的 C++ 标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

set(CAR_RL_BACKEND "AUTO" CACHE STRING
  "推理后端：AUTO、TENSORRT、ONNXRUNTIME或UNAVAILABLE")
set_property(CACHE CAR_RL_BACKEND PROPERTY STRINGS
  AUTO TENSORRT ONNXRUNTIME UNAVAILABLE)
string(TOUPPER "${CAR_RL_BACKEND}" CAR_RL_BACKEND_NORMALIZED)
if(NOT CAR_RL_BACKEND_NORMALIZED MATCHES
  "^(AUTO|TENSORRT|ONNXRUNTIME|UNAVAILABLE)$")
  message(FATAL_ERROR "CAR_RL_BACKEND只允许AUTO/TENSORRT/ONNXRUNTIME/UNAVAILABLE")
endif()

# TensorRT 头文件位置，只在安装了 TensorRT 的目标机上有效
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include /usr/local/include
  PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu)
# TensorRT 推理运行库位置
find_library(TENSORRT_LIBRARY nvinfer
  PATHS /usr/lib /usr/local/lib
  PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu)
find_package(CUDAToolkit QUIET)

# 官方ONNX Runtime C/C++包根目录，可通过-DONNXRUNTIME_ROOT或环境变量提供
set(ONNXRUNTIME_ROOT "" CACHE PATH "ONNX Runtime C/C++发布包根目录")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT} $ENV{ONNXRUNTIME_ROOT}
  PATH_SUFFIXES include include/onnxruntime/core/session)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  HINTS ${ONNXRUNTIME_ROOT} $ENV{ONNXRUNTIME_ROOT}
  PATH_SUFFIXES lib lib64)

# 不依赖具体推理平台的运行库源文件列表
set(CAR_RL_RUNTIME_SOURCES
  src/inference_benchmark.cpp
  src/model_contract.cpp
  src/model_session.cpp
  src/observation.cpp
  src/observation_recording.cpp
)

if(CAR_RL_BACKEND_NORMALIZED STREQUAL "AUTO")
  if(TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY AND CUDAToolkit_FOUND)
    set(CAR_RL_SELECTED_BACKEND "TENSORRT")
  elseif(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
    set(CAR_RL_SELECTED_BACKEND "ONNXRUNTIME")
  else()
    set(CAR_RL_SELECTED_BACKEND "UNAVAILABLE")
  endif()
else()
  set(CAR_RL_SELECTED_BACKEND "${CAR_RL_BACKEND_NORMALIZED}")
endif()

if(CAR_RL_SELECTED_BACKEND STREQUAL "TENSORRT")
  if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY OR NOT CUDAToolkit_FOUND)
    message(FATAL_ERROR "请求TENSORRT后端，但未找到TensorRT头文件、nvinfer或CUDA Toolkit")
  endif()
  list(APPEND CAR_RL_RUNTIME_SOURCES src/tensorrt_backend.cpp)
  set(CAR_RL_HAS_TENSORRT TRUE)
  set(CAR_RL_HAS_ONNXRUNTIME FALSE)
  message(STATUS "car_rl: TensorRT后端已启用")
elseif(CAR_RL_SELECTED_BACKEND STREQUAL "ONNXRUNTIME")
  if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR "请求ONNXRUNTIME后端，但未找到onnxruntime_cxx_api.h或libonnxruntime")
  endif()
  list(APPEND CAR_RL_RUNTIME_SOURCES src/onnxruntime_backend.cpp)
  set(CAR_RL_HAS_TENSORRT FALSE)
  set(CAR_RL_HAS_ONNXRUNTIME TRUE)
  message(STATUS "car_rl: ONNX Runtime CPU后端已启用")
elseif(CAR_RL_SELECTED_BACKEND STREQUAL "UNAVAILABLE")
  list(APPEND CAR_RL_RUNTIME_SOURCES src/unavailable_backend.cpp)
  set(CAR_RL_HAS_TENSORRT FALSE)
  set(CAR_RL_HAS_ONNXRUNTIME FALSE)
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
if(CAR_RL_HAS_ONNXRUNTIME)
  target_include_directories(car_rl_runtime PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(car_rl_runtime ${ONNXRUNTIME_LIBRARY})
  target_compile_definitions(car_rl_runtime PUBLIC CAR_RL_HAS_ONNXRUNTIME=1)
  get_filename_component(ONNXRUNTIME_LIBRARY_DIR ${ONNXRUNTIME_LIBRARY} DIRECTORY)
  set_target_properties(car_rl_runtime PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
    INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}")
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

add_executable(observation_recorder src/observation_recorder_node.cpp)
target_include_directories(observation_recorder PRIVATE include)
ament_target_dependencies(observation_recorder
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(observation_recorder car_rl_runtime yaml-cpp)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS
  car_rl_runtime
  car_rl_plugins
  model_tool
  contract_tool
  shadow_controller
  observation_recorder
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS
  scripts/get_model_worker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME get_model_worker)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

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

ament_package()
