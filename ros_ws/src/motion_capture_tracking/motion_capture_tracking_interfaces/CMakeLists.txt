cmake_minimum_required(VERSION 3.8)
project(motion_capture_tracking_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NamedPose.msg"
  "msg/NamedPoseArray.msg"
  "msg/TargetPosition.msg"
  "msg/TargetPositionArray.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs 
  ADD_LINTER_TESTS
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
