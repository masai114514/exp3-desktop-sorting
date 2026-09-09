cmake_minimum_required(VERSION 3.8)
project(sort_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# rosidl action 接口包：只装消息，不写 C++ 逻辑
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlace.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
