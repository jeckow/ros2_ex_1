cmake_minimum_required(VERSION 3.8)
project(turtle_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)    # ADD THIS DEPENDENCY

rosidl_generate_interfaces(${PROJECT_NAME}          # ADD THIS DEPENDENCY, no comma needed
  "msg/RotateStatus.msg"
  "srv/RotateStatus.srv"
 )

 ament_export_dependencies(rosidl_default_runtime)  # ADD THIS DEPENDENCYt_auto_find_test_dependencies()

ament_package()
