cmake_minimum_required(VERSION 3.8)
project(turtle_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtle_interfaces REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(turtlesim_ex1 src/turtlesim_ex1.cpp)
ament_target_dependencies(turtlesim_ex1 rclcpp turtlesim geometry_msgs)

add_executable(turtlesim_ex2 src/turtlesim_ex2.cpp)
ament_target_dependencies(turtlesim_ex2 rclcpp turtlesim geometry_msgs turtle_interfaces)

add_executable(turtlesim_ex1_initialize src/turtlesim_ex1_initialize.cpp)
ament_target_dependencies(turtlesim_ex1_initialize rclcpp turtlesim geometry_msgs)

add_executable(turtlesim_ex1_spawn src/turtlesim_ex1_spawn.cpp)
ament_target_dependencies(turtlesim_ex1_spawn rclcpp turtlesim)

add_executable(turtlesim_ex2_stop_rotate src/turtlesim_ex2_stop_rotate.cpp)
ament_target_dependencies(turtlesim_ex2_stop_rotate rclcpp turtle_interfaces)

install(TARGETS
  turtlesim_ex1
  turtlesim_ex2
  turtlesim_ex1_spawn
  turtlesim_ex1_initialize
  turtlesim_ex2_stop_rotate
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
