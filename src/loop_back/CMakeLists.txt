cmake_minimum_required(VERSION 3.5)
project(loop_back)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(time_interface REQUIRED)

add_executable(host_talker src/host_publisher_member_function.cpp)
ament_target_dependencies(host_talker rclcpp time_interface)
add_executable(remote_node src/remote_member_function.cpp)
ament_target_dependencies(remote_node rclcpp time_interface)
add_executable(host_listener src/host_subscriber_member_function.cpp)
ament_target_dependencies(host_listener rclcpp time_interface)


install(TARGETS
  host_talker
  remote_node
  host_listener
  DESTINATION lib/${PROJECT_NAME})

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()
