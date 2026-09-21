cmake_minimum_required(VERSION 3.8)
project(lifecycle_cpp_pkg)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sensor_station src/sensor_station.cpp)
ament_target_dependencies(sensor_station
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
)

add_executable(sensor_monitor src/sensor_monitor.cpp)
ament_target_dependencies(sensor_monitor
  rclcpp
  std_msgs
)

add_executable(lifecycle_manager src/lifecycle_manager.cpp)
ament_target_dependencies(lifecycle_manager
  rclcpp
  lifecycle_msgs
)

install(TARGETS
  sensor_station
  sensor_monitor
  lifecycle_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
