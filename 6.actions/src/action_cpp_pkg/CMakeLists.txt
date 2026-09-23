cmake_minimum_required(VERSION 3.8)
project(action_cpp_pkg)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(custom_interfaces REQUIRED)

# 1. Action Server Component & Standalone Executable
add_library(turtle_action_server_component SHARED
  src/turtle_action_server.cpp
)
target_include_directories(turtle_action_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(turtle_action_server_component
  PRIVATE "ACTION_CPP_PKG_BUILDING_DLL"
)
ament_target_dependencies(turtle_action_server_component
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  turtlesim
  custom_interfaces
)
rclcpp_components_register_node(
  turtle_action_server_component
  PLUGIN "action_cpp_pkg::TurtleActionServerNode"
  EXECUTABLE turtle_action_server
)
target_link_libraries(turtle_action_server turtle_action_server_component)
set_target_properties(turtle_action_server PROPERTIES INSTALL_RPATH "$ORIGIN/..")

# 2. Action Client Component & Standalone Executable
add_library(turtle_action_client_component SHARED
  src/turtle_action_client.cpp
)
target_include_directories(turtle_action_client_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(turtle_action_client_component
  PRIVATE "ACTION_CPP_PKG_BUILDING_DLL"
)
ament_target_dependencies(turtle_action_client_component
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  turtlesim
  custom_interfaces
)
rclcpp_components_register_node(
  turtle_action_client_component
  PLUGIN "action_cpp_pkg::TurtleActionClientNode"
  EXECUTABLE turtle_action_client
)
target_link_libraries(turtle_action_client turtle_action_client_component)
set_target_properties(turtle_action_client PROPERTIES INSTALL_RPATH "$ORIGIN/..")

# Export targets so dynamic library paths are recorded in ament environment hooks
ament_export_targets(export_turtle_action_components HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  turtlesim
  custom_interfaces
)

# Install shared library components
install(TARGETS
  turtle_action_server_component
  turtle_action_client_component
  EXPORT export_turtle_action_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install standalone executables
install(TARGETS
  turtle_action_server
  turtle_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
