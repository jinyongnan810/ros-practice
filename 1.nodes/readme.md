### Commands
```bash
# enable env
direnv allow
# create python pkg
ros2 pkg create node_py_pkg --build-type ament_python --dependencies rclpy
# create c++ pkg
ros2 pkg create node_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```