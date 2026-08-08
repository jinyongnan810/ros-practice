# Services
A Service is client/server system.

Service has following features:
- Service calls canbe synchronous or asynchronous, i.e. a client can call a service and wait for the response, or a client can call a service and continue doing other things while waiting for the response.
- One message type for request and one message type for response.

### Commands
```bash
# enable env
direnv allow

# create python pkg
ros2 pkg create service_py_pkg --build-type ament_python --dependencies rclpy
# create c++ pkg
ros2 pkg create service_cpp_pkg --build-type ament_cmake --dependencies rclcpp
# create custom interface pkg
ros2 pkg create custom_interfaces --build-type ament_cmake

# build all pkgs
colcon build
# build one pkg
colcon build --packages-select service_py_pkg
colcon build --packages-select service_cpp_pkg
colcon build --packages-select custom_interfaces

# run node
ros2 run service_py_pkg acc_server
ros2 run service_py_pkg acc_client
ros2 run service_cpp_pkg acc_server
ros2 run service_cpp_pkg acc_client
# with different names
ros2 run service_py_pkg acc_server --ros-args -r __node:=server
ros2 run service_py_pkg acc_client --ros-args -r __node:=client1
ros2 run service_py_pkg acc_client --ros-args -r __node:=client2
# with different service names
ros2 run service_py_pkg acc_server --ros-args -r __node:=server -r /accumulate:=/accumulate1
ros2 run service_py_pkg acc_client --ros-args -r __node:=client1 -r /accumulate:=/accumulate1

# get the type of the service
ros2 service type /accumulate
# check service interface
ros2 interface show custom_interfaces/srv/Acc
# call service directly from command line
# also can be called with rqt's Service Caller plugin
ros2 service call /accumulate custom_interfaces/srv/Acc "{a: 1, b: 2, c: 3}"


# check node graph
rqt_graph

```