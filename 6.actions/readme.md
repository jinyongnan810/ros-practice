# Actions
An Action is a client/server communication architecture designed for long-running, preemptible tasks.

An Action consists of three main parts:
- **Goal**: The client requests the server to perform a task. The server can accept or reject the goal.
- **Feedback**: While executing the goal, the server periodically sends progress updates back to the client.
- **Result**: Upon finishing the task, the server sends the final outcome (status and result data) back to the client.

Actions have following features:
- **Asynchronous & Non-blocking**: Clients can send a goal and continue doing other tasks while periodically receiving feedback.
- **Preemptible / Cancellable**: Clients can request cancellation of an active goal at any point during execution.
- **Composed of Topics & Services**: Under the hood, ROS 2 actions are implemented using multiple topics (feedback, status) and services (send goal, cancel goal, get result).

### Commands
```bash
# enable env
direnv allow

# create python pkg
ros2 pkg create action_py_pkg --build-type ament_python --dependencies rclpy
# create c++ pkg
ros2 pkg create action_cpp_pkg --build-type ament_cmake --dependencies rclcpp rclcpp_action
# create custom interface pkg (if defining a custom action, e.g. CountUntil.action)
ros2 pkg create custom_interfaces --build-type ament_cmake

# build all pkgs
colcon build
# build one pkg
colcon build --packages-select action_py_pkg
colcon build --packages-select action_cpp_pkg

# run node
ros2 run action_py_pkg acc_server
ros2 run action_py_pkg acc_client
ros2 run action_cpp_pkg acc_server
ros2 run action_cpp_pkg acc_client

# with different node names
ros2 run action_py_pkg acc_server --ros-args -r __node:=server
ros2 run action_py_pkg acc_client --ros-args -r __node:=client1
ros2 run action_py_pkg acc_client --ros-args -r __node:=client2

# with different action names
ros2 run action_py_pkg acc_server --ros-args -r __node:=server -r /accumulate:=/accumulate1
ros2 run action_py_pkg acc_client --ros-args -r __node:=client1 -r /accumulate:=/accumulate1

# list running nodes
ros2 node list

# see node info
ros2 node info /acc_server_node

# list actions
ros2 action list
# list actions with types
ros2 action list -t

# inspect action details (servers and clients)
ros2 action info /accumulate

# inspect action interface definition (.action)
ros2 interface show <package_name>/action/<ActionName>

# send action goal directly from CLI (with feedback)
ros2 action send_goal /accumulate <package_name>/action/<ActionName> "{<goal_field>: <value>}" --feedback

# check node graph
rqt_graph
```