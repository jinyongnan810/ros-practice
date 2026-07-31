# Topics
A topic is a named bus over which nodes exchange messages. A node sends messages by publishing them to a given topic. A node receives messages by subscribing to a given topic.

Topic has following features:
- Topic is unidirectional, i.e. a node can either publish or subscribe to a topic, but not both.
- Topic is asynchronous, i.e. a node can publish messages at any time, and a node can receive messages at any time.
- Topic is many-to-many, i.e. a node can publish messages to a topic, and multiple nodes can subscribe to the same topic, and vice versa.

### Commands
```bash
# enable env
direnv allow

# create python pkg
ros2 pkg create topic_py_pkg --build-type ament_python --dependencies rclpy
# create c++ pkg
ros2 pkg create topic_cpp_pkg --build-type ament_cmake --dependencies rclcpp

# build all pkgs
colcon build
# build one pkg
colcon build --packages-select topic_py_pkg
colcon build --packages-select topic_cpp_pkg

# run node
ros2 run node_py_pkg hello_world_node
ros2 run node_cpp_pkg hello_world_node
# with different names
ros2 run node_py_pkg hello_world_node --ros-args -r __node:=abc_node


# list running nodes
ros2 node list

# see node info
ros2 node info /hello_world_node 

# check node graph
rqt_graph

# demos
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

```