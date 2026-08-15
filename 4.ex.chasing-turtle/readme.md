# Chasing Turtle
A system that spawns some targets and a turtlebot that chases them. The targets are spawned in random locations and the turtlebot will chase the closest target.

### Commands
```bash
# enable env
direnv allow

# create python pkg
ros2 pkg create chasing_py_pkg --build-type ament_python --dependencies rclpy
# create c++ pkg
ros2 pkg create chasing_cpp_pkg --build-type ament_cmake --dependencies rclcpp

# build all pkgs
colcon build
# build one pkg
colcon build --packages-select chasing_py_pkg
colcon build --packages-select chasing_cpp_pkg

# run node
ros2 run chasing_py_pkg news_station_node
ros2 run chasing_py_pkg radio_node
ros2 run chasing_cpp_pkg news_station
ros2 run chasing_cpp_pkg radio
# with different names
ros2 run chasing_py_pkg news_station_node --ros-args -r __node:=station1_node
ros2 run chasing_py_pkg radio_node --ros-args -r __node:=radio1_node
ros2 run chasing_py_pkg radio_node --ros-args -r __node:=radio2_node


# list running nodes
ros2 node list

# see node info
ros2 node info /news_station_node

# see topics
ros2 topic list
# status of the topic
ros2 topic info /news
# frequency of the topic
ros2 topic hz /news
# bitrate of the topic
ros2 topic bw /news
# directly listen to the topic
ros2 topic echo /news
# directly publish to the topic (5 times a second)
ros2 topic pub -r 5 /news std_msgs/msg/String "{data: 'Hello from direct pub'}"
# rename the topic name
ros2 run topic_py_pkg news_station_node --ros-args -r __node:=station1_node -r news:=renamed_news
ros2 run topic_py_pkg radio_node --ros-args -r __node:=radio1_node -r news:=renamed_news
# record and replay the topic data
ros2 bag record -o pub_tests /news # use -a to record all topics
ros2 bag info pub_tests
ros2 bag play pub_tests


# check node graph
rqt_graph

```