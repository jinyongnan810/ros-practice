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

# turtlesim
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
# spawn target
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 5.5, theta: 0.0, name: 'turtle2'}"


# run project
ros2 launch chasing_cpp_pkg random_turtle_spawner.launch.xml
ros2 launch chasing_py_pkg random_turtle_spawner.launch.xml duration:=1.0



# check node graph
rqt_graph

```