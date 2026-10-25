## Install MoveIt2
```
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp

gedit ~/.bashrc
# add
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo apt install ros-jazzy-moveit
```

## Display the six-axis arm

The `robot_arm_description` package contains a primitive-shape six-axis arm,
including collision geometry, inertial properties, joint limits, and a `tool0`
frame. The joints are base yaw, shoulder pitch, elbow pitch, wrist roll,
wrist pitch, and tool roll. The arm is fixed to `world`.

Edit the named Xacro properties at the top of
`src/robot_arm_description/urdf/six_axis_arm.urdf.xacro` to change link sizes
and masses. Visuals, collisions, centres of mass, inertia tensors, and joint
offsets use these properties. The display launch file expands Xacro automatically.

From `9.moveit2` in a ROS 2 Jazzy terminal:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select robot_arm_description
source install/setup.bash
ros2 launch robot_arm_description display.launch.py
```

RViz opens with the robot model and grid. Use the joint state publisher GUI
sliders to move the six joints. Enable the TF display in RViz to inspect frames.
To display the zero pose without sliders:

```bash
ros2 launch robot_arm_description display.launch.py gui:=false
```

This package provides visualization only; MoveIt planning and ros2_control
configuration can be added separately.
