# URDF
- URDF (Unified Robot Description Format) is an XML format for representing a robot model.

### RViz
- tf is published by nodes, and can be visualized in rviz. tf is used to transform data between different coordinate frames. It answers:
  - How frames are placed relative to each other?
  - How they move relative to each other?
- axis: x red forward, y green right, z blue up
- yellow pink arrow: A is child of B. if parent frame B moves, child frame A moves with it.

### Components
- `links` are rigid bodies of the robot. They are connected by joints. Links can be visualized in rviz as a 3D model.
- `materials` are used to color the links. They can be defined in the urdf file or in a separate material file.
- `joints` are used to connect links. They can be of different types: revolute, continuous, prismatic, fixed, floating, planar. Joints can be visualized in rviz as arrows.

#### Setting origins
- First set origin of the joint, then set origin of the link. 

#### Joints types
| Type       | Description                                 | Usage                |
| ---------- | ------------------------------------------- | -------------------- |
| revolute   | rotates around a single axis                | elbow, knee          |
| continuous | rotates around a single axis without limits | wheel                |
| prismatic  | moves along a single axis                   | linear actuator      |
| fixed      | does not move                               | base of the robot    |
| floating   | can move freely in space                    | free-floating camera |
| planar     | moves in a plane                            | Air Hockey Pucks     |


### Commands
```bash
# auto source
direnv allow

# install urdf-tutorial
sudo apt install ros-jazzy-urdf-tutorial
# find all the packages installed
cd /opt/ros/jazzy/share
# go to urdf_tutorial package
cd urdf_tutorial/urdf
# launch the demo robot model
ros2 launch urdf_tutorial display.launch.py model:=/opt/ros/jazzy/share/urdf_tutorial/urdf/08-macroed.urdf.xacro
# launch own robot model
ros2 launch urdf_tutorial display.launch.py model:=/home/kin/shared/shared/ros-practice/5.urdf/simple_car.urdf
# get the urdf from /robot_description parameter
ros2 param get /robot_state_publisher robot_description
# get the joint states from /joint_states topic
ros2 topic echo /joint_states

# create package to include urdf files
ros2 pkg create simple_car_description

# run with robot_state_publisher and joint_state_publisher_gui
xacro simple_car.urdf -o /tmp/robot.urdf
ros2 run robot_state_publisher robot_state_publisher /tmp/robot.urdf
ros2 run joint_state_publisher_gui joint_state_publisher_gui
ros2 run rviz2 rviz2 -d src/simple_car_description/rviz/display.rviz

# check tf trees hierarchy
ros2 run tf2_tools view_frames
open frames_2026-08-19_07.55.13.pdf
```

### Launching this robot
This workspace already includes a small URDF package under `src/simple_car_description` with a model file and launch files.

```bash
cd /home/kin/shared/shared/ros-practice/5.urdf
source /opt/ros/jazzy/setup.bash
colcon build --packages-select simple_car_description --symlink-install
source install/setup.bash

# Python launch file
ros2 launch simple_car_description display.launch.py

# XML launch file
ros2 launch simple_car_description display.launch.xml
```

Both launch files use `src/simple_car_description/urdf/simple_car.urdf.xacro` by default. With `--symlink-install`, changes to that file do not require another build, but the launch process must be stopped and restarted because `robot_state_publisher` reads `robot_description` only at startup.

If `model:=.../simple_car.urdf` is passed explicitly, edits to `simple_car.urdf.xacro` will not be used.

The launch file starts:
- `robot_state_publisher` to publish TF from the URDF
- `joint_state_publisher_gui` to move the joints interactively
- `rviz2` for visualization

### Package layout
```text
5.urdf/
├── simple_car.urdf
└── src/
    └── simple_car_description/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   ├── display.launch.py
        │   └── display.launch.xml
        └── urdf/
          ├── simple_car.urdf
          └── simple_car.urdf.xacro
```

### Tips
- To make vscode syntax highlight for urdf files, open settings -> file associations -> add *.urdf and select XML.