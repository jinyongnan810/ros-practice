# URDF
- URDF (Unified Robot Description Format) is an XML format for representing a robot model.

### RViz
- tf is published by nodes, and can be visualized in rviz. tf is used to transform data between different coordinate frames. It answers:
  - How frames are placed relative to each other?
  - How they move relative to each other?
- axis: x red forward, y green right, z blue up
- yellow pink arrow: A is child of B. if parent frame B moves, child frame A moves with it.

### Commands
```bash
# install urdf-tutorial
sudo apt install ros-jazzy-urdf-tutorial
# find all the packages installed
cd /opt/ros/jazzy/share
# go to urdf_tutorial package
cd urdf_tutorial/urdf
# launch the demo robot model
ros2 launch urdf_tutorial display.launch.py model:=/opt/ros/jazzy/share/urdf_tutorial/urdf/08-macroed.urdf.xacro
# launch own robot model
ros2 launch urdf_tutorial display.launch.py model:=/home/kin/shared/shared/ros-practice/5.urdf/two_wheel_car.urdf

# check tf trees hierarchy
ros2 run tf2_tools view_frames
open frames_2026-08-19_07.55.13.pdf
```

### Tips
- To make vscode syntax highlight for urdf files, open settings -> file associations -> add *.urdf and select XML.