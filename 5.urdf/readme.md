# URDF
- URDF (Unified Robot Description Format) is an XML format for representing a robot model.

### RViz
- tf is published by nodes, and can be visualized in rviz. tf is used to transform data between different coordinate frames. It answers:
  - How frames are placed relative to each other?
  - How they move relative to each other?
- axis: x red forward, y green right, z blue up
- yellow pink arrow: A is child of B. if parent frame B moves, child frame A moves with it.

### Components
- `links` are rigid bodies of the robot. They are connected by joints. Each link typically contains:
  - `<visual>`: Defines the 3D graphical appearance (geometry, material/color, meshes). Visualized in RViz and Gazebo.
  - `<collision>`: Defines the simplified boundary geometry used by physics engines for collision detection.
  - `<inertial>`: Defines the mass and mass distribution of the link. **Required for physics simulation in Gazebo**. Links without inertial properties are treated as static or ignored by physics engines.
    - `<mass value="..."/>`: Total mass in kilograms ($kg$).
    - `<origin xyz="..." rpy="..."/>`: Position and orientation of the **Center of Mass (COM)** relative to the link coordinate frame.
    - `<inertia ixx="..." ixy="..." ixz="..." iyy="..." iyz="..." izz="..."/>`: 3x3 symmetric moment of inertia matrix (in $kg \cdot m^2$), measuring resistance to rotational acceleration around principal axes.
- `materials` are used to color the links. They can be defined in the urdf file or in a separate material file.
- `joints` are used to connect links. They can be of different types: revolute, continuous, prismatic, fixed, floating, planar. Joints can be visualized in rviz as arrows.

#### Inertia Formulas for Common Geometries
- **Solid Box** (dimensions $x, y, z$, mass $m$):
  $$I_{xx} = \frac{1}{12} m (y^2 + z^2), \quad I_{yy} = \frac{1}{12} m (x^2 + z^2), \quad I_{zz} = \frac{1}{12} m (x^2 + y^2)$$
- **Solid Cylinder** (radius $r$, length $h$ along $z$-axis, mass $m$):
  $$I_{xx} = I_{yy} = \frac{1}{12} m (3r^2 + h^2), \quad I_{zz} = \frac{1}{2} m r^2$$
- **Solid Sphere** (radius $r$, mass $m$):
  $$I_{xx} = I_{yy} = I_{zz} = \frac{2}{5} m r^2$$

- More in this: https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors

> **Note on Inertia Visualization**: In Gazebo and RViz, the inertia tensor is visualized as an *equivalent uniform inertia box*. For cylinders with radial symmetry ($I_{xx} = I_{yy}$), this equivalent box has a square cross-section ($\sqrt{3}r \times \sqrt{3}r$) along the circular face.

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

# use xacro to generate urdf from xacro
xacro simple_car.urdf.xacro -o simple_car.urdf
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
        ├── config/
        │   └── gazebo_bridge.yaml
        ├── launch/
        │   ├── display.launch.py
        │   ├── display.launch.xml
        │   ├── gazebo.launch.py
        │   └── gazebo.launch.xml
        ├── meshes/
        │   └── visual/
        │       └── waffle_base.stl
        ├── urdf/
        │   ├── simple_car.gazebo.xacro
        │   ├── simple_car.inertias.xacro
        │   ├── simple_car.materials.xacro
        │   ├── simple_car.properties.xacro
        │   ├── simple_car.urdf
        │   ├── simple_car.urdf.xacro
        │   └── simple_car.wheel.xacro
        └── worlds/
            └── forest.sdf
```

### Tips
- To make vscode syntax highlight for urdf files, open settings -> file associations -> add *.urdf and select XML.



# Gazebo

- Gazebo Sim (modern Gazebo / `ros_gz`) simulates the physical dynamics, contacts, sensors, and visuals of the robot.

### Gazebo Fuel Resources (`app.gazebosim.org`)
- Gazebo Sim natively downloads simulation assets (models, worlds, textures) directly from **Gazebo Fuel** (`https://fuel.gazebosim.org` / `app.gazebosim.org`).
- In world files (`.sdf`), models can be referenced by their Fuel URI:
  - `https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree`
  - `https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree`
- When starting the simulation, Gazebo Sim automatically resolves and caches these Fuel models locally under `~/.gz/fuel/fuel.gazebosim.org/`.
- Once downloaded, subsequent launches use the local cache even without an active internet connection.

### Key Concepts
- **Joint State Publisher Plugin (`gz-sim-joint-state-publisher-system`)**: Gazebo system plugin that monitors joint states (positions/velocities of movable joints) and publishes them on `<topic>joint_states</topic>`.
- **Differential Drive Plugin (`gz-sim-diff-drive-system`)**: Subscribes to `cmd_vel` to drive the left and right wheels, and computes / publishes odometry (`odom`) and TF transforms (`odom -> base_link`).
- **ROS-Gz Bridge (`ros_gz_bridge`)**: Translates topics between Gazebo and ROS 2:
  - `/clock`: Translates `gz.msgs.Clock` $\rightarrow$ `rosgraph_msgs/msg/Clock` so `use_sim_time:=true` keeps ROS nodes in sync with physics time.
  - `/joint_states`: Translates `gz.msgs.Model` $\rightarrow$ `sensor_msgs/msg/JointState` for `robot_state_publisher` and RViz.
  - `/cmd_vel`: Translates `geometry_msgs/msg/Twist` (ROS 2) $\rightarrow$ `gz.msgs.Twist` (Gazebo) to drive the wheels.
  - `/odom`: Translates `gz.msgs.Odometry` $\rightarrow$ `nav_msgs/msg/Odometry`.
  - `/tf`: Translates `gz.msgs.Pose_V` $\rightarrow$ `tf2_msgs/msg/TFMessage` (`odom -> base_link`).
- **Resource Path (`GZ_SIM_RESOURCE_PATH`)**: Tells Gazebo where packages/meshes are located. Resolves `package://<pkg>/...` (translated to `model://<pkg>/...`) to local files.
- **Model Spawning**: The `ros_gz_sim` `create` node reads `robot_description` published by `robot_state_publisher` and dynamically inserts the robot entity into the Gazebo world.

### What is Odometry (`odom`)?
**Odometry** is the method of estimating a robot's position and orientation (pose) over time relative to where it started, calculated by integrating motion sensor data (such as wheel rotation from encoders).

- **`nav_msgs/msg/Odometry` Message Components**:
  - **`pose.pose`**: The estimated position $(x, y, z)$ and orientation (quaternion $x, y, z, w$) in the world-fixed starting frame (`odom`).
  - **`pose.covariance`**: $6 \times 6$ matrix representing confidence/uncertainty in the position and orientation estimate.
  - **`twist.twist`**: The robot's current linear velocities ($v_x, v_y, v_z$) and angular velocities ($\omega_x, \omega_y, \omega_z$) in the robot's local body frame (`base_link`).
  - **`twist.covariance`**: $6 \times 6$ matrix representing confidence/uncertainty in velocity.

- **The `odom` Coordinate Frame (REP-105)**:
  - The **`odom`** frame is a world-fixed frame initialized at the robot's starting position ($0, 0, 0$).
  - The TF transform **`odom -> base_link`** continuously tracks where the robot's base is relative to its start.

- **Drift & Localization**:
  - Odometry is fast, smooth, and continuous, but accumulates **drift** over time due to wheel slip, bumps, and sensor noise.
  - In navigation stacks (Nav2 / SLAM), odometry provides high-frequency local motion tracking, while global localization (e.g., AMCL / LiDAR SLAM) corrects long-term drift against a global **`map`** frame (`map -> odom -> base_link`).

## Commands
```bash
# install gazebo packages
sudo apt install ros-jazzy-ros-gz

# start gazebo GUI directly
gz sim

# load and run the SDF world file directly in Gazebo Sim (-r runs simulation unpaused)
gz sim src/simple_car_description/worlds/forest.sdf -r
# or from the installed package share directory
gz sim $(ros2 pkg prefix --share simple_car_description)/worlds/forest.sdf -r

# check gazebo topics
gz topic -l

# build package
cd /home/kin/shared/shared/ros-practice/5.urdf
source /opt/ros/jazzy/setup.bash
colcon build --packages-select simple_car_description --symlink-install
source install/setup.bash

# 1. Launch robot in Forest World (default)
ros2 launch simple_car_description gazebo.launch.py       # Python
ros2 launch simple_car_description gazebo.launch.xml      # XML

# 2. Launch in Forest World with RViz enabled
ros2 launch simple_car_description gazebo.launch.py rviz:=true   # Python
ros2 launch simple_car_description gazebo.launch.xml rviz:=true  # XML

# 3. Launch with empty world instead of forest
ros2 launch simple_car_description gazebo.launch.py world:=empty.sdf   # Python
ros2 launch simple_car_description gazebo.launch.xml world:=empty.sdf  # XML

# 4. Customize spawn position (e.g. x=1.0, y=2.0, z=0.1)
ros2 launch simple_car_description gazebo.launch.py x:=1.0 y:=2.0 z:=0.1   # Python
ros2 launch simple_car_description gazebo.launch.xml x:=1.0 y:=2.0 z:=0.1  # XML

# 5. Drive the robot using keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

### Teleop Keyboard Controls (`teleop_twist_keyboard`)

|        Key        | Action               | Description                                  |
| :---------------: | :------------------- | :------------------------------------------- |
|      **`i`**      | **Forward**          | Drive straight forward                       |
| **`,`** *(comma)* | **Backward**         | Drive straight backward                      |
|      **`j`**      | **Turn Left**        | Rotate in place counter-clockwise            |
|      **`l`**      | **Turn Right**       | Rotate in place clockwise                    |
|      **`u`**      | **Forward + Left**   | Curve forward to the left                    |
|      **`o`**      | **Forward + Right**  | Curve forward to the right                   |
|      **`m`**      | **Backward + Left**  | Curve backward to the left                   |
|  **`.`** *(dot)*  | **Backward + Right** | Curve backward to the right                  |
|      **`k`**      | **Stop**             | Stop all movement (sets velocities to `0.0`) |
|  *any other key*  | **Stop**             | Any unmapped key stops the robot             |

> **Keypad Layout**:
> ```text
>   u   i   o       (↖  ↑  ↗)
>   j   k   l       (← stop →)
>   m   ,   .       (↙  ↓  ↘)
> ```

#### Speed Adjustments
|        Key        | Action                     | Effect                                       |
| :---------------: | :------------------------- | :------------------------------------------- |
| **`q`** / **`z`** | **Linear & Angular Speed** | Increase / Decrease all speeds by **10%**    |
| **`w`** / **`x`** | **Linear Speed Only**      | Increase / Decrease linear speed by **10%**  |
| **`e`** / **`c`** | **Angular Speed Only**     | Increase / Decrease angular speed by **10%** |