# ROS 2 Control (`8.ros2_control`)

A workspace dedicated to **`ros2_control`** practices, covering hardware interfaces (`mock_components`), resource managers (`controller_manager`), transmissions, controllers (`diff_drive_controller`), and RViz2 visualization.

---

## 📁 Package Architecture & Directory Layout

```text
8.ros2_control/
├── .envrc                          # direnv environment setup (sources install/setup.bash)
├── .vscode/                        # VS Code C++ & CMake settings
└── src/
    ├── robot_bringup/              # ros2_control configuration & bringup
    │   ├── CMakeLists.txt          # ament_cmake build rules
    │   ├── package.xml             # Dependencies (controller_manager, diff_drive_controller, etc.)
    │   ├── config/
    │   │   └── robot_controllers.yaml # controller_manager parameters & controllers
    │   └── launch/
    │       ├── robot.launch.xml    # Orchestrates robot_state_publisher, controller_manager, spawners & RViz2
    │       └── teleop.launch.xml   # Keyboard teleop configured with TwistStamped for diff_drive_controller
    └── robot_description/          # Robot kinematic & visual description package
        ├── CMakeLists.txt          # ament_cmake build rules
        ├── package.xml             # Dependencies (xacro, robot_state_publisher, rviz2, etc.)
        ├── launch/
        │   └── display.launch.xml  # XML launch file to preview model with joint_state_publisher_gui
        ├── meshes/                 # 3D visual mesh assets
        │   └── visual/
        │       └── waffle_base.stl
        ├── rviz/
        │   └── display.rviz        # Pre-configured RViz display profile
        └── urdf/                   # Modular Xacro kinematic model (Gazebo-free)
            ├── simple_car.urdf.xacro       # Top-level entry point
            ├── simple_car.properties.xacro # Dimensions, offsets & masses
            ├── simple_car.materials.xacro  # Color/material definitions
            ├── simple_car.inertias.xacro   # Inertia calculation macros
            ├── simple_car.wheel.xacro      # Wheel link & joint macro
            ├── simple_car.arm.xacro        # 2-DOF robotic arm macro
            └── simple_car.ros2_control.xacro # ros2_control tag with mock_components hardware
```

---

## 🚀 Quickstart & Execution Guide

### 1. Build the Workspace

From inside the `8.ros2_control` directory:

```bash
cd 8.ros2_control
colcon build --symlink-install
source install/setup.bash
```

> [!TIP]
> With `--symlink-install`, any edits made to `.xacro`, `.xml`, `.yaml`, or `.rviz` files take effect immediately on the next launch without requiring a rebuild.

---

### 2. Method A: Unified Launch via `robot.launch.xml` (Recommended)

Launch the complete `ros2_control` stack including `robot_state_publisher`, `ros2_control_node`, controller spawners, and RViz2:

```bash
ros2 launch robot_bringup robot.launch.xml
```

To run headless (without launching RViz2):

```bash
ros2 launch robot_bringup robot.launch.xml use_rviz:=false
```

---

### 3. Method B: Step-by-Step Manual CLI Commands

To understand how each node interacts within the `ros2_control` architecture, you can run them manually across separate terminals:

#### Terminal 1: Start `robot_state_publisher`
Parses the Xacro robot description and publishes `/robot_description` and TF transforms (launch file safely passes the multi-line XML string as a typed parameter):

```bash
ros2 launch robot_description rsp.launch.xml
```

#### Terminal 2: Start `controller_manager` (`ros2_control_node`)
Instantiates the real-time resource manager and hardware interfaces (`mock_components/GenericSystem`). In ROS 2 Jazzy, `ros2_control_node` automatically subscribes to `/robot_description` published by `robot_state_publisher`:

```bash
ros2 run controller_manager ros2_control_node --ros-args \
  --params-file $(ros2 pkg prefix --share robot_bringup)/config/robot_controllers.yaml
```

#### Terminal 3: Spawn & Activate Controllers
Load and activate the controllers defined in `robot_controllers.yaml`:

```bash
# 1. Activate joint state broadcaster (publishes /joint_states)
ros2 run controller_manager spawner joint_state_broadcaster

# 2. Activate differential drive controller (listens to cmd_vel, computes odometry)
ros2 run controller_manager spawner diff_drive_controller
```

#### Terminal 4: Inspect & Drive the Robot

```bash
# Verify active hardware interfaces (command and state interfaces)
ros2 control list_hardware_interfaces

# Verify controller states (both should show: active [active])
ros2 control list_controllers

# Send a one-time velocity command (add `--once` to the command to publish once; after 0.5s cmd_vel_timeout the robot brakes)
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped '{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"},
  twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}
}'

# Echo odometry updates published by diff_drive_controller
ros2 topic echo /diff_drive_controller/odom

# Visualize in RViz2
rviz2 -d $(ros2 pkg prefix --share robot_description)/rviz/display.rviz
```

---

### 4. Keyboard Teleoperation (`teleop_twist_keyboard`)

Drive the robot interactively using the keyboard. In ROS 2 Jazzy, `diff_drive_controller` expects `geometry_msgs/msg/TwistStamped`, so `stamped:=true` and `frame_id:=base_link` must be set.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r /cmd_vel:=/diff_drive_controller/cmd_vel \
  -p stamped:=true \
  -p frame_id:=base_link
```

#### Keypad Layout & Movement Controls

```text
    u   i   o       (↖  ↑  ↗)
    j   k   l       (← stop →)
    m   ,   .       (↙  ↓  ↘)
```

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
| **`k`** / *other* | **Stop**             | Stop all movement (sets velocities to `0.0`) |

#### Speed Adjustment Controls

|   Key   | Action                     | Effect                          |
| :-----: | :------------------------- | :------------------------------ |
| **`w`** | **Increase Linear Speed**  | Boosts linear speed by +10%     |
| **`x`** | **Decrease Linear Speed**  | Lowers linear speed by -10%     |
| **`e`** | **Increase Angular Speed** | Boosts rotational speed by +10% |
| **`c`** | **Decrease Angular Speed** | Lowers rotational speed by -10% |

---

### 5. Kinematics-Only GUI Preview (without `ros2_control`)

To preview the URDF and test joint movements with graphical sliders without starting `ros2_control`:

```bash
ros2 launch robot_description display.launch.xml
```
