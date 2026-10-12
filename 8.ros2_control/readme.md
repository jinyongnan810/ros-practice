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
    │   ├── package.xml             # Dependencies (controller_manager, rclpy, python3-tk, diff_drive_controller, etc.)
    │   ├── config/
    │   │   └── robot_controllers.yaml # controller_manager parameters & controllers
    │   ├── scripts/
    │   │   └── position_command_gui.py # Sliders commanding platform and arm through ros2_control
    │   └── launch/
    │       ├── robot.launch.xml    # Orchestrates state publishers, controller_manager, spawners & RViz2
    │       └── teleop.launch.xml   # Keyboard teleop configured with TwistStamped for diff_drive_controller
    ├── robot_hardware/             # ros2_control hardware interface & motor driver package
    │   ├── CMakeLists.txt          # ament_cmake build rules (compiles robot_hardware plugin library)
    │   ├── package.xml             # Dependencies (hardware_interface, dynamixel_sdk, etc.)
    │   ├── robot_hardware.xml      # Pluginlib plugin description file
    │   ├── include/
    │   │   └── robot_hardware/
    │   │       ├── base_link_hardware_interface.hpp # Hardware interface header (SystemInterface)
    │   │       └── xl330_driver.hpp # Dynamixel XL-330 motor driver
    │   └── src/
    │       └── base_link_hardware_interface.cpp # Hardware interface implementation
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
        ├── worlds/                 # Local ground plane and forest from 5.urdf
        └── urdf/                   # Modular Xacro with optional Gazebo support
            ├── simple_car.urdf.xacro       # Top-level entry point
            ├── simple_car.properties.xacro # Dimensions, offsets & masses
            ├── simple_car.materials.xacro  # Color/material definitions
            ├── simple_car.inertias.xacro   # Inertia calculation macros
            ├── simple_car.wheel.xacro      # Wheel link & joint macro
            ├── simple_car.arm.xacro        # 2-DOF robotic arm macro
            ├── simple_car.gazebo.xacro # Camera and gz_ros2_control system plugin
            └── simple_car.ros2_control.xacro # Separate components with mock/Gazebo backends
```

---

## 🚀 Quickstart & Execution Guide

### 1. Install Dynamixel SDK & Hardware Dependencies

The `robot_hardware` package communicates with Dynamixel XL-330 smart actuators via the official ROBOTIS **Dynamixel SDK** (`dynamixel_sdk`).

Install the pre-built ROS 2 binary package for your ROS distribution (e.g. ROS 2 Jazzy):

```bash
sudo apt update
sudo apt install -y ros-jazzy-dynamixel-sdk
```

> [!NOTE]
> If you are using a different ROS 2 distribution (such as Humble or Iron), use the dynamic environment variable:
> ```bash
> sudo apt install -y ros-${ROS_DISTRO}-dynamixel-sdk
> ```

> [!TIP]
> **Serial Port Permissions (`/dev/ttyUSB*` / `/dev/ttyACM*`)**:
> To access USB-to-Dynamixel serial adapters (such as ROBOTIS U2D2 or OpenRB-150) without root privileges, add your user to the `dialout` group:
> ```bash
> sudo usermod -aG dialout $USER
> ```
> *Note: Log out and log back in (or restart your terminal session) for the new group membership to take effect.*

---

### 2. Build the Workspace

From inside the `8.ros2_control` directory:

```bash
cd 8.ros2_control
colcon build --symlink-install
source install/setup.bash
```

> [!TIP]
> With `--symlink-install`, any edits made to `.xacro`, `.xml`, `.yaml`, or `.rviz` files take effect immediately on the next launch without requiring a rebuild.

---

### 3. Hardware Interface Operation Modes (`robot_hardware`)

The custom `BaseLinkHardwareInterface` controls the 4-wheel differential drive chassis and supports two reading strategies:

| Feature / Criteria      | **Direct Reading (`getPositionRadian`)**                                               | **Integration (`get_state + vel * dt`)**                                      |
| :---------------------- | :------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------- |
| **Primary Use Case**    | **Physical Robot Hardware**                                                            | **Software Testing / Simulation / Demos**                                     |
| **Physics Awareness**   | ✅ Detects wheel slips, stalls, motor inertia, terrain bumps, external pushes           | ❌ Assumes ideal motor response; reports movement even if the wheel is stalled |
| **Error Accumulation**  | ✅ Zero numerical integration drift (reads actual encoder counter)                      | ⚠️ Accumulates numerical errors ($O(\Delta t)$) over time                      |
| **Hardware Dependency** | ❌ Requires physical motors connected over serial/USB (`/dev/ttyUSB0`)                  | ✅ Can run anywhere (laptops, CI pipelines, headless VMs)                      |
| **Odometry Accuracy**   | ✅ Realistic odometry for SLAM / Navigation ($\Delta \theta = \theta_t - \theta_{t-1}$) | ⚠️ Ideal kinematic odometry (pure dead reckoning)                              |

#### Automatic Fallback
- **No Hardware Connected**: If `/dev/ttyUSB0` cannot be opened at `on_configure`, the interface automatically logs a warning and proceeds in **simulation mode** (`pos += vel * dt`). This allows testing controllers, RViz2, and keyboard teleop without physical hardware.
- **Switching to Real Hardware**: When physical Dynamixel XL-330 servos are connected, simply uncomment the real hardware read/write blocks in [`base_link_hardware_interface.cpp`](src/robot_hardware/src/base_link_hardware_interface.cpp).

---

### Separate Platform and Arm Hardware Interfaces

The Xacro defines three independent hardware components: `simple_car` (four drive wheels), `rotating_platform` (`wheel_joint`), and `robotic_arm` (`arm_joint_1`, `arm_joint_2`). Each defaults to `mock_components/GenericSystem`, so the existing controllers and GUI work unchanged.

To use a position component with physical actuators, replace its mock plugin and `calculate_dynamics` parameter with the commented `robot_hardware/WheelJointHardwareInterface` or `robot_hardware/ArmJointsHardwareInterface` plugin and its serial parameters. Each plugin accepts `port_name` and `baud_rate` (default 57600), position commands, and position/velocity states. Configure motor IDs using each joint's `dxl_id` parameter; the Xacro examples use 14 for the platform and 15–16 for the arm. Without these parameters, plugin defaults are 5 and 6–7 respectively.

Each component owns a serial connection. Use separate adapters/ports for the base, platform, and arm; the commented examples use `/dev/ttyUSB0`, `/dev/ttyUSB1`, and `/dev/ttyUSB2`. These independent plugins do not coordinate access to a shared serial bus.

The new position plugins read encoder feedback and write position targets when connected. Activation holds the measured position before enabling torque; deactivation disables torque. If the connection cannot be initialized, they fall back to mock position feedback with velocity computed over the control period. The current XL330 driver does not propagate per-motor communication errors, so opening a port alone does not confirm actuator communication.

---

### 4. Method A: Unified Launch via `robot.launch.xml` (Recommended)

Launch the complete `ros2_control` stack including `robot_state_publisher`, `ros2_control_node`, controller spawners, and RViz2:

```bash
ros2 launch robot_bringup robot.launch.xml
```

To control the arm and rotating platform with sliders, enable the position command GUI:

```bash
ros2 launch robot_bringup robot.launch.xml use_position_command_gui:=true
```

The GUI publishes `std_msgs/msg/Float64MultiArray` position targets in radians to `/arm_controller/commands` (ordered as `arm_joint_1`, `arm_joint_2`) and `/wheel_joint_controller/commands` (`wheel_joint`). It subscribes to `/joint_states` for actual positions and never publishes synthetic joint states. Slider ranges come from the selected URDF's joint limits. Sliders wait for fresh feedback and a command subscriber, initialize from measured positions without sending commands, and send targets when moved. Targets are direct position setpoints, without trajectory interpolation.

The current `mock_components/GenericSystem` setup moves the mock joints. With a hardware plugin supporting these position interfaces, the same commands move physical actuators. The drive wheels use `BaseLinkHardwareInterface`; separate `WheelJointHardwareInterface` and `ArmJointsHardwareInterface` plugins provide the platform and arm position interfaces.

To run headless (without launching RViz2):

```bash
ros2 launch robot_bringup robot.launch.xml use_rviz:=false
```

---

### Gazebo Sim with ros2_control (ROS 2 Jazzy / Harmonic)

> [!TIP]
> **When to use Gazebo Sim with ros2_control**
>
> Use Gazebo to test the same controller setup against a simulated robot with physics before connecting real hardware. For this robot, it lets you test driving and odometry with `diff_drive_controller`, platform and arm movement with the position controllers and GUI, gravity, collisions, friction, and navigation using simulated camera data.
>
> | Workflow | Best use |
> | :--- | :--- |
> | Mock hardware + RViz | Quickly check controller configuration, topics, and joint motion without physics. |
> | Gazebo + ros2_control | Test those controllers with simulated physics and sensors. |
> | Physical hardware interfaces | Operate and test the actual motors. |
>
> Gazebo selects `GazeboSimSystem` as the hardware backend while keeping the same controller commands. Its plugin creates the controller manager inside the simulator. Simulation helps catch problems early; real hardware still needs testing for motor response, communication, and calibration.

Install the simulation packages (the `gz_ros2_control` packages you already installed supply `GazeboSimSystem`):

```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control
```

Build and launch from `8.ros2_control`:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_bringup gazebo.launch.xml use_position_command_gui:=true
```

Add `use_rviz:=true` for RViz. The default world is a local ground plane derived from `5.urdf`, requiring no Fuel downloads. To use the original forest world:

```bash
ros2 launch robot_bringup gazebo.launch.xml \
  world:="$(ros2 pkg prefix --share robot_description)/worlds/forest.sdf"
```

The forest may download its tree models on first launch. The camera attachment, wheel friction, arm dynamics, and world setup are reused from `5.urdf`.

The launch passes `use_gazebo:=true` and the controller YAML path to Xacro. All three hardware components select `gz_ros2_control/GazeboSimSystem`; the Gazebo system plugin creates the controller manager inside the simulator. The spawner waits for that manager and activates the joint state broadcaster, differential drive controller, platform controller, and arm controller. Use this launch by itself; `robot.launch.xml` starts a separate manager for the mock/physical workflow.

The clock and camera topics are bridged from Gazebo. Joint states, odometry, TF, and commands come directly from ros2_control, so this model omits the standalone Gazebo drive, position-control, and joint-state plugins used in `5.urdf`. Existing controller commands and the position GUI apply to the simulated robot:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic pub --once /wheel_joint_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.5]}'
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.5, -0.5]}'
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true -p frame_id:=base_link -p use_sim_time:=true
```

For direct Xacro conversion, supply both arguments:

```bash
xacro src/robot_description/urdf/simple_car.urdf.xacro use_gazebo:=true \
  controller_config:="$(ros2 pkg prefix --share robot_bringup)/config/robot_controllers.yaml"
```

Without `use_gazebo:=true`, the existing mock components remain selected. See the [official Jazzy gz_ros2_control documentation](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html) for the simulation plugin configuration.

---

### 5. Method B: Step-by-Step Manual CLI Commands

To understand how each node interacts within the `ros2_control` architecture, you can run them manually across separate terminals. This procedure uses `joint_state_broadcaster` for controller feedback, matching the unified launch's default behavior:

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

# 3. Activate rotating platform position controller
ros2 run controller_manager spawner wheel_joint_controller

# 4. Activate robotic arm position controller
ros2 run controller_manager spawner arm_controller
```

#### Terminal 4: Inspect & Drive the Robot

```bash
# Verify active hardware interfaces (command and state interfaces)
ros2 control list_hardware_interfaces

# Verify controller states (all should show: active [active])
ros2 control list_controllers

# Send a one-time velocity command (add `--once` to the command to publish once; after 0.5s cmd_vel_timeout the robot brakes)
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped '{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"},
  twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}
}'

# Command rotating platform position (rad)
ros2 topic pub --once /wheel_joint_controller/commands std_msgs/msg/Float64MultiArray '{data: [1.57]}'

# Command robotic arm joints positions [arm_joint_1, arm_joint_2] (rad)
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.5, -0.5]}'

# Echo odometry updates published by diff_drive_controller
ros2 topic echo /diff_drive_controller/odom

# Visualize in RViz2
rviz2 -d $(ros2 pkg prefix --share robot_description)/rviz/display.rviz
```

---

### 6. Keyboard Teleoperation (`teleop_twist_keyboard`)

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

### 7. Kinematics-Only GUI Preview (without `ros2_control`)

To preview the URDF and test joint movements with graphical sliders without starting `ros2_control`:

```bash
ros2 launch robot_description display.launch.xml
```
