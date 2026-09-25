# ROS 2 Control (`8.ros2_control`)

A workspace dedicated to **`ros2_control`** practices, covering hardware interfaces, resource managers, transmissions, controllers, and RViz2 visualization.

---

## 📁 Package Architecture & Directory Layout

```text
8.ros2_control/
├── .envrc                          # direnv environment setup (sources install/setup.bash)
├── .vscode/                        # VS Code C++ & CMake settings
└── src/
    └── robot_description/          # Robot kinematic & visual description package
        ├── CMakeLists.txt          # ament_cmake build rules
        ├── package.xml             # Dependencies (xacro, robot_state_publisher, rviz2, etc.)
        ├── launch/
        │   └── display.launch.xml  # XML launch file to preview model in RViz2
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
            └── simple_car.arm.xacro        # 2-DOF robotic arm macro
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
> With `--symlink-install`, any edits made to `.xacro`, `.xml`, or `.rviz` files take effect immediately on the next launch without requiring a rebuild.

---

### 2. Visualize in RViz2

Launch the XML visualization file:

```bash
ros2 launch robot_description display.launch.xml
```

This will launch:
1. `robot_state_publisher` - Parses Xacro URDF and publishes `/robot_description` and transforms (`/tf`).
2. `joint_state_publisher_gui` - Provides a graphical slider interface to manipulate each joint (`wheel_joint`, `arm_joint_1`, `arm_joint_2`, etc.).
3. `rviz2` - Preloaded with the display configuration showing the robot model, grid, and TF frames.
