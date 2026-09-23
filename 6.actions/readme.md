# Actions
An Action is a client/server communication architecture designed for long-running, preemptible tasks.

An Action consists of three main parts:
- **Goal**: The client requests the server to perform a task. The server can accept or reject the goal.
- **Feedback**: While executing the goal, the server periodically sends progress updates back to the client.
- **Result**: Upon finishing the task, the server sends the final outcome (status and result data) back to the client.

Actions have following features:
- **Asynchronous & Non-blocking**: Clients can send a goal and continue doing other tasks while periodically receiving feedback.
- **Preemptible / Cancellable**: Clients can request cancellation of an active goal at any point during execution.
- **Composed of Topics & Services**: Under the hood, ROS 2 actions are implemented using multiple topics (feedback, status) and services (send goal, cancel goal, get result).

---

## 📋 Commands Guide

### 1. Workspace & Package Setup
```bash
# Enable environment (if using direnv)
direnv allow

# Create packages
ros2 pkg create action_py_pkg --build-type ament_python --dependencies rclpy
ros2 pkg create action_cpp_pkg --build-type ament_cmake --dependencies rclcpp rclcpp_action rclcpp_components
ros2 pkg create custom_interfaces --build-type ament_cmake

# Build workspace
colcon build

# Build individual packages
colcon build --packages-select custom_interfaces
colcon build --packages-select action_py_pkg
colcon build --packages-select action_cpp_pkg

# Source workspace overlay
source install/setup.bash
```

### 2. Launching — Turtlesim 2D Simulation
Launch turtlesim alongside the action server (and optionally the client).

```bash
# Launch turtlesim and action server (Python)
ros2 launch action_py_pkg turtle_action.launch.py

# Launch turtlesim, server, AND client (Python)
ros2 launch action_py_pkg turtle_action.launch.py launch_client:=true
ros2 launch action_py_pkg turtle_action.launch.xml launch_client:=true

# Launch turtlesim, server, AND client (C++)
ros2 launch action_cpp_pkg turtle_action.launch.py launch_client:=true
ros2 launch action_cpp_pkg turtle_action.launch.xml launch_client:=true

# Launch with custom client navigation parameters
ros2 launch action_py_pkg turtle_action.launch.py launch_client:=true target_x:=3.0 target_y:=7.0 linear_velocity:=2.5
```

### 3. Launching — ROS 2 Components & Composition
Run nodes composed inside a single `component_container` process for shared-memory zero-copy IPC and reduced resource overhead.

```bash
# Launch server and client composed in a component container (Python launch)
ros2 launch action_cpp_pkg turtle_action_component.launch.py launch_client:=true

# Launch via XML
ros2 launch action_cpp_pkg turtle_action_component.launch.xml launch_client:=true

# Inspect available component types
ros2 component types

# List active containers and loaded components
ros2 component list

# Dynamically load a component into a running container at runtime
ros2 component load /turtle_action_container action_cpp_pkg action_cpp_pkg::TurtleActionClientNode \
  --node-name turtle_action_client_2 \
  -p target_x:=2.0 -p target_y:=3.0 -p linear_velocity:=2.5

# Unload a component by its unique ID
ros2 component unload /turtle_action_container 3
```

### 4. Launching — 3D Gazebo Simulation (TurtleBot3)
Simulate realistic physics navigation in Gazebo Sim (Harmonic in ROS 2 Jazzy).

```bash
# Install simulation dependencies (if not already installed)
sudo apt install -y ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-simulations

# Launch Gazebo world with TurtleBot3 and action server (C++)
ros2 launch action_cpp_pkg turtlebot3_action.launch.py

# Launch Gazebo, TurtleBot3, AND action client (C++)
ros2 launch action_cpp_pkg turtlebot3_action.launch.py launch_client:=true target_x:=2.0 target_y:=2.0

# Launch Gazebo, TurtleBot3, AND action client (Python)
ros2 launch action_py_pkg turtlebot3_action.launch.py launch_client:=true target_x:=2.0 target_y:=2.0
ros2 launch action_py_pkg turtlebot3_action.launch.xml launch_client:=true target_x:=2.0 target_y:=2.0
```

### 5. Running Individual Nodes (`ros2 run`)
Run individual nodes in separate terminal windows for manual debugging.

```bash
# Terminal 1: Turtlesim window
ros2 run turtlesim turtlesim_node

# Terminal 2: Action server (choose Python or C++)
ros2 run action_py_pkg turtle_action_server
ros2 run action_cpp_pkg turtle_action_server

# Terminal 3: Action client (default target: 8.5, 8.5)
ros2 run action_py_pkg turtle_action_client
ros2 run action_cpp_pkg turtle_action_client

# Run client with custom target coordinates and linear speed
ros2 run action_py_pkg turtle_action_client --ros-args -p target_x:=3.0 -p target_y:=7.0 -p linear_velocity:=2.5

# Run client with auto-cancellation after 2 seconds
ros2 run action_py_pkg turtle_action_client --ros-args -p target_x:=1.0 -p target_y:=1.0 -p cancel_after_sec:=2.0
```

### 6. Introspection, CLI Testing & Preemption
```bash
# List active nodes and inspect node info
ros2 node list
ros2 node info /turtle_action_server

# List actions
ros2 action list
ros2 action list -t

# Inspect action server/client endpoints
ros2 action info /move_to_goal

# View action interface structure (Goal, Result, Feedback)
ros2 interface show custom_interfaces/action/MoveToGoal

# Send an action goal directly from CLI (with live feedback)
ros2 action send_goal /move_to_goal custom_interfaces/action/MoveToGoal "{target_x: 8.0, target_y: 8.0, linear_velocity: 2.0}" --feedback

# Test goal preemption: send Goal 1, then immediately send Goal 2 to preempt it
# Terminal A:
ros2 action send_goal /move_to_goal custom_interfaces/action/MoveToGoal "{target_x: 2.0, target_y: 2.0, linear_velocity: 1.0}" --feedback
# Terminal B (while Goal 1 is still moving):
ros2 action send_goal /move_to_goal custom_interfaces/action/MoveToGoal "{target_x: 9.0, target_y: 9.0, linear_velocity: 2.5}" --feedback

# Visualize computation graph
rqt_graph
```

---

### 💡 Tips & Best Practices

#### 1. Rate-Regulated Loop vs. Fixed `sleep()`
Always prefer ROS 2 `Rate` objects over fixed `time.sleep()` / `sleep()` in control and action execution loops:
- **Python**: `loop_rate = self.create_rate(10)` $\rightarrow$ `loop_rate.sleep()`
- **C++**: `rclcpp::Rate loop_rate(10);` $\rightarrow$ `loop_rate.sleep();`

**Key Advantages**:
- **Drift Compensation**: If computation (math, coordinate transforms, feedback publishing) takes 20 ms, a fixed `sleep(0.1)` yields a 120 ms cycle (8.3 Hz instead of 10 Hz), causing clock drift. `Rate.sleep()` automatically measures the elapsed execution time and sleeps only the remaining 80 ms to maintain a precise 10 Hz cycle.
- **Simulation Time Awareness (`use_sim_time`)**: `Rate` respects the ROS clock (`/clock`). When Gazebo or simulation time slows down, speeds up, or pauses, `Rate.sleep()` synchronizes accordingly. In contrast, fixed `sleep()` strictly uses the host's real wall-clock time.

#### 2. Goal Preemption Policy
ROS 2 actions are unopinionated about concurrency by default. For a single actuator/robot:
- Track `active_goal_handle` protected by a lock (`threading.Lock` / `std::mutex`).
- When a new goal arrives, assign it as the active goal.
- On each tick of the running loop, check `if goal_handle != active_goal_handle`. If preempted, call `goal_handle.abort()` and exit gracefully without stopping the turtle so the new goal takes over steering immediately.

#### 3. Concurrency in Python Action Servers
If `execute_callback` runs a continuous control loop, a single-threaded executor would block incoming subscriber callbacks (such as `/turtle1/pose`). To keep pose updates flowing:
- Assign both the subscriber and the action server to a `ReentrantCallbackGroup`.
- Run the node using `MultiThreadedExecutor`.

#### 4. ROS 2 Components & Composition (`rclcpp_components`)
Rather than running each node in its own operating system process:
- **Shared Memory & Zero-Copy IPC**: Composing nodes into a single `component_container` allows inter-node communication to bypass network serialization and socket buffers when `use_intra_process_comms` is enabled.
- **Resource Efficiency**: Multiple nodes share a single process footprint and executor, significantly lowering thread overhead, CPU context switching, and memory usage.
- **Dual-Mode Build**: By using `rclcpp_components_register_node(..., EXECUTABLE ...)`, nodes are compiled into reusable shared libraries (`.so` / `.dylib`) for dynamic loading into containers, while automatically generating standalone executables for traditional `ros2 run` invocations.
- **Dynamic Loading & Unloading**: Components can be loaded, configured, and unloaded dynamically at runtime without restarting the host container or other sibling nodes.

#### 5. 3D Simulation & Odometry Feedback (Gazebo + TurtleBot3)
While 2D Turtlesim provides a canvas `[0.0, 11.0]` with absolute coordinates (`turtlesim/msg/Pose`), real and 3D simulated robots (such as TurtleBot3 in Gazebo Sim) operate in continuous open Cartesian space using standard ROS navigation messages:
- **Pose Source (`nav_msgs/msg/Odometry`)**: The robot's position and orientation quaternion are continuously published to `/odom`. The action server extracts the planar yaw $\theta = \text{atan2}(2(wz + xy), 1 - 2(y^2 + z^2))$ and tracks traveled Euclidean distance.
- **Velocity Topic (`/cmd_vel`) & `TwistStamped` in ROS 2 Jazzy**: Wheel differential drive commands are sent to `/cmd_vel`. In ROS 2 Jazzy Gazebo Harmonic (`turtlebot3_gazebo`), the `ros_gz_bridge` requires `geometry_msgs/msg/TwistStamped` on `/cmd_vel` rather than bare `Twist`. The action server automatically detects `ROS_DISTRO` (or accepts `use_stamped_vel:=true/false`), publishing `TwistStamped` with headers to `/cmd_vel` while maintaining backwards-compatible `Twist` publishing to `/turtle1/cmd_vel` for Turtlesim.
- **Simulation Clock (`use_sim_time:=true`)**: Synchronizes the action control loop rate (`Rate(10)`) with Gazebo's `/clock` topic, compensating for physics stepping speed and simulation pause.
- **Realistic Physical Speeds & Clamped Steering**: Unlike 2D sprites in Turtlesim that can instantly accelerate to 2.0 m/s, physical robots like TurtleBot3 Burger have wheel slip and actuator limits (max linear speed $\approx 0.22\text{ m/s}$, max angular speed $\approx 2.84\text{ rad/s}$). The action server clamps angular speed to $\pm 2.0\text{ rad/s}$ and launch files default linear velocity to $0.22\text{ m/s}$ for smooth, stable physics simulation.