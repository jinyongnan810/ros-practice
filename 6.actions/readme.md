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

### Commands
```bash
# enable env
direnv allow

# create python pkg
ros2 pkg create action_py_pkg --build-type ament_python --dependencies rclpy
# create c++ pkg
ros2 pkg create action_cpp_pkg --build-type ament_cmake --dependencies rclcpp rclcpp_action
# create custom interface pkg (if defining a custom action, e.g. CountUntil.action)
ros2 pkg create custom_interfaces --build-type ament_cmake

# build all pkgs
colcon build
# build one pkg
colcon build --packages-select custom_interfaces
colcon build --packages-select action_py_pkg
colcon build --packages-select action_cpp_pkg

# launch turtlesim and server node together (default: client disabled)
ros2 launch action_py_pkg turtle_action.launch.py

# launch turtlesim, server, AND client all together (standalone processes)
ros2 launch action_py_pkg turtle_action.launch.py launch_client:=true
# or using XML
ros2 launch action_py_pkg turtle_action.launch.xml launch_client:=true
# or using C++ package
ros2 launch action_cpp_pkg turtle_action.launch.py launch_client:=true

# launch via ROS 2 Component Container (shared process composition)
ros2 launch action_cpp_pkg turtle_action_component.launch.py launch_client:=true
# or using XML
ros2 launch action_cpp_pkg turtle_action_component.launch.xml launch_client:=true

# component CLI commands
ros2 component types
ros2 component list
# manually load components into container
ros2 component load /turtle_action_container action_cpp_pkg action_cpp_pkg::TurtleActionClientNode \
  --node-name turtle_action_client_2 \
  -p target_x:=2.0 -p target_y:=3.0 -p linear_velocity:=2.5
# unload component by ID
ros2 component unload /turtle_action_container 3

# run nodes manually
ros2 run turtlesim turtlesim_node
ros2 run action_py_pkg turtle_action_server
ros2 run action_cpp_pkg turtle_action_server

# run client with default target (8.5, 8.5)
ros2 run action_py_pkg turtle_action_client
ros2 run action_cpp_pkg turtle_action_client

# run client with custom target coordinates and speed
ros2 run action_py_pkg turtle_action_client --ros-args -p target_x:=3.0 -p target_y:=7.0 -p linear_velocity:=2.5

# run client with automatic cancellation after 2 seconds
ros2 run action_py_pkg turtle_action_client --ros-args -p target_x:=1.0 -p target_y:=1.0 -p cancel_after_sec:=2.0

# list running nodes
ros2 node list

# see node info
ros2 node info /turtle_action_server

# list actions
ros2 action list
# list actions with types
ros2 action list -t

# inspect action details (servers and clients)
ros2 action info /move_to_goal

# inspect action interface definition (.action)
ros2 interface show custom_interfaces/action/MoveToGoal

# send action goal directly from CLI (with feedback)
ros2 action send_goal /move_to_goal custom_interfaces/action/MoveToGoal "{target_x: 8.0, target_y: 8.0, linear_velocity: 2.0}" --feedback

# test preemption: send goal 1, then immediately send goal 2 from another terminal
# terminal A:
ros2 action send_goal /move_to_goal custom_interfaces/action/MoveToGoal "{target_x: 2.0, target_y: 2.0, linear_velocity: 1.0}" --feedback
# terminal B (while goal 1 is still moving):
ros2 action send_goal /move_to_goal custom_interfaces/action/MoveToGoal "{target_x: 9.0, target_y: 9.0, linear_velocity: 2.5}" --feedback

# check node graph
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