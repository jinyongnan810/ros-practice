### Installation
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

### Bringup turtlebot3
```bash
# Start turtle bot
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Control the bot
export TURTLEBOT3_MODEL=waffle
sudo apt install ros-jazzy-turtlebot3-teleop
ros2 run turtlebot3_teleop teleop_keyboard

# Show lidar
`...`button -> Visualize lidar -> scan


```