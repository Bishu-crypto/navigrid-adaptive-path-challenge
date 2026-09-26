# NaviGrid: The Adaptive Path Challenge
Complete ROS 2 Jazzy & Gazebo Harmonic (GZ Sim 8) Autonomous Navigation Stack

---

## Architecture Overview

NaviGrid is an autonomous mobile robot navigation stack designed for dynamic, multi-elevation warehouse environments featuring:
- **Arena (30m x 30m)**: Enclosed boundary walls with standard LiDAR-reflective surface, Start Zone A (-11.0, 0.0), Goal Zone B (+11.0, 0.0).
- **Two Distinct Paths**:
  1. *Direct Incline Ramp*: Elevated bridge shortcut (14°–15° slope) connecting A to B along the shortest Euclidean line.
  2. *Zig-Zag Ground Floor Path*: Flat ($z=0$) winding corridor connecting A to B via warehouse racks and a narrow chokepoint corridor.
- **Dynamic Agents**:
  - Two animated pedestrian actors walking fixed crossing trajectories.
  - Secondary AMR patrolling perpendicular to the route on velocity control.
- **Mobile Robot (Differential Drive)**:
  - 2D LiDAR (360° FOV, 20 Hz, `/scan`)
  - 3-axis IMU (50 Hz, `/imu`)
  - Wheel Odometry (50 Hz, `/odom`, with `/tf` broadcasting)
- **Native Gazebo Harmonic Bridge (`harmonic_bridge`)**:
  - Direct C++ bridge using `gz-transport13` and `gz-msgs10` eliminating version mismatches.
- **Dynamic Safety Override System (`navigrid_safety`)**:
  - Standalone high-priority node enforcing $d_{safe} = k \cdot v^2 + d_{min}$.
  - Twist Priority Multiplexer arbitrating `/cmd_vel_safety` > `/cmd_vel_nav` $\to$ `/cmd_vel`.

---

## Workspace Structure

```
navigrid_ws/src/
├── navigrid_description/   # Robot URDF/xacro, sensors, RViz config
├── navigrid_gazebo/        # 30m x 30m Harmonic SDF world, models, actors
├── navigrid_bringup/       # Native C++ harmonic bridge, top-level launch files
├── navigrid_nav/           # Nav2 params, costmaps, planner tuning, SLAM config
└── navigrid_safety/        # High-priority safety override & twist multiplexer
```

---

## Installation & Build Instructions

### 1. Prerequisites
- ROS 2 (Humble / Jazzy)
- Gazebo Sim 8 (Harmonic)
- Standard build tools (`colcon`, `ament_cmake`, `python3-colcon-common-extensions`)

### 2. Clean Workspace Build
```bash
cd navigrid_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Simulation & Navigation Stack

### 1. Launch Complete Simulation Arena + Robot Stack
To launch Gazebo Harmonic with GUI:
```bash
source install/setup.bash
ros2 launch navigrid_bringup navigrid.launch.py
```

To launch headlessly (ideal for servers and CI):
```bash
source install/setup.bash
ros2 launch navigrid_bringup navigrid.launch.py headless:=true
```

### 2. Verify System Topics & Frequencies
In a separate terminal:
```bash
source install/setup.bash
ros2 topic list
ros2 topic hz /scan /imu /odom /tf /clock
```

Expected frequencies:
- `/scan`: ~20 Hz
- `/imu`: ~50 Hz
- `/odom`: ~50 Hz
- `/tf`: ~100 Hz
- `/clock`: ~500 Hz

---

## Sending Navigation Goals to Destination B

### Method A: Via ROS 2 Action Client (`NavigateToPose`)
To send the robot autonomously to Destination B:
```bash
source install/setup.bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 11.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### Method B: Via Goal Pose Topic (`/goal_pose`)
```bash
source install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 11.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

---

## Safety Override Demonstration

The safety node constantly monitors the forward stopping distance envelope:
$$d_{safe} = k \cdot v^2 + d_{min}$$

Defaults: $k = 0.5$, $d_{min} = 0.5\text{ m}$.
When an obstacle or dynamic actor breaches this distance, `/cmd_vel_safety` immediately forces a zero-velocity emergency stop via `twist_priority_mux`.

To test manual command vs. safety override:
```bash
# Send test velocity to Nav channel
ros2 topic pub -r 10 /cmd_vel_nav geometry_msgs/msg/Twist "{linear: {x: 0.6}}"

# Observe dynamic override log output when pedestrian or AMR approaches
```
