# ros_pid_ga_tuner — GA-Tuned PID Control for a Differential-Drive Robot in ROS

This package demonstrates the navigation of a **differential-drive robot** using a **PID controller** whose parameters are automatically optimized via a **Genetic Algorithm (GA)**. The objective is to combine classical control with intelligent optimization to improve goal-reaching performance in simulation.

---

## Overview

`ros_pid_ga_tuner` provides a complete simulated pipeline for goal-oriented navigation of a differential-drive robot running on **ROS Noetic**. Key components:

- Custom URDF robot model (differential-drive chassis with colored wheels and caster)
- PID controller for linear and angular velocity control
- Genetic Algorithm tuner to optimize PID gains (Kp, Ki, Kd) for both linear and angular controllers
- TF broadcasting of robot pose
- Trajectory publishing as `/robot_path` for RViz visualization
- ROS launch file with optional goal input parameters (`goal_x`, `goal_y`)

This repository is suitable for education, demonstrations, assignments, and as a foundation for further research.

---

## Features / Highlights

- **GA-tuned PID control**: automatically search for good PID gains to minimize trajectory error and settling time.
- **Real-time TF pose broadcasting** for frame transformations and visualization.
- **Path visualization** in RViz via `/robot_path` topic.
- **Modular Python node** (pid_ga.py) that is easy to extend and tune.
- **Launch-time goal parameters** to run experiments with different target positions.

---

## Repository structure

```text
ros_pid_ga_tuner/
│
├─ package.xml
├─ CMakeLists.txt
├─ README.md
├─ .gitignore
│
├─ launch/
│  └─ pid_ga_robot.launch
│
├─ urdf/
│  └─ differential_robot.urdf
│
├─ src/
│  └─ pid_ga.py
│
├─ rviz/
│  └─ pid_ga.rviz
│
└─ docs/
   └─ screenshots/
       └─ robot_path_screenshot.png
```

---

## Quick start (run demo)

1. Make sure you have a Catkin workspace and place the package in `src/`.

2. Build:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. Make the Python node executable:

```bash
chmod +x src/ros_pid_ga_tuner/src/robot_pid_ga.py
```

4. Launch the simulation with default goal:

```bash
roslaunch ros_pid_ga_tuner pid_ga_robot.launch
```

5. Launch with custom goal:

```bash
roslaunch ros_pid_ga_tuner pid_ga_robot.launch goal_x:=3.0 goal_y:=2.0
```

6. Open RViz (if not auto-started) and set Fixed Frame = `world`. Visualize `/robot_path`, TF frames, and the robot model.

---

## Nodes (brief)

### `robot_pid_ga.py`  
Main Python node implementing:
- PID controllers for linear and angular motion  
- Genetic Algorithm optimizer to tune PID gains  
- TF broadcasting of the robot pose  
- Publishing of `/robot_path` (nav_msgs/Path) for visualization  

---

## GA & PID configuration (examples)

- **Goal position**: `goal_x`, `goal_y` (launch params)
- **GA parameters**: population size, number of generations, mutation rate  
- **PID limits**: `v_max` (max linear velocity), `w_max` (max angular velocity)

Tune these for better stability and performance.

---

## Messages / Topics

### Publishes:
- `/cmd_vel` — robot control output 
- `/robot_path` — trajectory for RViz 
- TF: `world` → `body` 

### Subscribes:
- `/odom` (optional if using ideal internal simulation)

No custom messages or services required.

---

## Troubleshooting & Notes

- If robot oscillates or overshoots: 
  tune GA population/generations, adjust PID limits, or add anti-windup.
- If GA converges poorly: 
  increase mutation rate, population, or modify fitness function.
- Uses ideal odometry (no sensor noise).

---

## Suggested further work

- Integrate real robot odometry & sensors 
- Add obstacle avoidance (local planner) 
- Extend GA to optimize over multiple goals 
- Visualize PID error and GA fitness over generations 
