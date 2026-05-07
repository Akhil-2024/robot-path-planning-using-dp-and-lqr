# Launch Folder

The `launch/` folder contains ROS2 launch files used to start multiple nodes and simulation components together.

## Purpose

Launch files simplify execution by:
- starting Gazebo
- spawning the robot
- launching planner node
- launching controller node

using a single command.

---

## Example Launch Structure

```text
launch/
│
├── robot_sim.launch.py
└── planner_controller.launch.py
```

---

## Example Launch Command

```bash
ros2 launch robot_dp_lqr robot_sim.launch.py
```

---

## Benefits of Launch Files

- reduces terminal commands
- automates simulation startup
- improves project organization
- simplifies experimentation

---

## Typical Components Started

The launch file may include:
- Gazebo simulator
- robot entity spawning
- DP planner node
- LQR follower node
- visualization tools

---

## Future Improvement

Future versions can integrate:
- RViz visualization
- SLAM
- sensor plugins
- multi-robot launch support
