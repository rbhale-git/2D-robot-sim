# Claude Code — Project Context

## Project
2D Autonomous Mobile Robot Simulator — a top-down Python simulation for learning
robotics fundamentals: differential-drive kinematics, PD heading control, LIDAR
ray casting, and A* grid path planning.

## Stack
- **Python 3.10+**
- **Dependencies:** `numpy`, `matplotlib` only (no ROS, no pygame)

## Run
```bash
pip install -r requirements.txt
python main.py
```

## Module layout

| File            | Role                                                              |
|-----------------|-------------------------------------------------------------------|
| `main.py`       | Entry point — wires subsystems, runs matplotlib animation loop    |
| `robot.py`      | `Robot` class: state (x, y, θ, v, ω), Euler kinematics, trail    |
| `world.py`      | `World` class: obstacles, bounds, collision, occupancy grid       |
| `controller.py` | PD heading control, speed modulation, waypoint tracking, avoidance|
| `planner.py`    | A* on occupancy grid, grid ↔ world conversion, path downsampling  |
| `sensors.py`    | `LidarSensor`: ray casting vs circles and walls                   |
| `utils.py`      | `wrap_to_pi`, `clamp`, `distance`, `angle_to`                     |

## Key design decisions
- Fixed `dt = 0.05 s` Euler integration (simple and predictable for learning).
- Obstacles inflated by robot radius when building the occupancy grid — A* paths
  keep the robot centre safely away from obstacle surfaces.
- LIDAR reactive avoidance is an **overlay** on top of A* waypoint following:
  A* handles global routing; LIDAR provides last-moment safety corrections.
- Mouse-click event sets a new goal and triggers a full A* replan.

## Controller tuning (quick reference)
| Parameter    | Effect                                           | Typical range |
|--------------|--------------------------------------------------|---------------|
| `kp_omega`   | Proportional heading gain (turn speed)           | 2 – 5         |
| `kd_omega`   | Derivative heading gain (damps oscillation)      | 0.1 – 0.6     |
| `max_v`      | Linear speed cap (m/s)                           | 0.5 – 1.5     |
| `avoid_dist` | LIDAR avoidance trigger distance (m)             | 0.4 – 0.8     |
| `avoid_gain` | Avoidance steering strength                      | 0.5 – 2.0     |

## Phase progression (for learning)
| Phase | Focus                    |
|-------|--------------------------|
| 1     | Kinematics only          |
| 2     | PD goal seeking          |
| 3     | LIDAR reactive avoidance |
| 4     | A* path planning (full)  |
