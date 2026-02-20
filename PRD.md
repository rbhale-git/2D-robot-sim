# PRD: 2D Autonomous Mobile Robot Simulator

## Overview

Build a top-down 2D simulation where a circular differential-drive robot navigates to a goal while avoiding obstacles. The project focuses on robotics fundamentals (kinematics, control, simple sensing) before introducing heavier stacks like ROS. It should serve as a clean, beginner-friendly learning tool.

---

## Tech Stack & Constraints

- **Language:** Python 3.10+
- **Dependencies:** `numpy` and `matplotlib` only — no ROS, no pygame
- **Run command:** `pip install -r requirements.txt` then `python main.py`
- **Code style:** Readable, well-commented, with docstrings and type hints

---

## Project Structure

```
robot-sim/
├── main.py            # Entry point: sets up world, robot, planner, loop, visualization
├── robot.py           # Robot class: state, integration, geometry
├── world.py           # World class: obstacles, boundaries, occupancy grid helper
├── controller.py      # PID + waypoint tracking + reactive obstacle avoidance
├── planner.py         # A* implementation, grid conversion utilities
├── sensors.py         # LIDAR simulation (ray casting)
├── utils.py           # wrap_to_pi, clamp, geometry helpers
├── requirements.txt
├── CLAUDE.md          # Claude Code project context
└── README.md          # Documentation and learning guide
```

---

## Features

### 1. Simulation Loop

- Fixed timestep integration (`dt = 0.05s`)
- Robot state: `x`, `y`, `theta` (heading), `v` (linear velocity), `omega` (angular velocity)
- Differential-drive kinematics:
  ```
  x_dot   = v * cos(theta)
  y_dot   = v * sin(theta)
  theta_dot = omega
  ```
- Clamp `v` and `omega` to configurable max limits

### 2. World + Obstacles

- 2D bounded world (e.g., 10m × 10m)
- Obstacles are circles defined by `(x, y, radius)`
- Collision detection: robot is a circle; collision if `distance(robot, obstacle) < r_robot + r_obstacle`
- Keep robot inside world bounds
- Provide a default set of obstacles so behavior is visible immediately on first run

### 3. Goal Setting + Visualization

- Matplotlib live animation / interactive plot
- Display elements:
  - Robot: circle with a heading line
  - Obstacles: filled circles
  - Goal: marker
  - Planned path (A* waypoints)
  - Robot trail (history of positions)
- **Mouse click** on the plot sets a new goal in real time
- Start with a default goal if the user doesn't click

### 4. Controller

- Heading controller that steers the robot toward the current target waypoint:
  ```
  heading_error = wrap_to_pi(desired_theta - theta)
  ```
- PD (or PID) controller for `omega`
- Linear speed `v`:
  - Reduces when heading error is large
  - Slows down near the goal (proportional to distance)
- Stop when within goal tolerance (e.g., `0.2m`)
- Comment recommended gain values and how to tune them

### 5. Simulated LIDAR

- Simple ray casting: `N` rays over a configurable FOV (e.g., 180°), max range `R`
- For each ray, compute distance to nearest obstacle via circle–ray intersection math
- Use LIDAR readings for reactive avoidance:
  - If obstacle too close in front sector → bias `omega` to steer away and/or reduce `v`
- Keep implementation simple and well-explained

### 6. Path Planning (A*)

- Grid-based A* planner
- Build an occupancy grid from the obstacle list (inflate obstacles by robot radius for safety)
- Plan from robot position to goal on the grid
- Return a list of waypoints in world coordinates
- Waypoint-following logic:
  - Advance to the next waypoint when the robot is close to the current one
  - Controller always tracks the current waypoint

---

## Module Responsibilities

| File | Responsibility |
|---|---|
| `main.py` | Wires everything together: creates world, robot, planner, controller, sensors; runs the simulation loop and matplotlib animation |
| `robot.py` | `Robot` class holding state (`x, y, theta, v, omega`), `update()` for kinematics integration, geometry (radius) |
| `world.py` | `World` class with obstacle list, boundary dimensions, collision checks, and occupancy grid generation |
| `controller.py` | PD/PID heading controller, speed modulation, waypoint tracking, reactive avoidance layer using LIDAR data |
| `planner.py` | A* search on an occupancy grid, grid ↔ world coordinate conversion, path smoothing (optional) |
| `sensors.py` | `LidarSensor` class: ray casting against circular obstacles, returns array of range measurements |
| `utils.py` | Helper functions: `wrap_to_pi()`, `clamp()`, `distance()`, geometry utilities |

---

## Quality & Behavior Expectations

- The robot should **smoothly drive to the goal** in open space (no oscillation)
- With obstacles, the robot should **plan around them** via A* waypoints
- LIDAR provides **last-moment reactive safety** if the robot gets too close to an obstacle
- Reasonable default gains and saturations — no unstable oscillations out of the box
- Code should be **concise but correct**; prioritize clarity over cleverness

---

## Implementation Phases

Build and verify incrementally:

| Phase | Focus | What to test |
|---|---|---|
| **Phase 1** | Kinematics only | Robot drives forward in a straight line, state updates correctly |
| **Phase 2** | PID goal seeking | Robot turns toward and drives to a goal point, stops on arrival |
| **Phase 3** | LIDAR + reactive avoidance | Robot detects obstacles and steers around them |
| **Phase 4** | A* path planning | Robot plans a collision-free path and follows waypoints to the goal |

---

## Verification Checklist

After implementation, confirm:

1. `python main.py` launches a matplotlib window showing the robot, obstacles, and goal
2. Clicking in the plot sets a new goal and the robot drives toward it
3. The robot stops within tolerance (~0.2m) of the goal
4. A* path is visible as a line/dots from robot to goal, routing around obstacles
5. If the robot approaches an obstacle, LIDAR-based avoidance kicks in and prevents collision

---

## README Requirements

The `README.md` must include:

- Project description (what it is and why it exists)
- Install and run instructions
- Controls (mouse click to set goal)
- Module overview table
- Learning path (Phase 1–4 as described above)
- 2–3 screenshot placeholders (markdown image tags with placeholder text)
- "Next Extensions" section: Kalman filter, MPC, multi-robot, ROS 2 port

---

## Future Extensions (Out of Scope for MVP)

- Extended Kalman Filter for state estimation with noisy sensors
- Model Predictive Control (MPC) replacing PID
- Multi-robot coordination
- ROS 2 port with `nav2` integration
- Dynamic / moving obstacles
- More sophisticated path planners (RRT, RRT*)
