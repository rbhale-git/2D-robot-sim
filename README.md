---
title: 2D Robot Simulator Demo
sdk: gradio
app_file: app.py
pinned: false
---

# 2D Autonomous Mobile Robot Simulator

A top-down 2D simulation of a circular differential-drive robot navigating to a
user-selected goal while avoiding obstacles.  Built with **Python**, **NumPy**,
and **Matplotlib** only — no ROS, no pygame.

Designed as a **beginner-friendly learning tool** for core robotics concepts:

- Differential-drive kinematics
- PD heading control with speed modulation
- Simulated LIDAR (ray casting) and reactive obstacle avoidance
- A\* grid path planning with occupancy-grid inflation

---

## Install & Run

```bash
pip install -r requirements.txt
python main.py
```

A Matplotlib window opens automatically.  **Click anywhere in the plot** to set
a new goal — the robot plans a path and drives there.

> **Backend note:** `main.py` uses the `TkAgg` backend (ships with standard
> Python).  If you see a backend error, edit the `matplotlib.use(...)` line in
> `main.py` to `"Qt5Agg"` or another backend available in your environment.

### Hugging Face Spaces (demo sharing)

This repo now includes a web demo entrypoint:

```bash
python app.py
```

For Spaces deployment:

1. Create a new Hugging Face Space (SDK: `Gradio`).
2. Push this repository to the Space repo.
3. Spaces will install `requirements.txt` and launch `app.py`.

---

## Controls

| Action | Effect |
|--------|--------|
| **Left-click** in the plot | Set a new goal; triggers A\* replan |

Clicks inside an obstacle are silently ignored.

---

## What You'll See

| Element | Description |
|---------|-------------|
| Blue circle with white line | Robot body and heading indicator |
| Red circles | Static circular obstacles |
| Orange star | Current goal |
| Yellow dashed line | A\* planned path (waypoints) |
| Dark blue trail | History of robot positions |
| Cyan rays | Simulated LIDAR beams |
| HUD (top-left) | Live pose, velocity, and status |

---

## Module Overview

| File | Responsibility |
|------|----------------|
| `main.py` | Entry point: constructs all subsystems, runs the animation loop, handles mouse input |
| `robot.py` | `Robot` class: state `(x, y, θ, v, ω)`, Euler kinematic integration, position trail |
| `world.py` | `World` class: obstacle list, boundary checks, collision detection, occupancy grid |
| `controller.py` | PD heading controller, speed modulation, waypoint tracking, LIDAR avoidance overlay |
| `planner.py` | A\* search on occupancy grid, grid ↔ world conversion, path downsampling |
| `sensors.py` | `LidarSensor`: ray casting against circular obstacles and world walls |
| `utils.py` | `wrap_to_pi`, `clamp`, `distance`, `angle_to` |

---

## Learning Path

The simulator is designed to be understood **incrementally**:

| Phase | Focus | What to observe |
|-------|-------|-----------------|
| **1 — Kinematics** | `robot.py` + `utils.py` | Robot drives straight; position integrates correctly |
| **2 — PD Control** | `controller.py` (no waypoints) | Robot steers toward goal, slows near it, stops on arrival |
| **3 — LIDAR Avoidance** | `sensors.py` + `controller._reactive_avoidance` | Robot detects nearby obstacles and steers away reactively |
| **4 — A\* Planning** | `planner.py` + full `main.py` | Robot follows a planned, collision-free path around obstacles |

---

## Screenshots

![Simulator overview](docs/screenshot_overview.png)
*Robot navigating around obstacles to reach the goal*

![A* path](docs/screenshot_astar.png)
*A\* planned path shown as a yellow dashed line*

![LIDAR rays](docs/screenshot_lidar.png)
*LIDAR beams radiating from the robot*

---

## Next Extensions

Ideas for going beyond the MVP:

- **Extended Kalman Filter** — state estimation with noisy sensor readings
- **Model Predictive Control (MPC)** — replace PD with horizon-based optimisation
- **Multi-robot coordination** — multiple robots that avoid each other
- **ROS 2 port** — wrap modules in `rclpy` nodes and integrate with `nav2`
- **Dynamic obstacles** — moving objects the robot must react to in real time
- **Advanced planners** — RRT, RRT\*, or Theta\* for smoother, faster paths
