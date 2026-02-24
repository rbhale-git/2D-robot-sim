---
title: 2D Robot Simulator Demo
sdk: gradio
app_file: app.py
pinned: false
---

# 2D Autonomous Mobile Robot Simulator

A top-down 2D simulator where a circular differential-drive robot navigates to a goal while avoiding obstacles.

This project is built as a beginner-friendly robotics learning tool, covering:
- Differential-drive kinematics
- PD heading control with speed modulation
- Simulated LIDAR (ray casting)
- A* path planning on an inflated occupancy grid

## Live Demo

Hugging Face Space:
- https://huggingface.co/spaces/rbhale/2D-robot-sim

## Quick Start

Requirements:
- Python 3.10+

Install:

```bash
pip install -r requirements.txt
```

Run desktop simulator (interactive Matplotlib window):

```bash
python main.py
```

Run web demo locally (Gradio):

```bash
python app.py
```

## Controls

Desktop mode (`main.py`):

| Action | Effect |
|---|---|
| Left-click inside plot | Set a new goal and trigger A* replan |
| Randomize World button | New obstacle map, robot reset, random valid goal |

Notes:
- Clicks inside obstacles are ignored.
- Robot is hard-clamped to world bounds.

Web demo mode (`app.py`):
- Configure goal, world layout, seed, max steps, and frame stride.
- Click `Run Simulation` to generate a replay GIF and run summary.

## Project Structure

| File | Responsibility |
|---|---|
| `main.py` | Desktop entrypoint with interactive animation and click-to-goal controls |
| `app.py` | Web demo entrypoint for Hugging Face Spaces using Gradio |
| `robot.py` | Robot state, velocity saturation, Euler integration, and trail history |
| `world.py` | World bounds, circular obstacles, collision checks, occupancy grid generation |
| `controller.py` | PD heading control, speed modulation, waypoint tracking, reactive LIDAR avoidance |
| `planner.py` | Grid-based A* planner, world/grid conversion, path downsampling |
| `sensors.py` | LIDAR ray casting against obstacles and walls |
| `utils.py` | Geometry and math helpers (`wrap_to_pi`, `clamp`, etc.) |

## Simulation Pipeline

Each control cycle runs in this order:
1. Sense: LIDAR scan from current robot pose
2. Plan: A* path from robot to goal on inflated occupancy grid
3. Control: PD heading + speed modulation + reactive avoidance overlay
4. Integrate: Euler update of robot kinematics
5. Render: robot, path, goal, rays, and telemetry

## Key Parameters

| Parameter | Typical Range | Effect |
|---|---|---|
| `kp_omega` | 2.0 to 5.0 | Higher turns faster, too high can oscillate |
| `kd_omega` | 0.1 to 3.0 | Damps heading oscillation |
| `max_v` | 0.5 to 1.5 m/s | Linear speed cap |
| `avoid_dist` | 0.4 to 0.8 m | Distance threshold for avoidance |
| `avoid_gain` | 0.5 to 2.0 | Avoidance steering strength |

## Hugging Face Deployment

This repository is already configured for Spaces via README front matter:
- `sdk: gradio`
- `app_file: app.py`

To deploy updates:

```bash
git add .
git commit -m "Update simulator"
git push hf HEAD:main
```

If `hf` remote is not set:

```bash
git remote add hf https://huggingface.co/spaces/rbhale/2D-robot-sim
```

## Troubleshooting

- `No module named gradio`:
  - Run `pip install -r requirements.txt`
- Matplotlib backend error in desktop mode:
  - Change backend in `main.py` from `TkAgg` to an available backend
- Push rejected on Spaces:
  - `git fetch hf main` then retry push
- No A* path found:
  - Goal may be blocked after inflation; set another goal or randomize world

## Learning Path

1. Kinematics (`robot.py`)
2. PD goal seeking (`controller.py`)
3. LIDAR reactive avoidance (`sensors.py`)
4. A* planning and waypoint following (`planner.py`)

## Screenshots

![Simulator overview](docs/screenshot_overview.png)
Robot navigating around obstacles to reach the goal.

![A* path](docs/screenshot_astar.png)
A* planned path shown as a dashed line.

![LIDAR rays](docs/screenshot_lidar.png)
LIDAR beams radiating from the robot.

## Next Extensions

- Extended Kalman Filter for state estimation under noise
- Model Predictive Control (MPC)
- Multi-robot coordination
- ROS 2 port (`rclpy`, `nav2`)
- Dynamic obstacles
- Advanced planners such as RRT or Theta*
