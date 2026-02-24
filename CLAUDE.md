# Claude Code - Project Context

## Project

2D Autonomous Mobile Robot Simulator.

A top-down Python simulation for learning:
- Differential-drive kinematics
- PD heading control
- LIDAR ray casting
- A* grid path planning

This repo supports both desktop interaction and web demo sharing on Hugging Face Spaces.

## Stack

- Python 3.10+
- Core simulation: `numpy`, `matplotlib`
- Web demo: `gradio`, `Pillow`

## Run Commands

Desktop:

```bash
pip install -r requirements.txt
python main.py
```

Web demo:

```bash
python app.py
```

## Module Layout

| File | Role |
|---|---|
| `main.py` | Desktop entrypoint, animation loop, mouse goal setting, randomize button |
| `app.py` | Gradio web app that runs simulation rollouts and emits replay GIFs |
| `robot.py` | Robot state (`x, y, theta, v, omega`), Euler integration, trail |
| `world.py` | Obstacles, bounds checks, collision checks, occupancy grid |
| `controller.py` | PD heading control, speed modulation, waypoint tracking, LIDAR avoidance |
| `planner.py` | A* planner on occupancy grid, coordinate transforms, path simplification |
| `sensors.py` | LIDAR simulation via ray casting against circles and walls |
| `utils.py` | Utility math helpers (`wrap_to_pi`, `clamp`, `distance`, `angle_to`) |

## Key Design Decisions

- Fixed timestep (`dt = 0.05 s`) for predictable learning behavior.
- Occupancy-grid obstacle inflation for safer path planning clearance.
- LIDAR avoidance is an overlay on top of waypoint tracking.
- Desktop mode emphasizes interactivity; web mode emphasizes shareable demos.

## Controller Tuning Reference

| Parameter | Effect | Typical range |
|---|---|---|
| `kp_omega` | Heading proportional gain | 2.0 to 5.0 |
| `kd_omega` | Heading derivative damping | 0.1 to 3.0 |
| `max_v` | Linear speed cap (m/s) | 0.5 to 1.5 |
| `avoid_dist` | Avoidance trigger distance (m) | 0.4 to 0.8 |
| `avoid_gain` | Avoidance steering strength | 0.5 to 2.0 |

## Learning Phases

1. Kinematics only
2. PD goal seeking
3. LIDAR reactive avoidance
4. A* waypoint planning

## Deployment Notes

- Hugging Face Space: `https://huggingface.co/spaces/rbhale/2D-robot-sim`
- README front matter controls deployment entrypoint:
  - `sdk: gradio`
  - `app_file: app.py`
