# PRD: 2D Autonomous Mobile Robot Simulator

## Overview

Build a top-down 2D simulation where a circular differential-drive robot navigates to a goal while avoiding obstacles.

The product has two runtime modes:
- Desktop interactive simulator (`main.py`, Matplotlib UI)
- Shareable web demo (`app.py`, Gradio, Hugging Face Spaces)

The project is intended as a clear learning tool for core robotics concepts before moving to larger stacks.

## Tech Stack and Constraints

- Language: Python 3.10+
- Core simulation dependencies: `numpy`, `matplotlib`
- Web demo dependencies: `gradio`, `Pillow`
- No ROS, no pygame
- Code style: readable, documented, typed where useful

## Project Structure

```text
2D-robot-sim/
|- main.py           # Desktop interactive simulator
|- app.py            # Gradio web demo entrypoint
|- robot.py          # Robot state and kinematic integration
|- world.py          # Obstacles, bounds, collision, occupancy grid
|- controller.py     # PD control, waypoint logic, reactive avoidance
|- planner.py        # A* planner and coordinate conversion
|- sensors.py        # LIDAR ray casting
|- utils.py          # Math and geometry helpers
|- requirements.txt
|- README.md
|- CLAUDE.md
```

## Functional Requirements

### 1. Simulation Loop

- Fixed timestep integration (`dt = 0.05 s`)
- Robot state: `x`, `y`, `theta`, `v`, `omega`
- Differential-drive kinematics:
  - `x_dot = v * cos(theta)`
  - `y_dot = v * sin(theta)`
  - `theta_dot = omega`
- Velocity commands must be clamped to configured limits

### 2. World and Obstacles

- 2D bounded world (default `10 m x 10 m`)
- Obstacles represented as circles `(x, y, radius)`
- Collision when robot circle overlaps obstacle circle
- Robot must stay within world boundaries

### 3. Goal and Planning

- User can set a goal in desktop mode via mouse click
- Occupancy grid built with obstacle inflation by robot radius
- A* plans from current robot position to goal
- Controller tracks path as waypoints

### 4. Controller

- Heading error to active target:
  - `wrap_to_pi(desired_theta - theta)`
- PD angular controller for `omega`
- Linear speed modulation by heading alignment and target distance
- Stop when within goal tolerance

### 5. LIDAR and Reactive Avoidance

- Configurable number of rays, FOV, and max range
- Ray intersections against circles and world walls
- Front-sector proximity reduces speed and biases steering away from threat

### 6. Visualization

Desktop mode should render:
- Robot body and heading
- Obstacles
- Goal marker
- Planned path
- Robot trail
- LIDAR rays
- Basic telemetry text

Web demo should provide:
- Input controls (goal, layout, seed, step budget)
- Generated replay GIF
- Text summary of run outcome

## Quality Expectations

- Smooth open-space goal seeking without unstable oscillation
- Obstacle-aware pathing using A*
- Reactive avoidance near hazards
- Reasonable default tuning out of the box
- Clear and maintainable module boundaries

## Verification Checklist

Desktop:
1. `python main.py` launches window with robot, obstacles, and goal
2. Click sets a goal and triggers replanning
3. Robot reaches goal within tolerance in feasible scenarios

Web demo:
1. `python app.py` launches Gradio app
2. Running simulation produces GIF and summary
3. Random world mode produces reproducible scenarios with seed

Deployment:
1. Hugging Face Space builds from `main` branch
2. Space serves `app.py` via `sdk: gradio`

## Out of Scope (Current)

- State estimation (EKF/UKF)
- MPC
- Multi-robot collaboration
- Dynamic obstacle prediction
- ROS 2 integration
