"""
Hugging Face Spaces demo app for the 2D robot simulator.

This keeps the core simulator logic in the existing modules and renders a
headless animated GIF for web sharing.
"""
from __future__ import annotations

import math
import tempfile
from typing import List, Tuple

import gradio as gr
import matplotlib

# Use a headless backend for cloud runtime environments.
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from controller import Controller
from planner import AStarPlanner
from robot import Robot
from sensors import LidarSensor
from utils import clamp
from world import World

DT = 0.05
GRID_RES = 0.2
ROBOT_SPAWN = (1.0, 1.0, 0.3)


def _random_obstacles(rng: np.random.Generator, world: World, robot: Robot) -> List[Tuple[float, float, float]]:
    """Generate a random non-overlapping obstacle layout."""
    margin = robot.radius + 0.3
    n_obs = int(rng.integers(5, 14))
    obstacles: List[Tuple[float, float, float]] = []

    for _ in range(n_obs * 40):
        if len(obstacles) >= n_obs:
            break

        ox = rng.uniform(margin + 0.5, world.width - margin - 0.5)
        oy = rng.uniform(margin + 0.5, world.height - margin - 0.5)
        or_ = rng.uniform(0.25, 0.75)

        if math.hypot(ox - ROBOT_SPAWN[0], oy - ROBOT_SPAWN[1]) < or_ + robot.radius + 1.0:
            continue

        overlaps = any(math.hypot(ox - ex, oy - ey) < or_ + er + 0.15 for ex, ey, er in obstacles)
        if overlaps:
            continue

        obstacles.append((ox, oy, or_))

    return obstacles


def _create_scene(world: World, robot: Robot, goal: Tuple[float, float], lidar: LidarSensor):
    """Build matplotlib artists and return figure + artist handles."""
    bg_dark = "#1a1a2e"
    bg_panel = "#16213e"
    accent = "#0f3460"

    fig, ax = plt.subplots(figsize=(5.2, 5.2), dpi=120)
    fig.patch.set_facecolor(bg_panel)
    ax.set_facecolor(bg_dark)
    ax.set_xlim(0, world.width)
    ax.set_ylim(0, world.height)
    ax.set_aspect("equal")
    ax.set_title("2D Robot Simulator Demo", color="white", fontsize=11)
    ax.tick_params(colors="gray")
    for spine in ax.spines.values():
        spine.set_edgecolor("#444")

    for ox, oy, or_ in world.obstacles:
        patch = plt.Circle((ox, oy), or_, color="#e94560", alpha=0.85, zorder=3)
        ax.add_patch(patch)

    (trail_line,) = ax.plot([], [], "-", color=accent, lw=1.4, alpha=0.75, zorder=2)
    (path_line,) = ax.plot([], [], "--", color="#e2d96e", lw=1.2, alpha=0.9, zorder=4)
    robot_patch = plt.Circle((robot.x, robot.y), robot.radius, color="#4cc9f0", zorder=5)
    ax.add_patch(robot_patch)
    (heading_line,) = ax.plot([], [], "-", color="white", lw=2, zorder=6)
    (goal_marker,) = ax.plot([goal[0]], [goal[1]], "*", color="#f8961e", ms=14, zorder=7)

    lidar_lines = [
        ax.plot([], [], "-", color="#a8dadc", lw=0.4, alpha=0.30, zorder=2)[0]
        for _ in range(lidar.num_rays)
    ]
    status_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        color="white",
        fontsize=8.0,
        va="top",
        fontfamily="monospace",
    )

    artists = {
        "trail_line": trail_line,
        "path_line": path_line,
        "robot_patch": robot_patch,
        "heading_line": heading_line,
        "goal_marker": goal_marker,
        "lidar_lines": lidar_lines,
        "status_text": status_text,
    }
    return fig, artists


def _render_frame(
    fig,
    artists,
    robot: Robot,
    goal: Tuple[float, float],
    waypoints: List[Tuple[float, float]],
    lidar: LidarSensor,
    lidar_ranges: np.ndarray,
    reached: bool,
    step_idx: int,
) -> Image.Image:
    """Render one frame and return it as a PIL image."""
    artists["robot_patch"].center = (robot.x, robot.y)

    hl = robot.radius * 1.7
    artists["heading_line"].set_data(
        [robot.x, robot.x + hl * math.cos(robot.theta)],
        [robot.y, robot.y + hl * math.sin(robot.theta)],
    )
    artists["trail_line"].set_data(robot.trail_x, robot.trail_y)

    if waypoints:
        px, py = zip(*waypoints)
        artists["path_line"].set_data(px, py)
    else:
        artists["path_line"].set_data([], [])

    for ray_line, offset, rng in zip(artists["lidar_lines"], lidar.ray_angles, lidar_ranges):
        angle = robot.theta + offset
        ex = robot.x + rng * math.cos(angle)
        ey = robot.y + rng * math.sin(angle)
        ray_line.set_data([robot.x, ex], [robot.y, ey])

    dist_goal = math.hypot(goal[0] - robot.x, goal[1] - robot.y)
    wp_text = f"waypoints={len(waypoints)}" if waypoints else "direct"
    artists["status_text"].set_text(
        f"step={step_idx:4d}\n"
        f"x={robot.x:5.2f} y={robot.y:5.2f} th={math.degrees(robot.theta):+6.1f}\n"
        f"v={robot.v:4.2f} w={robot.omega:+5.2f}\n"
        f"dist_goal={dist_goal:.2f} {wp_text}\n"
        + ("GOAL REACHED" if reached else "")
    )

    fig.canvas.draw()
    rgba = np.asarray(fig.canvas.buffer_rgba(), dtype=np.uint8)
    return Image.fromarray(rgba[:, :, :3], mode="RGB")


def run_simulation(
    goal_x: float,
    goal_y: float,
    world_layout: str,
    seed: float,
    max_steps: int,
    frame_stride: int,
):
    """Run the simulator and return an animated GIF path and summary text."""
    seed_int = int(seed) if seed is not None else 7
    rng = np.random.default_rng(seed_int)

    world = World(width=10.0, height=10.0)
    robot = Robot(
        x=ROBOT_SPAWN[0],
        y=ROBOT_SPAWN[1],
        theta=ROBOT_SPAWN[2],
        radius=0.2,
        max_v=1.5,
        max_omega=2.5,
    )
    lidar = LidarSensor(num_rays=36, fov=math.pi, max_range=3.0)
    controller = Controller(
        kp_omega=3.5,
        kd_omega=2.6,
        omega_smooth=0.25,
        max_v=1.0,
        goal_tolerance=0.2,
        waypoint_tolerance=0.35,
        avoid_dist=0.6,
        avoid_gain=1.5,
        num_rays=lidar.num_rays,
    )
    planner = AStarPlanner(resolution=GRID_RES)

    if world_layout == "Random":
        world.obstacles = _random_obstacles(rng, world, robot)

    goal = (
        clamp(float(goal_x), 0.5, world.width - 0.5),
        clamp(float(goal_y), 0.5, world.height - 0.5),
    )
    occ_grid, origin_x, origin_y = world.build_occupancy_grid(GRID_RES, robot.radius + GRID_RES / 2)
    waypoints = planner.plan(robot.x, robot.y, goal[0], goal[1], occ_grid, origin_x, origin_y) or []
    controller.set_waypoints(waypoints)

    control_goal = waypoints[-1] if waypoints else goal
    lidar_ranges = lidar.scan(robot.x, robot.y, robot.theta, world.obstacles, world.width, world.height)
    reached = False
    collided = False
    steps_taken = 0

    fig, artists = _create_scene(world, robot, control_goal, lidar)
    frames: List[Image.Image] = []
    effective_stride = max(int(frame_stride), max(1, int(max_steps) // 90))
    frames.append(
        _render_frame(fig, artists, robot, control_goal, waypoints, lidar, lidar_ranges, reached, 0)
    )

    for step in range(1, int(max_steps) + 1):
        lidar_ranges = lidar.scan(robot.x, robot.y, robot.theta, world.obstacles, world.width, world.height)
        v, omega, reached = controller.compute(
            robot.x,
            robot.y,
            robot.theta,
            control_goal[0],
            control_goal[1],
            lidar_ranges,
            DT,
        )
        robot.set_velocity(v, omega)
        robot.update(DT)
        robot.x = clamp(robot.x, robot.radius, world.width - robot.radius)
        robot.y = clamp(robot.y, robot.radius, world.height - robot.radius)

        if world.collides_with_obstacle(robot.x, robot.y, robot.radius * 0.98):
            collided = True

        steps_taken = step
        if step % effective_stride == 0 or reached or collided or step == int(max_steps):
            frames.append(
                _render_frame(
                    fig, artists, robot, control_goal, waypoints, lidar, lidar_ranges, reached, step
                )
            )

        if reached or collided:
            break

    plt.close(fig)

    tmp = tempfile.NamedTemporaryFile(prefix="robot_sim_", suffix=".gif", delete=False)
    tmp.close()
    duration_ms = max(20, int(1000 * DT * effective_stride))
    frames[0].save(
        tmp.name,
        save_all=True,
        append_images=frames[1:],
        duration=duration_ms,
        loop=0,
        optimize=True,
    )

    status = (
        f"World: {world_layout}\n"
        f"Seed: {seed_int}\n"
        f"Goal: ({goal[0]:.2f}, {goal[1]:.2f})\n"
        f"Path waypoints: {len(waypoints)}\n"
        f"Steps: {steps_taken}\n"
        f"Reached goal: {'yes' if reached else 'no'}\n"
        f"Collision detected: {'yes' if collided else 'no'}"
    )
    return tmp.name, status


def build_demo() -> gr.Blocks:
    """Build the Gradio UI."""
    with gr.Blocks(title="2D Robot Simulator Demo") as demo:
        gr.Markdown(
            """
            # 2D Robot Simulator Demo
            Run an autonomous robot simulation and generate an animated replay.
            """
        )

        with gr.Row():
            with gr.Column(scale=1):
                world_layout = gr.Radio(
                    choices=["Default", "Random"],
                    value="Default",
                    label="World Layout",
                )
                goal_x = gr.Slider(0.5, 9.5, value=8.0, step=0.1, label="Goal X (m)")
                goal_y = gr.Slider(0.5, 9.5, value=8.0, step=0.1, label="Goal Y (m)")
                seed = gr.Number(value=7, precision=0, label="Random Seed")
                max_steps = gr.Slider(100, 900, value=360, step=20, label="Max Simulation Steps")
                frame_stride = gr.Slider(1, 8, value=3, step=1, label="Render Every N Steps")
                run_btn = gr.Button("Run Simulation", variant="primary")

            with gr.Column(scale=1):
                gif_output = gr.Image(type="filepath", label="Simulation Replay (GIF)")
                status_output = gr.Textbox(label="Run Summary", lines=7)

        run_btn.click(
            fn=run_simulation,
            inputs=[goal_x, goal_y, world_layout, seed, max_steps, frame_stride],
            outputs=[gif_output, status_output],
        )

        gr.Examples(
            examples=[
                [8.0, 8.0, "Default", 7, 360, 3],
                [7.4, 1.4, "Default", 7, 360, 3],
                [8.6, 8.1, "Random", 21, 500, 4],
            ],
            inputs=[goal_x, goal_y, world_layout, seed, max_steps, frame_stride],
        )

    return demo


if __name__ == "__main__":
    app = build_demo()
    app.launch()
