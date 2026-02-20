"""
2D Autonomous Mobile Robot Simulator — entry point.

Usage
-----
    pip install -r requirements.txt
    python main.py

Controls
--------
    Click anywhere in the plot  → set a new goal (A* replans automatically).
    "Randomize" button          → generate a new random world and goal, reset robot.
"""
import math

import matplotlib
# Select an interactive backend before importing pyplot.
# TkAgg ships with standard Python on most platforms.
# If you see a backend error, try 'Qt5Agg' or 'macosx' instead.
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

from controller import Controller
from planner import AStarPlanner
from robot import Robot
from sensors import LidarSensor
from world import World

# ─── Simulation parameters ────────────────────────────────────────────────────
DT = 0.05        # Fixed timestep (s)
GRID_RES = 0.2   # A* occupancy grid resolution (m)

# Robot always spawns here so it's easy to observe each new scenario
ROBOT_SPAWN = (1.0, 1.0, 0.3)   # x, y, theta

# ─── World & subsystem construction ───────────────────────────────────────────
world = World(width=10.0, height=10.0)

robot = Robot(
    x=ROBOT_SPAWN[0], y=ROBOT_SPAWN[1], theta=ROBOT_SPAWN[2],
    radius=0.2, max_v=1.5, max_omega=2.5,
)

lidar = LidarSensor(num_rays=36, fov=math.pi, max_range=3.0)

controller = Controller(
    kp_omega=3.5,
    kd_omega=2.6,       # ζ = kd/(2√kp) = 2.6/(2×√3.5) ≈ 0.70 → near-critically damped
    omega_smooth=0.25,  # EMA alpha for output omega; tune down for smoother, up for snappier
    max_v=1.0,
    goal_tolerance=0.2,
    waypoint_tolerance=0.35,
    avoid_dist=0.6,
    avoid_gain=1.5,
    num_rays=lidar.num_rays,
)

planner = AStarPlanner(resolution=GRID_RES)

# Occupancy grid — rebuilt whenever the world changes (randomize button)
occ_grid, origin_x, origin_y = world.build_occupancy_grid(GRID_RES, robot.radius + GRID_RES / 2)

# ─── Shared mutable state ─────────────────────────────────────────────────────
goal = [8.0, 8.0]
waypoints: list = []
lidar_ranges: np.ndarray = np.full(lidar.num_rays, lidar.max_range)
reached = False


def replan() -> None:
    """Run A* from the robot's current position to the goal and load waypoints."""
    global waypoints
    path = planner.plan(
        robot.x, robot.y,
        goal[0], goal[1],
        occ_grid, origin_x, origin_y,
    )
    waypoints = path if path is not None else []
    controller.set_waypoints(waypoints)


# ─── Matplotlib figure setup ──────────────────────────────────────────────────
BG_DARK  = "#1a1a2e"
BG_PANEL = "#16213e"
ACCENT   = "#0f3460"

fig, ax = plt.subplots(figsize=(8, 8.6))   # extra height for button row
fig.patch.set_facecolor(BG_PANEL)
fig.subplots_adjust(left=0.07, right=0.97, top=0.96, bottom=0.10)

ax.set_facecolor(BG_DARK)
ax.set_xlim(0, world.width)
ax.set_ylim(0, world.height)
ax.set_aspect("equal")
ax.set_title("2D Robot Simulator  |  Click to set goal", color="white", fontsize=12)
ax.tick_params(colors="gray")
for spine in ax.spines.values():
    spine.set_edgecolor("#444")

# ─── Obstacle rendering helpers ───────────────────────────────────────────────
obstacle_patches: list = []   # kept so we can remove them on randomize

def draw_obstacles() -> None:
    """Remove any existing obstacle patches and draw the current world.obstacles."""
    global obstacle_patches
    for p in obstacle_patches:
        p.remove()
    obstacle_patches.clear()
    for ox, oy, or_ in world.obstacles:
        p = plt.Circle((ox, oy), or_, color="#e94560", alpha=0.85, zorder=3)
        ax.add_patch(p)
        obstacle_patches.append(p)

draw_obstacles()

# ─── Dynamic artists (updated every frame) ────────────────────────────────────
(trail_line,) = ax.plot([], [], "-", color=ACCENT, lw=1.5, alpha=0.7, zorder=2)
(path_line,)  = ax.plot([], [], "--", color="#e2d96e", lw=1.2, alpha=0.8, zorder=4, label="A* path")

robot_patch = plt.Circle((robot.x, robot.y), robot.radius, color="#4cc9f0", zorder=5)
ax.add_patch(robot_patch)

(heading_line,) = ax.plot([], [], "-", color="white", lw=2, zorder=6)
(goal_marker,)  = ax.plot([goal[0]], [goal[1]], "*", color="#f8961e", ms=14, zorder=7, label="Goal")

lidar_ray_lines = [
    ax.plot([], [], "-", color="#a8dadc", lw=0.4, alpha=0.25, zorder=2)[0]
    for _ in range(lidar.num_rays)
]

status_text = ax.text(
    0.02, 0.97, "",
    transform=ax.transAxes,
    color="white", fontsize=8.5, va="top",
    fontfamily="monospace",
)

ax.legend(
    loc="lower right",
    facecolor=ACCENT, edgecolor="#444",
    labelcolor="white", fontsize=8,
)


# ─── Randomize world ──────────────────────────────────────────────────────────
def _random_world(rng: np.random.Generator) -> None:
    """
    Replace world.obstacles with a freshly sampled random layout and pick a
    random goal.  Obstacles are guaranteed to:
      - Not overlap each other (with a small gap between them).
      - Not cover the robot spawn point.
      - Not cover the chosen goal.
    The goal is guaranteed to:
      - Be reachable (not inside any obstacle after inflation).
      - Be at least 2 m from the robot spawn.
    """
    global occ_grid, origin_x, origin_y, goal, reached

    margin   = robot.radius + 0.3   # keep obstacles away from world edges
    n_obs    = int(rng.integers(5, 14))   # 5–13 obstacles

    new_obs = []
    for _ in range(n_obs * 40):        # allow plenty of placement attempts
        if len(new_obs) >= n_obs:
            break
        ox  = rng.uniform(margin + 0.5, world.width  - margin - 0.5)
        oy  = rng.uniform(margin + 0.5, world.height - margin - 0.5)
        or_ = rng.uniform(0.25, 0.75)

        # Keep a clear zone around the robot spawn
        if math.hypot(ox - ROBOT_SPAWN[0], oy - ROBOT_SPAWN[1]) < or_ + robot.radius + 1.0:
            continue

        # Don't overlap existing obstacles (require a gap of 0.15 m)
        overlaps = any(
            math.hypot(ox - ex, oy - ey) < or_ + er + 0.15
            for ex, ey, er in new_obs
        )
        if overlaps:
            continue

        new_obs.append((ox, oy, or_))

    world.obstacles = new_obs

    # Pick a random goal that is free and far enough from the spawn
    inflation = robot.radius + GRID_RES / 2 + 0.1   # same margin used by A*
    for _ in range(500):
        gx = rng.uniform(1.0, world.width  - 1.0)
        gy = rng.uniform(1.0, world.height - 1.0)
        if math.hypot(gx - ROBOT_SPAWN[0], gy - ROBOT_SPAWN[1]) < 2.0:
            continue
        if world.collides_with_obstacle(gx, gy, inflation):
            continue
        goal[0], goal[1] = gx, gy
        break

    # Rebuild occupancy grid for the new obstacle layout
    occ_grid, origin_x, origin_y = world.build_occupancy_grid(
        GRID_RES, robot.radius + GRID_RES / 2
    )

    # Reset robot to its fixed spawn
    robot.x, robot.y, robot.theta = ROBOT_SPAWN
    robot.v, robot.omega = 0.0, 0.0
    robot.trail_x, robot.trail_y = [robot.x], [robot.y]

    # Reset controller internal state so old filtered values don't bleed in
    controller._prev_heading_error = 0.0
    controller._omega_prev         = 0.0
    controller._avoid_omega_prev   = 0.0

    reached = False


def on_randomize(_event) -> None:
    """Button callback: generate a new random world, goal, and replan."""
    rng = np.random.default_rng()
    _random_world(rng)

    # Redraw obstacles (remove old patches, add new ones)
    draw_obstacles()
    goal_marker.set_data([goal[0]], [goal[1]])

    replan()

    # Force a full canvas redraw so the blit background picks up the new
    # obstacle patches before the animation resumes blitting over it.
    fig.canvas.draw()


# ─── Randomize button ─────────────────────────────────────────────────────────
# Place in the bottom-centre of the figure (figure-fraction coordinates)
btn_ax = fig.add_axes([0.35, 0.01, 0.30, 0.055])
btn_ax.set_navigate(False)
randomize_btn = Button(btn_ax, "⟳  Randomize World", color=ACCENT, hovercolor="#e94560")
randomize_btn.label.set_color("white")
randomize_btn.label.set_fontsize(10)
randomize_btn.on_clicked(on_randomize)


# ─── Mouse callback ────────────────────────────────────────────────────────────
def on_click(event) -> None:
    """Set a new goal position on mouse click and trigger A* replan."""
    global reached
    if event.inaxes is not ax or event.xdata is None:
        return
    gx, gy = event.xdata, event.ydata
    if world.collides_with_obstacle(gx, gy, robot.radius):
        return
    goal[0], goal[1] = gx, gy
    goal_marker.set_data([gx], [gy])
    reached = False
    replan()


fig.canvas.mpl_connect("button_press_event", on_click)

# ─── Initial plan ──────────────────────────────────────────────────────────────
replan()


# ─── Animation step ────────────────────────────────────────────────────────────
def sim_step(_frame):
    global reached, lidar_ranges

    if not reached:
        # 1. Sense
        lidar_ranges = lidar.scan(
            robot.x, robot.y, robot.theta,
            world.obstacles, world.width, world.height,
        )

        # 2. Control
        v, omega, reached = controller.compute(
            robot.x, robot.y, robot.theta,
            goal[0], goal[1],
            lidar_ranges, DT,
        )
        robot.set_velocity(v, omega)

        # 3. Integrate kinematics
        robot.update(DT)

        # 4. Hard-clamp position to world bounds
        robot.x = clamp(robot.x, robot.radius, world.width  - robot.radius)
        robot.y = clamp(robot.y, robot.radius, world.height - robot.radius)

    # ── Update visual artists ──────────────────────────────────────────────────
    robot_patch.center = (robot.x, robot.y)

    hl = robot.radius * 1.7
    heading_line.set_data(
        [robot.x, robot.x + hl * math.cos(robot.theta)],
        [robot.y, robot.y + hl * math.sin(robot.theta)],
    )

    trail_line.set_data(robot.trail_x, robot.trail_y)

    if waypoints:
        px, py = zip(*waypoints)
        path_line.set_data(px, py)
    else:
        path_line.set_data([], [])

    for ray_line, offset, rng in zip(lidar_ray_lines, lidar.ray_angles, lidar_ranges):
        angle = robot.theta + offset
        ex = robot.x + rng * math.cos(angle)
        ey = robot.y + rng * math.sin(angle)
        ray_line.set_data([robot.x, ex], [robot.y, ey])

    dist_g = math.hypot(goal[0] - robot.x, goal[1] - robot.y)
    wp_str = f"WP {controller._wp_idx}/{len(waypoints)}" if waypoints else "direct"
    status_text.set_text(
        f"x={robot.x:5.2f}  y={robot.y:5.2f}  θ={math.degrees(robot.theta):+6.1f}°\n"
        f"v={robot.v:4.2f} m/s   ω={robot.omega:+5.2f} rad/s\n"
        f"dist_goal={dist_g:.2f} m   {wp_str}\n"
        + ("★ GOAL REACHED ★" if reached else "")
    )

    return (
        robot_patch, heading_line, trail_line, path_line,
        goal_marker, status_text,
        *lidar_ray_lines,
    )


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


# ─── Run ──────────────────────────────────────────────────────────────────────
anim = FuncAnimation(
    fig, sim_step,
    interval=int(DT * 1000),
    blit=True,
    cache_frame_data=False,
)

plt.show()
