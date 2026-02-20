"""
Simulated LIDAR sensor using ray casting against circular obstacles and world walls.
"""
import math
from typing import List, Tuple

import numpy as np


class LidarSensor:
    """Simulated 2-D LIDAR.

    Casts ``num_rays`` evenly-spaced rays over a configurable field of view
    (centred on the robot heading) and returns the range to the nearest surface
    for each ray.  Rays hit circular obstacles or world-boundary walls,
    whichever is closer.

    Ray angle convention
    --------------------
    ray_angles[0] = -fov/2  (leftmost ray, relative to robot heading)
    ray_angles[-1] = +fov/2 (rightmost ray)
    """

    def __init__(
        self,
        num_rays: int = 36,
        fov: float = math.pi,       # 180 ° field of view
        max_range: float = 3.0,
        noise_std: float = 0.0,     # Set > 0 for realistic Gaussian noise
    ) -> None:
        """
        Args:
            num_rays:  Number of LIDAR rays.
            fov:       Total field of view in radians (centred on heading).
            max_range: Maximum sensing range (m).
            noise_std: Std-dev of additive Gaussian range noise (m); 0 = noiseless.
        """
        self.num_rays = num_rays
        self.fov = fov
        self.max_range = max_range
        self.noise_std = noise_std

        # Angular offsets of each ray relative to the robot heading
        self.ray_angles = np.linspace(-fov / 2.0, fov / 2.0, num_rays)

    # ------------------------------------------------------------------
    # Internal geometry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _ray_circle_distance(
        rx: float, ry: float,   # ray origin
        dx: float, dy: float,   # unit direction
        cx: float, cy: float,   # circle centre
        cr: float,              # circle radius
    ) -> float:
        """Distance along a ray to the first intersection with a circle, or inf.

        Solves the quadratic |O + t*D - C|² = r² for the smallest positive t.
        """
        fx = rx - cx
        fy = ry - cy

        # a = |D|² = 1 for a unit-direction vector, but keep it generic
        a = dx * dx + dy * dy
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - cr * cr

        discriminant = b * b - 4.0 * a * c
        if discriminant < 0.0:
            return math.inf   # Ray misses the circle

        sqrt_d = math.sqrt(discriminant)
        t1 = (-b - sqrt_d) / (2.0 * a)
        t2 = (-b + sqrt_d) / (2.0 * a)

        # Return the smallest positive t (the entry point)
        if t1 > 1e-9:
            return t1
        if t2 > 1e-9:
            return t2
        return math.inf   # Intersection is behind the ray origin

    @staticmethod
    def _ray_wall_distance(
        rx: float, ry: float,
        angle: float,
        width: float, height: float,
    ) -> float:
        """Minimum distance from (rx, ry) along `angle` to any world boundary."""
        dx = math.cos(angle)
        dy = math.sin(angle)
        candidates: list[float] = []

        if dx > 1e-9:                    # right wall  x = width
            candidates.append((width - rx) / dx)
        elif dx < -1e-9:                 # left wall   x = 0
            candidates.append(-rx / dx)
        if dy > 1e-9:                    # top wall    y = height
            candidates.append((height - ry) / dy)
        elif dy < -1e-9:                 # bottom wall y = 0
            candidates.append(-ry / dy)

        valid = [d for d in candidates if d > 1e-9]
        return min(valid) if valid else math.inf

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def scan(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        obstacles: List[Tuple[float, float, float]],
        world_width: float,
        world_height: float,
    ) -> np.ndarray:
        """Perform one LIDAR scan from the robot's current pose.

        Args:
            robot_x, robot_y: Robot centre position (m).
            robot_theta:      Robot heading (rad).
            obstacles:        List of (cx, cy, radius) circular obstacles.
            world_width:      World width (m).
            world_height:     World height (m).

        Returns:
            ranges: Float array of length ``num_rays`` with range measurements (m).
                    Values are capped at ``max_range``.
        """
        ranges = np.empty(self.num_rays)

        for i, offset in enumerate(self.ray_angles):
            angle = robot_theta + offset
            dx = math.cos(angle)
            dy = math.sin(angle)

            # Start with the wall distance as the worst-case range
            min_dist = self._ray_wall_distance(
                robot_x, robot_y, angle, world_width, world_height
            )

            # Check every obstacle
            for cx, cy, cr in obstacles:
                d = self._ray_circle_distance(robot_x, robot_y, dx, dy, cx, cy, cr)
                if d < min_dist:
                    min_dist = d

            ranges[i] = min(min_dist, self.max_range)

        # Optional Gaussian noise
        if self.noise_std > 0.0:
            ranges += np.random.normal(0.0, self.noise_std, self.num_rays)
            np.clip(ranges, 0.0, self.max_range, out=ranges)

        return ranges
