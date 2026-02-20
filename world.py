"""
World class: boundaries, circular obstacles, collision detection, and occupancy grid.
"""
import numpy as np
from typing import List, Optional, Tuple


# Type alias for a circular obstacle: (centre_x, centre_y, radius)
Obstacle = Tuple[float, float, float]

# Default obstacle layout — spread across the map so the robot must navigate around them
DEFAULT_OBSTACLES: List[Obstacle] = [
    (3.0, 3.0, 0.6),
    (5.0, 2.0, 0.5),
    (6.5, 5.0, 0.7),
    (3.5, 7.0, 0.6),
    (7.0, 8.0, 0.5),
    (2.0, 5.5, 0.5),
    (8.0, 3.0, 0.4),
    (5.0, 6.5, 0.5),
]


class World:
    """2D bounded world containing circular obstacles.

    Coordinate system: x to the right, y upward, origin at bottom-left.
    """

    def __init__(
        self,
        width: float = 10.0,
        height: float = 10.0,
        obstacles: Optional[List[Obstacle]] = None,
    ) -> None:
        """
        Args:
            width:     World width in metres.
            height:    World height in metres.
            obstacles: List of (cx, cy, radius) circular obstacles.
                       Defaults to a built-in layout if None.
        """
        self.width = width
        self.height = height
        self.obstacles: List[Obstacle] = obstacles if obstacles is not None else DEFAULT_OBSTACLES

    # ------------------------------------------------------------------
    # Collision / bounds helpers
    # ------------------------------------------------------------------

    def in_bounds(self, x: float, y: float, margin: float = 0.0) -> bool:
        """Return True if (x, y) is inside the world with an optional margin."""
        return margin <= x <= self.width - margin and margin <= y <= self.height - margin

    def collides_with_obstacle(self, x: float, y: float, robot_radius: float) -> bool:
        """Return True if a circle at (x, y) with robot_radius overlaps any obstacle.

        Collision condition: distance(centres) < r_robot + r_obstacle
        """
        for ox, oy, or_ in self.obstacles:
            if (x - ox) ** 2 + (y - oy) ** 2 < (robot_radius + or_) ** 2:
                return True
        return False

    def is_valid_position(self, x: float, y: float, robot_radius: float) -> bool:
        """Return True when the robot can occupy (x, y) without collision or boundary violation."""
        return self.in_bounds(x, y, robot_radius) and not self.collides_with_obstacle(
            x, y, robot_radius
        )

    # ------------------------------------------------------------------
    # Occupancy grid for A*
    # ------------------------------------------------------------------

    def build_occupancy_grid(
        self,
        resolution: float,
        robot_radius: float,
    ) -> Tuple[np.ndarray, float, float]:
        """Build a binary occupancy grid with obstacles inflated by robot_radius.

        Inflation ensures A* plans a path that keeps the robot centre at a safe
        distance from every obstacle surface.

        Args:
            resolution:   Grid cell size in metres.
            robot_radius: Robot radius used for obstacle inflation (m).

        Returns:
            grid:     2-D uint8 array; 0 = free, 1 = occupied / collision zone.
            origin_x: World x coordinate of grid column 0.
            origin_y: World y coordinate of grid row 0.
        """
        cols = int(self.width / resolution) + 1
        rows = int(self.height / resolution) + 1
        grid = np.zeros((rows, cols), dtype=np.uint8)

        for r in range(rows):
            for c in range(cols):
                wx = c * resolution
                wy = r * resolution
                if not self.is_valid_position(wx, wy, robot_radius):
                    grid[r, c] = 1

        return grid, 0.0, 0.0
