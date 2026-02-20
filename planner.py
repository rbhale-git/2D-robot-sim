"""
A* path planner on a 2-D occupancy grid.

Grid coordinates   → world coordinates:   wx = col * resolution + origin_x
World coordinates  → grid coordinates:    col = int((wx - origin_x) / resolution)

The planner returns a simplified list of (x, y) waypoints in world space.
Downsampling to ~30 waypoints keeps the controller from being given
unnecessarily dense intermediate targets.
"""
import heapq
import math
from typing import Dict, List, Optional, Tuple

import numpy as np


class AStarPlanner:
    """Grid-based A* path planner with 8-connected neighbour search."""

    # 8-connected move table: (Δrow, Δcol, cost multiplier)
    _MOVES: List[Tuple[int, int, float]] = [
        (-1,  0, 1.0),
        ( 1,  0, 1.0),
        ( 0, -1, 1.0),
        ( 0,  1, 1.0),
        (-1, -1, math.sqrt(2)),
        (-1,  1, math.sqrt(2)),
        ( 1, -1, math.sqrt(2)),
        ( 1,  1, math.sqrt(2)),
    ]

    def __init__(self, resolution: float = 0.2) -> None:
        """
        Args:
            resolution: Grid cell size in metres.  Smaller = finer paths but slower planning.
        """
        self.resolution = resolution

    # ------------------------------------------------------------------
    # Coordinate conversion
    # ------------------------------------------------------------------

    def _to_grid(
        self, wx: float, wy: float, origin_x: float, origin_y: float
    ) -> Tuple[int, int]:
        col = int((wx - origin_x) / self.resolution)
        row = int((wy - origin_y) / self.resolution)
        return row, col

    def _to_world(
        self, row: int, col: int, origin_x: float, origin_y: float
    ) -> Tuple[float, float]:
        wx = col * self.resolution + origin_x
        wy = row * self.resolution + origin_y
        return wx, wy

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(
        self,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float,
        grid: np.ndarray,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
    ) -> Optional[List[Tuple[float, float]]]:
        """Run A* from (start_x, start_y) to (goal_x, goal_y).

        Args:
            start_x, start_y: Start world position (m).
            goal_x, goal_y:   Goal world position (m).
            grid:             Occupancy grid — 0 free, 1 occupied.
            origin_x, origin_y: World coords of grid cell (0, 0).

        Returns:
            List of (x, y) waypoints from start to goal in world coordinates,
            or None if no path exists.
        """
        rows, cols = grid.shape

        sr, sc = self._to_grid(start_x, start_y, origin_x, origin_y)
        gr, gc = self._to_grid(goal_x, goal_y, origin_x, origin_y)

        # Clamp to grid bounds
        sr = max(0, min(rows - 1, sr))
        sc = max(0, min(cols - 1, sc))
        gr = max(0, min(rows - 1, gr))
        gc = max(0, min(cols - 1, gc))

        # If start or goal land in a blocked cell, nudge to nearest free cell
        if grid[gr, gc] == 1:
            result = self._nearest_free(grid, gr, gc)
            if result is None:
                return None
            gr, gc = result

        if grid[sr, sc] == 1:
            result = self._nearest_free(grid, sr, sc)
            if result is None:
                return None
            sr, sc = result

        def heuristic(r: int, c: int) -> float:
            return math.hypot(r - gr, c - gc) * self.resolution

        # Min-heap: (f_score, g_score, row, col)
        open_heap: List[Tuple[float, float, int, int]] = [
            (heuristic(sr, sc), 0.0, sr, sc)
        ]
        g_score: Dict[Tuple[int, int], float] = {(sr, sc): 0.0}
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

        while open_heap:
            _, g, r, c = heapq.heappop(open_heap)

            # Reached the goal
            if (r, c) == (gr, gc):
                return self._reconstruct(came_from, gr, gc, origin_x, origin_y)

            # Skip stale heap entries
            if g > g_score.get((r, c), math.inf) + 1e-9:
                continue

            for dr, dc, step_cost in self._MOVES:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                if grid[nr, nc] == 1:
                    continue

                new_g = g_score[(r, c)] + step_cost * self.resolution
                if new_g < g_score.get((nr, nc), math.inf):
                    g_score[(nr, nc)] = new_g
                    came_from[(nr, nc)] = (r, c)
                    f = new_g + heuristic(nr, nc)
                    heapq.heappush(open_heap, (f, new_g, nr, nc))

        return None  # No path found

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _nearest_free(
        self, grid: np.ndarray, r: int, c: int
    ) -> Optional[Tuple[int, int]]:
        """BFS from (r, c) to find the nearest free cell."""
        rows, cols = grid.shape
        visited = {(r, c)}
        queue = [(r, c)]
        while queue:
            cr, cc = queue.pop(0)
            if grid[cr, cc] == 0:
                return cr, cc
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = cr + dr, cc + dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append((nr, nc))
        return None

    def _reconstruct(
        self,
        came_from: Dict[Tuple[int, int], Tuple[int, int]],
        goal_r: int,
        goal_c: int,
        origin_x: float,
        origin_y: float,
    ) -> List[Tuple[float, float]]:
        """Trace came_from back to the start and return world-coord waypoints."""
        path: List[Tuple[float, float]] = []
        node = (goal_r, goal_c)
        while node in came_from:
            path.append(self._to_world(node[0], node[1], origin_x, origin_y))
            node = came_from[node]
        path.append(self._to_world(node[0], node[1], origin_x, origin_y))  # start
        path.reverse()

        # Downsample to ~30 waypoints so the controller isn't given a dense grid trace
        step = max(1, len(path) // 30)
        simplified = path[::step]
        if simplified[-1] != path[-1]:
            simplified.append(path[-1])  # always include the exact goal cell

        return simplified
