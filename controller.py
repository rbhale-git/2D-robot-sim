"""
Controller: PD heading control, speed modulation, waypoint tracking,
and reactive LIDAR-based obstacle avoidance.

Tuning guide
------------
kp_omega  : Proportional heading gain.  Increase → faster turns; too high → oscillation.
            Typical range: 2–5.
kd_omega  : Derivative heading gain.  Damps oscillation.  Typical range: 0.1–0.6.
max_v     : Linear speed cap (m/s).  Lower = safer; higher = faster travel.
avoid_dist: LIDAR range threshold that triggers reactive avoidance (m).
            ~0.4–0.8 works well for the default world.
avoid_gain: Strength of the avoidance steering term.  Typical: 0.5–2.0.
"""
import math
from typing import List, Optional, Tuple

import numpy as np

from utils import clamp, wrap_to_pi


class Controller:
    """Controls a differential-drive robot to follow waypoints with LIDAR avoidance.

    Pipeline (each timestep)
    ------------------------
    1. Check goal tolerance → stop if reached.
    2. Advance waypoint index if close enough to the current waypoint.
    3. PD controller for heading error toward the active target.
    4. Modulate linear speed by heading error and distance to target.
    5. Overlay reactive avoidance from LIDAR readings.
    """

    def __init__(
        self,
        kp_omega: float = 3.5,
        kd_omega: float = 0.4,
        max_v: float = 1.0,
        goal_tolerance: float = 0.2,
        waypoint_tolerance: float = 0.35,
        avoid_dist: float = 0.6,
        avoid_gain: float = 1.5,
        num_rays: int = 36,
    ) -> None:
        self.kp_omega = kp_omega
        self.kd_omega = kd_omega
        self.max_v = max_v
        self.goal_tolerance = goal_tolerance
        self.waypoint_tolerance = waypoint_tolerance
        self.avoid_dist = avoid_dist
        self.avoid_gain = avoid_gain
        self.num_rays = num_rays

        self._prev_heading_error: float = 0.0
        self._waypoints: List[Tuple[float, float]] = []
        self._wp_idx: int = 0

    # ------------------------------------------------------------------
    # Waypoint management
    # ------------------------------------------------------------------

    def set_waypoints(self, waypoints: List[Tuple[float, float]]) -> None:
        """Replace the current waypoint list and reset the index."""
        self._waypoints = waypoints
        self._wp_idx = 0
        self._prev_heading_error = 0.0

    @property
    def has_waypoints(self) -> bool:
        return bool(self._waypoints) and self._wp_idx < len(self._waypoints)

    @property
    def current_waypoint(self) -> Optional[Tuple[float, float]]:
        return self._waypoints[self._wp_idx] if self.has_waypoints else None

    # ------------------------------------------------------------------
    # Main compute method
    # ------------------------------------------------------------------

    def compute(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        goal_x: float,
        goal_y: float,
        lidar_ranges: np.ndarray,
        dt: float,
    ) -> Tuple[float, float, bool]:
        """Compute velocity commands for the current timestep.

        Args:
            robot_x, robot_y: Current robot position (m).
            robot_theta:      Current robot heading (rad).
            goal_x, goal_y:   Global goal position (m).
            lidar_ranges:     Array of LIDAR range measurements (m).
            dt:               Timestep duration (s).

        Returns:
            v       : Linear velocity command (m/s).
            omega   : Angular velocity command (rad/s).
            reached : True when the robot is within goal_tolerance of the goal.
        """
        # 1. Goal check
        dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
        if dist_to_goal < self.goal_tolerance:
            self._prev_heading_error = 0.0
            return 0.0, 0.0, True

        # 2. Determine active target
        if self.has_waypoints:
            wp = self.current_waypoint
            if math.hypot(wp[0] - robot_x, wp[1] - robot_y) < self.waypoint_tolerance:
                self._wp_idx += 1
            if self.has_waypoints:
                target_x, target_y = self._waypoints[self._wp_idx]
            else:
                target_x, target_y = goal_x, goal_y
        else:
            target_x, target_y = goal_x, goal_y

        dist_to_target = math.hypot(target_x - robot_x, target_y - robot_y)

        # 3. PD heading controller
        desired_theta = math.atan2(target_y - robot_y, target_x - robot_x)
        heading_error = wrap_to_pi(desired_theta - robot_theta)
        d_err = (heading_error - self._prev_heading_error) / max(dt, 1e-6)
        self._prev_heading_error = heading_error

        omega = self.kp_omega * heading_error + self.kd_omega * d_err
        omega = clamp(omega, -3.0, 3.0)

        # 4. Speed modulation
        #    - cos factor: drive slowly when pointing the wrong way (avoids drifting sideways)
        #    - dist factor: slow down as robot approaches the target
        heading_factor = max(0.0, math.cos(heading_error))
        dist_factor = clamp(dist_to_target / 1.0, 0.1, 1.0)
        v = self.max_v * heading_factor * dist_factor

        # 5. Reactive LIDAR avoidance overlay
        v, omega = self._reactive_avoidance(v, omega, lidar_ranges)

        return v, omega, False

    # ------------------------------------------------------------------
    # Reactive avoidance
    # ------------------------------------------------------------------

    def _reactive_avoidance(
        self,
        v: float,
        omega: float,
        ranges: np.ndarray,
    ) -> Tuple[float, float]:
        """Bias velocity commands when obstacles are detected in the front sector.

        Strategy
        --------
        - Inspect the central ±30° "front sector" of the LIDAR sweep.
        - If any ray reads below avoid_dist → reduce speed proportionally.
        - Compute a left/right threat imbalance across all rays and add a
          corrective angular term (steer away from the more threatened side).

        Args:
            v:      Nominal linear velocity (m/s).
            omega:  Nominal angular velocity (rad/s).
            ranges: LIDAR range array.

        Returns:
            Adjusted (v, omega).
        """
        if ranges is None or len(ranges) == 0:
            return v, omega

        n = len(ranges)

        # Front sector: middle third of the ray array (~±30° for 180° FOV)
        f_start = n // 3
        f_end = 2 * n // 3
        min_front = float(np.min(ranges[f_start:f_end]))

        if min_front < self.avoid_dist:
            # Scale speed down to zero as obstacle closes in
            speed_scale = clamp(min_front / self.avoid_dist, 0.0, 1.0)
            v *= speed_scale

            # Threat = sum of inverse distances (closer → higher threat)
            left_threat = float(np.sum(1.0 / (ranges[: n // 2] + 0.1)))
            right_threat = float(np.sum(1.0 / (ranges[n // 2 :] + 0.1)))

            # Positive omega = CCW (turn left); steer away from the more-threatened side
            avoid_omega = self.avoid_gain * (right_threat - left_threat)
            omega = clamp(omega + avoid_omega, -3.0, 3.0)

        return v, omega
