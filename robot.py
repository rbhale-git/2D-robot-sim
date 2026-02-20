"""
Robot class: state, differential-drive kinematics, and geometry.
"""
import math
from utils import clamp


class Robot:
    """Circular differential-drive robot.

    State
    -----
    x, y    : position (m)
    theta   : heading (rad), wrapped to [-pi, pi]
    v       : linear velocity (m/s)
    omega   : angular velocity (rad/s)

    Kinematics (Euler integration)
    --------------------------------
    x_dot     = v * cos(theta)
    y_dot     = v * sin(theta)
    theta_dot = omega
    """

    def __init__(
        self,
        x: float = 1.0,
        y: float = 1.0,
        theta: float = 0.0,
        radius: float = 0.2,
        max_v: float = 1.5,
        max_omega: float = 2.5,
    ) -> None:
        """
        Args:
            x:         Initial x position (m).
            y:         Initial y position (m).
            theta:     Initial heading (rad).
            radius:    Robot body radius (m).
            max_v:     Maximum linear speed (m/s).
            max_omega: Maximum angular speed (rad/s).
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0

        self.radius = radius
        self.max_v = max_v
        self.max_omega = max_omega

        # Position history used to draw the robot trail
        self.trail_x: list[float] = [x]
        self.trail_y: list[float] = [y]

    def set_velocity(self, v: float, omega: float) -> None:
        """Set commanded velocities, clamped to configured limits.

        Args:
            v:     Linear velocity (m/s).
            omega: Angular velocity (rad/s).
        """
        self.v = clamp(v, -self.max_v, self.max_v)
        self.omega = clamp(omega, -self.max_omega, self.max_omega)

    def update(self, dt: float) -> None:
        """Integrate kinematics forward by one timestep using Euler's method.

        Args:
            dt: Timestep in seconds.
        """
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta = (self.theta + self.omega * dt + math.pi) % (2 * math.pi) - math.pi

        # Append to trail (cap length to avoid unbounded memory)
        self.trail_x.append(self.x)
        self.trail_y.append(self.y)
        if len(self.trail_x) > 600:
            self.trail_x = self.trail_x[-600:]
            self.trail_y = self.trail_y[-600:]
