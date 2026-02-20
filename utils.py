"""
Utility functions for the 2D robot simulator.
"""
import math


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to the range [-pi, pi].

    Args:
        angle: Angle in radians (any value).

    Returns:
        Equivalent angle in [-pi, pi].
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp a value to [min_val, max_val].

    Args:
        value: Input value.
        min_val: Lower bound.
        max_val: Upper bound.

    Returns:
        Value clamped to [min_val, max_val].
    """
    return max(min_val, min(max_val, value))


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance between two 2D points."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def angle_to(x1: float, y1: float, x2: float, y2: float) -> float:
    """Bearing angle (radians) from point (x1, y1) to point (x2, y2)."""
    return math.atan2(y2 - y1, x2 - x1)
