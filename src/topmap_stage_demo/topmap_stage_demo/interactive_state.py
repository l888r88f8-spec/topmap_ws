#!/usr/bin/env python3
"""Pure planning-request state for the interactive demo.

The ROS bridge never publishes the result of an older click.  Every endpoint
change or invalid click advances ``revision``; a service response is accepted
only when its captured revision is still current.  A stale failure is therefore
ignored, while a failure for the newest revision clears the previous successful
path and is explained through the status marker and the ROS log.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

DEFAULT_START: Tuple[float, float, float] = (2.0, 3.5, 0.0)
DEFAULT_GOAL: Tuple[float, float, float] = (28.4, 3.5, 2.0)


def _point3(value: Sequence[float], label: str) -> Tuple[float, float, float]:
    if len(value) != 3 or not all(
        isinstance(item, (int, float)) and math.isfinite(float(item)) for item in value
    ):
        raise ValueError("%s must be a finite xyz" % label)
    return (float(value[0]), float(value[1]), float(value[2]))


def _valid_path(points: Sequence[Sequence[float]]) -> List[Tuple[float, float, float]]:
    path = [_point3(point, "path point") for point in points]
    if len(path) < 2:
        raise ValueError("planner path must contain at least two points")
    return path


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Unit planar quaternion (x, y, z, w) for a finite RViz yaw."""
    if not isinstance(yaw, (int, float)) or not math.isfinite(float(yaw)):
        raise ValueError("yaw must be finite")
    half = float(yaw) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def sanitize_planar_orientation(quaternion) -> float:
    """Accept a ROS Quaternion or xyzw components and return its planar yaw."""
    if all(hasattr(quaternion, key) for key in ("x", "y", "z", "w")):
        quaternion = tuple(getattr(quaternion, key) for key in ("x", "y", "z", "w"))
    if len(quaternion) != 4 or not all(
        isinstance(item, (int, float)) and math.isfinite(float(item)) for item in quaternion
    ):
        raise ValueError("orientation must be a finite quaternion")
    x, y, z, w = (float(item) for item in quaternion)
    norm = math.hypot(x, y, z, w)
    if norm <= 1e-9:
        raise ValueError("orientation quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    if not math.isfinite(yaw):
        raise ValueError("orientation yields a non-finite yaw")
    return yaw


@dataclass
class ReplanCoordinator:
    start: Tuple[float, float, float] = DEFAULT_START
    goal: Tuple[float, float, float] = DEFAULT_GOAL
    start_yaw: float = 0.0
    revision: int = 0
    clear_serial: int = 0
    status: str = "waiting"
    status_text: str = "WAITING: planner service is not ready"
    last_path: Optional[List[Tuple[float, float, float]]] = None
    _label: str = field(default="", repr=False)

    def set_start(self, point: Sequence[float], yaw: float = 0.0) -> int:
        self.start = _point3(point, "start")
        if not isinstance(yaw, (int, float)) or not math.isfinite(float(yaw)):
            raise ValueError("start yaw must be finite")
        self.start_yaw = float(yaw)
        return self._changed("start changed")

    def set_goal(self, point: Sequence[float]) -> int:
        self.goal = _point3(point, "goal")
        return self._changed("goal changed")

    def _changed(self, reason: str) -> int:
        self.revision += 1
        self.clear_serial += 1
        self.last_path = None
        self.status = "pending"
        self.status_text = "PLANNING: %s (revision %d)" % (reason, self.revision)
        return self.revision

    def invalidate(self, reason: str) -> int:
        self.revision += 1
        self.clear_serial += 1
        self.last_path = None
        self.status = "invalid"
        self.status_text = "INVALID CLICK: %s (revision %d)" % (reason, self.revision)
        return self.revision

    def begin_plan(self, label: str = "") -> int:
        self.status = "pending"
        self.status_text = "PLANNING: %s (revision %d)" % (
            label or "endpoints", self.revision
        )
        return self.revision

    def accept_path(self, token: int, points: Sequence[Sequence[float]]) -> bool:
        if token != self.revision:
            return False
        self.last_path = _valid_path(points)
        self.status = "success"
        self.status_text = "SUCCESS: service path, %d points (revision %d)" % (
            len(self.last_path), self.revision
        )
        return True

    def fail(self, token: int, reason: str) -> bool:
        if token != self.revision:
            return False
        self.last_path = None
        self.clear_serial += 1
        self.status = "failed"
        self.status_text = "PLANNING FAILED: %s (revision %d)" % (reason, self.revision)
        return True

    def wait_for_service(self) -> None:
        if self.status != "pending":
            self.status = "waiting"
            self.status_text = "WAITING: planner service is not ready"
