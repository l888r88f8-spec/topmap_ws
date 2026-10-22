#!/usr/bin/env python3
"""Live ROS interaction check for the interactive planning demo.

Run the demo headless in one terminal:
    source /home/robot/topmap_ws/install/setup.bash
    ros2 launch topmap_stage_demo planning.launch.py rviz:=false

then in another:
    source /home/robot/topmap_ws/install/setup.bash
    python3 src/topmap_stage_demo/tools/verify_interactive.py

It publishes the same standard messages RViz's 2D Pose Estimate / 2D Goal Pose
tools publish and checks: default plan, upper-platform goal, reversed direction,
invalid outside click clearing the old path, and a fast goal change where only
the newest result may win.  It also compares the bridge's final path with the
planner_node's remapped service result topic, proving the bridge did not invent
or interpolate a path.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from topmap_msgs.srv import PlanPath
from visualization_msgs.msg import Marker, MarkerArray


def yaw_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def points_of(message: Path):
    return [
        (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        for pose in message.poses
    ]


def close(a, b, tolerance=2e-3) -> bool:
    return len(a) == len(b) and all(abs(x - y) <= tolerance for x, y in zip(a, b))


class InteractionCheck(Node):
    def __init__(self, timeout: float, service_name: str):
        super().__init__("topmap_interactive_check")
        self.timeout = timeout
        self.last_status = ""
        self.last_status_stamp = 0.0
        self.last_path = None
        self.path_history = []
        self.backend_path = None
        self.clear_seen = False

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        inputs = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.start_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/topmap_stage_demo/initialpose", inputs)
        self.goal_pub = self.create_publisher(PoseStamped, "/topmap_stage_demo/goal_pose", inputs)
        self.create_subscription(MarkerArray, "/topmap_stage_demo/planning_status",
                                 self._on_status, latched)
        self.create_subscription(Path, "/topmap_stage_demo/planned_path",
                                 self._on_path, latched)
        self.create_subscription(MarkerArray, "/topmap_stage_demo/final_path",
                                 self._on_path_markers, latched)
        self.create_subscription(Path, "/topmap_stage_demo/planner/final_path",
                                 self._on_backend_path, latched)
        self.client = self.create_client(PlanPath, service_name)

    def _on_status(self, message: MarkerArray) -> None:
        for marker in message.markers:
            if marker.type == Marker.TEXT_VIEW_FACING and marker.text:
                self.last_status = marker.text
                self.last_status_stamp = time.monotonic()

    def _on_path(self, message: Path) -> None:
        self.last_path = message
        points = points_of(message)
        if points:
            self.path_history.append(points)

    def _on_backend_path(self, message: Path) -> None:
        self.backend_path = message

    def _on_path_markers(self, message: MarkerArray) -> None:
        if any(marker.action == Marker.DELETEALL for marker in message.markers):
            self.clear_seen = True

    def spin_until(self, predicate, description: str):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        raise AssertionError(
            "%s timed out after %.0fs; status=%r last_path=%r"
            % (description, self.timeout, self.last_status,
               points_of(self.last_path) if self.last_path else None))

    def wait_service(self) -> None:
        def ready():
            rclpy.spin_once(self, timeout_sec=0.1)
            return self.client.service_is_ready()
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if ready():
                print("[ok] service %s is ready" % self.client.srv_name)
                return
        raise AssertionError("service %s never became ready" % self.client.srv_name)

    def publish_start(self, x: float, y: float, yaw: float = 0.0) -> None:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        # Deliberately z=0, exactly like RViz's planar tool.
        message.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_quaternion(yaw)
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.06853891909122467
        self.start_pub.publish(message)
        print("[send] /topmap_stage_demo/initialpose (%.3f, %.3f, z=0) yaw=%.3f" % (x, y, yaw))

    def publish_goal(self, x: float, y: float) -> None:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = float(x)
        message.pose.position.y = float(y)
        message.pose.position.z = 0.0
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)
        print("[send] /topmap_stage_demo/goal_pose (%.3f, %.3f, z=0)" % (x, y))

    def wait_success(self, start, goal, description: str) -> None:
        def done():
            if not self.last_status.startswith("SUCCESS") or self.last_path is None:
                return False
            points = points_of(self.last_path)
            return (len(points) >= 2 and close(points[0], start, 5e-2)
                    and close(points[-1], goal, 5e-2))
        self.spin_until(done, description)
        points = points_of(self.last_path)
        print("[ok] %s: %d points, start=%s goal=%s status=%r"
              % (description, len(points), points[0], points[-1], self.last_status))

    def compare_backend_path(self, description: str) -> None:
        assert self.backend_path is not None, "%s: no backend path" % description
        bridge = points_of(self.last_path)
        backend = points_of(self.backend_path)
        assert len(bridge) == len(backend), (
            "%s: bridge points=%d backend points=%d" % (description, len(bridge), len(backend)))
        for index, (a, b) in enumerate(zip(bridge, backend)):
            assert close(a, b, 1e-6), (
                "%s: bridge path differs from service result at %d: %s vs %s"
                % (description, index, a, b))
        print("[ok] %s: bridge path exactly matches planner service result (%d points)"
              % (description, len(bridge)))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=150.0,
                        help="timeout per planning result (s)")
    parser.add_argument("--service", default="/topmap_stage_demo/plan_path")
    args = parser.parse_args(argv)

    rclpy.init()
    node = InteractionCheck(args.timeout, args.service)
    try:
        node.wait_service()
        print("[wait] default safe endpoints plan once after service readiness")
        node.wait_success((2.0, 3.5, 0.0), (28.4, 3.5, 2.0), "default plan")
        node.compare_backend_path("default plan")

        print("[case] upper-platform goal")
        node.publish_start(2.0, 3.5, 0.0)
        node.publish_goal(28.0, 4.0)
        node.wait_success((2.0, 3.5, 0.0), (28.0, 4.0, 2.0), "upper-platform goal")
        node.compare_backend_path("upper-platform goal")

        print("[case] reversed direction from the platform")
        node.publish_start(28.0, 4.0, math.pi)
        node.publish_goal(3.0, 3.5)
        node.wait_success((28.0, 4.0, 2.0), (3.0, 3.5, 0.0), "reversed direction")
        node.compare_backend_path("reversed direction")

        print("[case] invalid click outside the mesh must clear the old path")
        node.publish_goal(100.0, 100.0)
        node.spin_until(
            lambda: "INVALID" in node.last_status and node.last_path is not None
            and len(node.last_path.poses) == 0,
            "invalid outside click clear")
        assert node.clear_seen, "final_path did not receive a DELETEALL clear marker"
        print("[ok] invalid outside click: status=%r, path cleared" % node.last_status)

        print("[case] rapid goal change: stale result must not win")
        node.publish_goal(3.5, 3.5)  # recover a valid path
        node.wait_success((28.0, 4.0, 2.0), (3.5, 3.5, 0.0), "recovery after invalid")
        node.path_history = []
        node.publish_start(2.0, 3.5, 0.0)
        node.publish_goal(28.0, 3.0)    # A, immediately superseded
        node.publish_goal(28.0, 4.5)    # B, newest
        node.wait_success((2.0, 3.5, 0.0), (28.0, 4.5, 2.0), "newest rapid goal")
        # Any non-empty path delivered from the rapid-change window must belong
        # to the newest goal, never the superseded one.
        for index, points in enumerate(node.path_history):
            if close(points[0], (2.0, 3.5, 0.0), 5e-2):
                assert not close(points[-1], (28.0, 3.0, 2.0), 5e-2), (
                    "superseded rapid-change goal A was published at index %d: %s"
                    % (index, points[-1]))
        print("[ok] rapid goal change: newest goal only; stale results discarded")
        node.compare_backend_path("newest rapid goal")

        print("PASS: interactive clicks, surface Z, reverse, invalid clear, stale discard")
        return 0
    except AssertionError as error:
        print("FAIL: %s" % error, file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
