#!/usr/bin/env python3
"""Interactive RViz client for the real TopMap planning service.

The production ``topmap_ros planner_node`` loads ``stages.topmap.bin`` once and
exposes the real ``topmap::HierarchicalPlanner`` through the demo-isolated
``/topmap_stage_demo/plan_path`` service.  This node is only a bridge:

* subscribe to RViz's standard ``/topmap_stage_demo/initialpose`` and ``/topmap_stage_demo/goal_pose`` tools;
* resolve the 2D click on the actual ``stages.json`` mesh (XY containment and
  barycentric z, never the RViz z=0 and never a nearest-3D snap);
* send an asynchronous ``PlanPath`` request; stale responses are discarded;
* publish the service-returned path verbatim (Marker gets only a small
  render-only Z lift, ``nav_msgs/Path`` gets none);
* clear the old successful path and publish a visible status marker on invalid
  clicks or planner failure.

It never runs the fixed replay publisher, so an old path cannot overwrite a new
plan.  The static cloud/regions/chords are built once by the existing stage
publisher helpers.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Dict, List, Sequence, Tuple

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from . import publisher as pub
from .interactive_state import (
    DEFAULT_GOAL,
    DEFAULT_START,
    ReplanCoordinator,
    quaternion_from_yaw,
    sanitize_planar_orientation,
)
from .surface_projection import (
    FRAME_ID,
    ProjectionError,
    SurfaceMesh,
    load_surface_mesh,
    validate_frame,
)

TOPIC_INITIALPOSE = "/topmap_stage_demo/initialpose"
TOPIC_GOAL_POSE = "/topmap_stage_demo/goal_pose"
TOPIC_SERVICE = "/topmap_stage_demo/plan_path"
TOPIC_PLANNING_START = pub.TOPIC_PREFIX + "/planning_start"
TOPIC_PLANNING_GOAL = pub.TOPIC_PREFIX + "/planning_goal"
TOPIC_FINAL_PATH = pub.TOPIC_PREFIX + "/final_path"
TOPIC_PLANNED_PATH = pub.TOPIC_PREFIX + "/planned_path"
TOPIC_TOPOLOGY_SEQUENCE = pub.TOPIC_PREFIX + "/planned_topology_sequence"
TOPIC_STATUS = pub.TOPIC_PREFIX + "/planning_status"

# Rendering-only lifts; never applied to nav_msgs/Path or any stored geometry.
PATH_Z_LIFT = 0.005
ENDPOINT_Z_LIFT = 0.040
STATUS_Z = 0.1
STATUS_POSITION = (15.2, -1.5, STATUS_Z)

FINAL_PATH_RGB = (1.00, 0.55, 0.02)
START_RGB = (0.10, 1.00, 0.20)
GOAL_RGB = (1.00, 0.10, 0.10)
STATUS_RGB = {
    "success": (0.10, 1.00, 0.20),
    "pending": (1.00, 0.85, 0.00),
    "waiting": (1.00, 0.85, 0.00),
    "failed": (1.00, 0.10, 0.10),
    "invalid": (1.00, 0.10, 0.10),
}


def _finite3(point: Sequence[float]) -> bool:
    return len(point) == 3 and all(
        isinstance(value, (int, float)) and math.isfinite(float(value)) for value in point
    )


def _point(point: Sequence[float], dz: float = 0.0) -> Point:
    return Point(x=float(point[0]), y=float(point[1]), z=float(point[2]) + dz)


def _marker(ns: str, marker_id: int, marker_type: int, scale, rgb) -> Marker:
    return pub._marker(ns, marker_id, marker_type, scale, rgb)


def build_path_markers(points: Sequence[Sequence[float]],
                       frame_id: str = FRAME_ID) -> MarkerArray:
    """The exact service-returned polyline, rendered with a small Z lift."""
    if len(points) < 2 or any(not _finite3(point) for point in points):
        raise ValueError("service path must contain at least two finite xyz points")
    core = _marker("planning/final_path", 1, Marker.LINE_STRIP,
                   (0.16, 0.16, 0.16), FINAL_PATH_RGB)
    core.header.frame_id = frame_id
    for point in points:
        core.points.append(_point(point, PATH_Z_LIFT))
    return MarkerArray(markers=[core])


def build_cleared_path_markers(frame_id: str = FRAME_ID) -> MarkerArray:
    marker = _marker("planning/final_path_clear", 0, Marker.CUBE, (1.0, 1.0, 1.0),
                     FINAL_PATH_RGB)
    marker.header.frame_id = frame_id
    marker.action = Marker.DELETEALL
    return MarkerArray(markers=[marker])


def build_path_message(points: Sequence[Sequence[float]],
                       frame_id: str = FRAME_ID) -> Path:
    """nav_msgs/Path with the service coordinates, no render-only lift."""
    message = Path()
    message.header.frame_id = frame_id
    message.header.stamp = Time()
    for point in points:
        if not _finite3(point):
            raise ValueError("service path point is not a finite xyz")
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position = _point(point, 0.0)
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def build_endpoint_text(ns: str, marker_id: int, text: str, point: Sequence[float],
                        rgb) -> Marker:
    return pub._text_marker(ns, marker_id, text, _point(point, ENDPOINT_Z_LIFT + 0.28),
                            0.45, rgb)


def build_start_markers(start: Sequence[float], yaw: float) -> MarkerArray:
    if not _finite3(start):
        raise ValueError("start is not a finite xyz")
    sphere = _marker("planning/start", 0, Marker.SPHERE, (0.36, 0.36, 0.36), START_RGB)
    sphere.pose.position = _point(start, ENDPOINT_Z_LIFT)
    arrow = _marker("planning/start_yaw", 2, Marker.ARROW, (0.80, 0.16, 0.16), START_RGB)
    arrow.pose.position = _point(start, ENDPOINT_Z_LIFT)
    qx, qy, qz, qw = quaternion_from_yaw(float(yaw))
    arrow.pose.orientation.x = qx
    arrow.pose.orientation.y = qy
    arrow.pose.orientation.z = qz
    arrow.pose.orientation.w = qw
    label = build_endpoint_text("planning/start_label", 1, "START", start, START_RGB)
    return MarkerArray(markers=[sphere, arrow, label])


def build_goal_markers(goal: Sequence[float]) -> MarkerArray:
    if not _finite3(goal):
        raise ValueError("goal is not a finite xyz")
    sphere = _marker("planning/goal", 0, Marker.SPHERE, (0.36, 0.36, 0.36), GOAL_RGB)
    sphere.pose.position = _point(goal, ENDPOINT_Z_LIFT)
    label = build_endpoint_text("planning/goal_label", 1, "GOAL", goal, GOAL_RGB)
    return MarkerArray(markers=[sphere, label])


def build_status_marker(status: str, text: str,
                        frame_id: str = FRAME_ID) -> Marker:
    rgb = STATUS_RGB.get(status, (1.0, 0.85, 0.0))
    safe = str(text).encode("ascii", "replace").decode("ascii")
    marker = pub._text_marker("planning/status", 0, safe,
                              Point(x=STATUS_POSITION[0], y=STATUS_POSITION[1],
                                    z=STATUS_POSITION[2]),
                              0.62, rgb)
    marker.header.frame_id = frame_id
    return marker


def build_topology_sequence(topology_path, stages: dict,
                            frame_id: str = FRAME_ID) -> MarkerArray:
    """Optional region/portal chain from the service response (not the path)."""
    region_centroids = {
        region["id"]: region["centroid"] for region in stages["topology"]["regions"]
    }
    portal_centers = {
        portal["id"]: portal["center"] for portal in stages["topology"]["portals"]
    }
    chain = _marker("planning/topology_sequence", 0, Marker.LINE_STRIP,
                    (0.055, 0.055, 0.055), (0.10, 1.00, 0.20))
    chain.header.frame_id = frame_id
    region_ids = list(topology_path.region_ids)
    portal_ids = list(topology_path.portal_ids)
    for index, region_id in enumerate(region_ids):
        centroid = region_centroids.get(int(region_id))
        if centroid is not None:
            chain.points.append(_point(centroid, 0.045))
        if index < len(portal_ids):
            center = portal_centers.get(int(portal_ids[index]))
            if center is not None:
                chain.points.append(_point(center, 0.045))
    nodes = _marker("planning/topology_portals", 0, Marker.SPHERE_LIST,
                    (0.12, 0.12, 0.12), (1.00, 0.85, 0.00))
    nodes.header.frame_id = frame_id
    for portal_id in portal_ids:
        center = portal_centers.get(int(portal_id))
        if center is not None:
            nodes.points.append(_point(center, 0.045))
            nodes.colors.append(pub._rgba((1.00, 0.85, 0.00)))
    return MarkerArray(markers=[chain, nodes])


def _stamp_markers(array: MarkerArray, stamp) -> None:
    for marker in array.markers:
        marker.header.stamp = stamp


def _path_points(response) -> List[Tuple[float, float, float]]:
    points = []
    for pose in response.path.poses:
        position = pose.pose.position
        point = (float(position.x), float(position.y), float(position.z))
        if not _finite3(point):
            raise ValueError("planner service returned a non-finite path point")
        points.append(point)
    return points


class InteractivePlannerNode:
    """Thin rclpy node kept out of module import so the builders stay testable."""

    def __init__(self, stages_path: str, pcd_path: str, service_name: str,
                 auto_plan: bool = True) -> None:  # pragma: no cover - live ROS
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from topmap_msgs.srv import PlanPath

        self._rclpy = rclpy
        self._Node = Node
        self._PlanPath = PlanPath
        self._stages = pub.load_stages(stages_path)
        pub.validate_stages(self._stages)
        self._mesh: SurfaceMesh = load_surface_mesh(stages_path)
        self._cloud = pub.build_source_cloud(pcd_path)
        self._stage_regions = pub.build_stage3_regions(
            self._stages, z_lift=pub.PLANNING_FILL_Z)
        self._stage_chords = pub.build_stage4_portal_chords(self._stages)
        for topic, array in (
            (pub.TOPIC_STAGE3_REGIONS, self._stage_regions),
            (pub.TOPIC_STAGE4_CHORDS, self._stage_chords),
        ):
            pub.validate_marker_array(topic, array)

        self._node = Node("topmap_interactive_planner")
        self._coord = ReplanCoordinator()
        self._auto_plan = bool(auto_plan)
        self._replan_needed = self._auto_plan
        self._inflight_token = None
        self._inflight_started = 0.0
        self._last_requested_revision = -1
        self._service_logged = False

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
        self._publishers = {
            "start": self._node.create_publisher(MarkerArray, TOPIC_PLANNING_START, latched),
            "goal": self._node.create_publisher(MarkerArray, TOPIC_PLANNING_GOAL, latched),
            "path": self._node.create_publisher(MarkerArray, TOPIC_FINAL_PATH, latched),
            "nav_path": self._node.create_publisher(Path, TOPIC_PLANNED_PATH, latched),
            "topology": self._node.create_publisher(MarkerArray, TOPIC_TOPOLOGY_SEQUENCE, latched),
            "status": self._node.create_publisher(MarkerArray, TOPIC_STATUS, latched),
            "cloud": self._node.create_publisher(type(self._cloud), pub.TOPIC_SOURCE_CLOUD, latched),
            "regions": self._node.create_publisher(MarkerArray, pub.TOPIC_STAGE3_REGIONS, latched),
            "chords": self._node.create_publisher(MarkerArray, pub.TOPIC_STAGE4_CHORDS, latched),
        }
        self._node.create_subscription(
            PoseWithCovarianceStamped, TOPIC_INITIALPOSE, self._on_initialpose, inputs)
        self._node.create_subscription(
            PoseStamped, TOPIC_GOAL_POSE, self._on_goal_pose, inputs)
        self._client = self._node.create_client(PlanPath, service_name)

        self._publish_static()
        self._publish_endpoints()
        self._publish_clear()
        self._publish_status()
        self._node.create_timer(0.25, self._tick)
        self._node.create_timer(10.0, self._refresh)
        self._node.get_logger().info(
            "interactive planner ready: mesh faces=%d vertices=%d service=%s "
            "inputs=%s,%s (planning defaults after service readiness)"
            % (self._mesh.triangle_count, self._mesh.vertex_count, service_name,
               TOPIC_INITIALPOSE, TOPIC_GOAL_POSE))

    def _on_initialpose(self, message) -> None:  # pragma: no cover - live ROS
        try:
            validate_frame(message.header.frame_id)
            yaw = sanitize_planar_orientation(message.pose.pose.orientation)
            point = self._mesh.project(message.pose.pose.position.x,
                                       message.pose.pose.position.y)
        except (ProjectionError, ValueError) as error:
            self._invalid("start: %s" % error)
            return
        self._coord.set_start(point.as_tuple(), yaw)
        self._replan_needed = True
        self._node.get_logger().info(
            "START click -> (%.3f, %.3f, %.3f) yaw=%.3f revision=%d"
            % (point.x, point.y, point.z, yaw, self._coord.revision))
        self._publish_endpoints()
        self._publish_clear()
        self._publish_status()

    def _on_goal_pose(self, message) -> None:  # pragma: no cover - live ROS
        try:
            validate_frame(message.header.frame_id)
            point = self._mesh.project(message.pose.position.x, message.pose.position.y)
        except (ProjectionError, ValueError) as error:
            self._invalid("goal: %s" % error)
            return
        self._coord.set_goal(point.as_tuple())
        self._replan_needed = True
        self._node.get_logger().info(
            "GOAL click -> (%.3f, %.3f, %.3f) revision=%d"
            % (point.x, point.y, point.z, self._coord.revision))
        self._publish_endpoints()
        self._publish_clear()
        self._publish_status()

    def _invalid(self, reason: str) -> None:  # pragma: no cover - live ROS
        self._coord.invalidate(reason)
        self._replan_needed = False
        self._node.get_logger().warning("invalid click discarded: %s" % reason)
        self._publish_clear()
        self._publish_status()

    def _tick(self) -> None:  # pragma: no cover - live ROS
        if self._inflight_token is not None:
            if time.monotonic() - self._inflight_started > 180.0:
                token = self._inflight_token
                self._inflight_token = None
                self._on_plan_failed(token, "planner service timed out after 180 s")
            return
        if not self._replan_needed:
            return
        if not self._client.service_is_ready():
            if not self._service_logged:
                self._service_logged = True
                self._node.get_logger().info(
                    "waiting for %s (UI stays responsive; no blocking wait)"
                    % self._client.srv_name)
            self._coord.wait_for_service()
            self._publish_status()
            return
        if not self._service_logged:
            self._service_logged = True
            self._node.get_logger().info("planner service ready: %s" % self._client.srv_name)
        self._start_plan()

    def _start_plan(self) -> None:  # pragma: no cover - live ROS
        token = self._coord.begin_plan("endpoints")
        if token != self._last_requested_revision:
            self._last_requested_revision = token
        request = self._PlanPath.Request()
        request.start.header.frame_id = FRAME_ID
        request.start.header.stamp = self._node.get_clock().now().to_msg()
        request.start.pose.position = _point(self._coord.start, 0.0)
        qx, qy, qz, qw = quaternion_from_yaw(self._coord.start_yaw)
        request.start.pose.orientation.x = qx
        request.start.pose.orientation.y = qy
        request.start.pose.orientation.z = qz
        request.start.pose.orientation.w = qw
        request.goal.header.frame_id = FRAME_ID
        request.goal.header.stamp = request.start.header.stamp
        request.goal.pose.position = _point(self._coord.goal, 0.0)
        request.goal.pose.orientation.w = 1.0

        self._replan_needed = False
        self._inflight_token = token
        self._inflight_started = time.monotonic()
        self._publish_clear()
        self._publish_status()
        self._node.get_logger().info(
            "requesting plan revision=%d start=%s goal=%s"
            % (token, self._coord.start, self._coord.goal))
        future = self._client.call_async(request)
        future.add_done_callback(
            lambda done, captured=token: self._on_plan_done(captured, done))

    def _on_plan_done(self, token: int, future) -> None:  # pragma: no cover - live ROS
        if self._inflight_token == token:
            self._inflight_token = None
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - the service must not kill the UI
            self._on_plan_failed(token, "service call raised: %s" % error)
            return
        if not response.success:
            self._on_plan_failed(
                token, response.error_message or "service returned success=false")
            return
        try:
            points = _path_points(response)
        except ValueError as error:
            self._on_plan_failed(token, str(error))
            return
        if not self._coord.accept_path(token, points):
            self._node.get_logger().info(
                "discarding stale planner result revision=%d current=%d"
                % (token, self._coord.revision))
            return
        self._publish_path(points)
        try:
            self._publish_topology(response.topology_path)
        except Exception:  # visualization only; never invalidate the real path
            self._node.get_logger().warning("could not publish optional topology markers")
        self._publish_status()
        self._publish_endpoints()
        length = sum(
            math.dist(points[index], points[index + 1])
            for index in range(len(points) - 1)
        )
        self._node.get_logger().info(
            "planner success revision=%d points=%d length=%.3fm endpoints=(%.3f,%.3f,%.3f)->"
            "(%.3f,%.3f,%.3f)" % (token, len(points), length, *points[0], *points[-1]))

    def _on_plan_failed(self, token: int, reason: str) -> None:  # pragma: no cover - live ROS
        if not self._coord.fail(token, reason):
            self._node.get_logger().info(
                "discarding stale planner failure revision=%d current=%d"
                % (token, self._coord.revision))
            return
        self._node.get_logger().error(
            "planner failed revision=%d: %s (old path cleared)" % (token, reason))
        self._publish_clear()
        self._publish_status()

    def _refresh(self) -> None:  # pragma: no cover - live ROS
        self._publish_static()
        self._publish_endpoints()
        self._publish_status()
        if self._coord.last_path:
            self._publish_path(self._coord.last_path)
        else:
            self._publish_clear()

    def _stamp(self):
        return self._node.get_clock().now().to_msg()

    def _publish_static(self) -> None:  # pragma: no cover - live ROS
        stamp = self._stamp()
        for topic, array, key in (
            (pub.TOPIC_STAGE3_REGIONS, self._stage_regions, "regions"),
            (pub.TOPIC_STAGE4_CHORDS, self._stage_chords, "chords"),
        ):
            _stamp_markers(array, stamp)
            self._publishers[key].publish(array)
        self._cloud.header.stamp = stamp
        self._publishers["cloud"].publish(self._cloud)

    def _publish_endpoints(self) -> None:  # pragma: no cover - live ROS
        stamp = self._stamp()
        start = build_start_markers(self._coord.start, self._coord.start_yaw)
        goal = build_goal_markers(self._coord.goal)
        _stamp_markers(start, stamp)
        _stamp_markers(goal, stamp)
        self._publishers["start"].publish(start)
        self._publishers["goal"].publish(goal)

    def _publish_clear(self) -> None:  # pragma: no cover - live ROS
        stamp = self._stamp()
        cleared = build_cleared_path_markers()
        _stamp_markers(cleared, stamp)
        self._publishers["path"].publish(cleared)
        self._publishers["topology"].publish(cleared)
        empty = build_path_message([])
        empty.header.stamp = stamp
        self._publishers["nav_path"].publish(empty)

    def _publish_path(self, points: Sequence[Sequence[float]]) -> None:  # pragma: no cover
        stamp = self._stamp()
        array = build_path_markers(points)
        _stamp_markers(array, stamp)
        self._publishers["path"].publish(array)
        message = build_path_message(points)
        message.header.stamp = stamp
        for pose in message.poses:
            pose.header.stamp = stamp
        self._publishers["nav_path"].publish(message)

    def _publish_topology(self, topology_path) -> None:  # pragma: no cover - live ROS
        array = build_topology_sequence(topology_path, self._stages)
        _stamp_markers(array, self._stamp())
        self._publishers["topology"].publish(array)

    def _publish_status(self) -> None:  # pragma: no cover - live ROS
        marker = build_status_marker(self._coord.status, self._coord.status_text)
        marker.header.stamp = self._stamp()
        self._publishers["status"].publish(MarkerArray(markers=[marker]))

    def spin(self) -> None:  # pragma: no cover - live ROS
        self._rclpy.spin(self._node)

    def destroy(self) -> None:  # pragma: no cover - live ROS
        self._node.destroy_node()


def default_stages_path() -> str:
    return pub.default_data_path()


def default_pcd_path() -> str:
    return pub.default_pcd_path()


def run_check(stages_path: str, pcd_path: str) -> int:
    from rclpy.serialization import serialize_message

    stages = pub.load_stages(stages_path)
    pub.validate_stages(stages)
    mesh = load_surface_mesh(stages_path)
    cloud = pub.build_source_cloud(pcd_path)
    start = mesh.project(DEFAULT_START[0], DEFAULT_START[1])
    goal = mesh.project(DEFAULT_GOAL[0], DEFAULT_GOAL[1])
    if abs(start.z - DEFAULT_START[2]) > 1e-3 or abs(goal.z - DEFAULT_GOAL[2]) > 1e-3:
        raise ValueError("default endpoints do not lie on the stages.json surface")
    start_markers = build_start_markers(start.as_tuple(), 0.0)
    goal_markers = build_goal_markers(goal.as_tuple())
    cleared = build_cleared_path_markers()
    status = build_status_marker("waiting", "WAITING: planner service is not ready")
    print("[check] stages: %s" % os.path.abspath(stages_path))
    print("[check] pcd:    %s" % os.path.abspath(pcd_path))
    print("[check] mesh:   vertices=%d faces=%d" % (mesh.vertex_count, mesh.triangle_count))
    print("[check] default start=%s -> z=%.6f" % (DEFAULT_START, start.z))
    print("[check] default goal=%s -> z=%.6f" % (DEFAULT_GOAL, goal.z))
    print("[check] projection: XY containment + barycentric interpolation only")
    print("[check] start markers=%d goal markers=%d clear action=%d"
          % (len(start_markers.markers), len(goal_markers.markers),
             cleared.markers[0].action))
    print("[check] status=%r" % status.text)
    print("[check] cloud points=%d bytes=%d" % (cloud.width, len(serialize_message(cloud))))
    print("[check] OK: no DDS node created, no planner service called")
    return 0


def main(argv: List[str] | None = None) -> int:
    raw = list(sys.argv if argv is None else argv)
    cleaned = pub._strip_launch_ros_args(raw)
    if not cleaned:
        cleaned = ["interactive_planner"]
    parser = argparse.ArgumentParser(
        prog=os.path.basename(cleaned[0]) or "interactive_planner",
        description="RViz interactive bridge to the real TopMap PlanPath service.",
    )
    parser.add_argument("--stages", default=default_stages_path(),
                        help="stages.json containing the actual mesh (default: share/data)")
    parser.add_argument("--pcd", default=default_pcd_path(),
                        help="original synthetic PCD for the static cloud")
    parser.add_argument("--service", default=TOPIC_SERVICE,
                        help="isolated PlanPath service name")
    parser.add_argument("--no-auto-plan", action="store_true",
                        help="do not plan the default safe endpoints automatically")
    parser.add_argument("--check", action="store_true",
                        help="validate projection and messages offline, no DDS node")
    args = parser.parse_args(cleaned[1:])

    try:
        if args.check:
            return run_check(args.stages, args.pcd)
        import rclpy
        from rclpy.executors import ExternalShutdownException

        rclpy.init()
        bridge = InteractivePlannerNode(args.stages, args.pcd, args.service,
                                        auto_plan=not args.no_auto_plan)
        try:
            bridge.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        return 0
    except KeyboardInterrupt:
        return 0
    except ProjectionError as error:
        sys.stderr.write("[interactive] projection error: %s\n" % error)
        return 2
    except ValueError as error:
        sys.stderr.write("[interactive] validation error: %s\n" % error)
        return 2
    except FileNotFoundError as error:
        sys.stderr.write("[interactive] missing file: %s\n" % error)
        return 3


if __name__ == "__main__":
    sys.exit(main())
