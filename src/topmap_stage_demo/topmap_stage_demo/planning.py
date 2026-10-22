#!/usr/bin/env python3
"""Clean planning replay view for ``topmap_stage_demo``.

This module is deliberately *not* a planner.  It replays the real, offline
``topmap::HierarchicalPlanner`` result stored in ``data/planning.json`` and
shows, in the same ``map`` frame as the original cloud:

* the original point cloud (untouched XYZ);
* the final 5 regions and the red portal chords;
* START (green sphere) and GOAL (red sphere) at the requested endpoints;
* the **planner-returned path verbatim** as a wide bright orange line and as a
  ``nav_msgs/Path``;
* an optional green topology sequence (off by default);
* an optional, clearly-labelled **render-only ground projection reference**
  built by sampling and projecting ``planner.path`` (off by default).

The last item is *not* the planner output and is *not* an independent
validation. The original ``planner.path`` is never replaced by the display-only
projected trace, in either the Marker or the
``nav_msgs/Path``.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import Dict, List, Sequence, Tuple

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from . import publisher as pub

FRAME_ID = pub.FRAME_ID
QOS_DEPTH = pub.QOS_DEPTH
QOS_RELIABLE = pub.QOS_RELIABLE
QOS_TRANSIENT_LOCAL = pub.QOS_TRANSIENT_LOCAL
REFRESH_PERIOD_S = pub.REFRESH_PERIOD_S

TOPIC_PREFIX = pub.TOPIC_PREFIX
TOPIC_SOURCE_CLOUD = pub.TOPIC_SOURCE_CLOUD
TOPIC_STAGE1_MESH = pub.TOPIC_STAGE1_MESH
TOPIC_STAGE1_EDGES = pub.TOPIC_STAGE1_EDGES
TOPIC_STAGE2_REGIONS = pub.TOPIC_STAGE2_REGIONS
TOPIC_STAGE3_REGIONS = pub.TOPIC_STAGE3_REGIONS
TOPIC_STAGE4_CHORDS = pub.TOPIC_STAGE4_CHORDS

TOPIC_PLANNING_START = TOPIC_PREFIX + "/planning_start"
TOPIC_PLANNING_GOAL = TOPIC_PREFIX + "/planning_goal"
TOPIC_FINAL_PATH = TOPIC_PREFIX + "/final_path"
TOPIC_PLANNED_PATH = TOPIC_PREFIX + "/planned_path"
TOPIC_TOPOLOGY_SEQUENCE = TOPIC_PREFIX + "/planned_topology_sequence"
TOPIC_SURFACE_TRACE = TOPIC_PREFIX + "/surface_trace_reference"

PLANNING_MARKER_TOPICS: Tuple[str, ...] = (
    TOPIC_STAGE3_REGIONS,
    TOPIC_STAGE4_CHORDS,
    TOPIC_PLANNING_START,
    TOPIC_PLANNING_GOAL,
    TOPIC_FINAL_PATH,
    TOPIC_TOPOLOGY_SEQUENCE,
    TOPIC_SURFACE_TRACE,
)
ALL_TOPICS = PLANNING_MARKER_TOPICS

# Rendering-only lifts.  They never change the stored geometry; the nav_msgs
# Path message uses the exact planning.json coordinates with no lift at all.
PATH_Z_LIFT = 0.005
SURFACE_TRACE_Z_LIFT = 0.018
TOPOLOGY_Z_LIFT = 0.045
ENDPOINT_Z_LIFT = 0.040

FINAL_PATH_RGB = (1.00, 0.55, 0.02)       # bright orange, clearly not green
FINAL_PATH_UNDERLAY_RGB = (0.12, 0.05, 0.00)
START_RGB = (0.10, 1.00, 0.20)            # green
GOAL_RGB = (1.00, 0.10, 0.10)             # red
TOPOLOGY_RGB = (0.10, 1.00, 0.20)         # optional green chain
PORTAL_NODE_RGB = (1.00, 0.85, 0.00)
SURFACE_TRACE_RGB = (0.10, 0.80, 1.00)    # optional cyan reference line

FINAL_PATH_WIDTH = 0.12
FINAL_PATH_UNDERLAY_WIDTH = 0.20
TOPOLOGY_WIDTH = 0.055
SURFACE_TRACE_WIDTH = 0.030

DEFAULT_STAGES = os.path.join("data", "stages.json")
DEFAULT_PLANNING = os.path.join("data", "planning.json")
DEFAULT_PCD = os.path.join("data", "scene_h_ground_original_ramp.pcd")


# ---------------------------------------------------------------------------
# paths
# ---------------------------------------------------------------------------
def default_stages_path() -> str:
    return pub.default_data_path()


def default_planning_path() -> str:
    return pub._share_path(*DEFAULT_PLANNING.split(os.sep))


def default_pcd_path() -> str:
    return pub.default_pcd_path()


def load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


# ---------------------------------------------------------------------------
# validation
# ---------------------------------------------------------------------------
def _require(condition: bool, message: str) -> None:
    pub._require(condition, message)


def _length(points: Sequence[Sequence[float]]) -> float:
    return sum(
        math.dist(points[index], points[index + 1])
        for index in range(len(points) - 1)
    )


def _close3(a: Sequence[float], b: Sequence[float], tolerance: float = 2e-3) -> bool:
    return len(a) == 3 and len(b) == 3 and all(
        abs(float(a[index]) - float(b[index])) <= tolerance for index in range(3)
    )


def validate_planning(planning: dict, stages: dict) -> None:
    """Reject failed, fabricated or internally inconsistent planning data."""
    _require(isinstance(planning, dict), "planning data must be an object")
    _require(planning.get("schema") == "topmap_stage_demo.planning",
             "unexpected planning schema")
    _require(planning.get("schema_version") == 1,
             "unsupported planning schema_version")
    _require(planning.get("mode") == "offline_precomputed_replay",
             "planning is not an offline precomputed replay")
    _require(planning.get("interactive_replanning") is False,
             "this demo only supports precomputed replays")
    _require(planning.get("frame_id") == FRAME_ID,
             "planning frame_id must be %r" % FRAME_ID)
    _require(planning.get("status") == "success",
             "planning status is %r, not 'success'" % planning.get("status"))
    _require(planning.get("error") is None, "successful planning still has an error")

    planner = planning.get("planner") or {}
    _require(planner.get("name") == "topmap::HierarchicalPlanner",
             "unexpected planner name")
    _require(planner.get("dense_surface_path_replay") is False,
             "planner.dense_surface_path_replay must be false: the displayed path "
             "is the planner-returned path, not a display-only replacement")

    request = planning.get("request") or {}
    start = request.get("start")
    goal = request.get("goal")
    _require(pub._finite3(start), "planning.request.start is not finite")
    _require(pub._finite3(goal), "planning.request.goal is not finite")

    path = planning.get("path")
    _require(isinstance(path, list) and len(path) >= 2,
             "planning.path must contain at least two points")
    for index, point in enumerate(path):
        _require(pub._finite3(point),
                 "planning.path[%d] is non-finite or not xyz" % index)
    _require(_close3(path[0], start),
             "planning.path endpoint start does not match request.start")
    _require(_close3(path[-1], goal),
             "planning.path endpoint goal does not match request.goal")

    counts = planning.get("counts") or {}
    _require(counts.get("path_points") == len(path),
             "counts.path_points disagrees with planning.path")
    measured_length = _length(path)
    declared_length = planning.get("path_length_m")
    _require(isinstance(declared_length, (int, float))
             and math.isfinite(float(declared_length)),
             "path_length_m is not finite")
    _require(abs(measured_length - float(declared_length))
             <= max(1e-3, 1e-4 * abs(float(declared_length))),
             "path_length_m disagrees with the planner path")

    topology = planning.get("topology") or {}
    regions = topology.get("regions")
    portals = topology.get("portals")
    _require(isinstance(regions, list) and isinstance(portals, list),
             "planning.topology regions/portals must be lists")
    _require(len(regions) == len(portals) + 1,
             "topology must have exactly one more region than portal")
    stage_regions = {region["id"] for region in stages["topology"]["regions"]}
    stage_portals = {portal["id"]: portal for portal in stages["topology"]["portals"]}
    for region_id in regions:
        _require(region_id in stage_regions,
                 "topology region %r is not a stage final region" % (region_id,))
    for index, portal_id in enumerate(portals):
        _require(portal_id in stage_portals,
                 "topology portal %r is not a valid stage portal" % (portal_id,))
        portal = stage_portals[portal_id]
        _require({portal["region_a"], portal["region_b"]}
                 == {regions[index], regions[index + 1]},
                 "topology portal %r does not connect consecutive regions" % (portal_id,))

    safety = planning.get("safety") or {}
    _require(safety.get("valid") is True, "safety validation is not valid")
    _require(safety.get("violating_indices") == [],
             "safety violating_indices is not empty")
    _require(safety.get("portal_order_matches") is True,
             "safety portal order does not match the planned topology")
    _require(safety.get("crossed_portals") == portals,
             "safety crossed_portals does not match topology.portals")
    for key in ("min_clearance", "max_step", "max_slope_deg", "max_curvature"):
        value = safety.get(key)
        _require(isinstance(value, (int, float)) and math.isfinite(float(value)),
                 "safety.%s is not finite" % key)

    trace = planning.get("surface_trace")
    if trace is not None:
        _require(trace.get("kind") == "render_only_surface_projection_reference",
                 "surface_trace identity is not the render-only reference")
        _require(trace.get("render_only") is True,
                 "surface_trace.render_only must be true")
        _require(trace.get("independent_validation") is False,
                 "surface_trace must not claim independent validation")
        _require(trace.get("is_planner_final_path") is False,
                 "surface_trace must not claim to be the planner final path")
        _require(trace.get("source") == "planner.path",
                 "surface_trace.source must be planner.path")
        _require(trace.get("projection_ok") is True,
                 "surface_trace.projection_ok is not true")
        trace_path = trace.get("path")
        _require(isinstance(trace_path, list) and len(trace_path) >= 2,
                 "surface_trace.path must contain at least two points")
        for index, point in enumerate(trace_path):
            _require(pub._finite3(point),
                     "surface_trace.path[%d] is non-finite or not xyz" % index)
        _require(counts.get("surface_trace_points") == len(trace_path),
                 "counts.surface_trace_points disagrees with surface_trace.path")
        _require(_close3(trace_path[0], path[0]),
                 "surface_trace start is not the projected planner start")
        _require(_close3(trace_path[-1], path[-1]),
                 "surface_trace goal is not the projected planner goal")


# ---------------------------------------------------------------------------
# marker helpers
# ---------------------------------------------------------------------------
def _point_xyz(coords: Sequence[float], dz: float = 0.0) -> Point:
    return Point(x=float(coords[0]), y=float(coords[1]), z=float(coords[2]) + dz)


def _line_marker(ns: str, marker_id: int, width: float, rgb) -> Marker:
    return pub._marker(ns, marker_id, Marker.LINE_STRIP, (width, width, width), rgb)


def _path_marker(points: Sequence[Sequence[float]], ns: str, marker_id: int,
                 width: float, rgb, dz: float) -> Marker:
    marker = _line_marker(ns, marker_id, width, rgb)
    for point in points:
        marker.points.append(_point_xyz(point, dz))
    return marker


def build_start_markers(planning: dict) -> MarkerArray:
    start = planning["request"]["start"]
    sphere = pub._marker("planning/start", 0, Marker.SPHERE,
                         (0.36, 0.36, 0.36), START_RGB)
    sphere.pose.position = _point_xyz(start, ENDPOINT_Z_LIFT)
    label = pub._text_marker(
        "planning/start_label", 1, "START",
        _point_xyz(start, ENDPOINT_Z_LIFT + 0.28), 0.45, START_RGB,
    )
    return MarkerArray(markers=[sphere, label])


def build_goal_markers(planning: dict) -> MarkerArray:
    goal = planning["request"]["goal"]
    sphere = pub._marker("planning/goal", 0, Marker.SPHERE,
                         (0.36, 0.36, 0.36), GOAL_RGB)
    sphere.pose.position = _point_xyz(goal, ENDPOINT_Z_LIFT)
    label = pub._text_marker(
        "planning/goal_label", 1, "GOAL",
        _point_xyz(goal, ENDPOINT_Z_LIFT + 0.28), 0.45, GOAL_RGB,
    )
    return MarkerArray(markers=[sphere, label])


def build_final_path_markers(planning: dict) -> MarkerArray:
    """The planner-returned path verbatim: dark underlay + bright orange core."""
    path = planning["path"]
    underlay = _path_marker(
        path, "planning/final_path_underlay", 0, FINAL_PATH_UNDERLAY_WIDTH,
        FINAL_PATH_UNDERLAY_RGB, PATH_Z_LIFT - 0.004,
    )
    core = _path_marker(
        path, "planning/final_path", 1, FINAL_PATH_WIDTH,
        FINAL_PATH_RGB, PATH_Z_LIFT,
    )
    return MarkerArray(markers=[underlay, core])


def build_path_message(planning: dict):
    """nav_msgs/Path carrying the exact, un-lifted planner polyline."""
    from nav_msgs.msg import Path

    message = Path()
    message.header.frame_id = FRAME_ID
    message.header.stamp = Time()
    for point in planning["path"]:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position = _point_xyz(point, 0.0)
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def build_topology_sequence(planning: dict, stages: dict) -> MarkerArray:
    """Optional green region -> portal -> region chain (never the final path)."""
    region_centroids = {
        region["id"]: region["centroid"] for region in stages["topology"]["regions"]
    }
    portal_centers = {
        portal["id"]: portal["center"] for portal in stages["topology"]["portals"]
    }
    topology = planning["topology"]
    chain = _line_marker(
        "planning/topology_sequence", 0, TOPOLOGY_WIDTH, TOPOLOGY_RGB,
    )
    for index, region_id in enumerate(topology["regions"]):
        chain.points.append(_point_xyz(region_centroids[region_id], TOPOLOGY_Z_LIFT))
        if index < len(topology["portals"]):
            portal_id = topology["portals"][index]
            chain.points.append(_point_xyz(portal_centers[portal_id], TOPOLOGY_Z_LIFT))

    nodes = pub._marker("planning/topology_portals", 0, Marker.SPHERE_LIST,
                        (0.12, 0.12, 0.12), PORTAL_NODE_RGB)
    for portal_id in topology["portals"]:
        nodes.points.append(_point_xyz(portal_centers[portal_id], TOPOLOGY_Z_LIFT))
        nodes.colors.append(pub._rgba(PORTAL_NODE_RGB))
    return MarkerArray(markers=[chain, nodes])


def build_surface_trace_reference(planning: dict) -> MarkerArray:
    """Optional render-only projection reference, never the planner output."""
    trace = planning.get("surface_trace") or {}
    trace_path = trace.get("path") or []
    if not trace_path:
        return MarkerArray(markers=[])
    line = _path_marker(
        trace_path, "planning/surface_trace_reference", 0,
        SURFACE_TRACE_WIDTH, SURFACE_TRACE_RGB, SURFACE_TRACE_Z_LIFT,
    )
    middle = trace_path[len(trace_path) // 2]
    label = pub._text_marker(
        "planning/surface_trace_label", 0,
        "SURFACE TRACE REFERENCE ONLY",
        _point_xyz(middle, SURFACE_TRACE_Z_LIFT + 0.30), 0.32, SURFACE_TRACE_RGB,
    )
    return MarkerArray(markers=[line, label])


# ---------------------------------------------------------------------------
# messages
# ---------------------------------------------------------------------------
def build_all(planning: dict, stages: dict) -> Dict[str, MarkerArray]:
    validate_planning(planning, stages)
    return {
        TOPIC_STAGE3_REGIONS: pub.build_stage3_regions(stages, z_lift=pub.PLANNING_FILL_Z),
        TOPIC_STAGE4_CHORDS: pub.build_stage4_portal_chords(stages),
        TOPIC_PLANNING_START: build_start_markers(planning),
        TOPIC_PLANNING_GOAL: build_goal_markers(planning),
        TOPIC_FINAL_PATH: build_final_path_markers(planning),
        TOPIC_TOPOLOGY_SEQUENCE: build_topology_sequence(planning, stages),
        TOPIC_SURFACE_TRACE: build_surface_trace_reference(planning),
    }


def _load_build_validate(stages_path: str, planning_path: str):
    stages = load_json(stages_path)
    planning = load_json(planning_path)
    messages = build_all(planning, stages)
    for topic, array in messages.items():
        pub.validate_marker_array(topic, array)
    return stages, planning, messages


# ---------------------------------------------------------------------------
# node
# ---------------------------------------------------------------------------
def run_node(stages_path: str, planning_path: str, pcd_path: str) -> int:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    stages, planning, messages = _load_build_validate(stages_path, planning_path)
    cloud = pub.build_source_cloud(pcd_path)
    path_message = build_path_message(planning)

    rclpy.init()
    try:
        node = Node("topmap_planning_demo")
        qos = QoSProfile(
            depth=QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE if QOS_RELIABLE else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if QOS_TRANSIENT_LOCAL else DurabilityPolicy.VOLATILE,
        )
        publishers = {
            topic: node.create_publisher(MarkerArray, topic, qos)
            for topic in PLANNING_MARKER_TOPICS
        }
        path_publisher = node.create_publisher(type(path_message), TOPIC_PLANNED_PATH, qos)
        cloud_publisher = node.create_publisher(type(cloud), TOPIC_SOURCE_CLOUD, qos)

        def publish_once() -> None:
            if not rclpy.ok():
                return
            try:
                stamp = node.get_clock().now().to_msg()
                for topic, array in messages.items():
                    for marker in array.markers:
                        marker.header.stamp = stamp
                    publishers[topic].publish(array)
                path_message.header.stamp = stamp
                for pose in path_message.poses:
                    pose.header.stamp = stamp
                path_publisher.publish(path_message)
                cloud.header.stamp = stamp
                cloud_publisher.publish(cloud)
            except Exception:
                if not rclpy.ok():
                    return
                raise

        publish_once()
        node.create_timer(REFRESH_PERIOD_S, publish_once)
        node.get_logger().info(
            "planning replay published: path_points=%d path_length=%.3fm "
            "surface_trace_points=%d (render-only reference) regions=%d portals=%d; "
            "the final path is planning.path verbatim; refreshing every %.0fs"
            % (len(planning["path"]), planning["path_length_m"],
               len((planning.get("surface_trace") or {}).get("path") or []),
               len(stages["topology"]["regions"]), len(stages["topology"]["portals"]),
               REFRESH_PERIOD_S)
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    return 0


# ---------------------------------------------------------------------------
# offline check
# ---------------------------------------------------------------------------
def run_check(stages_path: str, planning_path: str, pcd_path: str) -> int:
    from rclpy.serialization import serialize_message

    stages, planning, messages = _load_build_validate(stages_path, planning_path)
    cloud = pub.build_source_cloud(pcd_path)
    path_message = build_path_message(planning)

    print("[check] stages:   %s" % os.path.abspath(stages_path))
    print("[check] planning: %s" % os.path.abspath(planning_path))
    print("[check] pcd:      %s" % os.path.abspath(pcd_path))
    print(
        "[check] planning_status=%s path_points=%d path_length_m=%.6f "
        "surface_trace_points=%d regions=%d portals=%d"
        % (planning["status"], len(planning["path"]), planning["path_length_m"],
           len((planning.get("surface_trace") or {}).get("path") or []),
           len(stages["topology"]["regions"]), len(stages["topology"]["portals"]))
    )
    print("[check] final path source: planning.path verbatim "
          "(render-only Z lift %.3f m; nav_msgs/Path has no lift)" % PATH_Z_LIFT)
    print("[check] surface_trace: render-only projection reference; "
          "independent_validation=%s is_planner_final_path=%s"
          % ((planning.get("surface_trace") or {}).get("independent_validation"),
             (planning.get("surface_trace") or {}).get("is_planner_final_path")))

    total_markers = 0
    total_bytes = 0
    for topic in PLANNING_MARKER_TOPICS:
        array = messages[topic]
        size = len(serialize_message(array))
        total_markers += len(array.markers)
        total_bytes += size
        print("[check] %-52s markers=%-4d bytes=%d" % (topic, len(array.markers), size))
    path_bytes = len(serialize_message(path_message))
    print("[check] %-52s poses=%-4d bytes=%d"
          % (TOPIC_PLANNED_PATH, len(path_message.poses), path_bytes))
    cloud_bytes = len(serialize_message(cloud))
    print("[check] %-52s points=%-4d bytes=%d"
          % (TOPIC_SOURCE_CLOUD, cloud.width, cloud_bytes))
    print(
        "[check] OK: %d marker topics, %d markers, %d bytes serialized, "
        "nav_msgs/Path poses=%d, cloud=%d points, no DDS node created"
        % (len(PLANNING_MARKER_TOPICS), total_markers, total_bytes,
           len(path_message.poses), cloud.width)
    )
    return 0


# ---------------------------------------------------------------------------
# cli
# ---------------------------------------------------------------------------
def main(argv: List[str] | None = None) -> int:
    raw = list(sys.argv if argv is None else argv)
    cleaned = pub._strip_launch_ros_args(raw)
    if not cleaned:
        cleaned = ["planning_publisher"]

    parser = argparse.ArgumentParser(
        prog=os.path.basename(cleaned[0]) or "planning_publisher",
        description="Replay the real offline TopMap planning result in RViz.",
    )
    parser.add_argument("--stages", default=default_stages_path(),
                        help="path to stages.json (default: installed share/data)")
    parser.add_argument("--planning", default=default_planning_path(),
                        help="path to planning.json (default: installed share/data)")
    parser.add_argument("--pcd", default=default_pcd_path(),
                        help="path to the original ASCII XYZ PCD (default: installed share/data)")
    parser.add_argument("--check", action="store_true",
                        help="validate and serialize every planning message offline")
    args = parser.parse_args(cleaned[1:])

    try:
        if args.check:
            return run_check(args.stages, args.planning, args.pcd)
        return run_node(args.stages, args.planning, args.pcd)
    except KeyboardInterrupt:
        return 0
    except ValueError as error:
        sys.stderr.write("[planning] validation error: %s\n" % error)
        return 2
    except FileNotFoundError as error:
        sys.stderr.write("[planning] missing file: %s\n" % error)
        return 3


if __name__ == "__main__":
    sys.exit(main())
