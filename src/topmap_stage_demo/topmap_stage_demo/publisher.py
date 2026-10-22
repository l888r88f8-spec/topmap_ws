#!/usr/bin/env python3
"""Overlaid multi-stage RViz publisher for ``topmap_stage_demo``.

Every algorithm stage of the *same* synthetic scene is published in the
``map`` frame at the **same XY origin** as the original point cloud.  There
are **no translated copies**: ``STAGE_OFFSETS`` is ``(0, 0, 0)`` for all
stages and the raw point-cloud XYZ is written verbatim.  The only geometric
adjustment is a documented, rendering-only Z lift (``<= 0.03 m``) that keeps
coincident surfaces/wires from z-fighting.

Each stage is an independently toggleable ROS topic so the RViz layer tree
can show or hide it:

======================  =====================================================
topic suffix            layer (RViz group)
======================  =====================================================
``source_cloud``        原始点云 (gray points, alpha 1, always published)
``stage1_triangle_mesh``    三角化 表面 (fill alpha 0.06)
``stage1_triangle_edges``   三角化 网格边 (thin, opaque)
``stage2_grown_regions``    区域合并 初始 3 区域 (fill alpha 0.14)
``stage3_final_regions``    结构细分 最终 5 区域 (fill alpha 0.44)
``stage4_topology_links``   拓扑图 links (green R-P-R, all 4 valid portals)
``stage4_region_nodes``     拓扑图 region nodes (yellow centroids)
``stage4_portal_chords``    通道 red chords (left-right)
``stage4_portal_nodes``     通道 portal centers (yellow)
``stage4_boundary_chains``  通道 actual boundary chain (cyan, OFF by default)
``region_labels`` / ``portal_labels``  optional R#/P# labels (OFF by default)
======================  =====================================================

The module is split in two: every marker is built by a *pure* function that
takes plain ``dict`` data (loaded from ``data/stages.json``) and returns a
``visualization_msgs/MarkerArray``; the thin :class:`StagePublisher` node only
owns the publishers, the QoS profile and the refresh timer.  ``--check``
therefore validates and serializes every message, plus the ``PointCloud2``
source cloud, completely offline (no DDS, no node).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
from typing import Dict, Iterable, List, Sequence, Tuple

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rclpy.utilities import remove_ros_args
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# ---------------------------------------------------------------------------
# constants
# ---------------------------------------------------------------------------
FRAME_ID = "map"
QOS_DEPTH = 1
QOS_RELIABLE = True
QOS_TRANSIENT_LOCAL = True
REFRESH_PERIOD_S = 10.0

TOPIC_PREFIX = "/topmap_stage_demo"
TOPIC_SOURCE_CLOUD = TOPIC_PREFIX + "/source_cloud"
TOPIC_STAGE1_MESH = TOPIC_PREFIX + "/stage1_triangle_mesh"
TOPIC_STAGE1_EDGES = TOPIC_PREFIX + "/stage1_triangle_edges"
TOPIC_STAGE2_REGIONS = TOPIC_PREFIX + "/stage2_grown_regions"
TOPIC_STAGE3_REGIONS = TOPIC_PREFIX + "/stage3_final_regions"
TOPIC_STAGE4_LINKS = TOPIC_PREFIX + "/stage4_topology_links"
TOPIC_STAGE4_REGION_NODES = TOPIC_PREFIX + "/stage4_region_nodes"
TOPIC_STAGE4_PORTAL_NODES = TOPIC_PREFIX + "/stage4_portal_nodes"
TOPIC_STAGE4_CHORDS = TOPIC_PREFIX + "/stage4_portal_chords"
TOPIC_STAGE4_CHAINS = TOPIC_PREFIX + "/stage4_boundary_chains"
TOPIC_REGION_LABELS = TOPIC_PREFIX + "/region_labels"
TOPIC_PORTAL_LABELS = TOPIC_PREFIX + "/portal_labels"

MARKER_TOPICS: Tuple[str, ...] = (
    TOPIC_STAGE1_MESH,
    TOPIC_STAGE1_EDGES,
    TOPIC_STAGE2_REGIONS,
    TOPIC_STAGE3_REGIONS,
    TOPIC_STAGE4_LINKS,
    TOPIC_STAGE4_REGION_NODES,
    TOPIC_STAGE4_PORTAL_NODES,
    TOPIC_STAGE4_CHORDS,
    TOPIC_STAGE4_CHAINS,
    TOPIC_REGION_LABELS,
    TOPIC_PORTAL_LABELS,
)
ALL_TOPICS = MARKER_TOPICS

# Overlay requirement: every stage shares the original map coordinates.
STAGE_OFFSETS: Dict[str, Tuple[float, float, float]] = {
    "stage1": (0.0, 0.0, 0.0),
    "stage2": (0.0, 0.0, 0.0),
    "stage3": (0.0, 0.0, 0.0),
    "stage4": (0.0, 0.0, 0.0),
}

# Rendering-only Z lifts, all <= 0.03 m, so coincident surfaces/wires do not
# z-fight.  They never move a point in XY.
MESH_FILL_Z = 0.000
EDGE_Z = 0.008
GROWN_FILL_Z = 0.012
FINAL_FILL_Z = 0.018
PLANNING_FILL_Z = 0.001
LINK_Z = 0.022
CHORD_Z = 0.024
CHAIN_Z = 0.024
NODE_Z = 0.028
LABEL_Z = 0.030

# Per-layer blend alpha, applied through the marker's *global* alpha channel.
MESH_FILL_ALPHA = 0.06
GROWN_FILL_ALPHA = 0.14
FINAL_FILL_ALPHA = 0.44

# RViz 11.2.x only selects its transparent-alpha blending mode when at least
# one *per-vertex* alpha is below ``rviz_rendering::unit_alpha_threshold``
# (0.9998); ``marker.color.a`` is then multiplied into every vertex.  All
# per-vertex alphas stay at 1.0 (the requested convention) except this single
# near-unit trigger that merely enables blending.
BLEND_TRIGGER_ALPHA = 0.998

MESH_RGB = (0.70, 0.72, 0.76)
EDGE_RGB = (0.10, 0.11, 0.18)
LINK_RGB = (0.10, 0.90, 0.20)
REGION_NODE_RGB = (1.00, 0.85, 0.00)
PORTAL_NODE_RGB = (1.00, 1.00, 0.00)
CHORD_RGB = (1.00, 0.10, 0.10)
CHAIN_RGB = (0.00, 0.85, 0.90)
LABEL_RGB = (0.98, 0.98, 0.55)

DEFAULT_DATA = os.path.join("data", "stages.json")
DEFAULT_PCD = os.path.join("data", "scene_h_ground_original_ramp.pcd")


# ---------------------------------------------------------------------------
# path resolution (ament_index share, with a source-tree fallback)
# ---------------------------------------------------------------------------
def _share_path(*parts: str) -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        candidate = os.path.join(get_package_share_directory("topmap_stage_demo"), *parts)
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", *parts))


def default_data_path() -> str:
    return _share_path(*DEFAULT_DATA.split(os.sep))


def default_pcd_path() -> str:
    return _share_path(*DEFAULT_PCD.split(os.sep))


# ---------------------------------------------------------------------------
# pure helpers
# ---------------------------------------------------------------------------
def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _finite(value: float) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _finite3(point: Sequence[float]) -> bool:
    return len(point) == 3 and all(_finite(v) for v in point)


def load_stages(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def unique_edges(faces: Iterable[Sequence[int]]) -> List[List[int]]:
    """Unique undirected triangle edges, sorted, each appearing exactly once."""
    edges = set()
    for face in faces:
        _require(len(face) == 3, "every face must have exactly three indices")
        a, b, c = int(face[0]), int(face[1]), int(face[2])
        for u, v in ((a, b), (b, c), (c, a)):
            edges.add((u, v) if u < v else (v, u))
    return [[u, v] for u, v in sorted(edges)]


def _hsv_to_rgb(h: float, s: float, v: float) -> Tuple[float, float, float]:
    i = int(h * 6.0) % 6
    f = h * 6.0 - int(h * 6.0)
    p = v * (1.0 - s)
    q = v * (1.0 - f * s)
    t = v * (1.0 - (1.0 - f) * s)
    return [
        (v, t, p),
        (q, v, p),
        (p, v, t),
        (p, q, v),
        (t, p, v),
        (v, p, q),
    ][i]


def region_color(index: int) -> Tuple[float, float, float]:
    """Deterministic, well separated colour for a region id."""
    hue = (int(index) * 0.618033988749895) % 1.0
    return _hsv_to_rgb(hue, 0.62, 0.95)


# ---------------------------------------------------------------------------
# source cloud (raw XYZ, never translated)
# ---------------------------------------------------------------------------
def load_pcd_xyz(path: str) -> List[List[float]]:
    """Minimal ASCII XYZ PCD reader used only for the source cloud."""
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        lines = handle.readlines()
    header_end = None
    data = "ascii"
    for index, line in enumerate(lines):
        if line.startswith("DATA"):
            data = line.split()[1].strip().lower()
            header_end = index + 1
            break
    _require(header_end is not None, "%s: missing DATA header" % path)
    _require(data == "ascii", "%s: only ascii PCD is supported" % path)
    points = []
    for line in lines[header_end:]:
        parts = line.split()
        if len(parts) < 3:
            continue
        points.append([float(parts[0]), float(parts[1]), float(parts[2])])
    _require(points, "%s: no points parsed" % path)
    return points


def build_source_cloud(pcd_path: str):
    """Build the original PointCloud2 with the raw XYZ untouched."""
    from sensor_msgs.msg import PointCloud2, PointField

    points = load_pcd_xyz(pcd_path)
    cloud = PointCloud2()
    cloud.header.frame_id = FRAME_ID
    cloud.header.stamp = Time()
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12 * len(points)
    cloud.is_dense = True
    cloud.data = b"".join(
        struct.pack("<fff", p[0], p[1], p[2]) for p in points
    )
    return cloud


# ---------------------------------------------------------------------------
# validation
# ---------------------------------------------------------------------------
def _validate_regions(regions: Sequence[dict], face_count: int, label: str) -> None:
    _require(isinstance(regions, list) and regions, "%s regions must be a non-empty list" % label)
    seen = set()
    for region in regions:
        rid = region["id"]
        _require(rid not in seen, "%s region id %s duplicated" % (label, rid))
        seen.add(rid)
        _require(_finite3(region["centroid"]), "%s region %s has a non-finite centroid" % (label, rid))
        ids = region["triangle_ids"]
        _require(len(ids) > 0, "%s region %s has no triangles" % (label, rid))
        for tid in ids:
            _require(isinstance(tid, int) and 0 <= tid < face_count,
                     "%s region %s references out-of-range triangle %s" % (label, rid, tid))


def validate_stages(data: dict) -> None:
    """Reject malformed/NaN/out-of-range stage data before any message is built."""
    _require(isinstance(data, dict), "stages data must be an object")
    for key in ("counts", "mesh", "stages", "topology", "config"):
        _require(key in data, "missing top-level key %r" % key)

    counts = data["counts"]
    mesh = data["mesh"]
    vertices = mesh["vertices"]
    faces = mesh["faces"]

    _require(isinstance(vertices, list) and vertices, "mesh.vertices must be non-empty")
    for index, vertex in enumerate(vertices):
        _require(_finite3(vertex), "mesh.vertices[%d] is not a finite xyz" % index)
    _require(isinstance(faces, list) and faces, "mesh.faces must be non-empty")
    for index, face in enumerate(faces):
        _require(len(face) == 3, "mesh.faces[%d] does not have three indices" % index)
        for vid in face:
            _require(isinstance(vid, int) and 0 <= vid < len(vertices),
                     "mesh.faces[%d] references out-of-range vertex %s" % (index, vid))

    _require(counts["vertex_count"] == len(vertices),
             "counts.vertex_count disagrees with mesh.vertices")
    _require(counts["face_count"] == len(faces),
             "counts.face_count disagrees with mesh.faces")
    _require(counts["unique_edge_count"] == len(mesh["unique_edges"]),
             "counts.unique_edge_count disagrees with mesh.unique_edges")
    _require(unique_edges(faces) == [list(e) for e in mesh["unique_edges"]],
             "mesh.unique_edges is not the deduplicated edge set of mesh.faces")

    stages = data["stages"]
    for key in ("grown", "split", "final"):
        _require(key in stages, "missing stages.%s" % key)
        _validate_regions(stages[key]["regions"], len(faces), key)

    topology = data["topology"]
    _validate_regions(topology["regions"], len(faces), "topology")
    region_ids = {r["id"] for r in topology["regions"]}
    portals = topology["portals"]
    _require(isinstance(portals, list), "topology.portals must be a list")
    _require(len(portals) == counts["final_portal_count"],
             "counts.final_portal_count disagrees with topology.portals")
    _require(len(topology["regions"]) == counts["final_region_count"],
             "counts.final_region_count disagrees with topology.regions")
    seen_portals = set()
    for portal in portals:
        pid = portal["id"]
        _require(pid not in seen_portals, "portal id %s duplicated" % pid)
        seen_portals.add(pid)
        _require(portal["region_a"] in region_ids,
                 "portal %s region_a %s is not a final region" % (pid, portal["region_a"]))
        _require(portal["region_b"] in region_ids,
                 "portal %s region_b %s is not a final region" % (pid, portal["region_b"]))
        _require(portal["region_a"] != portal["region_b"],
                 "portal %s connects a region to itself" % pid)
        for name in ("left", "right", "center"):
            _require(_finite3(portal[name]), "portal %s %s is not finite" % (pid, name))
        chain = portal["boundary_chain"]
        chain_points = portal["boundary_points"]
        _require(len(chain) > 0, "portal %s has an empty boundary_chain" % pid)
        _require(len(chain) == len(chain_points),
                 "portal %s boundary_chain/boundary_points length mismatch" % pid)
        for vid in chain:
            _require(isinstance(vid, int) and 0 <= vid < len(vertices),
                     "portal %s boundary_chain references out-of-range vertex %s" % (pid, vid))
        for point in chain_points:
            _require(_finite3(point), "portal %s has a non-finite boundary point" % pid)


def validate_marker_array(topic: str, array: MarkerArray) -> None:
    """Validate the message conventions RViz relies on."""
    _require(isinstance(array, MarkerArray), "%s is not a MarkerArray" % topic)
    seen = set()
    for marker in array.markers:
        key = (marker.ns, marker.id)
        _require(key not in seen, "%s: duplicated marker %s/%s" % (topic, marker.ns, marker.id))
        seen.add(key)
        _require(marker.header.frame_id == FRAME_ID,
                 "%s: marker %s/%s frame is %r" % (topic, marker.ns, marker.id, marker.header.frame_id))
        _require(marker.action == Marker.ADD, "%s: marker action is not ADD" % topic)
        _require(marker.frame_locked is False, "%s: frame_locked must be false" % topic)
        _require(marker.lifetime.sec == 0 and marker.lifetime.nanosec == 0,
                 "%s: lifetime must be zero" % topic)
        for axis in ("x", "y", "z"):
            _require(getattr(marker.scale, axis) > 0.0,
                     "%s: marker %s/%s has a non-positive scale" % (topic, marker.ns, marker.id))
        orientation = marker.pose.orientation
        _require(
            _finite(orientation.x) and _finite(orientation.y) and _finite(orientation.z)
            and _finite(orientation.w),
            "%s: marker orientation is not finite" % topic,
        )
        _require(
            abs(orientation.x) < 1e-9 and abs(orientation.y) < 1e-9
            and abs(orientation.z) < 1e-9 and abs(orientation.w - 1.0) < 1e-9,
            "%s: marker orientation is not identity" % topic,
        )
        position = marker.pose.position
        _require(_finite(position.x) and _finite(position.y) and _finite(position.z),
                 "%s: marker position is not finite" % topic)
        for index, point in enumerate(marker.points):
            _require(_finite(point.x) and _finite(point.y) and _finite(point.z),
                     "%s: marker %s/%s point %d is not finite" % (topic, marker.ns, marker.id, index))
        for index, color in enumerate(marker.colors):
            for channel in (color.r, color.g, color.b, color.a):
                _require(_finite(channel) and 0.0 <= channel <= 1.0,
                         "%s: marker %s/%s colour %d is out of range" % (topic, marker.ns, marker.id, index))
        if marker.type in (Marker.TRIANGLE_LIST, Marker.LINE_LIST, Marker.SPHERE_LIST, Marker.POINTS):
            _require(len(marker.colors) == len(marker.points),
                     "%s: marker %s/%s colors must be per point" % (topic, marker.ns, marker.id))
        _require(marker.text.isascii(), "%s: marker text must be ASCII" % topic)


# ---------------------------------------------------------------------------
# marker builders
# ---------------------------------------------------------------------------
def _time() -> Time:
    return Time()


def _marker(ns: str, marker_id: int, marker_type: int, scale, rgb) -> Marker:
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = _time()
    marker.ns = ns
    marker.id = int(marker_id)
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    marker.color.r, marker.color.g, marker.color.b = float(rgb[0]), float(rgb[1]), float(rgb[2])
    marker.color.a = 1.0
    marker.frame_locked = False
    marker.lifetime.sec = 0
    marker.lifetime.nanosec = 0
    return marker


def _point(coords: Sequence[float], stage: str, dz: float = 0.0) -> Point:
    dx, dy, dz0 = STAGE_OFFSETS[stage]
    return Point(
        x=float(coords[0]) + dx,
        y=float(coords[1]) + dy,
        z=float(coords[2]) + dz0 + dz,
    )


def _rgba(rgb, alpha: float = 1.0) -> ColorRGBA:
    color = ColorRGBA()
    color.r, color.g, color.b, color.a = float(rgb[0]), float(rgb[1]), float(rgb[2]), float(alpha)
    return color


def _append_vertex(marker: Marker, point: Point, rgb) -> None:
    """Per-vertex colour with RGB and alpha 1.0 (the requested convention)."""
    marker.points.append(point)
    marker.colors.append(_rgba(rgb, 1.0))


def _set_layer_alpha(marker: Marker, alpha: float) -> None:
    """Put layer opacity on the global marker alpha, as requested.

    RViz multiplies ``marker.color.a`` into each per-vertex alpha, but its
    transparent-alpha material is only selected when a per-vertex alpha drops
    below ``unit_alpha_threshold``.  One near-unit trigger vertex enables the
    blend without changing the requested per-vertex ``alpha = 1`` convention.
    """
    marker.color.a = alpha
    if alpha < 1.0 and marker.colors:
        marker.colors[0].a = BLEND_TRIGGER_ALPHA


def _append_triangle(marker: Marker, v0: Point, v1: Point, v2: Point, rgb) -> None:
    _append_vertex(marker, v0, rgb)
    _append_vertex(marker, v1, rgb)
    _append_vertex(marker, v2, rgb)


def build_stage1_mesh(data: dict) -> MarkerArray:
    """Raw triangle surface at the original coordinates (fill alpha 0.06)."""
    mesh = data["mesh"]
    marker = _marker("stage1/mesh", 0, Marker.TRIANGLE_LIST, (1.0, 1.0, 1.0), MESH_RGB)
    vertices = mesh["vertices"]
    for face in mesh["faces"]:
        _append_triangle(
            marker,
            _point(vertices[face[0]], "stage1", MESH_FILL_Z),
            _point(vertices[face[1]], "stage1", MESH_FILL_Z),
            _point(vertices[face[2]], "stage1", MESH_FILL_Z),
            MESH_RGB,
        )
    _set_layer_alpha(marker, MESH_FILL_ALPHA)
    return MarkerArray(markers=[marker])


def build_stage1_edges(data: dict) -> MarkerArray:
    """Deduplicated triangle edges, drawn thin just above the surface."""
    mesh = data["mesh"]
    marker = _marker("stage1/edges", 0, Marker.LINE_LIST, (0.012, 0.012, 0.012), EDGE_RGB)
    vertices = mesh["vertices"]
    for a, b in mesh["unique_edges"]:
        _append_vertex(marker, _point(vertices[a], "stage1", EDGE_Z), EDGE_RGB)
        _append_vertex(marker, _point(vertices[b], "stage1", EDGE_Z), EDGE_RGB)
    return MarkerArray(markers=[marker])


def _build_region_array(data: dict, stage_key: str, stage: str, ns: str,
                        dz: float, alpha: float) -> MarkerArray:
    mesh = data["mesh"]
    regions = data["stages"][stage_key]["regions"]
    markers = []
    for region in regions:
        rgb = region_color(region["id"])
        marker = _marker(ns, region["id"], Marker.TRIANGLE_LIST, (1.0, 1.0, 1.0), rgb)
        for tid in region["triangle_ids"]:
            face = mesh["faces"][tid]
            _append_triangle(
                marker,
                _point(mesh["vertices"][face[0]], stage, dz),
                _point(mesh["vertices"][face[1]], stage, dz),
                _point(mesh["vertices"][face[2]], stage, dz),
                rgb,
            )
        _set_layer_alpha(marker, alpha)
        markers.append(marker)
    return MarkerArray(markers=markers)


def build_stage2_regions(data: dict) -> MarkerArray:
    """Initial grown regions (3) overlaid on the same mesh (fill alpha 0.14)."""
    return _build_region_array(data, "grown", "stage2", "stage2/grown_regions",
                               GROWN_FILL_Z, GROWN_FILL_ALPHA)


def build_stage3_regions(data: dict, z_lift: float = FINAL_FILL_Z) -> MarkerArray:
    """Final regions; planning views use a smaller lift than the stage overlay."""
    return _build_region_array(data, "final", "stage3", "stage3/final_regions",
                               z_lift, FINAL_FILL_ALPHA)


def _region_centroid(data: dict, region_id: int):
    for region in data["topology"]["regions"]:
        if region["id"] == region_id:
            return region["centroid"]
    raise ValueError("unknown region id %s" % region_id)


def build_stage4_topology_links(data: dict) -> MarkerArray:
    """Region centroid -> portal centre -> region centroid for EVERY valid portal."""
    marker = _marker("stage4/links", 0, Marker.LINE_LIST, (0.022, 0.022, 0.022), LINK_RGB)
    for portal in data["topology"]["portals"]:
        center = _point(portal["center"], "stage4", LINK_Z)
        for region_id in (portal["region_a"], portal["region_b"]):
            _append_vertex(marker, _point(_region_centroid(data, region_id), "stage4", LINK_Z), LINK_RGB)
            _append_vertex(marker, center, LINK_RGB)
    return MarkerArray(markers=[marker])


def build_stage4_region_nodes(data: dict) -> MarkerArray:
    marker = _marker("stage4/region_nodes", 0, Marker.SPHERE_LIST,
                     (0.16, 0.16, 0.16), REGION_NODE_RGB)
    for region in data["topology"]["regions"]:
        _append_vertex(marker, _point(region["centroid"], "stage4", NODE_Z), REGION_NODE_RGB)
    return MarkerArray(markers=[marker])


def build_stage4_portal_nodes(data: dict) -> MarkerArray:
    marker = _marker("stage4/portal_nodes", 0, Marker.SPHERE_LIST,
                     (0.14, 0.14, 0.14), PORTAL_NODE_RGB)
    for portal in data["topology"]["portals"]:
        _append_vertex(marker, _point(portal["center"], "stage4", NODE_Z), PORTAL_NODE_RGB)
    return MarkerArray(markers=[marker])


def build_stage4_portal_chords(data: dict) -> MarkerArray:
    """Red left-right chord for every valid portal."""
    marker = _marker("stage4/chords", 0, Marker.LINE_LIST, (0.034, 0.034, 0.034), CHORD_RGB)
    for portal in data["topology"]["portals"]:
        _append_vertex(marker, _point(portal["left"], "stage4", CHORD_Z), CHORD_RGB)
        _append_vertex(marker, _point(portal["right"], "stage4", CHORD_Z), CHORD_RGB)
    return MarkerArray(markers=[marker])


def build_stage4_boundary_chains(data: dict) -> MarkerArray:
    """Optional cyan boundary chain, one LINE_STRIP per valid portal (default off)."""
    markers = []
    for portal in data["topology"]["portals"]:
        marker = _marker("stage4/boundary_chains", portal["id"], Marker.LINE_STRIP,
                         (0.012, 0.012, 0.012), CHAIN_RGB)
        for point in portal["boundary_points"]:
            _append_vertex(marker, _point(point, "stage4", CHAIN_Z), CHAIN_RGB)
        markers.append(marker)
    return MarkerArray(markers=markers)


def _text_marker(ns: str, marker_id: int, text: str, position: Point, height, rgb) -> Marker:
    marker = _marker(ns, marker_id, Marker.TEXT_VIEW_FACING, (height, height, height), rgb)
    marker.pose.position = position
    marker.text = text
    return marker


def build_region_labels(data: dict) -> MarkerArray:
    """Optional R# labels for the final regions (RViz default: OFF)."""
    markers = []
    for index, region in enumerate(data["stages"]["final"]["regions"]):
        markers.append(_text_marker(
            "region_labels", index, "R%d" % region["id"],
            _point(region["centroid"], "stage3", LABEL_Z), 0.45, LABEL_RGB))
    return MarkerArray(markers=markers)


def build_portal_labels(data: dict) -> MarkerArray:
    """Optional P# labels at portal centres (RViz default: OFF)."""
    markers = []
    for portal in data["topology"]["portals"]:
        markers.append(_text_marker(
            "portal_labels", portal["id"], "P%d" % portal["id"],
            _point(portal["center"], "stage4", LABEL_Z), 0.40, CHORD_RGB))
    return MarkerArray(markers=markers)


BUILDERS = {
    TOPIC_STAGE1_MESH: build_stage1_mesh,
    TOPIC_STAGE1_EDGES: build_stage1_edges,
    TOPIC_STAGE2_REGIONS: build_stage2_regions,
    TOPIC_STAGE3_REGIONS: build_stage3_regions,
    TOPIC_STAGE4_LINKS: build_stage4_topology_links,
    TOPIC_STAGE4_REGION_NODES: build_stage4_region_nodes,
    TOPIC_STAGE4_PORTAL_NODES: build_stage4_portal_nodes,
    TOPIC_STAGE4_CHORDS: build_stage4_portal_chords,
    TOPIC_STAGE4_CHAINS: build_stage4_boundary_chains,
    TOPIC_REGION_LABELS: build_region_labels,
    TOPIC_PORTAL_LABELS: build_portal_labels,
}

# There are deliberately no stage-title markers: the four stages share one
# origin, so four titles would overlap each other and the geometry.


def build_all(data: dict) -> Dict[str, MarkerArray]:
    validate_stages(data)
    return {topic: builder(data) for topic, builder in BUILDERS.items()}


def serialize_all(messages: Dict[str, MarkerArray]) -> Dict[str, int]:
    from rclpy.serialization import serialize_message

    return {topic: len(serialize_message(array)) for topic, array in messages.items()}


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
def run_node(data_path: str, pcd_path: str) -> int:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    data, messages = _load_build_validate(data_path)
    cloud = build_source_cloud(pcd_path)

    rclpy.init()
    try:
        node = Node("topmap_stage_demo")
        qos = QoSProfile(
            depth=QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE if QOS_RELIABLE else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if QOS_TRANSIENT_LOCAL else DurabilityPolicy.VOLATILE,
        )
        publishers = {
            topic: node.create_publisher(MarkerArray, topic, qos) for topic in ALL_TOPICS
        }
        cloud_publisher = node.create_publisher(type(cloud), TOPIC_SOURCE_CLOUD, qos)

        counts = data["counts"]

        def publish_once() -> None:
            if not rclpy.ok():
                return
            try:
                stamp = node.get_clock().now().to_msg()
                for topic, array in messages.items():
                    for marker in array.markers:
                        marker.header.stamp = stamp
                    publishers[topic].publish(array)
                cloud.header.stamp = stamp
                cloud_publisher.publish(cloud)
            except Exception:
                if not rclpy.ok():
                    return
                raise

        publish_once()
        node.create_timer(REFRESH_PERIOD_S, publish_once)
        node.get_logger().info(
            "overlaid stages published: faces=%d grown=%d split=%d final_regions=%d "
            "final_portals=%d cloud_points=%d; refreshing every %.0fs (Ctrl-C to stop)"
            % (counts["face_count"], counts["grown_region_count"], counts["split_region_count"],
               counts["final_region_count"], counts["final_portal_count"], cloud.width,
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
def _load_build_validate(data_path: str):
    data = load_stages(data_path)
    messages = build_all(data)
    for topic, array in messages.items():
        validate_marker_array(topic, array)
    return data, messages


def run_check(data_path: str, pcd_path: str) -> int:
    from rclpy.serialization import serialize_message

    data, messages = _load_build_validate(data_path)
    sizes = serialize_all(messages)
    counts = data["counts"]
    cloud = build_source_cloud(pcd_path)
    cloud_bytes = len(serialize_message(cloud))
    print("[check] data: %s" % os.path.abspath(data_path))
    print("[check] pcd:  %s" % os.path.abspath(pcd_path))
    print(
        "[check] pipeline: vertices=%d faces=%d unique_edges=%d grown=%d split=%d "
        "final_regions=%d final_portals=%d"
        % (counts["vertex_count"], counts["face_count"], counts["unique_edge_count"],
           counts["grown_region_count"], counts["split_region_count"],
           counts["final_region_count"], counts["final_portal_count"])
    )
    total_markers = 0
    total_bytes = 0
    for topic in ALL_TOPICS:
        array = messages[topic]
        total_markers += len(array.markers)
        total_bytes += sizes[topic]
        print("[check] %-52s markers=%-4d bytes=%d" % (topic, len(array.markers), sizes[topic]))
    print("[check] %-52s points=%-4d bytes=%d" % (TOPIC_SOURCE_CLOUD, cloud.width, cloud_bytes))
    print(
        "[check] OK: %d topics, %d markers, %d bytes serialized, cloud=%d points, no DDS node created"
        % (len(ALL_TOPICS), total_markers, total_bytes, cloud.width)
    )
    return 0


# ---------------------------------------------------------------------------
# cli
# ---------------------------------------------------------------------------
def _strip_launch_ros_args(argv: List[str]) -> List[str]:
    """Drop the ``--ros-args`` section that ``ros2 launch`` appends.

    ``rclpy.utilities.remove_ros_args`` is the authoritative parser.  It raises
    ``UnknownROSArgsError`` when non-ROS arguments follow ``--ros-args`` (which
    happens if the section is not last), so fall back to keeping everything
    before ``--ros-args`` plus anything after an explicit ``--`` separator.
    """
    try:
        return remove_ros_args(args=argv)
    except Exception:
        if "--ros-args" not in argv:
            return argv
        head = list(argv[:argv.index("--ros-args")])
        rest = list(argv[argv.index("--ros-args") + 1:])
        if "--" in rest:
            head += rest[rest.index("--") + 1:]
        return head


def main(argv: List[str] | None = None) -> int:
    raw = list(sys.argv if argv is None else argv)
    cleaned = _strip_launch_ros_args(raw)
    if not cleaned:
        cleaned = ["stage_publisher"]

    parser = argparse.ArgumentParser(
        prog=os.path.basename(cleaned[0]) or "stage_publisher",
        description="Overlaid multi-stage TopMap RViz publisher.",
    )
    parser.add_argument("--data", default=default_data_path(),
                        help="path to stages.json (default: installed share/data)")
    parser.add_argument("--pcd", default=default_pcd_path(),
                        help="path to the original ASCII XYZ PCD (default: installed share/data)")
    parser.add_argument("--check", action="store_true",
                        help="validate and serialize every stage message and the source cloud offline")
    args = parser.parse_args(cleaned[1:])

    try:
        if args.check:
            return run_check(args.data, args.pcd)
        return run_node(args.data, args.pcd)
    except KeyboardInterrupt:
        return 0
    except ValueError as error:
        sys.stderr.write("[publisher] validation error: %s\n" % error)
        return 2
    except FileNotFoundError as error:
        sys.stderr.write("[publisher] missing file: %s\n" % error)
        return 3


if __name__ == "__main__":
    sys.exit(main())
