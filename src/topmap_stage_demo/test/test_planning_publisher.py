"""Contract tests for the planning-only RViz publisher.

The planning view deliberately publishes only the clean layers: raw cloud,
final regions, red portal chords, start/goal and the real planner path.  The
green topology chain is a *separate* optional layer and must never be confused
with the final path.
"""
import copy
import json
import math
import os

import pytest
from visualization_msgs.msg import Marker

from topmap_stage_demo import planning as plan
from topmap_stage_demo import publisher as pub

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, "..", "data"))
STAGES = os.path.join(DATA, "stages.json")
PLANNING = os.path.join(DATA, "planning.json")
PCD = os.path.join(DATA, "scene_h_ground_original_ramp.pcd")


def _load(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


@pytest.fixture(scope="module")
def stages():
    return _load(STAGES)


@pytest.fixture(scope="module")
def planning():
    return _load(PLANNING)


@pytest.fixture(scope="module")
def messages(planning, stages):
    return plan.build_all(planning, stages)


def _markers_of(messages, topic, marker_id=None):
    markers = messages[topic].markers
    if marker_id is not None:
        markers = [marker for marker in markers if marker.id == marker_id]
    return markers


def test_planning_view_has_only_the_clean_layers(messages):
    assert set(messages) == set(plan.PLANNING_MARKER_TOPICS)
    assert plan.TOPIC_STAGE1_MESH not in messages
    assert plan.TOPIC_STAGE1_EDGES not in messages
    assert plan.TOPIC_STAGE2_REGIONS not in messages
    assert plan.TOPIC_STAGE3_REGIONS in messages
    assert plan.TOPIC_STAGE4_CHORDS in messages
    for topic, array in messages.items():
        pub.validate_marker_array(topic, array)


def test_start_and_goal_markers_sit_on_the_requested_points(planning, messages):
    start = planning["request"]["start"]
    goal = planning["request"]["goal"]
    for topic, expected, text in (
        (plan.TOPIC_PLANNING_START, start, "START"),
        (plan.TOPIC_PLANNING_GOAL, goal, "GOAL"),
    ):
        spheres = [m for m in messages[topic].markers if m.type == Marker.SPHERE]
        assert len(spheres) == 1
        sphere = spheres[0]
        assert sphere.pose.position.x == pytest.approx(expected[0], abs=1e-9)
        assert sphere.pose.position.y == pytest.approx(expected[1], abs=1e-9)
        assert sphere.pose.position.z == pytest.approx(
            expected[2] + plan.ENDPOINT_Z_LIFT, abs=1e-9
        )
        assert 0.0 < plan.ENDPOINT_Z_LIFT <= 0.06

        labels = [m for m in messages[topic].markers if m.type == Marker.TEXT_VIEW_FACING]
        assert len(labels) == 1
        assert labels[0].text == text
        assert labels[0].text.isascii()

    start_sphere = [
        m for m in messages[plan.TOPIC_PLANNING_START].markers if m.type == Marker.SPHERE
    ][0]
    goal_sphere = [
        m for m in messages[plan.TOPIC_PLANNING_GOAL].markers if m.type == Marker.SPHERE
    ][0]
    assert (start_sphere.color.r, start_sphere.color.g, start_sphere.color.b) != (
        goal_sphere.color.r,
        goal_sphere.color.g,
        goal_sphere.color.b,
    )


def test_final_path_marker_is_wide_bright_and_keeps_real_geometry(planning, messages):
    path = planning["path"]
    markers = messages[plan.TOPIC_FINAL_PATH].markers
    assert len(markers) == 2, "expected a dark underlay plus the bright path"
    underlay, core = markers
    assert underlay.type == core.type == Marker.LINE_STRIP
    # The final path is planning.path verbatim, NOT the dense surface trace.
    assert len(core.points) == len(path)
    for point, expected in zip(core.points, path):
        assert point.x == pytest.approx(expected[0], abs=1e-9)
        assert point.y == pytest.approx(expected[1], abs=1e-9)
        assert point.z == pytest.approx(expected[2] + plan.PATH_Z_LIFT, abs=1e-9)
    assert 0.0 < plan.PATH_Z_LIFT <= 0.005
    assert core.scale.x >= 0.10
    assert underlay.scale.x > core.scale.x
    assert core.color.g < core.color.r, "the final path must not be the green topology chain"


def test_surface_trace_reference_is_separate_and_render_only(planning, messages):
    trace = planning["surface_trace"]["path"]
    markers = messages[plan.TOPIC_SURFACE_TRACE].markers
    lines = [marker for marker in markers if marker.type == Marker.LINE_STRIP]
    assert len(lines) == 1
    line = lines[0]
    assert line.ns == "planning/surface_trace_reference"
    assert len(line.points) == len(trace)
    for point, expected in zip(line.points, trace):
        assert point.x == pytest.approx(expected[0], abs=1e-9)
        assert point.y == pytest.approx(expected[1], abs=1e-9)
        assert point.z == pytest.approx(
            expected[2] + plan.SURFACE_TRACE_Z_LIFT, abs=1e-9
        )
    labels = [marker for marker in markers if marker.type == Marker.TEXT_VIEW_FACING]
    assert labels and "REFERENCE ONLY" in labels[0].text
    # It is explicitly not the final path layer.
    assert plan.TOPIC_SURFACE_TRACE != plan.TOPIC_FINAL_PATH



def test_nav_msgs_path_carries_the_same_real_polyline(planning):
    message = plan.build_path_message(planning)
    assert message.header.frame_id == "map"
    assert len(message.poses) == len(planning["path"])
    for pose, expected in zip(message.poses, planning["path"]):
        assert pose.pose.position.x == pytest.approx(expected[0], abs=1e-9)
        assert pose.pose.position.y == pytest.approx(expected[1], abs=1e-9)
        assert pose.pose.position.z == pytest.approx(expected[2], abs=1e-9)


def test_topology_sequence_is_a_separate_optional_green_layer(planning, stages, messages):
    marker_array = messages[plan.TOPIC_TOPOLOGY_SEQUENCE]
    assert len(marker_array.markers) == 2  # solid topology chain + portal nodes
    chain = marker_array.markers[0]
    assert chain.type == Marker.LINE_STRIP
    # regions + portals + regions = 2 * portals + 1 points for this scene.
    assert len(chain.points) == 2 * len(planning["topology"]["portals"]) + 1
    assert chain.scale.x < messages[plan.TOPIC_FINAL_PATH].markers[1].scale.x
    assert chain.color.g > chain.color.r and chain.color.g > chain.color.b
    # The chain is built from the topology nodes, not from path points.
    first = planning["topology"]["regions"][0]
    centroid = next(
        region["centroid"] for region in stages["topology"]["regions"] if region["id"] == first
    )
    assert chain.points[0].x == pytest.approx(centroid[0], abs=1e-6)
    assert chain.points[0].y == pytest.approx(centroid[1], abs=1e-6)


def test_validate_rejects_failed_or_fabricated_planning(planning, stages):
    assert plan.validate_planning(planning, stages) is None

    failed = copy.deepcopy(planning)
    failed["status"] = "failed"
    failed["error"] = {"code": 604, "code_name": "kSurfaceSearchFailed", "message": "boom"}
    with pytest.raises(ValueError, match="status"):
        plan.validate_planning(failed, stages)

    nan_path = copy.deepcopy(planning)
    nan_path["path"][len(nan_path["path"]) // 2][2] = math.nan
    with pytest.raises(ValueError, match="non-finite"):
        plan.validate_planning(nan_path, stages)

    wrong_endpoint = copy.deepcopy(planning)
    wrong_endpoint["path"][-1] = [28.0, 3.5, 2.0]
    with pytest.raises(ValueError, match="endpoint|goal"):
        plan.validate_planning(wrong_endpoint, stages)

    unsafe = copy.deepcopy(planning)
    unsafe["safety"]["valid"] = False
    unsafe["safety"]["violating_indices"] = [7]
    with pytest.raises(ValueError, match="safety"):
        plan.validate_planning(unsafe, stages)

    bad_topology = copy.deepcopy(planning)
    bad_topology["topology"]["portals"][0] = 99
    with pytest.raises(ValueError, match="portal"):
        plan.validate_planning(bad_topology, stages)

    dense_claim = copy.deepcopy(planning)
    dense_claim["planner"]["dense_surface_path_replay"] = True
    with pytest.raises(ValueError, match="dense"):
        plan.validate_planning(dense_claim, stages)

    fake_trace = copy.deepcopy(planning)
    fake_trace["surface_trace"]["is_planner_final_path"] = True
    with pytest.raises(ValueError, match="surface_trace"):
        plan.validate_planning(fake_trace, stages)



def test_offline_check_serializes_every_planning_topic(capsys):
    code = plan.main([
        "planning_publisher",
        "--stages", STAGES,
        "--planning", PLANNING,
        "--pcd", PCD,
        "--check",
        "--ros-args", "-p", "use_sim_time:=true",
        "-r", "__node:=renamed",
    ])
    assert code == 0
    out = capsys.readouterr().out
    assert "planning_status=success" in out
    assert "no DDS node created" in out
    for topic in plan.PLANNING_MARKER_TOPICS:
        assert topic in out
    assert plan.TOPIC_PLANNED_PATH in out
