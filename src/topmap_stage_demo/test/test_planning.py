"""Regression tests for the real offline planning export.

The planning JSON is *not* hand-authored: ``tools/export_stages.cpp`` runs the
production ``topmap::HierarchicalPlanner`` on the same mesh/topology as
``stages.json`` and records the returned path, topology sequence, planner stats
and an independent ``SafetyValidator`` report.

The returned planner path must stay near the actual mesh along entire segments,
including the flat/ramp/platform creases. The optional display-only reference
keeps its own provenance even when its geometry coincides with the real path.
"""
import json
import math
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, "..", "data"))
STAGES = os.path.join(DATA, "stages.json")
PLANNING = os.path.join(DATA, "planning.json")

RAMP_X0 = 17.0
RAMP_X1 = 26.4
RAMP_RISE = 2.0

EXPECTED_START = [2.0, 3.5, 0.0]
EXPECTED_GOAL = [28.4, 3.5, 2.0]
EXPECTED_REGIONS = [1, 2, 3, 0, 4]
EXPECTED_PORTALS = [2, 3, 0, 1]


def _load(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


@pytest.fixture(scope="module")
def planning():
    assert os.path.isfile(PLANNING), (
        "planning.json is missing; run tools/build_data.py to run the real "
        "HierarchicalPlanner offline"
    )
    return _load(PLANNING)


@pytest.fixture(scope="module")
def stages():
    return _load(STAGES)


def _ramp_z(x):
    return RAMP_RISE * (x - RAMP_X0) / (RAMP_X1 - RAMP_X0)


def _ground_z(x):
    if x <= RAMP_X0:
        return 0.0
    if x >= RAMP_X1:
        return RAMP_RISE
    return _ramp_z(x)


def _length(path):
    return sum(
        math.dist(path[i], path[i + 1]) for i in range(len(path) - 1)
    )


def test_planning_is_real_offline_planner_output(planning):
    """The file must identify itself as a real, precomputed planner replay."""
    assert planning["schema"] == "topmap_stage_demo.planning"
    assert planning["schema_version"] == 1
    assert planning["mode"] == "offline_precomputed_replay"
    assert planning["interactive_replanning"] is False
    assert planning["frame_id"] == "map"
    assert planning["status"] == "success"
    assert planning["error"] is None

    planner = planning["planner"]
    assert planner["name"] == "topmap::HierarchicalPlanner"
    assert "RegionLocator" in planner["pipeline"]
    assert "TopologyAStar" in planner["pipeline"]
    assert "SurfaceAStar" in planner["pipeline"]
    assert "SafetyValidator" in planner["pipeline"]

    # The replay uses the same optimization switches as the interactive planner;
    # its displayed geometry is still the actual returned path.
    assert planner["dense_surface_path_replay"] is False
    assert planner["displayed_path"].startswith("planner.path")
    config = planner["config"]
    assert config["shortcut_enabled"] is True
    assert config["path_surface_tolerance"] == pytest.approx(0.005)
    assert config["funnel_enabled"] is True
    assert config["smoothing_enabled"] is True
    assert config["smoothing_iterations"] == 20
    note = planner["demo_note"].lower()
    assert "shortcut" in note
    assert "funnel" in note and "path_surface_tolerance" in note
    assert "surface_trace" in note
    assert "funnel are disabled" not in note


def test_fixed_start_and_goal_are_the_requested_points(planning):
    request = planning["request"]
    assert request["start"] == pytest.approx(EXPECTED_START, abs=1e-6)
    assert request["goal"] == pytest.approx(EXPECTED_GOAL, abs=1e-6)

    path = planning["path"]
    # Do NOT require >= 20 points here: the real planner output for this fixed
    # replay is sparse.  Just verify it is a genuine path with matching counts.
    assert len(path) >= 2
    assert path[0] == pytest.approx(EXPECTED_START, abs=2e-3)
    assert path[-1] == pytest.approx(EXPECTED_GOAL, abs=2e-3)
    assert planning["counts"]["path_points"] == len(path)


def test_path_is_finite_complete_and_has_real_length(planning):
    path = planning["path"]
    for index, point in enumerate(path):
        assert len(point) == 3, index
        assert all(isinstance(value, (int, float)) for value in point), index
        assert all(math.isfinite(value) for value in point), index

    assert min(point[2] for point in path) >= -1e-3
    assert max(point[2] for point in path) <= RAMP_RISE + 1e-3
    length = _length(path)
    assert 20.0 < length < 40.0
    assert planning["path_length_m"] == pytest.approx(length, rel=1e-4)
    assert planning["counts"]["path_points"] == len(path)

    # The dense projection must stay separate: it is not the planner path, and
    # no projected sample may be counted as an original planner vertex.
    trace_path = planning["surface_trace"]["path"]
    assert len(trace_path) >= len(path)
    assert planning["counts"]["surface_trace_points"] == len(trace_path)

    # Every planner vertex is represented by the ground projection (it is the
    # starting point of that render-only post-process, not a replacement).
    for point in path:
        assert min(math.dist(point, projected) for projected in trace_path) <= 1e-2


def test_planner_path_stays_on_ground_along_complete_segments(planning):
    """Independently bound the complete polyline on this piecewise-planar scene.

    On each linear path segment, the vertical error is piecewise linear, so its
    maximum occurs at an endpoint or at the known x=17/26.4 slope transitions.
    Checking those crossings avoids missing a crease between sampling points.
    """
    path = planning["path"]
    tolerance = planning["planner"]["config"].get("path_surface_tolerance", 0.005)
    slope_angle = math.atan(RAMP_RISE / (RAMP_X1 - RAMP_X0))
    vertical_limit = tolerance / math.cos(slope_angle) + 2e-5
    maximum = 0.0
    for start, goal in zip(path, path[1:]):
        fractions = [0.0, 1.0]
        if abs(goal[0] - start[0]) > 1e-12:
            for crease in (RAMP_X0, RAMP_X1):
                crossing = (crease - start[0]) / (goal[0] - start[0])
                if 0.0 < crossing < 1.0:
                    fractions.append(crossing)
        for t in fractions:
            x = start[0] + t * (goal[0] - start[0])
            z = start[2] + t * (goal[2] - start[2])
            maximum = max(maximum, abs(z - _ground_z(x)))
    assert maximum <= vertical_limit, (maximum, vertical_limit)
    for x, _y, z in path:
        assert z == pytest.approx(_ground_z(x), abs=1e-3)
    assert max(p[2] for p in path) - min(p[2] for p in path) > 1.9


def test_planner_preserves_exact_ramp_foot_and_top(planning):
    """A long ramp chord must bend at the real terrain transitions."""
    path = planning["path"]
    indices = []
    for crease_x, crease_z in ((RAMP_X0, 0.0), (RAMP_X1, RAMP_RISE)):
        matches = [
            index for index, (x, y, z) in enumerate(path)
            if abs(x - crease_x) <= 2e-5 and abs(z - crease_z) <= 2e-5
            and 2.9 <= y <= 4.1
        ]
        assert matches, ("missing terrain crease vertex", crease_x, path)
        indices.append(matches[0])
    assert indices[0] < indices[1]


def test_topology_sequence_is_real_and_ordered(planning, stages):
    topology = planning["topology"]
    assert topology["regions"] == EXPECTED_REGIONS
    assert topology["portals"] == EXPECTED_PORTALS
    assert len(topology["regions"]) == len(topology["portals"]) + 1

    portals = {portal["id"]: portal for portal in stages["topology"]["portals"]}
    regions = {region["id"] for region in stages["topology"]["regions"]}
    for index, portal_id in enumerate(topology["portals"]):
        assert portal_id in portals
        portal = portals[portal_id]
        assert {portal["region_a"], portal["region_b"]} == {
            topology["regions"][index],
            topology["regions"][index + 1],
        }
    for region_id in topology["regions"]:
        assert region_id in regions


def test_safety_report_and_planner_stats_are_preserved(planning, stages):
    safety = planning["safety"]
    stats = planning["stats"]
    config = stages["config"]

    assert safety["valid"] is True
    assert safety["portal_order_matches"] is True
    assert safety["crossed_portals"] == planning["topology"]["portals"]
    assert safety["violating_indices"] == []
    for key in ("min_clearance", "max_step", "max_slope_deg", "max_curvature"):
        assert math.isfinite(safety[key]), key
    assert safety["min_clearance"] >= 0.0
    assert safety["max_step"] <= config["robot"]["max_step_height"] + 1e-3
    assert safety["max_slope_deg"] <= config["robot"]["max_slope_deg"] + 1e-3
    assert (
        safety["max_curvature"] <= config["robot"]["max_path_curvature"] + 1e-3
    )
    assert "not a full robot safety certification" in safety["claimed_scope"]

    assert stats["topology_expanded_nodes"] >= 1
    assert stats["surface_expanded_nodes"] >= 1
    assert stats["regions_in_corridor"] >= 1
    assert stats["triangles_in_corridor"] >= 1
    assert math.isfinite(stats["path_length"])
    assert stats["path_length"] == pytest.approx(planning["path_length_m"], rel=1e-6)
    assert safety["min_clearance"] == pytest.approx(stats["min_clearance"], rel=1e-6)
    assert safety["max_slope_deg"] == pytest.approx(stats["max_slope_deg"], rel=1e-6)
    assert safety["max_curvature"] == pytest.approx(stats["max_curvature"], rel=1e-6)
    assert planning["request"]["start"] == pytest.approx(EXPECTED_START, abs=1e-6)
    assert planning["request"]["goal"] == pytest.approx(EXPECTED_GOAL, abs=1e-6)
    assert planning["disclaimer"]


def test_surface_trace_is_render_only_reference_not_final_path(planning):
    """The dense trace is ground-adherent, but is NOT the planner final path."""
    trace = planning["surface_trace"]
    path = planning["path"]

    assert trace["kind"] == "render_only_surface_projection_reference"
    assert trace["render_only"] is True
    assert trace["independent_validation"] is False
    assert trace["is_planner_final_path"] is False
    assert trace["source"] == "planner.path"
    assert trace["source_path_points"] == len(path)
    assert trace["projection_ok"] is True

    trace_path = trace["path"]
    assert len(trace_path) == planning["counts"]["surface_trace_points"]
    assert len(trace_path) >= len(path)
    assert trace_path[0] == pytest.approx(EXPECTED_START, abs=2e-3)
    assert trace_path[-1] == pytest.approx(EXPECTED_GOAL, abs=2e-3)
    assert trace["length_m"] == pytest.approx(_length(trace_path), rel=1e-4)
    # The reference may coincide with a path that already follows the surface.
    assert abs(trace["length_m"] - planning["path_length_m"]) < 0.1

    for index, point in enumerate(trace_path):
        assert len(point) == 3, index
        assert all(math.isfinite(value) for value in point), index
        # Render-only projection must actually lie on the physical ground.
        assert point[2] == pytest.approx(_ground_z(point[0]), abs=1e-3), (
            index, point, _ground_z(point[0])
        )


def test_failed_plan_is_recorded_explicitly(planning):
    """A failure must be visible; the schema has no 'fake success' path."""
    assert "status" in planning
    if planning["status"] != "success":
        assert planning["error"] is not None
        assert planning["path"] == []
        raise AssertionError("planner failed: %s" % planning["error"])
