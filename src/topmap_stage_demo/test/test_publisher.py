"""Focused tests for the overlaid stage publisher.

Verifies the explicit requirements: zero XY offset for every stage (including
portal and cloud layers), the original cloud present by default with 13349
points and raw XYZ, no four stage-title markers, per-layer blend alphas, the
full 4-portal topology graph, and ROS ``--ros-args`` parsing.
"""
import math
import os
import struct

import pytest

from topmap_stage_demo import publisher as pub

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, "..", "data", "stages.json"))
PCD = os.path.normpath(os.path.join(HERE, "..", "data", "scene_h_ground_original_ramp.pcd"))

EXPECTED_POINTS = 13349
MESH_BOUNDS = {"x": (0.0, 30.4), "y": (0.0, 7.0), "z": (0.0, 2.0)}
MAX_Z_LIFT = 0.03


@pytest.fixture(scope="module")
def data():
    return pub.load_stages(DATA)


@pytest.fixture(scope="module")
def messages(data):
    return pub.build_all(data)


# ---------------------------------------------------------------------------
# zero XY offset / no translated copies
# ---------------------------------------------------------------------------
def test_stage_offsets_are_all_zero():
    assert set(pub.STAGE_OFFSETS) == {"stage1", "stage2", "stage3", "stage4"}
    for offset in pub.STAGE_OFFSETS.values():
        assert offset == (0.0, 0.0, 0.0)


def test_no_xy_translation_in_any_marker(messages):
    """Every published point stays in the single-scene XY box (no +35/+13)."""
    for topic, array in messages.items():
        for marker in array.markers:
            for point in marker.points:
                assert -1e-6 <= point.x <= MESH_BOUNDS["x"][1] + 1e-6, (topic, point.x)
                assert -1e-6 <= point.y <= MESH_BOUNDS["y"][1] + 1e-6, (topic, point.y)
                assert -1e-6 <= point.z <= MESH_BOUNDS["z"][1] + MAX_Z_LIFT + 1e-6, (topic, point.z)


def test_all_z_lifts_are_within_rendering_budget():
    lifts = [pub.MESH_FILL_Z, pub.EDGE_Z, pub.GROWN_FILL_Z, pub.FINAL_FILL_Z,
             pub.LINK_Z, pub.CHORD_Z, pub.CHAIN_Z, pub.NODE_Z, pub.LABEL_Z]
    assert max(lifts) <= MAX_Z_LIFT
    assert min(lifts) >= 0.0


def test_mesh_reproduced_at_original_vertex_xy(messages, data):
    """The stage1 mesh triangle vertices equal the raw mesh vertices in XY."""
    marker = messages[pub.TOPIC_STAGE1_MESH].markers[0]
    vertices = data["mesh"]["vertices"]
    faces = data["mesh"]["faces"]
    expected = []
    for face in faces:
        for vid in face:
            expected.append(vertices[vid])
    assert len(marker.points) == len(expected)
    for got, want in zip(marker.points, expected):
        assert got.x == pytest.approx(want[0], abs=1e-9)
        assert got.y == pytest.approx(want[1], abs=1e-9)


# ---------------------------------------------------------------------------
# original cloud: always present, raw, 13349 points
# ---------------------------------------------------------------------------
def test_source_cloud_default_path_resolves():
    resolved = pub.default_pcd_path()
    assert os.path.isfile(resolved), resolved
    assert os.path.isfile(pub.default_data_path())


def test_source_cloud_is_raw_and_complete():
    cloud = pub.build_source_cloud(PCD)
    assert cloud.header.frame_id == "map"
    assert cloud.width == EXPECTED_POINTS
    assert cloud.height == 1
    assert cloud.point_step == 12
    raw = pub.load_pcd_xyz(PCD)
    assert len(raw) == EXPECTED_POINTS
    for index, want in enumerate(raw):
        x, y, z = struct.unpack_from("<fff", cloud.data, index * 12)
        assert (x, y, z) == pytest.approx(tuple(want), abs=1e-6)


# ---------------------------------------------------------------------------
# no four titles, optional labels off by default in the topic set semantics
# ---------------------------------------------------------------------------
def test_no_stage_title_markers(messages):
    assert not any("title" in topic for topic in messages)
    for topic, array in messages.items():
        for marker in array.markers:
            assert "STAGE" not in marker.text.upper(), (topic, marker.text)


def test_expected_independent_layer_topics(messages):
    assert set(messages) == set(pub.MARKER_TOPICS)
    assert pub.TOPIC_SOURCE_CLOUD not in messages  # cloud is PointCloud2, not markers
    for topic in (pub.TOPIC_STAGE1_MESH, pub.TOPIC_STAGE1_EDGES, pub.TOPIC_STAGE2_REGIONS,
                  pub.TOPIC_STAGE3_REGIONS, pub.TOPIC_STAGE4_LINKS,
                  pub.TOPIC_STAGE4_REGION_NODES, pub.TOPIC_STAGE4_PORTAL_NODES,
                  pub.TOPIC_STAGE4_CHORDS, pub.TOPIC_STAGE4_CHAINS,
                  pub.TOPIC_REGION_LABELS, pub.TOPIC_PORTAL_LABELS):
        assert topic in messages


# ---------------------------------------------------------------------------
# blend style
# ---------------------------------------------------------------------------
def test_layer_blend_alphas(data, messages):
    mesh = messages[pub.TOPIC_STAGE1_MESH].markers[0]
    assert mesh.color.a == pytest.approx(0.06)
    for marker in messages[pub.TOPIC_STAGE2_REGIONS].markers:
        assert marker.color.a == pytest.approx(0.14)
    for marker in messages[pub.TOPIC_STAGE3_REGIONS].markers:
        assert marker.color.a == pytest.approx(0.44)
    # per-vertex colours stay at ~1.0 (only the blend trigger is slightly lower)
    for topic in (pub.TOPIC_STAGE1_MESH, pub.TOPIC_STAGE2_REGIONS, pub.TOPIC_STAGE3_REGIONS):
        for marker in messages[topic].markers:
            assert min(color.a for color in marker.colors) >= 0.99
            assert max(color.a for color in marker.colors) <= 1.0


# ---------------------------------------------------------------------------
# full topology graph, all 4 valid portals, never a planned path
# ---------------------------------------------------------------------------
def test_full_topology_graph_counts(data, messages):
    portals = data["topology"]["portals"]
    regions = data["topology"]["regions"]
    assert len(portals) == 4
    assert len(regions) == 5
    links = messages[pub.TOPIC_STAGE4_LINKS].markers[0]
    assert len(links.points) == 2 * len(portals) * 2  # R-P-R per portal
    chords = messages[pub.TOPIC_STAGE4_CHORDS].markers[0]
    assert len(chords.points) == 2 * len(portals)
    assert len(messages[pub.TOPIC_STAGE4_PORTAL_NODES].markers[0].points) == len(portals)
    assert len(messages[pub.TOPIC_STAGE4_REGION_NODES].markers[0].points) == len(regions)
    assert len(messages[pub.TOPIC_STAGE4_CHAINS].markers) == len(portals)


def test_portal_endpoints_match_json(data, messages):
    links = messages[pub.TOPIC_STAGE4_LINKS].markers[0]
    for portal in data["topology"]["portals"]:
        center = portal["center"]
        # the portal centre appears once per adjacent region (2x)
        hits = [p for p in links.points
                if p.x == pytest.approx(center[0], abs=1e-9)
                and p.y == pytest.approx(center[1], abs=1e-9)]
        assert len(hits) == 2


# ---------------------------------------------------------------------------
# ROS launch --ros-args must not break argparse
# ---------------------------------------------------------------------------
def test_ros_args_are_stripped_and_check_includes_cloud(capsys):
    # launch_ros appends the ROS section after the node's own arguments.
    code = pub.main(["stage_publisher",
                     "--data", DATA, "--pcd", PCD, "--check",
                     "--ros-args", "-p", "use_sim_time:=true",
                     "-r", "__node:=renamed"])
    assert code == 0
    out = capsys.readouterr().out
    assert "points=%d" % EXPECTED_POINTS in out
    assert "no DDS node created" in out
