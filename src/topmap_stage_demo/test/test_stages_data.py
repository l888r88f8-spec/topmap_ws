"""Contract tests against the actual generated ``stages.json``.

These assert the real pipeline numbers (13349 points / 25800 faces / 3 grown /
5 final / 4 valid portals / 1 component) instead of broad redundant checks.
"""
import os

import pytest

from topmap_stage_demo import publisher as pub

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, "..", "data", "stages.json"))


@pytest.fixture(scope="module")
def data():
    return pub.load_stages(DATA)


def test_actual_pipeline_counts(data):
    counts = data["counts"]
    assert counts["input_points"] == 13349
    assert counts["vertex_count"] == 13349
    assert counts["face_count"] == 25800
    assert counts["unique_edge_count"] == 39148
    assert counts["grown_region_count"] == 3
    assert counts["split_region_count"] == 5
    assert counts["final_region_count"] == 5
    assert counts["portal_valid_count"] == 4
    assert counts["final_portal_count"] == 4
    assert counts["connected_components"] == 1


def test_unique_edges_are_derived_from_faces(data):
    derived = pub.unique_edges(data["mesh"]["faces"])
    assert derived == [list(edge) for edge in data["mesh"]["unique_edges"]]
    assert len(derived) == data["counts"]["unique_edge_count"]


def test_mesh_spans_the_single_scene(data):
    vertices = data["mesh"]["vertices"]
    for axis, index in (("x", 0), ("y", 1), ("z", 2)):
        values = [v[index] for v in vertices]
        assert min(values) == pytest.approx(0.0)
        assert max(values) == pytest.approx({"x": 30.4, "y": 7.0, "z": 2.0}[axis])


def test_portals_connect_distinct_final_regions(data):
    region_ids = {region["id"] for region in data["topology"]["regions"]}
    assert len(region_ids) == 5
    for portal in data["topology"]["portals"]:
        assert portal["region_a"] in region_ids
        assert portal["region_b"] in region_ids
        assert portal["region_a"] != portal["region_b"]
        assert len(portal["boundary_chain"]) == len(portal["boundary_points"]) > 0


def test_validation_accepts_actual_data(data):
    pub.validate_stages(data)  # must not raise
