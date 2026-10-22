"""Verify the installed MapArchive is the same synthetic mesh/topology as stages.json.

The production planner_node consumes the binary archive while the click bridge
projects onto stages.json.  If those two sources ever diverged, a click could
resolve to a different surface than the planner uses.  This lightweight reader
pins the archive's actual vertices, faces, final region triangle sets and
portal endpoints against the stage export.
"""
import json
import os
import struct

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, "..", "data"))
STAGES = os.path.join(DATA, "stages.json")
ARCHIVE = os.path.join(DATA, "stages.topmap.bin")


class Reader:
    def __init__(self, payload):
        self.payload = payload
        self.offset = 0

    def take(self, fmt):
        size = struct.calcsize(fmt)
        assert self.offset + size <= len(self.payload), "truncated archive payload"
        values = struct.unpack_from(fmt, self.payload, self.offset)
        self.offset += size
        return values[0] if len(values) == 1 else values

    def u32(self):
        return self.take("<I")

    def i32(self):
        return self.take("<i")

    def f32(self):
        return self.take("<f")

    def bool(self):
        return bool(self.take("<B"))

    def vec_i32(self):
        return [self.i32() for _ in range(self.u32())]

    def vec3f(self):
        return [self.f32(), self.f32(), self.f32()]


def _sections(path):
    with open(path, "rb") as handle:
        payload = handle.read()
    offset = 0
    magic, version, endian, count = struct.unpack_from("<IIII", payload, offset)
    offset += 16
    sections = {}
    for _ in range(count):
        name_length = struct.unpack_from("<I", payload, offset)[0]
        offset += 4
        name = payload[offset:offset + name_length].decode("ascii")
        offset += name_length
        byte_length = struct.unpack_from("<Q", payload, offset)[0]
        offset += 8
        sections[name] = payload[offset:offset + byte_length]
        offset += byte_length
    return magic, version, sections


def _regions(payload):
    reader = Reader(payload)
    regions = []
    for _ in range(reader.u32()):
        region = {"id": reader.i32(), "triangle_ids": reader.vec_i32()}
        reader.vec3f()
        for _ in range(7):  # area, min_z, max_z, mean_slope, max_slope, slope_variance, roughness
            reader.f32()
        reader.bool()
        reader.bool()
        region["portal_ids"] = reader.vec_i32()
        regions.append(region)
    return regions


def _portals(payload):
    reader = Reader(payload)
    portals = []
    for _ in range(reader.u32()):
        portal = {"id": reader.i32(), "region_a": reader.i32(), "region_b": reader.i32()}
        portal["boundary_chain"] = reader.vec_i32()
        portal["left"] = reader.vec3f()
        portal["right"] = reader.vec3f()
        portal["center"] = reader.vec3f()
        for _ in range(6):
            reader.f32()
        reader.bool()
        portals.append(portal)
    return portals


@pytest.fixture(scope="module")
def data():
    with open(STAGES, "r", encoding="utf-8") as handle:
        stages = json.load(handle)
    magic, version, sections = _sections(ARCHIVE)
    vertices_reader = Reader(sections["vertices"])
    vertices = [vertices_reader.vec3f() for _ in range(vertices_reader.u32())]
    faces_reader = Reader(sections["faces"])
    faces = [list(faces_reader.take("<iii")) for _ in range(faces_reader.u32())]
    return stages, magic, version, sections, vertices, faces


def test_archive_mesh_is_the_same_actual_mesh_as_stages_json(data):
    stages, magic, version, _sections_map, vertices, faces = data
    assert magic != 0 and version == 1
    assert len(vertices) == stages["counts"]["vertex_count"] == 13349
    assert len(faces) == stages["counts"]["face_count"] == 25800
    for index in (0, 1, 100, 5000, 13348):
        for archive_value, stages_value in zip(vertices[index], stages["mesh"]["vertices"][index]):
            assert archive_value == pytest.approx(stages_value, abs=1e-6)
    for index in (0, 1, 100, 5000, 25799):
        assert faces[index] == stages["mesh"]["faces"][index]


def test_archive_regions_and_portals_match_stage_topology(data):
    stages, _, _, sections, _, _ = data
    archive_regions = _regions(sections["regions"])
    archive_portals = _portals(sections["portals"])
    stage_regions = {region["id"]: region for region in stages["stages"]["final"]["regions"]}
    stage_portals = {portal["id"]: portal for portal in stages["topology"]["portals"]}

    assert {region["id"] for region in archive_regions} == set(stage_regions)
    archive_region_portals = {region["id"]: [] for region in archive_regions}
    for portal in archive_portals:
        archive_region_portals[portal["region_a"]].append(portal["id"])
        archive_region_portals[portal["region_b"]].append(portal["id"])
    for region in archive_regions:
        assert region["triangle_ids"] == stage_regions[region["id"]]["triangle_ids"]
        assert sorted(archive_region_portals[region["id"]]) == sorted(
            stage_regions[region["id"]]["portal_ids"]
        )

    assert {portal["id"] for portal in archive_portals} == set(stage_portals)
    for portal in archive_portals:
        stage_portal = stage_portals[portal["id"]]
        assert (portal["region_a"], portal["region_b"]) == (
            stage_portal["region_a"],
            stage_portal["region_b"],
        )
        assert portal["boundary_chain"] == stage_portal["boundary_chain"]
        for key in ("left", "right", "center"):
            for archive_value, stages_value in zip(portal[key], stage_portal[key]):
                assert archive_value == pytest.approx(stages_value, abs=2e-5)
