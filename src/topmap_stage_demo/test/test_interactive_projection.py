"""Regression tests for 2D click -> actual mesh surface projection.

RViz 2D Pose Estimate / 2D Goal Pose publish a planar pose with z = 0.  The
interactive demo must not use that z and must not snap to the nearest 3D point:
it must resolve the click by XY triangle containment and barycentric height
interpolation on the *same* ``stages.json`` mesh.
"""
import math
import os

import pytest

from topmap_stage_demo.surface_projection import (
    FRAME_ID,
    ProjectionError,
    SurfaceMesh,
    load_surface_mesh,
    project_click,
    validate_frame,
)

HERE = os.path.dirname(os.path.abspath(__file__))
STAGES = os.path.normpath(os.path.join(HERE, "..", "data", "stages.json"))


@pytest.fixture(scope="module")
def mesh():
    return load_surface_mesh(STAGES)


def test_ground_platform_and_ramp_use_barycentric_surface_height(mesh):
    # Flat H ground.
    assert project_click(mesh, 2.0, 3.5).z == pytest.approx(0.0, abs=1e-4)
    # Upper platform: a z=0 RViz click must become the real 2.0 m surface.
    platform = project_click(mesh, 28.0, 3.5)
    assert platform.z == pytest.approx(2.0, abs=1e-4)
    # Ramp: actual interpolated face height, not z=0 and not a 3D nearest probe.
    ramp = project_click(mesh, 22.0, 3.5)
    expected = 2.0 * (22.0 - 17.0) / (26.4 - 17.0)
    assert ramp.z == pytest.approx(expected, abs=0.03)
    assert ramp.z > 0.9


def test_shared_edge_has_a_deterministic_stable_choice(mesh):
    # (0.05, 0.05) lies on the shared diagonal of the two triangles in cell 0.
    first = project_click(mesh, 0.05, 0.05)
    for _ in range(10):
        again = project_click(mesh, 0.05, 0.05)
        assert again == first
    assert first.z == pytest.approx(0.0, abs=1e-6)


def test_outside_nan_and_unsupported_frames_are_rejected(mesh):
    with pytest.raises(ProjectionError, match="outside|no triangle"):
        project_click(mesh, 100.0, 100.0)
    with pytest.raises(ProjectionError, match="finite"):
        project_click(mesh, math.nan, 3.5)
    with pytest.raises(ProjectionError, match="finite"):
        project_click(mesh, 2.0, math.inf)
    with pytest.raises(ProjectionError, match="outside|no triangle"):
        project_click(mesh, -2.0, 3.5)

    assert validate_frame(FRAME_ID) == FRAME_ID
    for bad in ("odom", "", "map "):
        with pytest.raises(ProjectionError, match="frame"):
            validate_frame(bad)


def test_hole_is_rejected_instead_of_snapping_to_a_far_triangle():
    # A square ring (outer 2x2, inner hole 0.5..1.5) made of four strips.
    vertices = [
        [0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 2.0, 0.0], [0.0, 2.0, 0.0],
        [0.5, 0.5, 0.0], [1.5, 0.5, 0.0], [1.5, 1.5, 0.0], [0.5, 1.5, 0.0],
    ]
    faces = [
        [0, 1, 5], [0, 5, 4],   # bottom strip
        [1, 2, 6], [1, 6, 5],   # right strip
        [2, 3, 7], [2, 7, 6],   # top strip
        [3, 0, 4], [3, 4, 7],   # left strip
    ]
    ring = SurfaceMesh(vertices, faces)
    assert project_click(ring, 0.25, 1.0).z == pytest.approx(0.0, abs=1e-6)
    with pytest.raises(ProjectionError, match="outside|no triangle"):
        project_click(ring, 1.0, 1.0)


def test_mesh_loaded_once_has_real_stages_geometry(mesh):
    assert mesh.triangle_count == 25800
    assert mesh.vertex_count == 13349
    # Projection never mutates the mesh or invents triangles.
    before = (mesh.triangle_count, mesh.vertex_count)
    project_click(mesh, 22.0, 3.5)
    assert (mesh.triangle_count, mesh.vertex_count) == before
