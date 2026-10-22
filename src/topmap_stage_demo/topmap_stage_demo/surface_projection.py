#!/usr/bin/env python3
"""Resolve a 2D RViz click to the real synthetic mesh surface.

RViz's 2D Pose Estimate / 2D Goal Pose tools publish a planar pose whose ``z``
is normally 0, even when the cursor is over the 2.0 m upper platform.  This
module never trusts that z and never uses a 3D nearest-point heuristic:

* build a spatial index over the *actual* ``stages.json`` XY triangles once;
* test XY triangle containment with barycentric coordinates;
* interpolate the triangle's real z at the click;
* reject non-finite input, unsupported frames and any point in a hole or
  outside the mesh instead of snapping to a far triangle.

When several triangles contain the same XY point (a shared edge, or overlapping
floors in another scene), the deterministic rule is: highest interpolated z
first, then lowest triangle index.  The current synthetic scene has one layer,
so this only makes shared-edge choices stable.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

FRAME_ID = "map"
CONTAINMENT_EPSILON = 1e-7
MAX_GRID_CELLS = 512


class ProjectionError(ValueError):
    """A click cannot be resolved to one actual mesh surface point."""


@dataclass(frozen=True)
class ProjectedPoint:
    x: float
    y: float
    z: float
    triangle_id: int

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)


def _finite(value: float) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def validate_frame(frame_id: str) -> str:
    """Only the demo's ``map`` frame is accepted; no TF lookup is performed."""
    if frame_id != FRAME_ID:
        raise ProjectionError(
            "click frame must be %r, got %r" % (FRAME_ID, frame_id)
        )
    return FRAME_ID


class SurfaceMesh:
    """XY-indexed triangle surface loaded once from the stage export."""

    def __init__(self, vertices: Sequence[Sequence[float]], faces: Sequence[Sequence[int]]):
        if not vertices:
            raise ProjectionError("mesh has no vertices")
        if not faces:
            raise ProjectionError("mesh has no faces")

        self._vertices: List[Tuple[float, float, float]] = []
        for index, vertex in enumerate(vertices):
            if len(vertex) != 3 or not all(_finite(value) for value in vertex):
                raise ProjectionError("mesh vertex %d is not a finite xyz" % index)
            self._vertices.append((float(vertex[0]), float(vertex[1]), float(vertex[2])))

        self._faces: List[Tuple[int, int, int]] = []
        triangles = []
        min_x = min_y = math.inf
        max_x = max_y = -math.inf
        for face_index, face in enumerate(faces):
            if len(face) != 3:
                raise ProjectionError("mesh face %d does not have three indices" % face_index)
            ids = tuple(int(value) for value in face)
            for vertex_id in ids:
                if not 0 <= vertex_id < len(self._vertices):
                    raise ProjectionError(
                        "mesh face %d references out-of-range vertex %d"
                        % (face_index, vertex_id)
                    )
            ax, ay, az = self._vertices[ids[0]]
            bx, by, bz = self._vertices[ids[1]]
            cx, cy, cz = self._vertices[ids[2]]
            denominator = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
            if abs(denominator) < 1e-14:
                # Vertical/degenerate XY triangle cannot be a 2D click target.
                continue
            tri = (
                face_index,
                ax, ay, az,
                bx, by, bz,
                cx, cy, cz,
                denominator,
                min(ax, bx, cx), max(ax, bx, cx),
                min(ay, by, cy), max(ay, by, cy),
            )
            triangles.append(tri)
            self._faces.append(ids)
            min_x = min(min_x, tri[11])
            max_x = max(max_x, tri[12])
            min_y = min(min_y, tri[13])
            max_y = max(max_y, tri[14])

        if not triangles:
            raise ProjectionError("mesh has no non-degenerate XY triangles")

        self._triangles = triangles
        self._min_x, self._max_x = min_x, max_x
        self._min_y, self._max_y = min_y, max_y
        span = max(max_x - min_x, max_y - min_y, 1e-6)
        self._cell_size = max(span / MAX_GRID_CELLS, 0.10)
        self._nx = max(1, int(math.floor((max_x - min_x) / self._cell_size)) + 1)
        self._ny = max(1, int(math.floor((max_y - min_y) / self._cell_size)) + 1)
        self._grid: List[List[int]] = [[] for _ in range(self._nx * self._ny)]
        for triangle_index, tri in enumerate(triangles):
            ix0 = self._cell_x(tri[11])
            ix1 = self._cell_x(tri[12])
            iy0 = self._cell_y(tri[13])
            iy1 = self._cell_y(tri[14])
            for iy in range(iy0, iy1 + 1):
                row = iy * self._nx
                for ix in range(ix0, ix1 + 1):
                    self._grid[row + ix].append(triangle_index)

    @property
    def vertex_count(self) -> int:
        return len(self._vertices)

    @property
    def triangle_count(self) -> int:
        return len(self._triangles)

    def _cell_x(self, x: float) -> int:
        index = int(math.floor((x - self._min_x) / self._cell_size))
        return min(max(index, 0), self._nx - 1)

    def _cell_y(self, y: float) -> int:
        index = int(math.floor((y - self._min_y) / self._cell_size))
        return min(max(index, 0), self._ny - 1)

    def project(self, x: float, y: float) -> ProjectedPoint:
        if not _finite(x) or not _finite(y):
            raise ProjectionError("click x/y must be finite")
        x = float(x)
        y = float(y)
        if not (self._min_x - CONTAINMENT_EPSILON <= x <= self._max_x + CONTAINMENT_EPSILON
                and self._min_y - CONTAINMENT_EPSILON <= y <= self._max_y + CONTAINMENT_EPSILON):
            raise ProjectionError(
                "click (%.6f, %.6f) is outside the map mesh" % (x, y)
            )

        candidates = self._grid[self._cell_y(y) * self._nx + self._cell_x(x)]
        best = None
        for triangle_index in candidates:
            tri = self._triangles[triangle_index]
            (triangle_id, ax, ay, az, bx, by, bz, cx, cy, cz, denominator, *_rest) = tri
            u = ((by - cy) * (x - cx) + (cx - bx) * (y - cy)) / denominator
            v = ((cy - ay) * (x - cx) + (ax - cx) * (y - cy)) / denominator
            w = 1.0 - u - v
            if u < -CONTAINMENT_EPSILON or v < -CONTAINMENT_EPSILON or w < -CONTAINMENT_EPSILON:
                continue
            # Clamp only the tiny boundary rounding error, never an outside point.
            u = max(0.0, u)
            v = max(0.0, v)
            w = max(0.0, w)
            total = u + v + w
            if total <= 0.0:
                continue
            u, v, w = u / total, v / total, w / total
            z = u * az + v * bz + w * cz
            key = (z, -triangle_id)
            if best is None or key > best[0]:
                best = (key, ProjectedPoint(x, y, float(z), triangle_id))

        if best is None:
            raise ProjectionError(
                "click (%.6f, %.6f) is outside the map mesh (no containing triangle)"
                % (x, y)
            )
        return best[1]


def load_surface_mesh(stages_path: str) -> SurfaceMesh:
    with open(stages_path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    mesh = data.get("mesh") if isinstance(data, dict) else None
    if not isinstance(mesh, dict):
        raise ProjectionError("%s: missing mesh object" % stages_path)
    return SurfaceMesh(mesh.get("vertices") or [], mesh.get("faces") or [])


def project_click(mesh: SurfaceMesh, x: float, y: float,
                  frame_id: str | None = None) -> ProjectedPoint:
    if frame_id is not None:
        validate_frame(frame_id)
    return mesh.project(x, y)
