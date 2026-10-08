#pragma once

#include "topmap_core/binary_io.hpp"
#include "topmap_surface/mesh_types.hpp"

namespace topmap {

// T11 mesh payload, built on the T04 little-endian primitives. Layout:
//   u32 vertex_count, vertex_count * vector3f
//   u32 triangle_count, triangle_count * {
//     3 * i32 vertex_ids, vector3f centroid, vector3f normal,
//     f32 slope, f32 roughness, f32 curvature, f32 area, f32 min_clearance,
//     bool traversable }
// Edges and connected components are deliberately not stored: they are
// re-derived by TriangleMesh::fromTriangleNodes.
Status writeTriangleMesh(BinaryWriter& w, const TriangleMesh& mesh);

// Rebuilds a TriangleMesh from a payload produced by writeTriangleMesh. A short
// read, an out-of-range vertex index or any other structural defect yields a
// kSerializationIoError and no partially-valid mesh.
Result<TriangleMesh> readTriangleMesh(BinaryReader& r);

}  // namespace topmap
