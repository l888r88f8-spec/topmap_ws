#include "topmap_surface/mesh_serialization.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace topmap {
namespace {

// Bytes of one serialized triangle:
//   3 * i32 vertex_ids + 3 * f32 centroid + 3 * f32 normal + 5 * f32 attrs + 1 bool
constexpr std::size_t kTriangleBytes = 3u * 4u + 3u * 4u + 3u * 4u + 5u * 4u + 1u;
constexpr std::size_t kVertexBytes = 3u * 4u;
constexpr std::uint32_t kMaxCount = std::numeric_limits<std::uint32_t>::max();

Status IoError(const std::string& message) {
  return Status::Error(ErrorCode::kSerializationIoError, message);
}

}  // namespace

Status writeTriangleMesh(BinaryWriter& w, const TriangleMesh& mesh) {
  if (mesh.vertices().size() > kMaxCount || mesh.size() > kMaxCount) {
    return IoError("mesh is too large for the serialized count fields");
  }
  w.writeU32(static_cast<std::uint32_t>(mesh.vertices().size()));
  for (const Eigen::Vector3f& v : mesh.vertices()) w.writeVector3f(v);

  w.writeU32(static_cast<std::uint32_t>(mesh.size()));
  for (const TriangleNode& t : mesh.triangles()) {
    for (PointId vid : t.vertex_ids) w.writeI32(vid);
    w.writeVector3f(t.centroid);
    w.writeVector3f(t.normal);
    w.writeF32(t.slope);
    w.writeF32(t.roughness);
    w.writeF32(t.curvature);
    w.writeF32(t.area);
    w.writeF32(t.min_clearance);
    w.writeBool(t.traversable);
  }
  return Status::Ok();
}

Result<TriangleMesh> readTriangleMesh(BinaryReader& r) {
  std::uint32_t vertex_count = 0;
  if (!r.readU32(vertex_count)) return IoError("short read on vertex count");
  // Bound the count by the bytes actually available before touching the heap.
  if (static_cast<std::size_t>(vertex_count) > r.remaining() / kVertexBytes) {
    return IoError("vertex count exceeds the remaining payload");
  }

  std::vector<Eigen::Vector3f> vertices;
  vertices.reserve(vertex_count);
  for (std::uint32_t i = 0; i < vertex_count; ++i) {
    Eigen::Vector3f v;
    if (!r.readVector3f(v)) return IoError("short read on vertex data");
    vertices.push_back(v);
  }

  std::uint32_t triangle_count = 0;
  if (!r.readU32(triangle_count)) return IoError("short read on triangle count");
  if (static_cast<std::size_t>(triangle_count) > r.remaining() / kTriangleBytes) {
    return IoError("triangle count exceeds the remaining payload");
  }

  std::vector<TriangleNode> triangles;
  triangles.reserve(triangle_count);
  for (std::uint32_t i = 0; i < triangle_count; ++i) {
    TriangleNode t;
    for (int k = 0; k < 3; ++k) {
      std::int32_t vid = 0;
      if (!r.readI32(vid)) return IoError("short read on triangle vertex ids");
      t.vertex_ids[k] = vid;
    }
    if (!r.readVector3f(t.centroid)) return IoError("short read on triangle centroid");
    if (!r.readVector3f(t.normal)) return IoError("short read on triangle normal");
    if (!r.readF32(t.slope) || !r.readF32(t.roughness) || !r.readF32(t.curvature) ||
        !r.readF32(t.area) || !r.readF32(t.min_clearance) || !r.readBool(t.traversable)) {
      return IoError("short read on triangle attributes");
    }
    for (PointId vid : t.vertex_ids) {
      if (vid < 0 || vid >= static_cast<PointId>(vertex_count)) {
        return IoError("triangle references a vertex id outside the vertex table");
      }
    }
    triangles.push_back(t);
  }

  if (triangles.empty()) return IoError("payload contains no triangles");

  Result<TriangleMesh> mesh = TriangleMesh::fromTriangleNodes(std::move(vertices), std::move(triangles));
  if (!mesh.ok()) return IoError(mesh.status().message);
  return mesh;
}

}  // namespace topmap
