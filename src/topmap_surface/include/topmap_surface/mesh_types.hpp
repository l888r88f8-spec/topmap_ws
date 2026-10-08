#pragma once
#include <array>
#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "topmap_core/error.hpp"
#include "topmap_core/ids.hpp"

namespace topmap {

struct TriangleEdge {
  TriangleId neighbor = kInvalidId;
  PointId v0 = kInvalidId;
  PointId v1 = kInvalidId;
  bool traversable = false;
};

struct TriangleNode {
  TriangleId id = kInvalidId;
  std::array<PointId, 3> vertex_ids{kInvalidId, kInvalidId, kInvalidId};
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
  float slope = 0.0f, roughness = 0.0f, curvature = 0.0f, area = 0.0f;
  // R8: geodesic distance to the nearest non-traversable triangle or mesh
  // boundary. Independent of the Region partition.
  float min_clearance = 0.0f;
  bool traversable = false;
  std::array<TriangleEdge, 3> edges{};
  // Adjacent triangle ids (edges order, ascending, boundary edges omitted).
  std::vector<TriangleId> neighbors() const;
};

// R7: vertex features are optional; when present their length must equal
// vertices.size().
struct MeshData {
  std::vector<Eigen::Vector3f> vertices;
  std::vector<std::array<PointId, 3>> faces;
  std::vector<float> vertex_roughness;   // may be empty
  std::vector<float> vertex_curvature;   // may be empty
};

struct MeshBuildParams {
  float max_edge_length = 0.30f;
  float max_normal_deviation_deg = 35.0f;
  float component_radius = 0.20f;
  bool prefer_layered = false;
  // R9: non-manifold edges no longer fail the whole graph; only a discarded
  // face ratio above this threshold does.
  float max_discarded_face_ratio = 0.05f;
};

class TriangleMesh {
 public:
  TriangleMesh() = default;
  static Result<TriangleMesh> fromMeshData(const MeshData& data, const MeshBuildParams& params);
  static Result<TriangleMesh> fromTriangleNodes(std::vector<Eigen::Vector3f> vertices,
                                                std::vector<TriangleNode> triangles);

  std::size_t size() const;
  const std::vector<Eigen::Vector3f>& vertices() const;
  const std::vector<TriangleNode>& triangles() const;
  std::vector<TriangleNode>& triangles();
  const TriangleNode& triangle(TriangleId id) const;
  std::vector<TriangleId> neighbors(TriangleId id) const;
  Result<std::vector<TriangleId>> kRing(TriangleId id, int rings) const;
  const std::vector<int>& componentIds() const;
  int componentId(TriangleId id) const;
  int discardedFaces() const;

 private:
  std::vector<Eigen::Vector3f> vertices_;
  std::vector<TriangleNode> triangles_;
  std::vector<int> component_ids_;
  int discarded_faces_ = 0;
};

// R8: geodesic distance to the boundary of the given Region. Distinct from
// TriangleNode.min_clearance and owned by the Region/Portal tasks
// (T14/T16/T22); declared here to complete the frozen A4.6 surface interface.
std::vector<float> regionBoundaryDistance(const TriangleMesh& mesh,
                                          const std::vector<TriangleId>& region_triangles);

}  // namespace topmap
