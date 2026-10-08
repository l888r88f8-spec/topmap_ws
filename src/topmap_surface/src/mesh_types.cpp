#include "topmap_surface/mesh_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include "topmap_core/geometry.hpp"

namespace topmap {
namespace {

constexpr float kMinArea = 1e-9f;

// Rotate a triangle's `vertex_ids` and `edges` in lock step so that edge slot k
// still corresponds to (vertex_ids[k], vertex_ids[(k+1)%3]).
void rotateTriangle(TriangleNode& t, int shift) {
  shift = ((shift % 3) + 3) % 3;
  if (shift == 0) return;
  const std::array<PointId, 3> v = t.vertex_ids;
  const std::array<TriangleEdge, 3> e = t.edges;
  for (int k = 0; k < 3; ++k) {
    t.vertex_ids[k] = v[(k + shift) % 3];
    t.edges[k] = e[(k + shift) % 3];
  }
}

// Fill `edges[k].neighbor` from shared mesh edges. The two smallest triangle ids
// on a non-manifold edge (more than two incident triangles) keep their
// adjacency; every further incident face is counted as discarded (R9). Returns
// the number of discarded faces. Boundary edges keep neighbor == kInvalidId.
int buildAdjacency(std::vector<TriangleNode>& triangles) {
  for (auto& t : triangles) {
    for (int k = 0; k < 3; ++k) {
      t.edges[k] = TriangleEdge{};
      t.edges[k].v0 = t.vertex_ids[k];
      t.edges[k].v1 = t.vertex_ids[(k + 1) % 3];
    }
  }

  std::map<std::pair<PointId, PointId>, std::vector<std::pair<int, int>>> edge_map;
  for (int ti = 0; ti < static_cast<int>(triangles.size()); ++ti) {
    const TriangleNode& t = triangles[ti];
    for (int k = 0; k < 3; ++k) {
      const PointId a = t.vertex_ids[k];
      const PointId b = t.vertex_ids[(k + 1) % 3];
      edge_map[{std::min(a, b), std::max(a, b)}].push_back({ti, k});
    }
  }

  std::set<int> discarded;
  for (auto& entry : edge_map) {
    std::vector<std::pair<int, int>>& occurrences = entry.second;
    if (occurrences.size() < 2) continue;  // boundary edge
    std::sort(occurrences.begin(), occurrences.end());
    const int t0 = occurrences[0].first, s0 = occurrences[0].second;
    const int t1 = occurrences[1].first, s1 = occurrences[1].second;
    triangles[t0].edges[s0].neighbor = t1;
    triangles[t1].edges[s1].neighbor = t0;
    for (std::size_t k = 2; k < occurrences.size(); ++k) {
      discarded.insert(occurrences[k].first);
    }
  }

  // Keep the frozen invariant that the first edge slot holds an interior edge
  // when the triangle has one, so consumers may rely on edges[0] being the
  // primary adjacency.
  for (auto& t : triangles) {
    int first_shared = -1;
    for (int k = 0; k < 3; ++k) {
      if (t.edges[k].neighbor != kInvalidId) {
        first_shared = k;
        break;
      }
    }
    if (first_shared > 0) rotateTriangle(t, first_shared);
  }
  return static_cast<int>(discarded.size());
}

// Breadth-first component labelling over shared-edge adjacency.
std::vector<int> computeComponents(const std::vector<TriangleNode>& triangles) {
  std::vector<int> component(triangles.size(), -1);
  int next_component = 0;
  std::vector<int> stack;
  for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
    if (component[i] != -1) continue;
    component[i] = next_component;
    stack.push_back(i);
    while (!stack.empty()) {
      const int cur = stack.back();
      stack.pop_back();
      for (const TriangleEdge& e : triangles[cur].edges) {
        if (e.neighbor != kInvalidId && component[e.neighbor] == -1) {
          component[e.neighbor] = next_component;
          stack.push_back(e.neighbor);
        }
      }
    }
    ++next_component;
  }
  return component;
}

}  // namespace

std::vector<TriangleId> TriangleNode::neighbors() const {
  std::vector<TriangleId> result;
  for (const TriangleEdge& e : edges) {
    if (e.neighbor != kInvalidId) result.push_back(e.neighbor);
  }
  std::sort(result.begin(), result.end());
  return result;
}

Result<TriangleMesh> TriangleMesh::fromMeshData(const MeshData& data,
                                                const MeshBuildParams& params) {
  if (data.faces.empty()) {
    return Status::Error(ErrorCode::kMeshEmpty, "mesh has no faces");
  }
  if (!data.vertex_roughness.empty() && data.vertex_roughness.size() != data.vertices.size()) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "vertex_roughness length does not match vertex count");
  }
  if (!data.vertex_curvature.empty() && data.vertex_curvature.size() != data.vertices.size()) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "vertex_curvature length does not match vertex count");
  }

  const PointId vertex_count = static_cast<PointId>(data.vertices.size());
  std::vector<TriangleNode> triangles;
  triangles.reserve(data.faces.size());
  int degenerate_faces = 0;
  for (const std::array<PointId, 3>& face : data.faces) {
    for (PointId vid : face) {
      if (vid < 0 || vid >= vertex_count) {
        return Status::Error(ErrorCode::kInvalidArgument, "face references an invalid vertex id");
      }
    }
    if (face[0] == face[1] || face[1] == face[2] || face[0] == face[2]) {
      ++degenerate_faces;
      continue;
    }
    const Eigen::Vector3f a = data.vertices[face[0]];
    const Eigen::Vector3f b = data.vertices[face[1]];
    const Eigen::Vector3f c = data.vertices[face[2]];
    const float area = triangleArea(a, b, c);
    if (!(area > kMinArea)) {
      ++degenerate_faces;
      continue;
    }

    TriangleNode node;
    node.id = static_cast<TriangleId>(triangles.size());
    node.vertex_ids = face;
    node.centroid = (a + b + c) / 3.0f;
    node.normal = orientNormalUp(triangleNormal(a, b, c), Eigen::Vector3f::UnitZ());
    node.area = area;
    node.slope = slopeFromNormal(node.normal);
    if (!data.vertex_roughness.empty()) {
      node.roughness = (data.vertex_roughness[face[0]] + data.vertex_roughness[face[1]] +
                        data.vertex_roughness[face[2]]) /
                       3.0f;
    }
    if (!data.vertex_curvature.empty()) {
      node.curvature = (data.vertex_curvature[face[0]] + data.vertex_curvature[face[1]] +
                        data.vertex_curvature[face[2]]) /
                       3.0f;
    }
    triangles.push_back(node);
  }

  const int non_manifold_faces = buildAdjacency(triangles);
  const int discarded = degenerate_faces + non_manifold_faces;
  const float discarded_ratio =
      static_cast<float>(discarded) / static_cast<float>(data.faces.size());
  if (discarded_ratio > params.max_discarded_face_ratio) {
    return Status::Error(ErrorCode::kMeshNotManifold,
                         "discarded face ratio exceeds max_discarded_face_ratio");
  }
  if (triangles.empty()) {
    return Status::Error(ErrorCode::kMeshEmpty, "mesh has no usable faces");
  }

  TriangleMesh mesh;
  mesh.vertices_ = data.vertices;
  mesh.triangles_ = std::move(triangles);
  mesh.discarded_faces_ = discarded;
  mesh.component_ids_ = computeComponents(mesh.triangles_);
  return mesh;
}

Result<TriangleMesh> TriangleMesh::fromTriangleNodes(std::vector<Eigen::Vector3f> vertices,
                                                     std::vector<TriangleNode> triangles) {
  if (triangles.empty()) {
    return Status::Error(ErrorCode::kMeshEmpty, "mesh has no triangles");
  }
  const PointId vertex_count = static_cast<PointId>(vertices.size());
  for (const TriangleNode& t : triangles) {
    for (PointId vid : t.vertex_ids) {
      if (vid < 0 || vid >= vertex_count) {
        return Status::Error(ErrorCode::kInvalidArgument, "triangle references an invalid vertex id");
      }
    }
  }
  for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
    triangles[i].id = static_cast<TriangleId>(i);
  }

  TriangleMesh mesh;
  mesh.vertices_ = std::move(vertices);
  mesh.triangles_ = std::move(triangles);
  mesh.discarded_faces_ = buildAdjacency(mesh.triangles_);
  mesh.component_ids_ = computeComponents(mesh.triangles_);
  return mesh;
}

std::size_t TriangleMesh::size() const { return triangles_.size(); }

const std::vector<Eigen::Vector3f>& TriangleMesh::vertices() const { return vertices_; }

const std::vector<TriangleNode>& TriangleMesh::triangles() const { return triangles_; }

std::vector<TriangleNode>& TriangleMesh::triangles() { return triangles_; }

const TriangleNode& TriangleMesh::triangle(TriangleId id) const { return triangles_[id]; }

std::vector<TriangleId> TriangleMesh::neighbors(TriangleId id) const {
  if (id < 0 || id >= static_cast<TriangleId>(triangles_.size())) return {};
  return triangles_[id].neighbors();
}

Result<std::vector<TriangleId>> TriangleMesh::kRing(TriangleId id, int rings) const {
  if (id < 0 || id >= static_cast<TriangleId>(triangles_.size()) || rings < 0) {
    return Status::Error(ErrorCode::kInvalidArgument, "kRing called with an invalid triangle id or ring count");
  }
  std::vector<int> distance(triangles_.size(), -1);
  std::queue<TriangleId> queue;
  distance[id] = 0;
  queue.push(id);
  std::vector<TriangleId> result;
  while (!queue.empty()) {
    const TriangleId current = queue.front();
    queue.pop();
    if (distance[current] >= rings) continue;
    for (TriangleId neighbor : triangles_[current].neighbors()) {
      if (distance[neighbor] == -1) {
        distance[neighbor] = distance[current] + 1;
        queue.push(neighbor);
        result.push_back(neighbor);
      }
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

const std::vector<int>& TriangleMesh::componentIds() const { return component_ids_; }

int TriangleMesh::componentId(TriangleId id) const {
  if (id < 0 || id >= static_cast<TriangleId>(component_ids_.size())) return kInvalidId;
  return component_ids_[id];
}

int TriangleMesh::discardedFaces() const { return discarded_faces_; }

}  // namespace topmap
