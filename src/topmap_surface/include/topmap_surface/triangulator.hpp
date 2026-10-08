#pragma once
#include <vector>

#include <Eigen/Core>

#include "topmap_core/error.hpp"
#include "topmap_surface/mesh_types.hpp"

namespace topmap {

enum class TriangulationMode { kDelaunay3DSurface = 0, kLayered2p5D = 1 };

struct TriangulationResult {
  MeshData mesh;
  TriangulationMode mode = TriangulationMode::kDelaunay3DSurface;
  int component_count = 0;
  int discarded_triangles = 0;
};

class SurfaceTriangulator {
 public:
  // R7: optional per-input-point features. When non-null their length must
  // equal points.size(); they are re-indexed to the output vertex order.
  static Result<TriangulationResult> triangulate(const std::vector<Eigen::Vector3f>& points,
                                                 const std::vector<Eigen::Vector3f>& normals,
                                                 const MeshBuildParams& params,
                                                 const std::vector<float>* roughness = nullptr,
                                                 const std::vector<float>* curvature = nullptr);
};

}  // namespace topmap
