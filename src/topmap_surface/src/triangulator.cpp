#include "topmap_surface/triangulator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>

#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include "topmap_core/geometry.hpp"

namespace topmap {
namespace {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using VertexBase = CGAL::Triangulation_vertex_base_with_info_2<int, Kernel>;
using FaceBase = CGAL::Triangulation_face_base_2<Kernel>;
using Tds = CGAL::Triangulation_data_structure_2<VertexBase, FaceBase>;
using Delaunay2 = CGAL::Delaunay_triangulation_2<Kernel, Tds>;

constexpr float kMinArea = 1e-12f;
constexpr float kMinNorm = 1e-12f;
constexpr float kPlaneRmsSplit = 0.10f;  // split a component into height bands above this
constexpr float kBandHeight = 0.5f;
constexpr float kBandOverlap = 0.1f;
constexpr float kPi = 3.14159265358979323846f;

struct IndexTriple {
  std::array<int, 3> v{0, 0, 0};
};

// Deterministic position-independent hash key for a uniform grid cell.
long long cellKey(int x, int y, int z) {
  return (static_cast<long long>(x) * 73856093LL) ^
         (static_cast<long long>(y) * 19349663LL) ^
         (static_cast<long long>(z) * 83492791LL);
}

class UnionFind {
 public:
  explicit UnionFind(int n) : parent_(static_cast<std::size_t>(n)) {
    for (int i = 0; i < n; ++i) parent_[static_cast<std::size_t>(i)] = i;
  }
  int find(int x) {
    while (parent_[static_cast<std::size_t>(x)] != x) {
      parent_[static_cast<std::size_t>(x)] = parent_[static_cast<std::size_t>(parent_[static_cast<std::size_t>(x)])];
      x = parent_[static_cast<std::size_t>(x)];
    }
    return x;
  }
  void unite(int a, int b) {
    const int ra = find(a), rb = find(b);
    if (ra != rb) parent_[static_cast<std::size_t>(ra)] = rb;
  }

 private:
  std::vector<int> parent_;
};

// 3D radius connected components. A uniform cell hash of side `radius` only
// requires checking the 27 neighbouring cells because any two points within
// `radius` differ by at most one cell per axis.
std::vector<std::vector<int>> connectedComponents(const std::vector<Eigen::Vector3f>& points,
                                                 float radius) {
  const int n = static_cast<int>(points.size());
  std::vector<std::vector<int>> groups;
  if (n == 0) return groups;
  if (!(radius > 0.0f) || !std::isfinite(radius)) radius = 0.2f;

  std::unordered_map<long long, std::vector<int>> grid;
  grid.reserve(static_cast<std::size_t>(n) * 2);
  std::vector<std::array<int, 3>> cells(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const std::array<int, 3> c{static_cast<int>(std::floor(points[i].x() / radius)),
                               static_cast<int>(std::floor(points[i].y() / radius)),
                               static_cast<int>(std::floor(points[i].z() / radius))};
    cells[static_cast<std::size_t>(i)] = c;
    grid[cellKey(c[0], c[1], c[2])].push_back(i);
  }

  UnionFind uf(n);
  const float r2 = radius * radius;
  for (int i = 0; i < n; ++i) {
    const std::array<int, 3>& c = cells[static_cast<std::size_t>(i)];
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const auto it = grid.find(cellKey(c[0] + dx, c[1] + dy, c[2] + dz));
          if (it == grid.end()) continue;
          for (int j : it->second) {
            if (j <= i) continue;
            if ((points[i] - points[j]).squaredNorm() <= r2) uf.unite(i, j);
          }
        }
      }
    }
  }

  std::unordered_map<int, int> root_to_group;
  for (int i = 0; i < n; ++i) {
    const int root = uf.find(i);
    auto it = root_to_group.find(root);
    if (it == root_to_group.end()) {
      root_to_group.emplace(root, static_cast<int>(groups.size()));
      groups.push_back({i});
    } else {
      groups[static_cast<std::size_t>(it->second)].push_back(i);
    }
  }
  for (auto& group : groups) std::sort(group.begin(), group.end());
  return groups;
}

struct PlaneFit {
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
  float rms = 0.0f;
};

PlaneFit fitPlane(const std::vector<Eigen::Vector3f>& points, const std::vector<int>& indices) {
  PlaneFit fit;
  if (indices.empty()) return fit;
  for (int i : indices) fit.centroid += points[static_cast<std::size_t>(i)];
  fit.centroid /= static_cast<float>(indices.size());

  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  for (int i : indices) {
    const Eigen::Vector3f d = points[static_cast<std::size_t>(i)] - fit.centroid;
    covariance.noalias() += d * d.transpose();
  }
  covariance /= static_cast<float>(indices.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  fit.normal = solver.eigenvectors().col(0);
  if (!fit.normal.allFinite() || fit.normal.norm() <= kMinNorm) {
    fit.normal = Eigen::Vector3f::UnitZ();
  } else {
    fit.normal.normalize();
    if (fit.normal.z() < 0.0f) fit.normal = -fit.normal;
  }
  const float smallest = solver.eigenvalues()(0);
  fit.rms = smallest > 0.0f ? std::sqrt(smallest) : 0.0f;
  return fit;
}

void planeBasis(const Eigen::Vector3f& normal, Eigen::Vector3f* u, Eigen::Vector3f* v) {
  const Eigen::Vector3f axis =
      std::fabs(normal.x()) < 0.9f ? Eigen::Vector3f::UnitX() : Eigen::Vector3f::UnitY();
  *u = normal.cross(axis);
  if (!u->allFinite() || u->norm() <= kMinNorm) *u = Eigen::Vector3f::UnitX();
  u->normalize();
  *v = normal.cross(*u);
  if (!v->allFinite() || v->norm() <= kMinNorm) *v = Eigen::Vector3f::UnitY();
  v->normalize();
}

float normalDeviationDeg(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  if (!a.allFinite() || !b.allFinite() || a.norm() <= kMinNorm || b.norm() <= kMinNorm) return 0.0f;
  const float cosine = clampf(a.normalized().dot(b.normalized()), -1.0f, 1.0f);
  return std::acos(cosine) * 180.0f / kPi;
}

bool triangleAccepted(const std::vector<Eigen::Vector3f>& points,
                      const std::vector<Eigen::Vector3f>& normals,
                      const std::array<int, 3>& tri, const MeshBuildParams& params,
                      std::array<int, 3>* key) {
  const int i = tri[0], j = tri[1], k = tri[2];
  if (i == j || j == k || i == k) return false;
  const Eigen::Vector3f& a = points[static_cast<std::size_t>(i)];
  const Eigen::Vector3f& b = points[static_cast<std::size_t>(j)];
  const Eigen::Vector3f& c = points[static_cast<std::size_t>(k)];
  if (!(triangleArea(a, b, c) > kMinArea)) return false;

  const float max_edge = params.max_edge_length;
  if ((a - b).norm() > max_edge || (b - c).norm() > max_edge || (c - a).norm() > max_edge) {
    return false;
  }

  const Eigen::Vector3f triangle_normal =
      orientNormalUp(triangleNormal(a, b, c), Eigen::Vector3f::UnitZ());
  const Eigen::Vector3f mean_normal = normals[static_cast<std::size_t>(i)] +
                                      normals[static_cast<std::size_t>(j)] +
                                      normals[static_cast<std::size_t>(k)];
  if (normalDeviationDeg(triangle_normal, mean_normal) > params.max_normal_deviation_deg) {
    return false;
  }

  *key = tri;
  std::sort(key->begin(), key->end());
  return true;
}

// Triangulate one planar band with CGAL's 2D Delaunay triangulation, filtering
// every candidate face through the shared edge/normal/area gate.
void triangulateBand(const std::vector<Eigen::Vector3f>& points,
                     const std::vector<Eigen::Vector3f>& normals,
                     const std::vector<int>& indices, const Eigen::Vector3f& origin,
                     const Eigen::Vector3f& u, const Eigen::Vector3f& v,
                     const MeshBuildParams& params, std::set<std::array<int, 3>>* seen,
                     std::vector<std::array<int, 3>>* accepted, int* kept, int* discarded) {
  if (indices.size() < 3) return;
  Delaunay2 dt;
  for (int index : indices) {
    const Eigen::Vector3f d = points[static_cast<std::size_t>(index)] - origin;
    auto vertex = dt.insert(Kernel::Point_2(d.dot(u), d.dot(v)));
    vertex->info() = index;
  }
  for (auto face = dt.finite_faces_begin(); face != dt.finite_faces_end(); ++face) {
    const std::array<int, 3> tri{face->vertex(0)->info(), face->vertex(1)->info(),
                                 face->vertex(2)->info()};
    std::array<int, 3> key{};
    if (!triangleAccepted(points, normals, tri, params, &key)) {
      ++(*discarded);
      continue;
    }
    if (!seen->insert(key).second) continue;  // duplicate from overlapping bands
    accepted->push_back(tri);
    ++(*kept);
  }
}

struct LayeredResult {
  bool ok = false;
  std::vector<std::array<int, 3>> triangles;
  int kept = 0;
  int discarded = 0;
};

LayeredResult runLayered(const std::vector<Eigen::Vector3f>& points,
                         const std::vector<Eigen::Vector3f>& normals,
                         const std::vector<std::vector<int>>& components,
                         const MeshBuildParams& params) {
  LayeredResult result;
  std::set<std::array<int, 3>> seen;
  for (const std::vector<int>& component : components) {
    if (component.size() < 3) continue;
    const PlaneFit fit = fitPlane(points, component);
    Eigen::Vector3f u, v;
    planeBasis(fit.normal, &u, &v);

    if (fit.rms <= kPlaneRmsSplit) {
      triangulateBand(points, normals, component, fit.centroid, u, v, params, &seen,
                      &result.triangles, &result.kept, &result.discarded);
      continue;
    }

    // Non-planar component: split into overlapping horizontal Z bands so the
    // bands share vertices and stitch into one surface.
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
    for (int index : component) {
      z_min = std::min(z_min, points[static_cast<std::size_t>(index)].z());
      z_max = std::max(z_max, points[static_cast<std::size_t>(index)].z());
    }
    const int band_count = std::max(1, static_cast<int>(std::ceil((z_max - z_min) / kBandHeight)));
    for (int band = 0; band < band_count; ++band) {
      const float lo = z_min + static_cast<float>(band) * kBandHeight - kBandOverlap;
      const float hi = z_min + static_cast<float>(band + 1) * kBandHeight + kBandOverlap;
      std::vector<int> band_indices;
      for (int index : component) {
        const float z = points[static_cast<std::size_t>(index)].z();
        if (z >= lo && z <= hi) band_indices.push_back(index);
      }
      triangulateBand(points, normals, band_indices, fit.centroid, u, v, params, &seen,
                      &result.triangles, &result.kept, &result.discarded);
    }
  }
  result.ok = !result.triangles.empty();
  return result;
}

struct AFSRResult {
  bool ok = false;
  std::vector<std::array<int, 3>> triangles;
  int kept = 0;
  int discarded = 0;
};

AFSRResult runAFSR(const std::vector<Eigen::Vector3f>& points,
                   const std::vector<Eigen::Vector3f>& normals, const MeshBuildParams& params) {
  AFSRResult result;

  // A near-collinear point set cannot carry a surface. Skip AFSR so CGAL's
  // degenerate-triangulation diagnostic is not emitted and let the layered
  // fallback report the failure.
  {
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const Eigen::Vector3f& p : points) centroid += p;
    centroid /= static_cast<float>(points.size());
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const Eigen::Vector3f& p : points) {
      const Eigen::Vector3f d = p - centroid;
      covariance.noalias() += d * d.transpose();
    }
    covariance /= static_cast<float>(points.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    const float lambda1 = solver.eigenvalues()(1);
    const float lambda2 = solver.eigenvalues()(2);
    if (lambda2 <= 1e-20f || lambda1 <= 1e-9f * lambda2) return result;
  }

  std::vector<Kernel::Point_3> cgal_points;
  cgal_points.reserve(points.size());
  for (const Eigen::Vector3f& p : points) {
    cgal_points.emplace_back(static_cast<double>(p.x()), static_cast<double>(p.y()),
                             static_cast<double>(p.z()));
  }

  std::vector<std::array<std::size_t, 3>> facets;
  try {
    CGAL::advancing_front_surface_reconstruction(cgal_points.begin(), cgal_points.end(),
                                                 std::back_inserter(facets));
  } catch (...) {
    return result;
  }

  std::set<std::array<int, 3>> seen;
  for (const std::array<std::size_t, 3>& facet : facets) {
    const std::array<int, 3> tri{static_cast<int>(facet[0]), static_cast<int>(facet[1]),
                                 static_cast<int>(facet[2])};
    std::array<int, 3> key{};
    if (!triangleAccepted(points, normals, tri, params, &key)) {
      ++result.discarded;
      continue;
    }
    if (!seen.insert(key).second) continue;
    result.triangles.push_back(tri);
    ++result.kept;
  }

  // A reconstruction is only trusted when it is dense enough and not mostly
  // discarded; otherwise the layered 2.5D path takes over.
  const int total = result.kept + result.discarded;
  const bool enough =
      result.kept >= static_cast<int>(points.size()) && total > 0 &&
      static_cast<float>(result.discarded) / static_cast<float>(total) <=
          params.max_discarded_face_ratio;
  result.ok = enough && !result.triangles.empty();
  return result;
}

MeshData compileMesh(const std::vector<Eigen::Vector3f>& points,
                     const std::vector<std::array<int, 3>>& triangles,
                     const std::vector<float>* roughness, const std::vector<float>* curvature) {
  std::set<int> used;
  for (const std::array<int, 3>& tri : triangles) {
    for (int v : tri) used.insert(v);
  }
  std::vector<int> ordered(used.begin(), used.end());

  std::unordered_map<int, int> remap;
  remap.reserve(ordered.size() * 2);
  MeshData mesh;
  mesh.vertices.reserve(ordered.size());
  for (std::size_t i = 0; i < ordered.size(); ++i) {
    remap.emplace(ordered[i], static_cast<int>(i));
    mesh.vertices.push_back(points[static_cast<std::size_t>(ordered[i])]);
  }
  if (roughness != nullptr) {
    mesh.vertex_roughness.reserve(ordered.size());
    for (int index : ordered) mesh.vertex_roughness.push_back((*roughness)[static_cast<std::size_t>(index)]);
  }
  if (curvature != nullptr) {
    mesh.vertex_curvature.reserve(ordered.size());
    for (int index : ordered) mesh.vertex_curvature.push_back((*curvature)[static_cast<std::size_t>(index)]);
  }
  mesh.faces.reserve(triangles.size());
  for (const std::array<int, 3>& tri : triangles) {
    mesh.faces.push_back({static_cast<PointId>(remap[tri[0]]), static_cast<PointId>(remap[tri[1]]),
                          static_cast<PointId>(remap[tri[2]])});
  }
  return mesh;
}

}  // namespace

Result<TriangulationResult> SurfaceTriangulator::triangulate(
    const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals,
    const MeshBuildParams& params, const std::vector<float>* roughness,
    const std::vector<float>* curvature) {
  if (points.empty()) {
    return Status::Error(ErrorCode::kPointCloudEmpty, "triangulation input is empty");
  }
  if (points.size() < 3) {
    return Status::Error(ErrorCode::kPointCloudTooSmall,
                         "triangulation needs at least 3 points");
  }
  if (normals.size() != points.size()) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "normals length does not match point count");
  }
  if (roughness != nullptr && roughness->size() != points.size()) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "roughness length does not match point count");
  }
  if (curvature != nullptr && curvature->size() != points.size()) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "curvature length does not match point count");
  }

  const std::vector<std::vector<int>> components =
      connectedComponents(points, params.component_radius);

  if (params.prefer_layered) {
    const LayeredResult layered = runLayered(points, normals, components, params);
    const int total = layered.kept + layered.discarded;
    if (!layered.ok || total == 0 ||
        static_cast<float>(layered.discarded) / static_cast<float>(total) >
            params.max_discarded_face_ratio) {
      return Status::Error(ErrorCode::kLayeredTriangulationFailed,
                           "layered 2.5D triangulation produced no acceptable mesh");
    }
    TriangulationResult out;
    out.mesh = compileMesh(points, layered.triangles, roughness, curvature);
    out.mode = TriangulationMode::kLayered2p5D;
    out.component_count = static_cast<int>(components.size());
    out.discarded_triangles = layered.discarded;
    return out;
  }

  AFSRResult afsr = runAFSR(points, normals, params);
  if (afsr.ok) {
    TriangulationResult out;
    out.mesh = compileMesh(points, afsr.triangles, roughness, curvature);
    out.mode = TriangulationMode::kDelaunay3DSurface;
    out.component_count = static_cast<int>(components.size());
    out.discarded_triangles = afsr.discarded;
    return out;
  }

  // Automatic degradation: 3D reconstruction failed, fall back to layered 2.5D.
  const LayeredResult layered = runLayered(points, normals, components, params);
  const int total = layered.kept + layered.discarded;
  if (!layered.ok || total == 0 ||
      static_cast<float>(layered.discarded) / static_cast<float>(total) >
          params.max_discarded_face_ratio) {
    return Status::Error(ErrorCode::kTriangulationFailed,
                         "both 3D AFSR and layered 2.5D triangulation failed");
  }

  TriangulationResult out;
  out.mesh = compileMesh(points, layered.triangles, roughness, curvature);
  out.mode = TriangulationMode::kLayered2p5D;
  out.component_count = static_cast<int>(components.size());
  out.discarded_triangles = layered.discarded;
  return out;
}

}  // namespace topmap
