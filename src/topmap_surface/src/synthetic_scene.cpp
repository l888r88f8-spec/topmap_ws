#include "topmap_surface/synthetic_scene.hpp"

#include <array>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace topmap {
namespace {

// Patches are described on the frozen 0.1 m lattice: index i maps to the world
// coordinate i * grid_step. This keeps every shared boundary on the very same
// lattice line so the two sides weld to the same vertex and T08 can build the
// shared-edge adjacency.
enum class PatchKind { kFlat, kRampX };

struct Patch {
  int layer = 0;
  int ix0 = 0, ix1 = 0, iy0 = 0, iy1 = 0;
  PatchKind kind = PatchKind::kFlat;
  float z0 = 0.0f, z1 = 0.0f;
};

Patch Flat(int layer, int x0, int x1, int y0, int y1, float z) {
  return Patch{layer, x0, x1, y0, y1, PatchKind::kFlat, z, z};
}

Patch RampX(int layer, int x0, int x1, int y0, int y1, float z0, float z1) {
  return Patch{layer, x0, x1, y0, y1, PatchKind::kRampX, z0, z1};
}

struct SceneSpec {
  std::vector<Patch> patches;
  int nx = 0;
  int ny = 0;
};

SceneSpec SpecFor(SyntheticScene s) {
  switch (s) {
    case SyntheticScene::kFlatPlane:
      return {{Flat(0, 0, 60, 0, 60, 0.0f)}, 60, 60};
    case SyntheticScene::kRamp:
      return {{Flat(0, 0, 40, 0, 40, 0.0f), RampX(0, 40, 134, 14, 26, 0.0f, 2.0f),
               Flat(0, 134, 174, 0, 40, 2.0f)},
              174, 40};
    case SyntheticScene::kOverlappingFloors:
      return {{Flat(0, 0, 50, 0, 50, 0.0f), Flat(1, 0, 50, 0, 50, 3.0f)}, 50, 50};
    case SyntheticScene::kMultiLevelRamps:
      // The frozen AABB is [0,32] and the ramps must be 12 degrees; a 2 m rise
      // at 12 degrees spans 9.4 m (exactly like kRamp), so the last platform
      // absorbs the remaining span.
      return {{Flat(0, 0, 40, 0, 40, 0.0f), RampX(0, 40, 134, 14, 26, 0.0f, 2.0f),
               Flat(0, 134, 174, 0, 40, 2.0f), RampX(0, 174, 268, 14, 26, 2.0f, 4.0f),
               Flat(0, 268, 320, 0, 40, 4.0f)},
              320, 40};
    case SyntheticScene::kHallNarrowHall:
      return {{Flat(0, 0, 70, 0, 70, 0.0f), Flat(0, 70, 100, 29, 41, 0.0f),
               Flat(0, 100, 170, 0, 70, 0.0f)},
              170, 70};
    case SyntheticScene::kUShape:
      return {{Flat(0, 0, 40, 0, 10, 0.0f), Flat(0, 0, 10, 10, 40, 0.0f),
               Flat(0, 0, 40, 40, 50, 0.0f)},
              40, 50};
    case SyntheticScene::kTShape:
      return {{Flat(0, 20, 60, 20, 60, 0.0f), Flat(0, 20, 60, 60, 100, 0.0f),
               Flat(0, 60, 100, 20, 60, 0.0f), Flat(0, 20, 60, 0, 20, 0.0f)},
              100, 100};
    case SyntheticScene::kSingleBridge:
      return {{Flat(0, 0, 50, 0, 50, 0.0f), Flat(0, 50, 80, 22, 37, 0.0f),
               Flat(0, 80, 130, 0, 50, 0.0f)},
              130, 50};
    case SyntheticScene::kParallelPortals:
      return {{Flat(0, 0, 50, 0, 80, 0.0f), Flat(0, 50, 80, 10, 22, 0.0f),
               Flat(0, 50, 80, 58, 70, 0.0f), Flat(0, 80, 130, 0, 80, 0.0f)},
              130, 80};
    case SyntheticScene::kLoop:
      return {{Flat(0, 0, 40, 0, 40, 0.0f), Flat(0, 80, 120, 0, 40, 0.0f),
               Flat(0, 80, 120, 80, 120, 0.0f), Flat(0, 0, 40, 80, 120, 0.0f),
               Flat(0, 40, 80, 14, 26, 0.0f), Flat(0, 94, 106, 40, 80, 0.0f),
               Flat(0, 40, 80, 94, 106, 0.0f), Flat(0, 14, 26, 40, 80, 0.0f)},
              120, 120};
  }
  return {};
}

float PatchZ(const Patch& p, int ix) {
  if (p.kind == PatchKind::kFlat) return p.z0;
  const float span = static_cast<float>(p.ix1 - p.ix0);
  const float t = static_cast<float>(ix - p.ix0) / span;
  return p.z0 + t * (p.z1 - p.z0);
}

}  // namespace

std::string toString(SyntheticScene s) {
  switch (s) {
    case SyntheticScene::kFlatPlane: return "flat_plane";
    case SyntheticScene::kRamp: return "ramp";
    case SyntheticScene::kOverlappingFloors: return "overlapping_floors";
    case SyntheticScene::kMultiLevelRamps: return "multi_level_ramps";
    case SyntheticScene::kHallNarrowHall: return "hall_narrow_hall";
    case SyntheticScene::kUShape: return "u_shape";
    case SyntheticScene::kTShape: return "t_shape";
    case SyntheticScene::kSingleBridge: return "single_bridge";
    case SyntheticScene::kParallelPortals: return "parallel_portals";
    case SyntheticScene::kLoop: return "loop";
  }
  return "unknown";
}

Result<SyntheticScene> parseSyntheticScene(const std::string& name) {
  for (SyntheticScene s : {SyntheticScene::kFlatPlane, SyntheticScene::kRamp,
                           SyntheticScene::kOverlappingFloors, SyntheticScene::kMultiLevelRamps,
                           SyntheticScene::kHallNarrowHall, SyntheticScene::kUShape,
                           SyntheticScene::kTShape, SyntheticScene::kSingleBridge,
                           SyntheticScene::kParallelPortals, SyntheticScene::kLoop}) {
    if (toString(s) == name) return s;
  }
  return Status::Error(ErrorCode::kInvalidArgument, "unknown synthetic scene: " + name);
}

Result<MeshData> makeSyntheticMesh(SyntheticScene s, const SyntheticSceneOptions& o) {
  if (!(o.grid_step > 0.0f)) {
    return Status::Error(ErrorCode::kInvalidArgument, "grid_step must be positive");
  }
  const SceneSpec spec = SpecFor(s);
  if (spec.patches.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument, "no geometry for synthetic scene");
  }
  const float step = o.grid_step;

  MeshData mesh;
  std::map<std::array<int, 3>, PointId> welded;  // (layer, ix, iy) -> vertex id
  const auto vertexAt = [&](int layer, int ix, int iy, float z) -> PointId {
    const std::array<int, 3> key{layer, ix, iy};
    const auto it = welded.find(key);
    if (it != welded.end()) return it->second;
    const PointId id = static_cast<PointId>(mesh.vertices.size());
    mesh.vertices.emplace_back(ix * step, iy * step, z);
    welded.emplace(key, id);
    return id;
  };

  for (const Patch& p : spec.patches) {
    const int nx = p.ix1 - p.ix0;
    const int ny = p.iy1 - p.iy0;
    if (nx <= 0 || ny <= 0) continue;
    std::vector<PointId> grid(static_cast<std::size_t>(nx + 1) * (ny + 1), kInvalidId);
    for (int j = 0; j <= ny; ++j) {
      for (int i = 0; i <= nx; ++i) {
        grid[static_cast<std::size_t>(j) * (nx + 1) + i] =
            vertexAt(p.layer, p.ix0 + i, p.iy0 + j, PatchZ(p, p.ix0 + i));
      }
    }
    for (int j = 0; j < ny; ++j) {
      for (int i = 0; i < nx; ++i) {
        const std::size_t a = static_cast<std::size_t>(j) * (nx + 1) + i;
        const std::size_t b = a + 1;
        const std::size_t d = a + (nx + 1);
        const std::size_t c = d + 1;
        mesh.faces.push_back({grid[a], grid[b], grid[c]});
        mesh.faces.push_back({grid[a], grid[c], grid[d]});
      }
    }
  }

  if (mesh.faces.empty()) {
    return Status::Error(ErrorCode::kMeshEmpty, "synthetic scene produced no faces");
  }
  return mesh;
}

Result<std::pair<Eigen::Vector3f, Eigen::Vector3f>> makeSyntheticEndpoints(
    SyntheticScene s, const SyntheticSceneOptions&) {
  switch (s) {
    case SyntheticScene::kFlatPlane:
      return std::make_pair(Eigen::Vector3f(0.5f, 0.5f, 0.05f), Eigen::Vector3f(5.5f, 5.5f, 0.05f));
    case SyntheticScene::kRamp:
      return std::make_pair(Eigen::Vector3f(2.0f, 2.0f, 0.05f), Eigen::Vector3f(15.4f, 2.0f, 2.05f));
    case SyntheticScene::kOverlappingFloors:
      return std::make_pair(Eigen::Vector3f(2.5f, 2.5f, 0.05f), Eigen::Vector3f(2.5f, 2.5f, 3.05f));
    case SyntheticScene::kMultiLevelRamps:
      return std::make_pair(Eigen::Vector3f(2.0f, 2.0f, 0.05f), Eigen::Vector3f(30.0f, 2.0f, 4.05f));
    case SyntheticScene::kHallNarrowHall:
      return std::make_pair(Eigen::Vector3f(3.5f, 3.5f, 0.05f), Eigen::Vector3f(13.5f, 3.5f, 0.05f));
    case SyntheticScene::kUShape:
      return std::make_pair(Eigen::Vector3f(3.5f, 0.5f, 0.05f), Eigen::Vector3f(3.5f, 4.5f, 0.05f));
    case SyntheticScene::kTShape:
      return std::make_pair(Eigen::Vector3f(4.0f, 9.0f, 0.05f), Eigen::Vector3f(9.0f, 4.0f, 0.05f));
    case SyntheticScene::kSingleBridge:
      return std::make_pair(Eigen::Vector3f(2.5f, 2.5f, 0.05f), Eigen::Vector3f(10.5f, 2.5f, 0.05f));
    case SyntheticScene::kParallelPortals:
      return std::make_pair(Eigen::Vector3f(2.5f, 4.0f, 0.05f), Eigen::Vector3f(10.5f, 4.0f, 0.05f));
    case SyntheticScene::kLoop:
      return std::make_pair(Eigen::Vector3f(2.0f, 2.0f, 0.05f), Eigen::Vector3f(10.0f, 10.0f, 0.05f));
  }
  return Status::Error(ErrorCode::kInvalidArgument, "no endpoints for synthetic scene");
}

}  // namespace topmap
