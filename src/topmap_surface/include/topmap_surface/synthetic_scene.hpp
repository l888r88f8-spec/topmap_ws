#pragma once

#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "topmap_core/error.hpp"
#include "topmap_surface/mesh_types.hpp"

namespace topmap {

enum class SyntheticScene {
  kFlatPlane = 0,
  kRamp,
  kOverlappingFloors,
  kMultiLevelRamps,
  kHallNarrowHall,
  kUShape,
  kTShape,
  kSingleBridge,
  kParallelPortals,
  kLoop,
};

// snake_case name, identical to the value accepted by the T28 CLI and by
// parseSyntheticScene.
std::string toString(SyntheticScene s);

// R17: CLI name -> enum. Unrecognised names yield kInvalidArgument.
Result<SyntheticScene> parseSyntheticScene(const std::string& name);

struct SyntheticSceneOptions {
  float grid_step = 0.10f;
  float level_height = 3.00f;
  float hall_size = 6.00f;
  float corridor_width = 1.20f;
};

// R15: the ten scenes have frozen coordinates (see the implementation plan
// table). makeSyntheticEndpoints returns those start/goal values verbatim.
Result<MeshData> makeSyntheticMesh(SyntheticScene s, const SyntheticSceneOptions& o = {});
Result<std::pair<Eigen::Vector3f, Eigen::Vector3f>> makeSyntheticEndpoints(
    SyntheticScene s, const SyntheticSceneOptions& o = {});

}  // namespace topmap
