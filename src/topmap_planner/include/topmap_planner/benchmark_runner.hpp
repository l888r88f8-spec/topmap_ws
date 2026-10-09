#pragma once

#include <string>
#include <vector>

#include "topmap_core/error.hpp"
#include "topmap_core/params.hpp"
#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_topology/topology_graph.hpp"

namespace topmap {

// A4.8 (R31): the three planning methods compared by the benchmark.
enum class PlanningMethod { kPointCloudAStar = 0, kMeshAStar = 1, kHierarchical = 2 };

// A4.8 (R31): one benchmark table row. `expected_unreachable` records whether
// the scene is unreachable *by design* (kOverlappingFloors); for those rows
// `success` means "the method correctly reported no path", so the T33
// acceptance script can keep a single success column.
struct BenchmarkRecord {
  std::string scene, method;
  bool success = false;
  bool expected_unreachable = false;
  int expanded_nodes = 0, triangle_count = 0, region_count = 0, portal_count = 0,
      error_code = 0;
  int invalid_cross_layer_edges = 0;
  bool reachability_preserved = false, portal_order_valid = false;
  double planning_time_sec = 0.0;
  float path_length = 0.0f, min_clearance = 0.0f;
  int regions_before_compression = 0, regions_after_compression = 0;
};

// Exact CSV contract. The header column order is frozen; benchmarkCsvRow emits
// the fields in the same order so writeBenchmarkCsv/readBenchmarkCsv round-trip.
std::string benchmarkCsvHeader();
std::string benchmarkCsvRow(const BenchmarkRecord& record);

// Atomic-ish write: the payload is rendered in memory and written to a sibling
// temporary file that is renamed over `path` only after a full successful
// write, so a failed write never truncates an existing file.
Status writeBenchmarkCsv(const std::string& path,
                         const std::vector<BenchmarkRecord>& records);

// Strict parser: rejects a wrong header, a wrong column count, a malformed
// number (including trailing junk) and a non-finite value. Empty files and
// header-only files are invalid/empty-row results respectively.
Result<std::vector<BenchmarkRecord>> readBenchmarkCsv(const std::string& path);

// Runs every (scene, method) pair through the production surface/topology
// pipeline and returns rows ordered scene-major, method-minor. Invalid global
// arguments (empty scene/method list, non-finite or non-positive point_radius)
// fail the whole call; a single method failure is recorded in its row instead.
Result<std::vector<BenchmarkRecord>> runBenchmark(const std::vector<SyntheticScene>& scenes,
                                                  const std::vector<PlanningMethod>& methods,
                                                  const TopmapConfig& config, float point_radius);

// A-01: counts Portals whose two Regions have disjoint height bands *and* whose
// own interface step exceeds `trav.max_step_height`. Disjoint Region bands alone
// are not a violation: a continuous ramp-to-flat transition has a ramp Region
// spanning the whole rise while the flat Regions sit at its ends, yet the shared
// edge step is below the robot limit. Only a genuinely discontinuous
// cross-floor join (large step) is counted.
int CountInvalidCrossLayerEdges(const TopologyGraph& graph, const TraversabilityParams& trav);

}  // namespace topmap
