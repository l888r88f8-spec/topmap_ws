#include "topmap_planner/benchmark_runner.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "topmap_core/geometry.hpp"
#include "topmap_planner/hierarchical_planner.hpp"
#include "topmap_planner/path_optimizer.hpp"
#include "topmap_planner/region_locator.hpp"
#include "topmap_planner/safety_validator.hpp"
#include "topmap_planner/surface_astar.hpp"
#include "topmap_planner/topology_astar.hpp"
#include "topmap_surface/traversability.hpp"
#include "topmap_topology/portal_extractor.hpp"
#include "topmap_topology/portal_validator.hpp"
#include "topmap_topology/region_growing.hpp"
#include "topmap_topology/region_split.hpp"
#include "topmap_topology/topology_compressor.hpp"
#include "topmap_topology/topology_graph.hpp"
#include "topmap_topology/topology_invariants.hpp"

namespace topmap {
namespace {

using Clock = std::chrono::steady_clock;
constexpr float kInf = std::numeric_limits<float>::infinity();

const char* const kCsvHeader =
    "scene,method,success,expected_unreachable,expanded_nodes,planning_time_sec,path_length,"
    "min_clearance,triangle_count,region_count,portal_count,invalid_cross_layer_edges,"
    "reachability_preserved,portal_order_valid,regions_before_compression,"
    "regions_after_compression,error_code";

bool IsFinite(float value) { return std::isfinite(value); }

Status Invalid(std::string message) {
  return Status::Error(ErrorCode::kInvalidArgument, std::move(message));
}

Status IoError(std::string message) {
  return Status::Error(ErrorCode::kSerializationIoError, std::move(message));
}

double SecondsBetween(const Clock::time_point& start, const Clock::time_point& end) {
  return std::chrono::duration<double>(end - start).count();
}

const char* MethodName(PlanningMethod method) {
  switch (method) {
    case PlanningMethod::kPointCloudAStar:
      return "point_cloud";
    case PlanningMethod::kMeshAStar:
      return "mesh";
    case PlanningMethod::kHierarchical:
      return "hierarchical";
  }
  return "unknown";
}

float PathLength(const std::vector<Eigen::Vector3f>& path) {
  float length = 0.0f;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    length += (path[i + 1] - path[i]).norm();
  }
  return length;
}

// T34: the post-processing corridor for Methods A/B. Both search on a full
// placeable-mesh graph, so their raw path is optimized over the same triangle
// set rather than a topology corridor.
Corridor FullMeshCorridor(const TriangleMesh& mesh, const TraversabilityParams& trav) {
  Corridor corridor;
  corridor.triangles.reserve(mesh.size());
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    if (TraversabilityEvaluator::isRobotPlaceable(mesh.triangle(id), trav)) {
      corridor.triangles.push_back(id);
    }
  }
  return corridor;
}

// ---------------------------------------------------------------------------
// Production scene pipeline (mirrors the shared planner test fixture, but lives
// in the library so the benchmark does not depend on test-only headers).
// ---------------------------------------------------------------------------

bool HasCrossRegionSharedEdge(const TriangleMesh& mesh,
                              const std::vector<SurfaceRegion>& regions) {
  std::vector<RegionId> owner(static_cast<std::size_t>(mesh.size()), kInvalidId);
  for (const SurfaceRegion& region : regions) {
    for (TriangleId triangle : region.triangle_ids) {
      if (triangle >= 0 && triangle < static_cast<TriangleId>(mesh.size())) {
        owner[static_cast<std::size_t>(triangle)] = region.id;
      }
    }
  }
  for (TriangleId triangle = 0; triangle < static_cast<TriangleId>(mesh.size()); ++triangle) {
    const RegionId current = owner[static_cast<std::size_t>(triangle)];
    if (current == kInvalidId) continue;
    for (const TriangleEdge& edge : mesh.triangle(triangle).edges) {
      if (edge.neighbor == kInvalidId || edge.neighbor < triangle) continue;
      const RegionId other = owner[static_cast<std::size_t>(edge.neighbor)];
      if (other != kInvalidId && other != current) return true;
    }
  }
  return false;
}

struct SceneData {
  TriangleMesh mesh;
  TopologyGraph graph;        // uncompressed, used by methods A/B
  TopologyGraph compressed;   // used by method C when compression is applied
  CompressionStats compression;
  bool reachability_preserved = false;
  Eigen::Vector3f start = Eigen::Vector3f::Zero();
  Eigen::Vector3f goal = Eigen::Vector3f::Zero();
};

Result<SceneData> BuildScene(SyntheticScene scene, const TopmapConfig& config) {
  auto data = makeSyntheticMesh(scene);
  if (!data.ok()) return data.status();

  auto mesh_result = TriangleMesh::fromMeshData(data.value(), {});
  if (!mesh_result.ok()) return mesh_result.status();
  TriangleMesh mesh = mesh_result.take();

  TraversabilityEvaluator::apply(mesh, config.traversability);

  auto grown = RegionGrower().grow(mesh, config.region);
  if (!grown.ok()) return grown.status();

  auto split = RegionSplitter().split(mesh, grown.value(), config.region);
  if (!split.ok()) return split.status();
  std::vector<SurfaceRegion> regions = split.take();

  std::vector<Portal> portals;
  auto extracted = PortalExtractor().extract(mesh, regions, config.portal);
  if (extracted.ok()) {
    portals = extracted.take();
  } else if (extracted.status().code == ErrorCode::kPortalExtractionFailed &&
             !HasCrossRegionSharedEdge(mesh, regions)) {
    portals.clear();  // Legitimate: regions exist but share no inter-Region edge.
  } else {
    return extracted.status();
  }

  if (!portals.empty()) {
    auto validated =
        PortalValidator().validate(mesh, regions, portals, config.robot, config.portal);
    if (!validated.ok()) return validated.status();
    portals = validated.take();
  }

  auto graph = TopologyGraph::build(regions, portals);
  if (!graph.ok()) return graph.status();

  SceneData prepared;
  prepared.mesh = std::move(mesh);
  prepared.graph = graph.take();

  auto compressed =
      TopologyCompressor().compress(prepared.graph, config.compression, config.region);
  if (!compressed.ok()) return compressed.status();
  prepared.compression = compressed.value().second;
  prepared.compressed = compressed.value().first;

  std::string reason;
  prepared.reachability_preserved = TopologyInvariants::sameConnectivity(
      TopologyInvariants::capture(prepared.graph),
      TopologyInvariants::capture(prepared.compressed), &reason);

  auto endpoints = makeSyntheticEndpoints(scene);
  if (!endpoints.ok()) return endpoints.status();
  prepared.start = endpoints.value().first;
  prepared.goal = endpoints.value().second;
  return prepared;
}

// A-01: a cross-layer connection is a Portal whose two Regions occupy disjoint
// height bands *and* whose interface step is not robot-traversable. The
// shared-edge mesh itself cannot produce a discontinuous join, so this is the
// transitive Region-level check. Disjoint bands alone are not enough: a
// continuous ramp-to-flat interface has a ramp Region spanning the full rise
// while the flat Regions pin to its ends, but the crossing step stays at or
// below `trav.max_step_height` and the connection is legitimate.
std::vector<PortalId> ExpectedPortalOrder(const TriangleMesh& mesh, const TopologyGraph& graph,
                                          const Eigen::Vector3f& start,
                                          const Eigen::Vector3f& goal,
                                          const TopmapConfig& config) {
  const RegionLocator locator;
  const auto start_region = locator.locate(mesh, graph, start, config.plan);
  const auto goal_region = locator.locate(mesh, graph, goal, config.plan);
  if (!start_region.ok() || !goal_region.ok()) return {};
  const auto path = TopologyAStar().search(graph, start_region.value(), goal_region.value(),
                                           config.plan.topology_weight, nullptr);
  if (!path.ok()) return {};
  return path.value().portals;
}

// ---------------------------------------------------------------------------
// Method A: radius-neighbour A* over the mesh vertices.
// ---------------------------------------------------------------------------

// Uniform-grid radius graph. Every undirected pair within `radius` is a
// candidate; the pair is kept only when its two nearest robot-placeable,
// front-facing triangles share a traversable edge, or when the segment midpoint
// lies on a placeable triangle incident to one endpoint (the nearest-triangle
// approximation of the plan). Candidate discovery uses a spatial hash so the
// build is O(V * local density), never O(V^2).
//
// Failure contract: the sampled SafetyValidator batch is mandatory. When it
// rejects its arguments (non-positive check_step, invalid traversability
// parameters, non-finite endpoint) this returns that error and the caller must
// treat the graph as unbuilt; it is never silently returned as an edgeless
// graph. A candidate list that is empty for a valid mesh simply yields an
// edgeless graph, which A* then reports as an ordinary no-path result.
struct PointGraph {
  std::vector<Eigen::Vector3f> points;
  std::vector<std::vector<int>> adjacency;
  std::vector<char> usable;
  std::vector<float> clearance;
};

Result<PointGraph> BuildPointGraph(const TriangleMesh& mesh, const TraversabilityParams& trav,
                                   float radius, float check_step) {
  const std::size_t count = mesh.vertices().size();
  PointGraph graph;
  graph.points = mesh.vertices();
  graph.adjacency.assign(count, std::vector<int>());
  graph.usable.assign(count, 0);
  graph.clearance.assign(count, 0.0f);

  std::vector<TriangleId> best_triangle(count, kInvalidId);
  std::vector<float> best_clearance(count, 0.0f);
  std::vector<int> component(count, -1);
  std::vector<std::vector<TriangleId>> incident(count);
  const float normal_limit = trav.max_normal_diff_rad();
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    const TriangleNode& triangle = mesh.triangle(id);
    if (!TraversabilityEvaluator::isRobotPlaceable(triangle, trav)) continue;
    if (angleBetween(triangle.normal, Eigen::Vector3f::UnitZ()) > normal_limit) continue;
    for (const PointId vertex : triangle.vertex_ids) {
      if (vertex < 0 || vertex >= static_cast<PointId>(count)) continue;
      const std::size_t index = static_cast<std::size_t>(vertex);
      incident[index].push_back(id);
      if (!graph.usable[index] || triangle.min_clearance > best_clearance[index] ||
          (triangle.min_clearance == best_clearance[index] && id < best_triangle[index])) {
        graph.usable[index] = 1;
        best_triangle[index] = id;
        best_clearance[index] = triangle.min_clearance;
        graph.clearance[index] = triangle.min_clearance;
      }
    }
  }
  for (std::size_t index = 0; index < count; ++index) {
    if (graph.usable[index]) component[index] = mesh.componentId(best_triangle[index]);
  }

  const auto shares_traversable_edge = [&](TriangleId a, TriangleId b) {
    if (a == b) return true;
    if (a < 0 || b < 0) return false;
    const TriangleNode& ta = mesh.triangle(a);
    for (const TriangleEdge& edge : ta.edges) {
      if (edge.neighbor != b) continue;
      return edge.traversable &&
             TraversabilityEvaluator::isEdgeTraversable(ta, mesh.triangle(b), trav);
    }
    return false;
  };

  const auto midpoint_on_incident = [&](const Eigen::Vector3f& midpoint,
                                        const std::vector<TriangleId>& triangles) {
    for (const TriangleId id : triangles) {
      const TriangleNode& triangle = mesh.triangle(id);
      const float distance =
          pointTriangleDistance(midpoint, mesh.vertices()[triangle.vertex_ids[0]],
                                mesh.vertices()[triangle.vertex_ids[1]],
                                mesh.vertices()[triangle.vertex_ids[2]]);
      if (distance <= trav.max_step_height) return true;
    }
    return false;
  };

  // Cheap, mesh-local pre-filter. A candidate must be placeable, in the same
  // connected component, and either share a traversable edge or have its
  // midpoint on a placeable incident triangle. The expensive per-sample
  // clearance check runs afterwards on the surviving candidates.
  const auto edge_candidate = [&](int u, int v) {
    if (!graph.usable[static_cast<std::size_t>(u)] ||
        !graph.usable[static_cast<std::size_t>(v)]) {
      return false;
    }
    if (component[static_cast<std::size_t>(u)] != component[static_cast<std::size_t>(v)]) {
      return false;
    }
    if (shares_traversable_edge(best_triangle[static_cast<std::size_t>(u)],
                                best_triangle[static_cast<std::size_t>(v)])) {
      return true;
    }
    const Eigen::Vector3f midpoint =
        0.5f * (graph.points[static_cast<std::size_t>(u)] +
                graph.points[static_cast<std::size_t>(v)]);
    return midpoint_on_incident(midpoint, incident[static_cast<std::size_t>(u)]) ||
           midpoint_on_incident(midpoint, incident[static_cast<std::size_t>(v)]);
  };

  if (graph.points.empty()) return graph;

  const float cell_size = std::max(radius, 1e-3f);
  const std::uint64_t offset = 1u << 20;
  const auto cell_of = [cell_size](float value) {
    return static_cast<int>(std::floor(value / cell_size));
  };
  const auto cell_key = [offset](int ix, int iy, int iz) {
    const std::uint64_t a = static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix + offset));
    const std::uint64_t b = static_cast<std::uint64_t>(static_cast<std::uint32_t>(iy + offset));
    const std::uint64_t c = static_cast<std::uint64_t>(static_cast<std::uint32_t>(iz + offset));
    return (a << 42) | (b << 21) | c;
  };

  std::unordered_map<std::uint64_t, std::vector<int>> grid;
  grid.reserve(count * 2u);
  for (std::size_t index = 0; index < count; ++index) {
    const Eigen::Vector3f& point = graph.points[index];
    grid[cell_key(cell_of(point.x()), cell_of(point.y()), cell_of(point.z()))].push_back(
        static_cast<int>(index));
  }

  const float radius_squared = radius * radius;
  struct Candidate {
    int u = -1;
    int v = -1;
  };
  std::vector<Candidate> candidates;
  for (std::size_t index = 0; index < count; ++index) {
    if (!graph.usable[index]) continue;
    const int u = static_cast<int>(index);
    const Eigen::Vector3f& point = graph.points[index];
    const int cx = cell_of(point.x()), cy = cell_of(point.y()), cz = cell_of(point.z());
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const auto it = grid.find(cell_key(cx + dx, cy + dy, cz + dz));
          if (it == grid.end()) continue;
          for (const int v : it->second) {
            if (v <= u) continue;
            if ((graph.points[static_cast<std::size_t>(v)] - point).squaredNorm() >
                radius_squared) {
              continue;
            }
            if (edge_candidate(u, v)) candidates.push_back(Candidate{u, v});
          }
        }
      }
    }
  }

  // T33: enforce the same per-sample clearance/placeability constraint that
  // SafetyValidator applies, but over the whole candidate set in one pass with a
  // shared spatial index. A chord that leaves the placeable free space is
  // dropped, so A* can only return a path validate() accepts.
  if (!candidates.empty()) {
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> segments;
    segments.reserve(candidates.size());
    for (const Candidate& candidate : candidates) {
      segments.emplace_back(graph.points[static_cast<std::size_t>(candidate.u)],
                            graph.points[static_cast<std::size_t>(candidate.v)]);
    }
    const auto safe = SafetyValidator().validateSegments(segments, mesh, trav, check_step);
    if (!safe.ok()) return safe.status();
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      if (!safe.value()[i]) continue;
      const std::size_t u = static_cast<std::size_t>(candidates[i].u);
      const std::size_t v = static_cast<std::size_t>(candidates[i].v);
      graph.adjacency[u].push_back(candidates[i].v);
      graph.adjacency[v].push_back(candidates[i].u);
    }
  }
  for (std::vector<int>& neighbors : graph.adjacency) {
    std::sort(neighbors.begin(), neighbors.end());
  }
  return graph;
}

int NearestUsableVertex(const PointGraph& graph, const Eigen::Vector3f& query) {
  int best = -1;
  float best_distance = kInf;
  for (std::size_t index = 0; index < graph.points.size(); ++index) {
    if (!graph.usable[index]) continue;
    const float distance = (graph.points[index] - query).norm();
    if (best < 0 || distance < best_distance) {
      best = static_cast<int>(index);
      best_distance = distance;
    }
  }
  return best;
}

struct QueueNode {
  float f = 0.0f;
  float g = 0.0f;
  int id = -1;
};

struct QueueGreater {
  bool operator()(const QueueNode& a, const QueueNode& b) const {
    if (a.f != b.f) return a.f > b.f;
    if (a.g != b.g) return a.g > b.g;
    return a.id > b.id;
  }
};

// ---------------------------------------------------------------------------
// Method evaluators.
// ---------------------------------------------------------------------------

struct MethodOutcome {
  Status status;
  int expanded_nodes = 0;
  double planning_time_sec = 0.0;
  float path_length = 0.0f;
  float min_clearance = 0.0f;
  bool portal_order_valid = false;
  bool found_path = false;
};

MethodOutcome EvaluatePointCloudAStar(const SceneData& data, const TopmapConfig& config,
                                      float point_radius) {
  MethodOutcome outcome;
  const Clock::time_point begin = Clock::now();
  const Result<PointGraph> built =
      BuildPointGraph(data.mesh, config.traversability, point_radius, config.plan.check_step);
  if (!built.ok()) {
    // BuildPointGraph only fails on invalid arguments, so surface the exact
    // error instead of disguising it as a no-path result.
    outcome.status = built.status();
    outcome.planning_time_sec = SecondsBetween(begin, Clock::now());
    return outcome;
  }
  const PointGraph& graph = built.value();

  const int start_vertex = NearestUsableVertex(graph, data.start);
  if (start_vertex < 0) {
    outcome.status = Status::Error(ErrorCode::kStartNotOnSurface,
                                   "point-cloud graph has no usable vertex near the start");
    outcome.planning_time_sec = SecondsBetween(begin, Clock::now());
    return outcome;
  }
  const int goal_vertex = NearestUsableVertex(graph, data.goal);
  if (goal_vertex < 0) {
    outcome.status = Status::Error(ErrorCode::kGoalNotOnSurface,
                                   "point-cloud graph has no usable vertex near the goal");
    outcome.planning_time_sec = SecondsBetween(begin, Clock::now());
    return outcome;
  }

  const std::size_t count = graph.points.size();
  std::vector<float> g(count, kInf);
  std::vector<int> parent(count, -1);
  std::vector<char> closed(count, 0);
  const auto heuristic = [&](int id) {
    return (graph.points[static_cast<std::size_t>(id)] -
            graph.points[static_cast<std::size_t>(goal_vertex)])
        .norm();
  };
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueGreater> open;
  g[static_cast<std::size_t>(start_vertex)] = 0.0f;
  open.push(QueueNode{heuristic(start_vertex), 0.0f, start_vertex});
  while (!open.empty()) {
    const QueueNode node = open.top();
    open.pop();
    if (closed[static_cast<std::size_t>(node.id)]) continue;
    closed[static_cast<std::size_t>(node.id)] = 1;
    ++outcome.expanded_nodes;
    if (node.id == goal_vertex) break;
    for (const int next : graph.adjacency[static_cast<std::size_t>(node.id)]) {
      if (closed[static_cast<std::size_t>(next)]) continue;
      const std::size_t current_index = static_cast<std::size_t>(node.id);
      const std::size_t next_index = static_cast<std::size_t>(next);
      const float geometric_step =
          (graph.points[next_index] - graph.points[current_index]).norm();
      const float clearance =
          std::min(graph.clearance[current_index], graph.clearance[next_index]);
      const float floor = std::max(0.1f * config.traversability.required_clearance, 1e-3f);
      const float margin =
          std::max(clearance - config.traversability.required_clearance, floor);
      const float clearance_multiplier =
          config.traversability.required_clearance > 0.0f
              ? 1.0f + config.traversability.required_clearance / margin
              : 1.0f;
      const float step = geometric_step * clearance_multiplier;
      if (!IsFinite(step)) continue;
      const float candidate = g[static_cast<std::size_t>(node.id)] + step;
      if (!IsFinite(candidate)) continue;
      if (candidate < g[static_cast<std::size_t>(next)]) {
        g[static_cast<std::size_t>(next)] = candidate;
        parent[static_cast<std::size_t>(next)] = node.id;
        open.push(QueueNode{candidate + heuristic(next), candidate, next});
      }
    }
  }
  if (g[static_cast<std::size_t>(goal_vertex)] == kInf) {
    outcome.planning_time_sec = SecondsBetween(begin, Clock::now());
    outcome.status = Status::Error(ErrorCode::kSurfaceSearchFailed,
                                   "point-cloud A* found no traversable vertex path");
    return outcome;
  }

  std::vector<Eigen::Vector3f> path;
  for (int id = goal_vertex; id != -1; id = parent[static_cast<std::size_t>(id)]) {
    path.push_back(graph.points[static_cast<std::size_t>(id)]);
  }
  std::reverse(path.begin(), path.end());
  if (path.size() == 1u) path.push_back(path.front());

  // T34: post-process the raw vertex chain on the full placeable-mesh corridor
  // before validation, exactly like the hierarchical method. A geometric
  // post-processing failure is surfaced as kPathOptimizationFailed (605); an
  // optimized path that still fails validation is kPathValidationFailed (606).
  const Corridor corridor = FullMeshCorridor(data.mesh, config.traversability);
  auto optimized = PathOptimizer().optimize(path, data.mesh, corridor, config.plan, config.robot,
                                            config.traversability);
  outcome.planning_time_sec = SecondsBetween(begin, Clock::now());
  if (!optimized.ok()) {
    outcome.status = optimized.status();
    return outcome;
  }
  const std::vector<Eigen::Vector3f>& final_path = optimized.value();
  outcome.path_length = PathLength(final_path);
  const PathValidationReport report =
      SafetyValidator().validate(final_path, data.mesh, config.traversability, config.robot,
                                 config.plan.check_step);
  outcome.min_clearance = report.min_clearance;
  if (!report.valid) {
    outcome.status = Status::Error(ErrorCode::kPathValidationFailed,
                                   "point-cloud path failed the SafetyValidator re-check");
    return outcome;
  }
  const std::vector<PortalId> expected =
      ExpectedPortalOrder(data.mesh, data.graph, data.start, data.goal, config);
  outcome.portal_order_valid =
      SafetyValidator::portalOrderMatches(final_path, data.mesh, data.graph, expected);
  outcome.found_path = true;
  outcome.status = Status::Ok();
  return outcome;
}

MethodOutcome EvaluateMeshAStar(const SceneData& data, const TopmapConfig& config) {
  MethodOutcome outcome;
  Corridor corridor = FullMeshCorridor(data.mesh, config.traversability);
  for (const SurfaceRegion& region : data.graph.regions()) {
    corridor.regions.push_back(region.id);
  }

  const std::vector<Eigen::Vector3f> waypoints{data.start, data.goal};
  const Clock::time_point begin = Clock::now();
  auto result = SurfaceAStar().search(data.mesh, corridor, waypoints, config.traversability,
                                      &outcome.expanded_nodes);
  if (!result.ok()) {
    outcome.planning_time_sec = SecondsBetween(begin, Clock::now());
    outcome.status = result.status();
    return outcome;
  }
  const std::vector<Eigen::Vector3f>& path = result.value();

  auto optimized = PathOptimizer().optimize(path, data.mesh, corridor, config.plan, config.robot,
                                            config.traversability);
  outcome.planning_time_sec = SecondsBetween(begin, Clock::now());
  if (!optimized.ok()) {
    outcome.status = optimized.status();
    return outcome;
  }
  const std::vector<Eigen::Vector3f>& final_path = optimized.value();
  outcome.path_length = PathLength(final_path);
  const PathValidationReport report =
      SafetyValidator().validate(final_path, data.mesh, config.traversability, config.robot,
                                 config.plan.check_step);
  outcome.min_clearance = report.min_clearance;
  if (!report.valid) {
    outcome.status = Status::Error(ErrorCode::kPathValidationFailed,
                                   "mesh path failed the SafetyValidator re-check");
    return outcome;
  }
  const std::vector<PortalId> expected =
      ExpectedPortalOrder(data.mesh, data.graph, data.start, data.goal, config);
  outcome.portal_order_valid =
      SafetyValidator::portalOrderMatches(final_path, data.mesh, data.graph, expected);
  outcome.found_path = true;
  outcome.status = Status::Ok();
  return outcome;
}

MethodOutcome EvaluateHierarchical(const SceneData& data, const TopmapConfig& config) {
  MethodOutcome outcome;
  const TopologyGraph& graph = data.compressed;
  PlanRequest request;
  request.start = data.start;
  request.goal = data.goal;

  const Clock::time_point begin = Clock::now();
  auto result = HierarchicalPlanner(config).plan(data.mesh, graph, request);
  outcome.planning_time_sec = SecondsBetween(begin, Clock::now());
  if (!result.ok()) {
    outcome.status = result.status();
    return outcome;
  }
  const PlanResult& plan = result.value();
  outcome.found_path = true;
  // A-05 compares Method C's corridor Surface A* with Method B's full-mesh
  // Surface A*. Keep the metric in that shared graph domain; topology-search
  // expansions remain available separately in PlanStats.
  outcome.expanded_nodes = plan.stats.surface_expanded_nodes;
  outcome.path_length = plan.stats.path_length;
  outcome.min_clearance = plan.stats.min_clearance;
  outcome.portal_order_valid =
      SafetyValidator::portalOrderMatches(plan.path, data.mesh, graph, plan.topology.portals);
  outcome.status = Status::Ok();
  return outcome;
}

BenchmarkRecord MakeRecord(SyntheticScene scene, PlanningMethod method, const SceneData& data,
                           const TopmapConfig& config, float point_radius) {
  BenchmarkRecord record;
  record.scene = toString(scene);
  record.method = MethodName(method);
  record.expected_unreachable = (scene == SyntheticScene::kOverlappingFloors);
  record.triangle_count = static_cast<int>(data.mesh.size());
  record.region_count = static_cast<int>(data.graph.regions().size());
  record.portal_count = static_cast<int>(data.graph.portals().size());
  record.invalid_cross_layer_edges = CountInvalidCrossLayerEdges(data.graph, config.traversability);
  record.reachability_preserved = data.reachability_preserved;
  record.regions_before_compression = data.compression.regions_before;
  record.regions_after_compression = data.compression.regions_after;

  MethodOutcome outcome;
  switch (method) {
    case PlanningMethod::kPointCloudAStar:
      outcome = EvaluatePointCloudAStar(data, config, point_radius);
      break;
    case PlanningMethod::kMeshAStar:
      outcome = EvaluateMeshAStar(data, config);
      break;
    case PlanningMethod::kHierarchical:
      outcome = EvaluateHierarchical(data, config);
      break;
  }

  record.expanded_nodes = outcome.expanded_nodes;
  record.planning_time_sec = outcome.planning_time_sec;
  record.path_length = outcome.path_length;
  record.min_clearance = outcome.min_clearance;
  record.portal_order_valid = outcome.portal_order_valid;
  record.error_code = static_cast<int>(outcome.status.code);

  if (record.expected_unreachable) {
    // R31: success means "correctly reported no path". Method C must reject the
    // scene in the topology stage; the geometric methods have their own genuine
    // no-path codes.
    const ErrorCode code = outcome.status.code;
    const bool no_path = !outcome.found_path &&
                         (code == ErrorCode::kTopologyUnreachable ||
                          code == ErrorCode::kSurfaceSearchFailed ||
                          code == ErrorCode::kStartNotOnSurface ||
                          code == ErrorCode::kGoalNotOnSurface);
    record.success =
        no_path && (method != PlanningMethod::kHierarchical ||
                    code == ErrorCode::kTopologyUnreachable);
    if (record.success) {
      // T33 result contract: every correctly-rejected unreachable row exposes
      // the unified planning-stage unreachable code (602), whichever method
      // produced the genuine no-path verdict. A method that wrongly found a
      // path keeps its own code and reports success == false.
      record.error_code = static_cast<int>(ErrorCode::kTopologyUnreachable);
    }
  } else {
    record.success = outcome.found_path;
  }
  return record;
}

// ---------------------------------------------------------------------------
// CSV helpers.
// ---------------------------------------------------------------------------

std::string FormatDouble(double value) {
  std::ostringstream out;
  out << std::setprecision(9) << value;
  return out.str();
}

std::string FormatFloat(float value) {
  std::ostringstream out;
  out << std::setprecision(9) << value;
  return out.str();
}

std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = line.find(',', start);
    if (comma == std::string::npos) {
      fields.push_back(line.substr(start));
      break;
    }
    fields.push_back(line.substr(start, comma - start));
    start = comma + 1;
  }
  return fields;
}

bool ParseInt(const std::string& text, int* out) {
  if (text.empty()) return false;
  int value = 0;
  const char* begin = text.data();
  const char* end = text.data() + text.size();
  const auto result = std::from_chars(begin, end, value);
  if (result.ec != std::errc() || result.ptr != end) return false;
  *out = value;
  return true;
}

bool ParseBool(const std::string& text, bool* out) {
  int value = 0;
  if (!ParseInt(text, &value)) return false;
  if (value != 0 && value != 1) return false;
  *out = (value == 1);
  return true;
}

bool ParseDouble(const std::string& text, double* out) {
  if (text.empty()) return false;
  double value = 0.0;
  const char* begin = text.data();
  const char* end = text.data() + text.size();
  const auto result = std::from_chars(begin, end, value);
  if (result.ec != std::errc() || result.ptr != end || !std::isfinite(value)) return false;
  *out = value;
  return true;
}

bool ParseFloat(const std::string& text, float* out) {
  if (text.empty()) return false;
  float value = 0.0f;
  const char* begin = text.data();
  const char* end = text.data() + text.size();
  const auto result = std::from_chars(begin, end, value);
  if (result.ec != std::errc() || result.ptr != end || !std::isfinite(value)) return false;
  *out = value;
  return true;
}

std::string StripCarriageReturn(std::string line) {
  if (!line.empty() && line.back() == '\r') line.pop_back();
  return line;
}

}  // namespace

int CountInvalidCrossLayerEdges(const TopologyGraph& graph, const TraversabilityParams& trav) {
  int count = 0;
  for (const Portal& portal : graph.portals()) {
    const SurfaceRegion* a = graph.region(portal.region_a);
    const SurfaceRegion* b = graph.region(portal.region_b);
    if (a == nullptr || b == nullptr) continue;
    const bool disjoint_bands = a->max_z < b->min_z || b->max_z < a->min_z;
    if (!disjoint_bands) continue;
    // The Regions occupy separate height bands, so the connection only counts as
    // a wrong cross-layer edge when the Portal's own crossing step is beyond the
    // configured robot limit. A continuous ramp join stays at or below it.
    if (portal.max_step > trav.max_step_height) ++count;
  }
  return count;
}

std::string benchmarkCsvHeader() { return std::string(kCsvHeader); }

std::string benchmarkCsvRow(const BenchmarkRecord& record) {
  std::ostringstream out;
  out << record.scene << ',' << record.method << ',' << (record.success ? 1 : 0) << ','
      << (record.expected_unreachable ? 1 : 0) << ',' << record.expanded_nodes << ','
      << FormatDouble(record.planning_time_sec) << ',' << FormatFloat(record.path_length) << ','
      << FormatFloat(record.min_clearance) << ',' << record.triangle_count << ','
      << record.region_count << ',' << record.portal_count << ','
      << record.invalid_cross_layer_edges << ',' << (record.reachability_preserved ? 1 : 0)
      << ',' << (record.portal_order_valid ? 1 : 0) << ','
      << record.regions_before_compression << ',' << record.regions_after_compression << ','
      << record.error_code;
  return out.str();
}

Status writeBenchmarkCsv(const std::string& path,
                         const std::vector<BenchmarkRecord>& records) {
  std::ostringstream payload;
  payload << kCsvHeader << '\n';
  for (const BenchmarkRecord& record : records) {
    payload << benchmarkCsvRow(record) << '\n';
  }
  const std::string content = payload.str();
  const std::string temporary = path + ".tmp";

  {
    std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
    if (!out) return IoError("cannot open temporary benchmark file: " + temporary);
    out.write(content.data(), static_cast<std::streamsize>(content.size()));
    out.flush();
    if (!out) {
      out.close();
      std::remove(temporary.c_str());
      return IoError("failed while writing benchmark file: " + temporary);
    }
  }
  if (std::rename(temporary.c_str(), path.c_str()) != 0) {
    std::remove(temporary.c_str());
    return IoError("cannot replace benchmark file: " + path);
  }
  return Status::Ok();
}

Result<std::vector<BenchmarkRecord>> readBenchmarkCsv(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return IoError("cannot open benchmark file: " + path);

  std::string line;
  if (!std::getline(in, line)) return Invalid("benchmark file is empty: " + path);
  if (StripCarriageReturn(line) != kCsvHeader) {
    return Invalid("benchmark CSV header does not match the frozen contract");
  }

  std::vector<BenchmarkRecord> records;
  while (std::getline(in, line)) {
    line = StripCarriageReturn(std::move(line));
    if (line.empty()) continue;
    const std::vector<std::string> fields = SplitCsv(line);
    if (fields.size() != 17u) {
      return Invalid("benchmark CSV row has " + std::to_string(fields.size()) +
                     " columns, expected 17");
    }
    BenchmarkRecord record;
    record.scene = fields[0];
    record.method = fields[1];
    if (!ParseBool(fields[2], &record.success)) return Invalid("invalid success field");
    if (!ParseBool(fields[3], &record.expected_unreachable)) {
      return Invalid("invalid expected_unreachable field");
    }
    if (!ParseInt(fields[4], &record.expanded_nodes)) return Invalid("invalid expanded_nodes");
    if (!ParseDouble(fields[5], &record.planning_time_sec)) {
      return Invalid("invalid planning_time_sec");
    }
    if (!ParseFloat(fields[6], &record.path_length)) return Invalid("invalid path_length");
    if (!ParseFloat(fields[7], &record.min_clearance)) return Invalid("invalid min_clearance");
    if (!ParseInt(fields[8], &record.triangle_count)) return Invalid("invalid triangle_count");
    if (!ParseInt(fields[9], &record.region_count)) return Invalid("invalid region_count");
    if (!ParseInt(fields[10], &record.portal_count)) return Invalid("invalid portal_count");
    if (!ParseInt(fields[11], &record.invalid_cross_layer_edges)) {
      return Invalid("invalid invalid_cross_layer_edges");
    }
    if (!ParseBool(fields[12], &record.reachability_preserved)) {
      return Invalid("invalid reachability_preserved");
    }
    if (!ParseBool(fields[13], &record.portal_order_valid)) {
      return Invalid("invalid portal_order_valid");
    }
    if (!ParseInt(fields[14], &record.regions_before_compression)) {
      return Invalid("invalid regions_before_compression");
    }
    if (!ParseInt(fields[15], &record.regions_after_compression)) {
      return Invalid("invalid regions_after_compression");
    }
    if (!ParseInt(fields[16], &record.error_code)) return Invalid("invalid error_code");
    records.push_back(std::move(record));
  }
  return records;
}

Result<std::vector<BenchmarkRecord>> runBenchmark(const std::vector<SyntheticScene>& scenes,
                                                  const std::vector<PlanningMethod>& methods,
                                                  const TopmapConfig& config,
                                                  float point_radius) {
  if (scenes.empty()) return Invalid("runBenchmark requires at least one scene");
  if (methods.empty()) return Invalid("runBenchmark requires at least one method");
  if (!IsFinite(point_radius) || !(point_radius > 0.0f)) {
    return Invalid("point_radius must be finite and positive");
  }

  std::vector<BenchmarkRecord> records;
  records.reserve(scenes.size() * methods.size());
  for (const SyntheticScene scene : scenes) {
    auto prepared = BuildScene(scene, config);
    if (!prepared.ok()) return prepared.status();
    for (const PlanningMethod method : methods) {
      records.push_back(MakeRecord(scene, method, prepared.value(), config, point_radius));
    }
  }
  return records;
}

}  // namespace topmap
