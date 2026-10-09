#include "topmap_planner/hierarchical_planner.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "topmap_planner/corridor_builder.hpp"
#include "topmap_planner/path_optimizer.hpp"
#include "topmap_planner/region_locator.hpp"
#include "topmap_planner/safety_validator.hpp"
#include "topmap_planner/surface_astar.hpp"
#include "topmap_planner/topology_astar.hpp"
#include "topmap_surface/traversability.hpp"

namespace topmap {
namespace {

using Clock = std::chrono::steady_clock;

bool IsFinite(float value) { return std::isfinite(value); }

bool IsFinite(const Eigen::Vector3f& point) {
  return IsFinite(point.x()) && IsFinite(point.y()) && IsFinite(point.z());
}

Status Invalid(std::string message) {
  return Status::Error(ErrorCode::kInvalidArgument, std::move(message));
}

double SecondsBetween(const Clock::time_point& start, const Clock::time_point& end) {
  return std::chrono::duration<double>(end - start).count();
}

bool ValidateTraversability(const TraversabilityParams& trav) {
  const float fields[] = {
      trav.max_slope_deg,        trav.max_normal_diff_deg,   trav.max_slope_diff_deg,
      trav.max_step_height,      trav.max_roughness,         trav.max_surface_curvature,
      trav.min_triangle_area,    trav.required_clearance,
  };
  for (const float value : fields) {
    if (!IsFinite(value) || value < 0.0f) return false;
  }
  return true;
}

// Step 1: reject a malformed request or configuration before any stage runs, so
// a bad parameter can never be misreported as a start/goal localization failure.
Status ValidateRequestConfig(const TriangleMesh& mesh, const TopologyGraph& graph,
                             const PlanRequest& request, const TopmapConfig& config) {
  if (!IsFinite(request.start)) return Invalid("plan request start is not finite");
  if (!IsFinite(request.goal)) return Invalid("plan request goal is not finite");
  if (mesh.size() == 0u) return Invalid("cannot plan on an empty mesh");
  if (graph.regions().empty()) return Invalid("cannot plan on an empty topology graph");

  const PlanParams& plan = config.plan;
  if (!IsFinite(plan.max_project_radius) || !(plan.max_project_radius > 0.0f)) {
    return Invalid("plan.max_project_radius must be finite and positive");
  }
  if (!IsFinite(plan.max_project_normal_diff_deg) ||
      plan.max_project_normal_diff_deg < 0.0f) {
    return Invalid("plan.max_project_normal_diff_deg must be finite and non-negative");
  }
  if (plan.corridor_expansion_rings < 0) {
    return Invalid("plan.corridor_expansion_rings must be non-negative");
  }
  if (!IsFinite(plan.check_step) || !(plan.check_step > 0.0f)) {
    return Invalid("plan.check_step must be finite and positive");
  }
  if (!IsFinite(plan.topology_weight) || !(plan.topology_weight > 0.0f)) {
    return Invalid("plan.topology_weight must be finite and positive");
  }
  if (plan.smoothing_iterations < 0) {
    return Invalid("plan.smoothing_iterations must be non-negative");
  }
  if (!IsFinite(config.robot.max_path_curvature) || !(config.robot.max_path_curvature > 0.0f)) {
    return Invalid("robot.max_path_curvature must be finite and positive");
  }
  if (!ValidateTraversability(config.traversability)) {
    return Invalid("traversability parameters must be finite and non-negative");
  }
  return Status::Ok();
}

// The documented fallback corridor: every robot-placeable triangle of the mesh.
// The Region set is the whole graph so PlanStats.regions_in_corridor still
// describes a searchable corridor rather than the (partial) topology path.
Corridor MakeFullMeshCorridor(const TriangleMesh& mesh, const TopologyGraph& graph,
                              const TraversabilityParams& trav) {
  Corridor corridor;
  corridor.triangles.reserve(mesh.size());
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    if (TraversabilityEvaluator::isRobotPlaceable(mesh.triangle(id), trav)) {
      corridor.triangles.push_back(id);
    }
  }
  corridor.regions.reserve(graph.regions().size());
  for (const SurfaceRegion& region : graph.regions()) {
    corridor.regions.push_back(region.id);
  }
  return corridor;
}

float PathLength(const std::vector<Eigen::Vector3f>& path) {
  float length = 0.0f;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    length += (path[i + 1] - path[i]).norm();
  }
  return length;
}

}  // namespace

HierarchicalPlanner::HierarchicalPlanner(TopmapConfig config) : config_(std::move(config)) {}

Result<PlanResult> HierarchicalPlanner::plan(const TriangleMesh& mesh, const TopologyGraph& graph,
                                             const PlanRequest& request) const {
  const Status validation = ValidateRequestConfig(mesh, graph, request, config_);
  if (!validation.ok()) return validation;

  // 1. Start/goal localization (R6: the plan params carry the projection radius
  // and the normal-consistency gate).
  const RegionLocator locator;
  const auto start_region = locator.locate(mesh, graph, request.start, config_.plan);
  if (!start_region.ok()) {
    return Status::Error(ErrorCode::kStartNotOnSurface,
                         "start localization failed: " + start_region.status().message);
  }
  const auto goal_region = locator.locate(mesh, graph, request.goal, config_.plan);
  if (!goal_region.ok()) {
    return Status::Error(ErrorCode::kGoalNotOnSurface,
                         "goal localization failed: " + goal_region.status().message);
  }

  // 2. Topology A* over the Region-Portal bipartite graph.
  int topology_expanded = 0;
  const Clock::time_point topology_begin = Clock::now();
  const auto topology = TopologyAStar().search(graph, start_region.value(), goal_region.value(),
                                               config_.plan.topology_weight, &topology_expanded);
  const double topology_seconds = SecondsBetween(topology_begin, Clock::now());
  if (!topology.ok()) {
    return Status::Error(ErrorCode::kTopologyUnreachable,
                         "topology search failed: " + topology.status().message);
  }
  const TopologyPath& topology_path = topology.value();

  // Ordered Portal geometry and the segmented-surface waypoint sequence.
  std::vector<Portal> ordered_portals;
  ordered_portals.reserve(topology_path.portals.size());
  std::vector<Eigen::Vector3f> waypoints;
  waypoints.reserve(topology_path.portals.size() + 2u);
  waypoints.push_back(request.start);
  for (const PortalId portal_id : topology_path.portals) {
    const Portal* portal = graph.portal(portal_id);
    if (portal == nullptr) {
      return Status::Error(ErrorCode::kTopologyUnreachable,
                           "topology path references a portal that is not in the graph");
    }
    ordered_portals.push_back(*portal);
    waypoints.push_back(portal->center);
  }
  waypoints.push_back(request.goal);

  // 3. Corridor: build, retry once with one extra expansion ring, then fall back
  // to the full placeable mesh if configured.
  bool fallback_used = false;
  bool corridor_is_full = false;
  Corridor corridor;

  const auto build_corridor = [&](int expansion_rings) -> Result<Corridor> {
    // R29: deterministic test-only injection; both attempts must fail.
    if (config_.plan.force_corridor_failure) {
      return Status::Error(ErrorCode::kCorridorBuildFailed,
                           "corridor build failure injected by plan.force_corridor_failure");
    }
    return CorridorBuilder().build(mesh, graph, topology_path, expansion_rings);
  };

  auto corridor_result = build_corridor(config_.plan.corridor_expansion_rings);
  if (!corridor_result.ok() && corridor_result.status().code == ErrorCode::kCorridorBuildFailed) {
    corridor_result = build_corridor(config_.plan.corridor_expansion_rings + 1);
  }
  if (!corridor_result.ok()) {
    if (corridor_result.status().code != ErrorCode::kCorridorBuildFailed) {
      return corridor_result.status();
    }
    if (!config_.plan.fallback_full_mesh) {
      return Status::Error(ErrorCode::kCorridorBuildFailed,
                           "corridor build failed and plan.fallback_full_mesh is disabled: " +
                               corridor_result.status().message);
    }
    corridor = MakeFullMeshCorridor(mesh, graph, config_.traversability);
    if (corridor.triangles.empty()) {
      return Status::Error(ErrorCode::kCorridorBuildFailed,
                           "full-mesh fallback corridor has no robot-placeable triangle");
    }
    fallback_used = true;
    corridor_is_full = true;
  } else {
    corridor = corridor_result.take();
  }

  // 4. Segmented surface A* through start, every ordered Portal center, goal.
  int surface_expanded = 0;
  double surface_seconds = 0.0;
  const auto run_surface = [&](const Corridor& search_corridor) {
    int expanded = 0;
    const Clock::time_point begin = Clock::now();
    auto found = SurfaceAStar().search(mesh, search_corridor, waypoints, config_.traversability,
                                       &expanded);
    surface_seconds += SecondsBetween(begin, Clock::now());
    surface_expanded += expanded;
    return found;
  };

  auto surface_result = run_surface(corridor);
  if (!surface_result.ok() && surface_result.status().code == ErrorCode::kSurfaceSearchFailed &&
      config_.plan.fallback_full_mesh && !corridor_is_full) {
    corridor = MakeFullMeshCorridor(mesh, graph, config_.traversability);
    if (!corridor.triangles.empty()) {
      fallback_used = true;
      corridor_is_full = true;
      surface_result = run_surface(corridor);
    }
  }
  if (!surface_result.ok()) {
    if (surface_result.status().code != ErrorCode::kSurfaceSearchFailed) {
      return surface_result.status();
    }
    return Status::Error(ErrorCode::kSurfaceSearchFailed,
                         "surface search failed: " + surface_result.status().message);
  }
  const std::vector<Eigen::Vector3f> raw_path = surface_result.take();

  // 5. Optimization over the ordered Portal geometry so the funnel really runs.
  bool optimization_succeeded = false;
  std::vector<Eigen::Vector3f> optimized_path;
  double optimization_seconds = 0.0;
  {
    const Clock::time_point begin = Clock::now();
    auto optimized = PathOptimizer().optimize(raw_path, mesh, corridor, ordered_portals,
                                              config_.plan, config_.robot, config_.traversability);
    optimization_seconds = SecondsBetween(begin, Clock::now());
    if (optimized.ok()) {
      optimization_succeeded = true;
      optimized_path = optimized.take();
    } else if (optimized.status().code == ErrorCode::kPathOptimizationFailed) {
      // Retain the raw T24 discrete path; the plan is still usable.
      fallback_used = true;
    } else {
      return optimized.status();
    }
  }

  // 6. Safety validation of the optimized candidate, with a raw-path retry.
  const SafetyValidator validator;
  const std::vector<Eigen::Vector3f>& candidate =
      optimization_succeeded ? optimized_path : raw_path;
  PathValidationReport report = validator.validate(candidate, mesh, config_.traversability,
                                                   config_.robot, config_.plan.check_step);
  std::vector<Eigen::Vector3f> final_path = candidate;
  if (!report.valid) {
    if (!optimization_succeeded) {
      return Status::Error(ErrorCode::kPathValidationFailed,
                           "the raw surface path failed safety validation");
    }
    const PathValidationReport raw_report =
        validator.validate(raw_path, mesh, config_.traversability, config_.robot,
                           config_.plan.check_step);
    if (!raw_report.valid) {
      return Status::Error(ErrorCode::kPathValidationFailed,
                           "optimized and raw paths both failed safety validation "
                           "(optimized violations=" +
                               std::to_string(report.violating_indices.size()) +
                               ", max_curvature=" + std::to_string(report.max_curvature) +
                               "; raw violations=" +
                               std::to_string(raw_report.violating_indices.size()) +
                               ", max_curvature=" + std::to_string(raw_report.max_curvature) +
                               ")");
    }
    final_path = raw_path;
    report = raw_report;
    fallback_used = true;
  }

  if (!SafetyValidator::portalOrderMatches(final_path, mesh, graph,
                                           topology_path.portals)) {
    return Status::Error(ErrorCode::kPathValidationFailed,
                         "safe path does not preserve the topology Portal order");
  }

  // 7. Assemble the result; every PlanStats field describes the returned path.
  PlanResult result;
  result.status = Status::Ok();
  result.path = std::move(final_path);
  result.topology = topology_path;

  PlanStats& stats = result.stats;
  stats.regions_in_corridor = static_cast<int>(corridor.regions.size());
  stats.triangles_in_corridor = static_cast<int>(corridor.triangles.size());
  stats.topology_expanded_nodes = topology_expanded;
  stats.surface_expanded_nodes = surface_expanded;
  stats.topology_time_sec = topology_seconds;
  stats.surface_time_sec = surface_seconds;
  stats.optimization_time_sec = optimization_seconds;
  stats.path_length = PathLength(result.path);
  stats.min_clearance = report.min_clearance;
  stats.max_slope_deg = report.max_slope_deg;
  stats.max_curvature = report.max_curvature;
  stats.fallback_used = fallback_used;
  return result;
}

}  // namespace topmap
