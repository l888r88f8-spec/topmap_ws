// export_stages.cpp
//
// Standalone, read-only exporter for the four-stage RViz demonstration.
//
// It runs the *actual* topmap pipeline on a synthetic PCD fixture:
//   load PCD -> process -> surface features -> triangulate -> fromMeshData ->
//   traversability -> region grow -> region split -> portal extract ->
//   portal validate -> topology graph -> topology compress
//
// and writes every intermediate stage to data/stages.json.  No triangle is
// deleted, filtered or merged away for presentation: the grown, split and final
// partitions are preserved side by side, together with every valid portal
// (boundary chain included) and the compressed final graph.
//
// When a fifth path is supplied it additionally runs the *production*
// topmap::HierarchicalPlanner on the same mesh/graph with the fixed demo
// endpoints and writes data/planning.json.  Nothing in that file is hand-drawn
// or interpolated: it is the planner-returned polyline, its true topology
// sequence, its PlanStats, and an independent topmap::SafetyValidator report.
// The demo launch replays this precomputed result; it is not interactive.
//
// Usage:
//   export_stages <input.pcd> <config.yaml> <scene_metadata.json> <out.json> \
//                 [planning_out.json] [archive_out.bin]
//
// Exit codes: 2 usage, 3 config, 4 PCD, 5 process, 6 features, 7 triangulate,
//             8 fromMeshData, 9 growing, 10 split, 11 portal extract,
//             12 portal validate, 13 graph build, 14 compress,
//             15 output file, 16 real planner failed, 17 planner returned a
//             path that failed the independent safety/portal-order check,
//             1 exported data failed self-validation.

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "topmap_core/params.hpp"
#include "topmap_core/geometry.hpp"
#include "topmap_planner/hierarchical_planner.hpp"
#include "topmap_planner/plan_types.hpp"
#include "topmap_planner/safety_validator.hpp"
#include "topmap_surface/point_cloud_processor.hpp"
#include "topmap_surface/surface_features.hpp"
#include "topmap_surface/traversability.hpp"
#include "topmap_surface/triangulator.hpp"
#include "topmap_topology/portal_extractor.hpp"
#include "topmap_topology/map_archive.hpp"
#include "topmap_topology/portal_validator.hpp"
#include "topmap_topology/region_growing.hpp"
#include "topmap_topology/region_split.hpp"
#include "topmap_topology/topology_compressor.hpp"
#include "topmap_topology/topology_graph.hpp"

namespace {

bool finite3(const Eigen::Vector3f& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

void writeVec(std::ostream& o, const Eigen::Vector3f& v) {
  o << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}

void writePointArray(std::ostream& o, const std::vector<Eigen::Vector3f>& points) {
  o << '[';
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i) o << ',';
    writeVec(o, points[i]);
  }
  o << ']';
}

void writeIntArray(std::ostream& o, const std::vector<int>& values) {
  o << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i) o << ',';
    o << values[i];
  }
  o << ']';
}

// Reads a small UTF-8 text file (the scene metadata JSON) verbatim.  Used only
// to embed the fixture metadata into the export.
std::string readTextFile(const std::string& path, bool* ok) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    *ok = false;
    return {};
  }
  std::ostringstream buffer;
  buffer << in.rdbuf();
  *ok = true;
  return buffer.str();
}

std::string trim(const std::string& s) {
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) return {};
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

void writeRegion(std::ostream& o, const topmap::SurfaceRegion& r) {
  o << "{\"id\":" << r.id << ",\"area\":" << r.area << ",\"centroid\":";
  writeVec(o, r.centroid);
  o << ",\"is_ramp\":" << (r.is_ramp ? "true" : "false")
    << ",\"is_topology_critical\":" << (r.is_topology_critical ? "true" : "false")
    << ",\"min_z\":" << r.min_z << ",\"max_z\":" << r.max_z
    << ",\"mean_slope\":" << r.mean_slope << ",\"max_slope\":" << r.max_slope
    << ",\"slope_variance\":" << r.slope_variance << ",\"roughness\":" << r.roughness
    << ",\"triangle_count\":" << r.triangle_ids.size() << ",\"portal_ids\":[";
  for (std::size_t i = 0; i < r.portal_ids.size(); ++i) {
    if (i) o << ',';
    o << r.portal_ids[i];
  }
  o << "],\"triangle_ids\":[";
  for (std::size_t i = 0; i < r.triangle_ids.size(); ++i) {
    if (i) o << ',';
    o << r.triangle_ids[i];
  }
  o << "]}";
}

void writeRegions(std::ostream& o, const std::vector<topmap::SurfaceRegion>& regions) {
  o << '[';
  for (std::size_t i = 0; i < regions.size(); ++i) {
    if (i) o << ',';
    writeRegion(o, regions[i]);
  }
  o << ']';
}

void writeFloatArray(std::ostream& o, const std::vector<float>& values) {
  o << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i) o << ',';
    o << values[i];
  }
  o << ']';
}

// Closest point on a triangle, clamped to its interior/edges.  Used only to
// render the planner path on the actual surface; the planner path itself is
// preserved unchanged in ``planning.json``.
Eigen::Vector3f closestPointOnTriangle(const Eigen::Vector3f& p, const Eigen::Vector3f& a,
                                       const Eigen::Vector3f& b, const Eigen::Vector3f& c) {
  const Eigen::Vector3f ab = b - a, ac = c - a, ap = p - a;
  const float d1 = ab.dot(ap), d2 = ac.dot(ap);
  if (d1 <= 0.0f && d2 <= 0.0f) return a;

  const Eigen::Vector3f bp = p - b;
  const float d3 = ab.dot(bp), d4 = ac.dot(bp);
  if (d3 >= 0.0f && d4 <= d3) return b;

  const float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    const float v = d1 / (d1 - d3);
    return a + v * ab;
  }

  const Eigen::Vector3f cp = p - c;
  const float d5 = ab.dot(cp), d6 = ac.dot(cp);
  if (d6 >= 0.0f && d5 <= d6) return c;

  const float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    const float w = d2 / (d2 - d6);
    return a + w * ac;
  }

  const float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + w * (c - b);
  }

  const float denom = 1.0f / (va + vb + vc);
  const float v = vb * denom, w = vc * denom;
  return a + ab * v + ac * w;
}

// Render-only surface trace: sample the *planner-returned* polyline at the
// planner's own check_step and project each sample onto the nearest
// robot-placeable, front-facing triangle.  This is the geometry the planner's
// SafetyValidator actually checks (it also samples and projects); it is not a
// new plan, a hand-drawn route, or a way to hide a failed planner result.  The
// original planner polyline stays verbatim in planning.json["path"].
bool projectPathToSurface(const std::vector<Eigen::Vector3f>& path,
                          const topmap::TriangleMesh& mesh, const topmap::TopmapConfig& config,
                          std::vector<Eigen::Vector3f>* trace, std::string* error) {
  trace->clear();
  if (path.size() < 2u) {
    *error = "planner path has fewer than two points";
    return false;
  }
  const float max_distance = config.plan.max_project_radius;
  const float max_normal_rad = config.traversability.max_normal_diff_rad();
  const float back_face_tolerance = config.traversability.max_step_height;
  const auto project = [&](const Eigen::Vector3f& point, Eigen::Vector3f* out) {
    topmap::TriangleId best = topmap::kInvalidId;
    float best_distance = std::numeric_limits<float>::infinity();
    for (topmap::TriangleId id = 0; id < static_cast<topmap::TriangleId>(mesh.size()); ++id) {
      const topmap::TriangleNode& triangle = mesh.triangle(id);
      if (!topmap::TraversabilityEvaluator::isRobotPlaceable(triangle, config.traversability)) {
        continue;
      }
      if (topmap::angleBetween(triangle.normal, Eigen::Vector3f::UnitZ()) > max_normal_rad) {
        continue;
      }
      if (triangle.normal.dot(point - triangle.centroid) < -back_face_tolerance) continue;
      const float distance = topmap::pointTriangleDistance(
          point, mesh.vertices()[triangle.vertex_ids[0]], mesh.vertices()[triangle.vertex_ids[1]],
          mesh.vertices()[triangle.vertex_ids[2]]);
      if (best == topmap::kInvalidId || distance < best_distance) {
        best = id;
        best_distance = distance;
      }
    }
    if (best == topmap::kInvalidId || best_distance > max_distance) return false;
    const topmap::TriangleNode& triangle = mesh.triangle(best);
    *out = closestPointOnTriangle(point, mesh.vertices()[triangle.vertex_ids[0]],
                                  mesh.vertices()[triangle.vertex_ids[1]],
                                  mesh.vertices()[triangle.vertex_ids[2]]);
    return true;
  };

  Eigen::Vector3f projected;
  if (!project(path.front(), &projected)) {
    *error = "cannot project the planner path start onto the surface";
    return false;
  }
  trace->push_back(projected);
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3f& a = path[i];
    const Eigen::Vector3f& b = path[i + 1];
    const float length = (b - a).norm();
    if (!std::isfinite(length)) {
      *error = "planner path segment is not finite";
      return false;
    }
    const int samples = std::max(1, static_cast<int>(std::ceil(length / config.plan.check_step)));
    for (int s = 1; s <= samples; ++s) {
      const float t = static_cast<float>(s) / static_cast<float>(samples);
      const Eigen::Vector3f point = a + t * (b - a);
      if (!project(point, &projected)) {
        *error = "cannot project a planner path sample onto the surface";
        return false;
      }
      if ((projected - trace->back()).squaredNorm() > 1e-12f) trace->push_back(projected);
    }
  }
  if (trace->size() < 2u) {
    *error = "surface trace has fewer than two points";
    return false;
  }
  return true;
}

// Runs the production planner once and records the real result.  A failure is
// written explicitly (status/error/path=[]) and returned as a non-zero exit
// code; there is deliberately no code path that invents a successful path.
int writePlanningFile(const std::string& planning_path, const std::string& pcd_path,
                      const std::string& config_path, const topmap::TopmapConfig& config,
                      const topmap::TriangleMesh& mesh, const topmap::TopologyGraph& graph,
                      const topmap::PlanRequest& request) {
  // Use the same configuration as the interactive planner. Surface adherence is
  // enforced by the production optimizer and independently rechecked below;
  // the optional display-only reference never replaces the returned path.
  topmap::TopmapConfig planning_config = config;

  const auto plan_result = topmap::HierarchicalPlanner(planning_config).plan(mesh, graph, request);
  const bool planner_ok = plan_result.ok();

  topmap::PathValidationReport safety_report;
  std::vector<topmap::PortalId> crossed_portals;
  std::vector<Eigen::Vector3f> surface_trace;
  bool safety_ok = false;
  bool portal_order_ok = false;
  bool trace_ok = false;
  std::string trace_error;
  if (planner_ok) {
    const topmap::PlanResult& plan = plan_result.value();
    const topmap::SafetyValidator validator;
    safety_report = validator.validate(plan.path, mesh, planning_config.traversability,
                                      planning_config.robot, planning_config.plan.check_step,
                                      planning_config.plan.path_surface_tolerance);
    crossed_portals = topmap::SafetyValidator::crossedPortals(plan.path, mesh, graph);
    portal_order_ok = topmap::SafetyValidator::portalOrderMatches(plan.path, mesh, graph,
                                                                 plan.topology.portals);
    safety_ok = safety_report.valid;
    trace_ok = projectPathToSurface(plan.path, mesh, planning_config, &surface_trace,
                                    &trace_error);
  }

  std::string error_code_name = "kOk";
  std::string error_message;
  if (!planner_ok) {
    error_code_name = topmap::toString(plan_result.status().code);
    error_message = plan_result.status().message;
  } else if (!safety_ok) {
    error_code_name = "kPathValidationFailed";
    error_message = "independent SafetyValidator rejected the planner-returned path";
  } else if (!portal_order_ok) {
    error_code_name = "kPathValidationFailed";
    error_message = "safe path does not preserve the planned topology Portal order";
  } else if (!trace_ok) {
    error_code_name = "kPathValidationFailed";
    error_message = "surface projection for rendering failed: " + trace_error;
  }
  const bool success = planner_ok && safety_ok && portal_order_ok && trace_ok;

  std::ofstream o(planning_path);
  if (!o) {
    std::cerr << "cannot open planning output: " << planning_path << '\n';
    return 15;
  }
  o << std::setprecision(9);
  o << '{';
  o << "\"schema\":\"topmap_stage_demo.planning\",\"schema_version\":1,";
  o << "\"generator\":\"export_stages.cpp / topmap::HierarchicalPlanner\",";
  o << "\"mode\":\"offline_precomputed_replay\",\"interactive_replanning\":false,";
  o << "\"frame_id\":\"map\",";
  o << "\"source\":{\"pcd\":" << std::quoted(pcd_path) << ",\"config\":"
    << std::quoted(config_path) << ",\"scene\":\"flat_H_ground_original_ramp\"},";
  o << "\"request\":{\"start\":";
  writeVec(o, request.start);
  o << ",\"goal\":";
  writeVec(o, request.goal);
  o << "},";
  o << "\"planner\":{\"name\":\"topmap::HierarchicalPlanner\",\"pipeline\":["
       "\"RegionLocator\",\"TopologyAStar\",\"CorridorBuilder\",\"SurfaceAStar\","
       "\"PathOptimizer\",\"SafetyValidator\"],\"dense_surface_path_replay\":false,"
       "\"displayed_path\":\"planner.path (verbatim; RViz adds only a render-only Z lift)\","
       "\"demo_note\":\"The same shortcut, funnel and smoothing configuration as the "
       "interactive planner is used. The planner-returned path is constrained to the "
       "actual mesh by path_surface_tolerance. The optional surface_trace is only a "
       "render-only reference and never replaces planner.path.\",\"config\":{";
  o << "\"topology_weight\":" << planning_config.plan.topology_weight
    << ",\"max_project_radius\":" << planning_config.plan.max_project_radius
    << ",\"path_surface_tolerance\":" << planning_config.plan.path_surface_tolerance
    << ",\"max_project_normal_diff_deg\":"
    << planning_config.plan.max_project_normal_diff_deg
    << ",\"corridor_expansion_rings\":" << planning_config.plan.corridor_expansion_rings
    << ",\"fallback_full_mesh\":"
    << (planning_config.plan.fallback_full_mesh ? "true" : "false")
    << ",\"shortcut_enabled\":"
    << (planning_config.plan.shortcut_enabled ? "true" : "false")
    << ",\"funnel_enabled\":" << (planning_config.plan.funnel_enabled ? "true" : "false")
    << ",\"smoothing_enabled\":"
    << (planning_config.plan.smoothing_enabled ? "true" : "false")
    << ",\"smoothing_iterations\":" << planning_config.plan.smoothing_iterations
    << ",\"check_step\":" << planning_config.plan.check_step
    << ",\"robot\":{\"width\":" << planning_config.robot.width
    << ",\"length\":" << planning_config.robot.length
    << ",\"max_slope_deg\":" << planning_config.robot.max_slope_deg
    << ",\"max_step_height\":" << planning_config.robot.max_step_height
    << ",\"safety_distance\":" << planning_config.robot.safety_distance
    << ",\"max_path_curvature\":" << planning_config.robot.max_path_curvature
    << ",\"required_clearance\":" << planning_config.robot.required_clearance() << "}}},";
  o << "\"status\":\"" << (success ? "success" : "failed") << "\",";
  o << "\"error\":";
  if (success) {
    o << "null";
  } else {
    o << "{\"code\":" << static_cast<int>(planner_ok
                                             ? topmap::ErrorCode::kPathValidationFailed
                                             : plan_result.status().code)
      << ",\"code_name\":" << std::quoted(error_code_name)
      << ",\"message\":" << std::quoted(error_message) << '}';
  }
  o << ",\"path\":";
  if (planner_ok) {
    writePointArray(o, plan_result.value().path);
  } else {
    o << "[]";
  }

  const topmap::PlanStats* stats = planner_ok ? &plan_result.value().stats : nullptr;
  o << ",\"path_length_m\":" << (stats != nullptr ? stats->path_length : 0.0f);
  float surface_trace_length = 0.0f;
  for (std::size_t i = 0; i + 1 < surface_trace.size(); ++i) {
    surface_trace_length += (surface_trace[i + 1] - surface_trace[i]).norm();
  }
  o << ",\"counts\":{\"path_points\":"
    << (planner_ok ? plan_result.value().path.size() : 0u)
    << ",\"surface_trace_points\":" << surface_trace.size() << "},";
  o << "\"surface_trace\":{\"kind\":\"render_only_surface_projection_reference\","
       "\"render_only\":true,\"independent_validation\":false,\"is_planner_final_path\":false,"
       "\"projection_ok\":"
    << (trace_ok ? "true" : "false") << ",\"source\":\"planner.path\",\"source_path_points\":"
    << (planner_ok ? plan_result.value().path.size() : 0u)
    << ",\"sample_step_m\":"
    << planning_config.plan.check_step << ",\"length_m\":" << surface_trace_length
    << ",\"path\":";
  writePointArray(o, surface_trace);
  o << ",\"note\":\"Render-only projection reference: samples planner.path every "
       "check_step and snaps each sample to the nearest robot-placeable front-facing "
       "triangle, matching the SafetyValidator's own sample/projection model. It is a "
       "dense post-processed curve, NOT planner.path, NOT the displayed final path, and "
       "NOT an independent validation; the sparse planner-returned path is preserved "
       "verbatim in planning.json.path.\"},";

  std::vector<int> topology_regions;
  std::vector<int> topology_portals;
  if (planner_ok) {
    for (const topmap::RegionId id : plan_result.value().topology.regions) {
      topology_regions.push_back(static_cast<int>(id));
    }
    for (const topmap::PortalId id : plan_result.value().topology.portals) {
      topology_portals.push_back(static_cast<int>(id));
    }
  }
  o << "\"topology\":{\"regions\":";
  writeIntArray(o, topology_regions);
  o << ",\"portals\":";
  writeIntArray(o, topology_portals);
  o << ",\"cost\":" << (planner_ok ? plan_result.value().topology.cost : 0.0f) << "},";

  o << "\"stats\":{";
  o << "\"regions_in_corridor\":" << (stats != nullptr ? stats->regions_in_corridor : 0)
    << ",\"triangles_in_corridor\":" << (stats != nullptr ? stats->triangles_in_corridor : 0)
    << ",\"topology_expanded_nodes\":" << (stats != nullptr ? stats->topology_expanded_nodes : 0)
    << ",\"surface_expanded_nodes\":" << (stats != nullptr ? stats->surface_expanded_nodes : 0)
    << ",\"topology_time_sec\":" << (stats != nullptr ? stats->topology_time_sec : 0.0)
    << ",\"surface_time_sec\":" << (stats != nullptr ? stats->surface_time_sec : 0.0)
    << ",\"optimization_time_sec\":" << (stats != nullptr ? stats->optimization_time_sec : 0.0)
    << ",\"path_length\":" << (stats != nullptr ? stats->path_length : 0.0f)
    << ",\"min_clearance\":" << (stats != nullptr ? stats->min_clearance : 0.0f)
    << ",\"max_slope_deg\":" << (stats != nullptr ? stats->max_slope_deg : 0.0f)
    << ",\"max_curvature\":" << (stats != nullptr ? stats->max_curvature : 0.0f)
    << ",\"fallback_used\":" << ((stats != nullptr && stats->fallback_used) ? "true" : "false")
    << "},";

  o << "\"safety\":{\"valid\":" << (safety_ok ? "true" : "false")
    << ",\"min_clearance\":" << (planner_ok ? safety_report.min_clearance : 0.0f)
    << ",\"max_step\":" << (planner_ok ? safety_report.max_step : 0.0f)
    << ",\"max_slope_deg\":" << (planner_ok ? safety_report.max_slope_deg : 0.0f)
    << ",\"max_curvature\":" << (planner_ok ? safety_report.max_curvature : 0.0f)
    << ",\"violating_indices\":[";
  if (planner_ok) {
    for (std::size_t i = 0; i < safety_report.violating_indices.size(); ++i) {
      if (i) o << ',';
      o << safety_report.violating_indices[i];
    }
  }
  o << "],\"portal_order_matches\":" << (portal_order_ok ? "true" : "false")
    << ",\"crossed_portals\":[";
  for (std::size_t i = 0; i < crossed_portals.size(); ++i) {
    if (i) o << ',';
    o << static_cast<int>(crossed_portals[i]);
  }
  o << "],\"check_step\":" << planning_config.plan.check_step
    << ",\"checked_with\":\"topmap::SafetyValidator::validate + crossedPortals + "
       "portalOrderMatches\",\"claimed_scope\":\"path safety validation only; not a full "
       "robot safety certification\"},";
  o << "\"disclaimer\":\"Fixed demo endpoints, precomputed offline by the production "
       "HierarchicalPlanner and replayed by RViz; this launch is not interactive "
       "replanning. The returned path is checked against the dedicated surface tolerance, "
       "clearance, ground-steering curvature and Portal sequence. This is not a full robot "
       "safety certification. The separate surface_trace is a render-only reference and "
       "never replaces the actual planner output.\"";
  o << "}\n";
  o.close();
  if (!o) {
    std::cerr << "failed while writing planning output: " << planning_path << '\n';
    return 15;
  }

  if (success) {
    std::cout << "planning_ok=1 path_points=" << plan_result.value().path.size()
              << " path_length=" << plan_result.value().stats.path_length
              << " surface_trace_points=" << surface_trace.size()
              << " surface_trace_length=" << surface_trace_length
              << " regions=" << topology_regions.size()
              << " portals=" << topology_portals.size()
              << " min_clearance=" << safety_report.min_clearance
              << " max_slope_deg=" << safety_report.max_slope_deg
              << " max_curvature=" << safety_report.max_curvature
              << " fallback=" << (plan_result.value().stats.fallback_used ? 1 : 0) << '\n';
    return 0;
  }
  std::cerr << "planning failed (" << error_code_name << "): " << error_message << '\n';
  return planner_ok ? 17 : 16;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 5 && argc != 6 && argc != 7) {
    std::cerr << "usage: export_stages <input.pcd> <config.yaml> "
                 "<scene_metadata.json> <out.json> [planning_out.json] "
                 "[archive_out.bin]\n";
    return 2;
  }
  const std::string pcd_path = argv[1];
  const std::string config_path = argv[2];
  const std::string metadata_path = argv[3];
  const std::string out_path = argv[4];
  const std::string planning_path = argc >= 6 ? argv[5] : std::string();
  const std::string archive_path = argc == 7 ? argv[6] : std::string();

  // ---- 0. configuration ----------------------------------------------------
  auto config_result = topmap::loadTopmapConfig(config_path);
  if (!config_result.ok()) {
    std::cerr << "config load failed: " << config_result.status().message << '\n';
    return 3;
  }
  const topmap::TopmapConfig& config = config_result.value();

  // ---- 1. actual PCD pipeline ---------------------------------------------
  auto input = topmap::PointCloudProcessor::loadPCD(pcd_path);
  if (!input.ok()) {
    std::cerr << "loadPCD failed: " << input.status().message << '\n';
    return 4;
  }
  auto processed = topmap::PointCloudProcessor::process(input.value(), config.map);
  if (!processed.ok()) {
    std::cerr << "process failed: " << processed.status().message << '\n';
    return 5;
  }
  auto features = topmap::SurfaceFeatureEstimator::estimate(processed.value(), config.map);
  if (!features.ok()) {
    std::cerr << "feature estimation failed: " << features.status().message << '\n';
    return 6;
  }

  std::vector<Eigen::Vector3f> points, normals;
  std::vector<float> roughness, curvature;
  points.reserve(features.value().size());
  normals.reserve(features.value().size());
  roughness.reserve(features.value().size());
  curvature.reserve(features.value().size());
  for (const auto& f : features.value()) {
    points.push_back(f.position);
    normals.push_back(f.normal);
    roughness.push_back(f.roughness);
    curvature.push_back(f.curvature);
  }

  topmap::MeshBuildParams mesh_params;
  mesh_params.max_edge_length = config.map.max_edge_length;
  mesh_params.max_normal_deviation_deg = config.map.max_normal_deviation_deg;
  mesh_params.prefer_layered = config.map.prefer_layered_triangulation;

  auto triangulated = topmap::SurfaceTriangulator::triangulate(
      points, normals, mesh_params, &roughness, &curvature);
  if (!triangulated.ok()) {
    std::cerr << "triangulation failed code=" << static_cast<int>(triangulated.status().code)
              << '\n';
    return 7;
  }
  const topmap::MeshData& raw_mesh = triangulated.value().mesh;
  const int triangulated_face_count = static_cast<int>(raw_mesh.faces.size());

  auto mesh_result = topmap::TriangleMesh::fromMeshData(raw_mesh, mesh_params);
  if (!mesh_result.ok()) {
    std::cerr << "fromMeshData failed: " << mesh_result.status().message << '\n';
    return 8;
  }
  topmap::TriangleMesh& mesh = mesh_result.value();
  topmap::TraversabilityEvaluator::apply(mesh, config.traversability);

  // ---- 2. topology pipeline ------------------------------------------------
  auto grown = topmap::RegionGrower().grow(mesh, config.region);
  if (!grown.ok()) {
    std::cerr << "region growing failed: " << grown.status().message << '\n';
    return 9;
  }
  auto split = topmap::RegionSplitter().split(mesh, grown.value(), config.region);
  if (!split.ok()) {
    std::cerr << "region split failed: " << split.status().message << '\n';
    return 10;
  }
  auto candidates = topmap::PortalExtractor().extract(mesh, split.value(), config.portal);
  if (!candidates.ok()) {
    std::cerr << "portal extraction failed: " << candidates.status().message << '\n';
    return 11;
  }
  auto validated = topmap::PortalValidator().validateDetailed(
      mesh, split.value(), candidates.value(), config.robot, config.portal);
  if (!validated.ok()) {
    std::cerr << "portal validation failed: " << validated.status().message << '\n';
    return 12;
  }
  const std::vector<topmap::Portal>& valid_portals = validated.value().portals;
  const std::vector<topmap::PortalRejection>& rejections = validated.value().rejections;

  auto graph = topmap::TopologyGraph::build(split.value(), valid_portals);
  if (!graph.ok()) {
    std::cerr << "topology graph build failed: " << graph.status().message << '\n';
    return 13;
  }
  auto compressed = topmap::TopologyCompressor().compress(
      graph.value(), config.compression, config.region);
  if (!compressed.ok()) {
    std::cerr << "topology compression failed: " << compressed.status().message << '\n';
    return 14;
  }
  const topmap::TopologyGraph& final_graph = compressed.value().first;
  const topmap::CompressionStats& compression_stats = compressed.value().second;

  // The Region structs carry no portal back-references by themselves; take them
  // from the (validated) graph so every exported region lists its real portals.
  std::vector<topmap::SurfaceRegion> split_regions = split.value();
  for (auto& r : split_regions) r.portal_ids = graph.value().portalsOf(r.id);
  std::vector<topmap::SurfaceRegion> final_regions = final_graph.regions();
  for (auto& r : final_regions) r.portal_ids = final_graph.portalsOf(r.id);

  // ---- 3. derived geometry -------------------------------------------------
  const std::vector<Eigen::Vector3f>& vertices = mesh.vertices();
  const std::vector<topmap::TriangleNode>& triangles = mesh.triangles();
  std::vector<Eigen::Vector3f> centroids, tri_normals;
  std::vector<float> areas, slopes, min_clearances, tri_roughness, tri_curvature;
  std::vector<char> traversable;
  centroids.reserve(triangles.size());
  tri_normals.reserve(triangles.size());
  areas.reserve(triangles.size());
  slopes.reserve(triangles.size());
  min_clearances.reserve(triangles.size());
  tri_roughness.reserve(triangles.size());
  tri_curvature.reserve(triangles.size());
  traversable.reserve(triangles.size());
  for (const auto& t : triangles) {
    centroids.push_back(t.centroid);
    tri_normals.push_back(t.normal);
    areas.push_back(t.area);
    slopes.push_back(t.slope);
    min_clearances.push_back(t.min_clearance);
    tri_roughness.push_back(t.roughness);
    tri_curvature.push_back(t.curvature);
    traversable.push_back(t.traversable ? 1 : 0);
  }

  // Unique undirected triangle edges (i < j), sorted, exactly once each.  The
  // RViz stage-1 wireframe draws these so shared edges are not drawn twice.
  std::set<std::pair<int, int>> edge_set;
  for (const auto& t : triangles) {
    for (int e = 0; e < 3; ++e) {
      int a = t.vertex_ids[e];
      int b = t.vertex_ids[(e + 1) % 3];
      if (a > b) std::swap(a, b);
      edge_set.emplace(a, b);
    }
  }
  std::vector<std::pair<int, int>> unique_edges(edge_set.begin(), edge_set.end());

  int traversable_count = 0;
  for (char value : traversable) traversable_count += value;

  // ---- 4. validation (no filtering, only diagnostics) ----------------------
  std::vector<std::string> errors;
  const int vertex_count = static_cast<int>(vertices.size());
  const int face_count = static_cast<int>(triangles.size());
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    if (!finite3(vertices[i])) {
      errors.push_back("vertex " + std::to_string(i) + " is not finite");
      if (errors.size() > 32) break;
    }
  }
  for (std::size_t i = 0; i < triangles.size(); ++i) {
    if (!finite3(triangles[i].centroid) || !finite3(triangles[i].normal)) {
      errors.push_back("triangle " + std::to_string(i) + " has a non-finite centroid/normal");
      if (errors.size() > 32) break;
    }
    for (int k = 0; k < 3; ++k) {
      const int id = triangles[i].vertex_ids[k];
      if (id < 0 || id >= vertex_count) {
        errors.push_back("triangle " + std::to_string(i) + " vertex index out of range");
        break;
      }
    }
    if (errors.size() > 32) break;
  }
  const auto check_regions = [&](const std::vector<topmap::SurfaceRegion>& regions,
                                 const char* label) {
    for (const auto& r : regions) {
      if (!finite3(r.centroid)) {
        errors.push_back(std::string(label) + " region " + std::to_string(r.id) +
                         " has a non-finite centroid");
      }
      for (topmap::TriangleId id : r.triangle_ids) {
        if (id < 0 || id >= face_count) {
          errors.push_back(std::string(label) + " region " + std::to_string(r.id) +
                           " references an out-of-range triangle");
          break;
        }
      }
      if (errors.size() > 32) break;
    }
  };
  check_regions(grown.value(), "grown");
  check_regions(split_regions, "split");
  check_regions(final_regions, "final");
  for (const auto& p : final_graph.portals()) {
    if (!finite3(p.left) || !finite3(p.right) || !finite3(p.center)) {
      errors.push_back("portal " + std::to_string(p.id) + " has a non-finite endpoint");
    }
    if (!final_graph.hasRegion(p.region_a) || !final_graph.hasRegion(p.region_b)) {
      errors.push_back("portal " + std::to_string(p.id) + " references an unknown region");
    }
    if (p.boundary_chain.empty()) {
      errors.push_back("portal " + std::to_string(p.id) + " has an empty boundary chain");
    }
    for (topmap::PointId pid : p.boundary_chain) {
      if (pid < 0 || pid >= vertex_count) {
        errors.push_back("portal " + std::to_string(p.id) +
                         " boundary chain references an out-of-range vertex");
        break;
      }
    }
    if (errors.size() > 32) break;
  }

  // ---- 4b. save the same mesh/topology as a loadable MapArchive -----------
  // The online topmap_ros planner_node loads this file once and replans with the
  // production HierarchicalPlanner.  It is deliberately written from the very
  // same mesh and compressed final_graph that produce stages.json, not rebuilt
  // from the JSON later.
  if (!archive_path.empty()) {
    topmap::MapArchive archive;
    archive.format_version = topmap::kTopmapFormatVersion;
    archive.metadata.frame_id = "map";
    archive.metadata.robot = config.robot;
    archive.metadata.map = config.map;
    archive.metadata.point_count = static_cast<std::uint32_t>(mesh.vertices().size());
    archive.metadata.triangle_count = static_cast<std::uint32_t>(mesh.size());
    archive.metadata.region_count =
        static_cast<std::uint32_t>(final_graph.regions().size());
    archive.metadata.portal_count =
        static_cast<std::uint32_t>(final_graph.portals().size());
    archive.metadata.connected_components =
        static_cast<std::uint32_t>(final_graph.connectedComponents());
    archive.metadata.regions_before_compression =
        static_cast<std::uint32_t>(graph.value().regions().size());
    archive.mesh = mesh;
    archive.graph = final_graph;
    const topmap::Status saved = topmap::saveMapArchive(archive_path, archive);
    if (!saved.ok()) {
      std::cerr << "map archive write failed: " << saved.message << '\n';
      return 15;
    }
    std::cout << "archive_ok=1 archive_vertices=" << archive.mesh.vertices().size()
              << " archive_faces=" << archive.mesh.size()
              << " archive_regions=" << archive.graph.regions().size()
              << " archive_portals=" << archive.graph.portals().size() << '\n';
  }

  // ---- 5. serialize --------------------------------------------------------
  bool metadata_read_ok = false;
  const std::string metadata_raw = trim(readTextFile(metadata_path, &metadata_read_ok));

  std::ofstream o(out_path);
  if (!o) {
    std::cerr << "cannot open output: " << out_path << '\n';
    return 15;
  }
  o << std::setprecision(7);

  int cycle_rank = static_cast<int>(final_graph.portals().size()) -
                   static_cast<int>(final_regions.size()) +
                   final_graph.connectedComponents();

  o << '{';
  o << "\"schema\":\"topmap_stage_demo.stages\",\"schema_version\":1,";
  o << "\"generator\":\"export_stages.cpp\",";

  o << "\"source\":{\"pcd\":" << std::quoted(pcd_path)
    << ",\"config\":" << std::quoted(config_path)
    << ",\"metadata_file\":" << std::quoted(metadata_path)
    << ",\"metadata\":";
  if (metadata_read_ok && !metadata_raw.empty()) {
    o << metadata_raw;
  } else {
    o << "null";
  }
  o << ",\"scene\":\"flat_H_ground_original_ramp\","
       "\"type\":\"program_generated_synthetic_scene\","
       "\"note\":\"Synthetic fixture. Presentation copies are translations of "
       "the same data; all triangles are preserved.\"},";

  o << "\"config\":{";
  o << "\"robot\":{\"width\":" << config.robot.width << ",\"length\":" << config.robot.length
    << ",\"max_slope_deg\":" << config.robot.max_slope_deg
    << ",\"max_step_height\":" << config.robot.max_step_height
    << ",\"safety_distance\":" << config.robot.safety_distance
    << ",\"max_path_curvature\":" << config.robot.max_path_curvature
    << ",\"required_width\":" << config.robot.required_width()
    << ",\"required_clearance\":" << config.robot.required_clearance() << "},";
  o << "\"map\":{\"voxel_size\":" << config.map.voxel_size
    << ",\"outlier_k\":" << config.map.outlier_k
    << ",\"outlier_std_ratio\":" << config.map.outlier_std_ratio
    << ",\"normal_search_radius\":" << config.map.normal_search_radius
    << ",\"max_edge_length\":" << config.map.max_edge_length
    << ",\"max_normal_deviation_deg\":" << config.map.max_normal_deviation_deg
    << ",\"prefer_layered_triangulation\":"
    << (config.map.prefer_layered_triangulation ? "true" : "false")
    << ",\"keep_raw_cloud\":" << (config.map.keep_raw_cloud ? "true" : "false") << "},";
  o << "\"traversability\":{\"max_slope_deg\":" << config.traversability.max_slope_deg
    << ",\"max_normal_diff_deg\":" << config.traversability.max_normal_diff_deg
    << ",\"max_slope_diff_deg\":" << config.traversability.max_slope_diff_deg
    << ",\"max_step_height\":" << config.traversability.max_step_height
    << ",\"allow_slope_transition\":"
    << (config.traversability.allow_slope_transition ? "true" : "false")
    << ",\"max_roughness\":" << config.traversability.max_roughness
    << ",\"max_surface_curvature\":" << config.traversability.max_surface_curvature
    << ",\"min_triangle_area\":" << config.traversability.min_triangle_area
    << ",\"required_clearance\":" << config.traversability.required_clearance << "},";
  o << "\"region\":{\"normal_weight\":" << config.region.normal_weight
    << ",\"slope_weight\":" << config.region.slope_weight
    << ",\"roughness_weight\":" << config.region.roughness_weight
    << ",\"height_weight\":" << config.region.height_weight
    << ",\"region_score_threshold\":" << config.region.region_score_threshold
    << ",\"max_height_span\":" << config.region.max_height_span
    << ",\"slope_regime_deg\":" << config.region.slope_regime_deg
    << ",\"height_span_across_regime_only\":"
    << (config.region.height_span_across_regime_only ? "true" : "false")
    << ",\"max_slope_variance\":" << config.region.max_slope_variance
    << ",\"min_region_area\":" << config.region.min_region_area
    << ",\"min_region_triangles\":" << config.region.min_region_triangles
    << ",\"bottleneck_ratio\":" << config.region.bottleneck_ratio
    << ",\"bottleneck_abs_ceiling\":" << config.region.bottleneck_abs_ceiling
    << ",\"min_side_area\":" << config.region.min_side_area
    << ",\"concavity_branch_min_arm_length\":" << config.region.concavity_branch_min_arm_length
    << ",\"concavity_branch_angle_deg\":" << config.region.concavity_branch_angle_deg
    << ",\"ramp_slope_deg\":" << config.region.ramp_slope_deg << "},";
  o << "\"portal\":{\"boundary_min_edges\":" << config.portal.boundary_min_edges
    << ",\"width_margin\":" << config.portal.width_margin << "},";
  o << "\"compression\":{\"max_merge_cost\":" << config.compression.max_merge_cost
    << ",\"preserve_topology\":" << (config.compression.preserve_topology ? "true" : "false")
    << ",\"forbid_cross_regime_merge\":"
    << (config.compression.forbid_cross_regime_merge ? "true" : "false") << "},";
  o << "\"plan\":{\"max_project_radius\":" << config.plan.max_project_radius
    << ",\"path_surface_tolerance\":" << config.plan.path_surface_tolerance
    << ",\"max_project_normal_diff_deg\":" << config.plan.max_project_normal_diff_deg
    << ",\"corridor_expansion_rings\":" << config.plan.corridor_expansion_rings
    << ",\"fallback_full_mesh\":" << (config.plan.fallback_full_mesh ? "true" : "false")
    << ",\"shortcut_enabled\":" << (config.plan.shortcut_enabled ? "true" : "false")
    << ",\"funnel_enabled\":" << (config.plan.funnel_enabled ? "true" : "false")
    << ",\"smoothing_enabled\":" << (config.plan.smoothing_enabled ? "true" : "false")
    << ",\"smoothing_iterations\":" << config.plan.smoothing_iterations
    << ",\"check_step\":" << config.plan.check_step
    << ",\"topology_weight\":" << config.plan.topology_weight << "}},";

  o << "\"filters\":{\"preserved_all_triangles\":"
    << (mesh.discardedFaces() == 0 ? "true" : "false")
    << ",\"removed_triangles\":" << mesh.discardedFaces()
    << ",\"note\":\"No triangle is deleted or filtered. Tiny synthetic "
       "triangulation boundary bridge faces are intentionally kept.\"},";

  o << "\"counts\":{"
    << "\"input_points\":" << input.value().size()
    << ",\"processed_points\":" << processed.value().size()
    << ",\"feature_points\":" << features.value().size()
    << ",\"triangulated_face_count\":" << triangulated_face_count
    << ",\"vertex_count\":" << vertex_count
    << ",\"face_count\":" << face_count
    << ",\"removed_triangles\":" << mesh.discardedFaces()
    << ",\"unique_edge_count\":" << unique_edges.size()
    << ",\"traversable_triangle_count\":" << traversable_count
    << ",\"grown_region_count\":" << grown.value().size()
    << ",\"split_region_count\":" << split.value().size()
    << ",\"final_region_count\":" << final_regions.size()
    << ",\"portal_candidate_count\":" << candidates.value().size()
    << ",\"portal_valid_count\":" << valid_portals.size()
    << ",\"portal_rejected_count\":" << rejections.size()
    << ",\"final_portal_count\":" << final_graph.portals().size()
    << ",\"connected_components\":" << final_graph.connectedComponents()
    << ",\"cycle_rank\":" << cycle_rank
    << ",\"compression_merges_applied\":" << compression_stats.merges_applied
    << ",\"compression_merges_rejected\":" << compression_stats.merges_rejected
    << ",\"compression_rejected_by_invariant\":" << compression_stats.rejected_by_invariant
    << ",\"compression_rejected_by_regime\":" << compression_stats.rejected_by_regime
    << ",\"triangulation_mode\":"
    << std::quoted(triangulated.value().mode == topmap::TriangulationMode::kDelaunay3DSurface
                       ? "3D_AFSR"
                       : "layered_2.5D_Delaunay")
    << ",\"triangulation_components\":" << triangulated.value().component_count
    << ",\"triangulation_discarded\":" << triangulated.value().discarded_triangles << "},";

  o << "\"mesh\":{\"vertices\":[";
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    if (i) o << ',';
    writeVec(o, vertices[i]);
  }
  o << "],\"faces\":[";
  for (std::size_t i = 0; i < triangles.size(); ++i) {
    if (i) o << ',';
    o << '[' << triangles[i].vertex_ids[0] << ',' << triangles[i].vertex_ids[1] << ','
      << triangles[i].vertex_ids[2] << ']';
  }
  o << "],\"unique_edges\":[";
  for (std::size_t i = 0; i < unique_edges.size(); ++i) {
    if (i) o << ',';
    o << '[' << unique_edges[i].first << ',' << unique_edges[i].second << ']';
  }
  o << "],\"centroids\":[";
  for (std::size_t i = 0; i < centroids.size(); ++i) {
    if (i) o << ',';
    writeVec(o, centroids[i]);
  }
  o << "],\"normals\":[";
  for (std::size_t i = 0; i < tri_normals.size(); ++i) {
    if (i) o << ',';
    writeVec(o, tri_normals[i]);
  }
  o << "],\"areas\":";
  writeFloatArray(o, areas);
  o << ",\"slopes\":";
  writeFloatArray(o, slopes);
  o << ",\"min_clearances\":";
  writeFloatArray(o, min_clearances);
  o << ",\"roughness\":";
  writeFloatArray(o, tri_roughness);
  o << ",\"curvature\":";
  writeFloatArray(o, tri_curvature);
  o << ",\"traversable\":[";
  for (std::size_t i = 0; i < traversable.size(); ++i) {
    if (i) o << ',';
    o << (traversable[i] ? "true" : "false");
  }
  o << "]},";

  o << "\"stages\":{";
  o << "\"grown\":{\"label\":\"initial grown regions\",\"regions\":";
  writeRegions(o, grown.value());
  o << "},\"split\":{\"label\":\"split regions\",\"regions\":";
  writeRegions(o, split_regions);
  o << "},\"final\":{\"label\":\"final compressed regions\",\"regions\":";
  writeRegions(o, final_regions);
  o << "}},";

  o << "\"topology\":{\"regions\":";
  writeRegions(o, final_regions);
  o << ",\"portals\":[";
  bool first = true;
  for (const auto& p : final_graph.portals()) {
    if (!first) o << ',';
    first = false;
    o << "{\"id\":" << p.id << ",\"region_a\":" << p.region_a << ",\"region_b\":" << p.region_b
      << ",\"left\":";
    writeVec(o, p.left);
    o << ",\"right\":";
    writeVec(o, p.right);
    o << ",\"center\":";
    writeVec(o, p.center);
    o << ",\"raw_width\":" << p.raw_width << ",\"valid_width\":" << p.valid_width
      << ",\"mean_slope\":" << p.mean_slope << ",\"max_step\":" << p.max_step
      << ",\"min_clearance\":" << p.min_clearance
      << ",\"traversal_cost\":" << p.traversal_cost
      << ",\"is_critical\":" << (p.is_critical ? "true" : "false")
      << ",\"boundary_chain\":[";
    for (std::size_t i = 0; i < p.boundary_chain.size(); ++i) {
      if (i) o << ',';
      o << p.boundary_chain[i];
    }
    o << "],\"boundary_points\":[";
    for (std::size_t i = 0; i < p.boundary_chain.size(); ++i) {
      if (i) o << ',';
      writeVec(o, vertices[static_cast<std::size_t>(p.boundary_chain[i])]);
    }
    o << "]}";
  }
  o << "],\"rejected_portals\":[";
  first = true;
  for (const auto& r : rejections) {
    if (!first) o << ',';
    first = false;
    o << "{\"id\":" << r.id << ",\"reason\":" << std::quoted(topmap::toString(r.reason))
      << ",\"error_code\":" << static_cast<int>(r.code) << ",\"detail\":" << std::quoted(r.detail)
      << "}";
  }
  o << "],\"connected_components\":" << final_graph.connectedComponents()
    << ",\"cycle_rank\":" << cycle_rank << "},";

  o << "\"validation\":{\"ok\":" << (errors.empty() ? "true" : "false") << ",\"errors\":[";
  for (std::size_t i = 0; i < errors.size(); ++i) {
    if (i) o << ',';
    o << std::quoted(errors[i]);
  }
  o << "]}}";
  o << '\n';
  o.close();

  if (!errors.empty()) {
    std::cerr << "self-validation failed with " << errors.size() << " error(s)\n";
    for (const auto& e : errors) std::cerr << "  " << e << '\n';
    return 1;
  }

  std::cout << "input=" << input.value().size() << " processed=" << processed.value().size()
            << " vertices=" << vertex_count << " faces=" << face_count
            << " unique_edges=" << unique_edges.size() << " mode="
            << (triangulated.value().mode == topmap::TriangulationMode::kDelaunay3DSurface
                    ? "3D_AFSR"
                    : "layered_2.5D_Delaunay")
            << " grown=" << grown.value().size() << " split=" << split.value().size()
            << " final_regions=" << final_regions.size()
            << " candidates=" << candidates.value().size()
            << " valid_portals=" << valid_portals.size()
            << " final_portals=" << final_graph.portals().size()
            << " components=" << final_graph.connectedComponents() << '\n';

  // ---- 6. optional real plan on the same mesh/graph ------------------------
  if (!planning_path.empty()) {
    topmap::PlanRequest request;
    request.start = Eigen::Vector3f(2.0f, 3.5f, 0.0f);   // left hall, safe interior
    request.goal = Eigen::Vector3f(28.4f, 3.5f, 2.0f);   // upper platform interior
    const int planning_code =
        writePlanningFile(planning_path, pcd_path, config_path, config, mesh, final_graph,
                          request);
    if (planning_code != 0) return planning_code;
  }
  return 0;
}
