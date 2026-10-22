// Standalone offline timing harness for the production topmap planner.
//
// Loads a saved MapArchive and the real TopmapConfig, then runs the production
// topmap::HierarchicalPlanner. Archive loading and graph/mesh setup are outside
// every timed section. The reported plan wall time is one complete plan() call;
// the optimizer time is PlanStats.optimization_time_sec from that call.
//
// The default request is the demo start/goal. One warm-up run is discarded and
// five measured runs are reported (median included). Each measured path is
// independently checked with topmap::SafetyValidator and compared with the
// first measured path so a timing comparison cannot silently change semantics.
//
// Usage:
//   measure_planning <archive.bin> <config.yaml> <output.json> [warmup] [runs]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "topmap_core/params.hpp"
#include "topmap_planner/hierarchical_planner.hpp"
#include "topmap_planner/plan_types.hpp"
#include "topmap_planner/safety_validator.hpp"
#include "topmap_surface/traversability.hpp"
#include "topmap_topology/map_archive.hpp"

namespace {

using Clock = std::chrono::steady_clock;

double SecondsBetween(Clock::time_point begin, Clock::time_point end) {
  return std::chrono::duration<double>(end - begin).count();
}

void WriteVec(std::ostream& out, const Eigen::Vector3f& v) {
  out << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}

void WritePath(std::ostream& out, const std::vector<Eigen::Vector3f>& path) {
  out << '[';
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (i) out << ',';
    WriteVec(out, path[i]);
  }
  out << ']';
}

void WriteIntArray(std::ostream& out, const std::vector<int>& values) {
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i) out << ',';
    out << values[i];
  }
  out << ']';
}

double Median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const std::size_t mid = values.size() / 2u;
  if (values.size() % 2u == 1u) return values[mid];
  return 0.5 * (values[mid - 1u] + values[mid]);
}

float MaxPathDifference(const std::vector<Eigen::Vector3f>& a,
                        const std::vector<Eigen::Vector3f>& b) {
  if (a.size() != b.size()) return std::numeric_limits<float>::infinity();
  float worst = 0.0f;
  for (std::size_t i = 0; i < a.size(); ++i) {
    worst = std::max(worst, (a[i] - b[i]).cwiseAbs().maxCoeff());
  }
  return worst;
}

struct Run {
  double plan_wall_sec = 0.0;
  topmap::PlanResult plan;
  topmap::PathValidationReport safety;
  bool portal_order_ok = false;
  bool safety_ok = false;
  float path_diff_from_first = 0.0f;
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4 || argc > 6) {
    std::cerr << "usage: measure_planning <archive.bin> <config.yaml> <output.json> "
                 "[warmup=1] [runs=5]\n";
    return 2;
  }
  const std::string archive_path = argv[1];
  const std::string config_path = argv[2];
  const std::string output_path = argv[3];
  const int warmup = argc >= 5 ? std::max(0, std::atoi(argv[4])) : 1;
  const int measured = argc >= 6 ? std::max(1, std::atoi(argv[5])) : 5;

  // Everything below this point is setup, never timed.
  auto config = topmap::loadTopmapConfig(config_path);
  if (!config.ok()) {
    std::cerr << "config error: " << config.status().message << '\n';
    return 3;
  }
  auto archive = topmap::loadMapArchive(archive_path);
  if (!archive.ok()) {
    std::cerr << "archive error: " << archive.status().message << '\n';
    return 4;
  }

  // loadMapArchive restores stored attributes but not derived edge
  // traversability; planner_node re-derives it the same way. This is map setup,
  // so it happens before any timer.
  topmap::TraversabilityEvaluator::apply(archive.value().mesh,
                                         config.value().traversability);

  const topmap::PlanRequest request{Eigen::Vector3f(2.0f, 3.5f, 0.0f),
                                    Eigen::Vector3f(28.4f, 3.5f, 2.0f)};
  const topmap::HierarchicalPlanner planner(config.value());
  const topmap::SafetyValidator validator;

  for (int i = 0; i < warmup; ++i) {
    const auto result = planner.plan(archive.value().mesh, archive.value().graph, request);
    if (!result.ok()) {
      std::cerr << "warm-up plan failed: " << result.status().message << '\n';
      return 5;
    }
  }

  std::vector<Run> runs;
  runs.reserve(static_cast<std::size_t>(measured));
  for (int i = 0; i < measured; ++i) {
    Run run;
    const Clock::time_point begin = Clock::now();
    auto result = planner.plan(archive.value().mesh, archive.value().graph, request);
    run.plan_wall_sec = SecondsBetween(begin, Clock::now());
    if (!result.ok()) {
      std::cerr << "measured plan failed: " << result.status().message << '\n';
      return 6;
    }
    run.plan = std::move(result.value());
    run.safety = validator.validate(
        run.plan.path, archive.value().mesh, config.value().traversability,
        config.value().robot, config.value().plan.check_step,
        config.value().plan.path_surface_tolerance);
    run.portal_order_ok = topmap::SafetyValidator::portalOrderMatches(
        run.plan.path, archive.value().mesh, archive.value().graph,
        run.plan.topology.portals);
    run.safety_ok = run.safety.valid && run.portal_order_ok;
    run.path_diff_from_first =
        runs.empty() ? 0.0f : MaxPathDifference(runs.front().plan.path, run.plan.path);
    runs.push_back(std::move(run));
  }

  std::vector<double> optimization_times;
  std::vector<double> wall_times;
  for (const Run& run : runs) {
    optimization_times.push_back(run.plan.stats.optimization_time_sec);
    wall_times.push_back(run.plan_wall_sec);
  }
  const double optimization_median = Median(optimization_times);
  const double wall_median = Median(wall_times);

  std::ofstream out(output_path);
  if (!out) {
    std::cerr << "cannot open output: " << output_path << '\n';
    return 7;
  }
  out << std::setprecision(10);
  out << '{';
  out << "\"schema\":\"topmap_stage_demo.measure_planning\",\"schema_version\":1,";
  out << "\"archive\":" << std::quoted(archive_path)
      << ",\"config\":" << std::quoted(config_path)
      << ",\"request\":{\"start\":";
  WriteVec(out, request.start);
  out << ",\"goal\":";
  WriteVec(out, request.goal);
  out << "},\"setup_excluded_from_timers\":true,\"warmup_runs\":" << warmup
      << ",\"measured_runs\":" << measured;
  out << ",\"method\":{\"plan_wall_sec\":\"steady_clock around one "
         "HierarchicalPlanner::plan call, excluding archive/config load\","
         "\"optimization_time_sec\":\"PlanStats.optimization_time_sec from the same "
         "call\",\"median\":\"middle value; mean of two middle values for an even "
         "count\"},";
  out << "\"triangle_count\":" << archive.value().mesh.size()
      << ",\"region_count\":" << archive.value().graph.regions().size()
      << ",\"median_optimization_time_sec\":" << optimization_median
      << ",\"median_plan_wall_sec\":" << wall_median;
  out << ",\"runs\":[";
  for (std::size_t i = 0; i < runs.size(); ++i) {
    const Run& run = runs[i];
    if (i) out << ',';
    out << "{\"optimization_time_sec\":" << run.plan.stats.optimization_time_sec
        << ",\"plan_wall_sec\":" << run.plan_wall_sec
        << ",\"surface_time_sec\":" << run.plan.stats.surface_time_sec
        << ",\"topology_time_sec\":" << run.plan.stats.topology_time_sec
        << ",\"path_points\":" << run.plan.path.size()
        << ",\"path_length\":" << run.plan.stats.path_length
        << ",\"fallback_used\":" << (run.plan.stats.fallback_used ? "true" : "false")
        << ",\"safety_valid\":" << (run.safety_ok ? "true" : "false")
        << ",\"portal_order_matches\":" << (run.portal_order_ok ? "true" : "false")
        << ",\"path_diff_from_first\":" << run.path_diff_from_first << '}';
  }
  out << "],\"path\":";
  WritePath(out, runs.front().plan.path);
  out << ",\"topology\":{\"regions\":";
  std::vector<int> regions;
  for (const topmap::RegionId id : runs.front().plan.topology.regions) {
    regions.push_back(static_cast<int>(id));
  }
  WriteIntArray(out, regions);
  out << ",\"portals\":";
  std::vector<int> portals;
  for (const topmap::PortalId id : runs.front().plan.topology.portals) {
    portals.push_back(static_cast<int>(id));
  }
  WriteIntArray(out, portals);
  out << ",\"cost\":" << runs.front().plan.topology.cost << '}';
  const Run& first = runs.front();
  out << ",\"safety\":{\"valid\":" << (first.safety_ok ? "true" : "false")
      << ",\"min_clearance\":" << first.safety.min_clearance
      << ",\"max_step\":" << first.safety.max_step
      << ",\"max_slope_deg\":" << first.safety.max_slope_deg
      << ",\"max_curvature\":" << first.safety.max_curvature
      << ",\"violating_indices\":[";
  for (std::size_t i = 0; i < first.safety.violating_indices.size(); ++i) {
    if (i) out << ',';
    out << first.safety.violating_indices[i];
  }
  out << "],\"portal_order_matches\":" << (first.portal_order_ok ? "true" : "false")
      << ",\"checked_with\":\"topmap::SafetyValidator::validate + portalOrderMatches\"},";
  out << "\"stats\":" << '{'
      << "\"regions_in_corridor\":" << first.plan.stats.regions_in_corridor
      << ",\"triangles_in_corridor\":" << first.plan.stats.triangles_in_corridor
      << ",\"topology_expanded_nodes\":" << first.plan.stats.topology_expanded_nodes
      << ",\"surface_expanded_nodes\":" << first.plan.stats.surface_expanded_nodes
      << ",\"path_length\":" << first.plan.stats.path_length
      << ",\"min_clearance\":" << first.plan.stats.min_clearance
      << ",\"max_slope_deg\":" << first.plan.stats.max_slope_deg
      << ",\"max_curvature\":" << first.plan.stats.max_curvature
      << ",\"fallback_used\":" << (first.plan.stats.fallback_used ? "true" : "false")
      << "}}";
  out << '\n';
  out.close();
  if (!out) {
    std::cerr << "failed writing output: " << output_path << '\n';
    return 7;
  }

  bool all_safe = true;
  bool all_identical = true;
  for (const Run& run : runs) {
    all_safe = all_safe && run.safety_ok;
    all_identical = all_identical && run.path_diff_from_first == 0.0f;
  }
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "median_optimization_time_sec=" << optimization_median << '\n';
  std::cout << "median_plan_wall_sec=" << wall_median << '\n';
  std::cout << "path_points=" << first.plan.path.size()
            << " path_length=" << first.plan.stats.path_length
            << " safety_valid=" << (all_safe ? 1 : 0)
            << " identical_runs=" << (all_identical ? 1 : 0) << '\n';
  return all_safe ? 0 : 8;
}
