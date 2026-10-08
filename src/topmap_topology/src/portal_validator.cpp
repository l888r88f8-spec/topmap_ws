#include "topmap_topology/portal_validator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "topmap_surface/traversability.hpp"

namespace topmap {
namespace {

std::pair<PointId, PointId> edgeKey(PointId a, PointId b) {
  return {std::min(a, b), std::max(a, b)};
}

// Area-weighted mean slope over the faces that touch the crossing edges.
float weightedMeanSlope(const TriangleMesh& mesh, const std::set<TriangleId>& triangles) {
  double sum = 0.0;
  double area = 0.0;
  for (TriangleId t : triangles) {
    const TriangleNode& node = mesh.triangle(t);
    sum += static_cast<double>(node.slope) * node.area;
    area += node.area;
  }
  return area > 0.0 ? static_cast<float>(sum / area) : 0.0f;
}

}  // namespace

const char* toString(PortalRejectionReason reason) {
  switch (reason) {
    case PortalRejectionReason::kNone: return "none";
    case PortalRejectionReason::kInvalidRegion: return "invalid_region";
    case PortalRejectionReason::kInvalidChain: return "invalid_chain";
    case PortalRejectionReason::kTooNarrow: return "too_narrow";
    case PortalRejectionReason::kSlope: return "slope";
    case PortalRejectionReason::kStep: return "step";
    case PortalRejectionReason::kClearance: return "clearance";
  }
  return "unknown";
}

float portalLocalWidth(const TriangleMesh& mesh, const std::vector<TriangleId>& region_union,
                       TriangleId seed) {
  if (seed < 0 || seed >= static_cast<TriangleId>(mesh.size())) return 0.0f;
  const std::vector<float> distance = regionBoundaryDistance(mesh, region_union);
  return 2.0f * distance[static_cast<std::size_t>(seed)];
}

Result<PortalValidationReport> PortalValidator::validateDetailed(
    const TriangleMesh& mesh, const std::vector<SurfaceRegion>& regions,
    const std::vector<Portal>& candidates, const RobotParams& robot,
    const PortalParams& params) const {
  if (mesh.size() == 0) {
    return Status::Error(ErrorCode::kMeshEmpty, "cannot validate portals on an empty mesh");
  }
  if (regions.empty()) {
    return Status::Error(ErrorCode::kRegionEmpty, "cannot validate portals without regions");
  }
  if (candidates.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument, "no portal candidates to validate");
  }
  if (params.boundary_min_edges < 1) {
    return Status::Error(ErrorCode::kInvalidArgument, "portal.boundary_min_edges must be >= 1");
  }
  if (!(robot.required_width() > 0.0f)) {
    return Status::Error(ErrorCode::kInvalidArgument, "robot.required_width() must be positive");
  }

  // Region ownership, mirroring PortalExtractor's contract.
  std::vector<RegionId> triangle_region(static_cast<std::size_t>(mesh.size()), kInvalidId);
  std::set<RegionId> region_ids;
  for (const SurfaceRegion& region : regions) {
    if (region.id < 0) {
      return Status::Error(ErrorCode::kInvalidArgument, "region has a negative id");
    }
    if (!region_ids.insert(region.id).second) {
      return Status::Error(ErrorCode::kInvalidArgument,
                           "duplicate RegionId " + std::to_string(region.id));
    }
    for (TriangleId t : region.triangle_ids) {
      if (t < 0 || t >= static_cast<TriangleId>(mesh.size())) {
        return Status::Error(ErrorCode::kInvalidArgument,
                             "region references an out-of-range triangle id");
      }
      if (triangle_region[t] != kInvalidId) {
        return Status::Error(ErrorCode::kInvalidArgument,
                             "triangle belongs to more than one region");
      }
      triangle_region[t] = region.id;
    }
  }

  PortalValidationReport report;
  for (const Portal& candidate : candidates) {
    if (candidate.region_a == candidate.region_b || region_ids.count(candidate.region_a) == 0 ||
        region_ids.count(candidate.region_b) == 0) {
      return Status::Error(ErrorCode::kInvalidArgument,
                           "portal references a missing or identical region");
    }
    if (candidate.boundary_chain.size() < 2u) {
      report.rejections.push_back(PortalRejection{candidate.id, PortalRejectionReason::kInvalidChain,
                                                  ErrorCode::kPortalValidationFailed,
                                                  "boundary chain has fewer than two vertices"});
      continue;
    }

    std::set<std::pair<PointId, PointId>> chain_edges;
    for (std::size_t k = 1; k < candidate.boundary_chain.size(); ++k) {
      chain_edges.insert(edgeKey(candidate.boundary_chain[k - 1], candidate.boundary_chain[k]));
    }

    // The crossing is the set of mesh edges between the two Regions whose
    // endpoints belong to this chain. valid_width is sampled along the chain
    // from the region-union boundary-distance field (R8's
    // regionBoundaryDistance, never TriangleNode.min_clearance): for each
    // crossing edge the free distance on the two sides of the chain normal is
    // 2 * regionBoundaryDistance(seed), and the narrowest sample over the whole
    // chain is the portal width.
    std::vector<TriangleId> region_union;
    for (const SurfaceRegion& region : regions) {
      if (region.id != candidate.region_a && region.id != candidate.region_b) continue;
      region_union.insert(region_union.end(), region.triangle_ids.begin(),
                          region.triangle_ids.end());
    }
    const std::vector<float> free_distance = regionBoundaryDistance(mesh, region_union);

    float valid_width = std::numeric_limits<float>::infinity();
    float best_clearance = -1.0f;
    float max_step = 0.0f;
    std::set<TriangleId> touched;
    for (TriangleId t = 0; t < static_cast<TriangleId>(mesh.size()); ++t) {
      if (triangle_region[t] != candidate.region_a) continue;
      for (const TriangleEdge& e : mesh.triangle(t).edges) {
        const TriangleId u = e.neighbor;
        if (u == kInvalidId || triangle_region[u] != candidate.region_b) continue;
        if (e.v0 == kInvalidId || e.v1 == kInvalidId) continue;
        if (chain_edges.count(edgeKey(e.v0, e.v1)) == 0) continue;
        const float side_a = 2.0f * free_distance[static_cast<std::size_t>(t)];
        const float side_b = 2.0f * free_distance[static_cast<std::size_t>(u)];
        valid_width = std::min(valid_width, std::min(side_a, side_b));
        // min_clearance stays the R8 footprint-erosion yardstick; it is never
        // mixed with the region-union width field.
        const float capacity =
            std::min(mesh.triangle(t).min_clearance, mesh.triangle(u).min_clearance);
        best_clearance = std::max(best_clearance, capacity);
        max_step = std::max(max_step, TraversabilityEvaluator::heightJump(mesh.triangle(t),
                                                                          mesh.triangle(u)));
        touched.insert(t);
        touched.insert(u);
      }
    }

    if (best_clearance < 0.0f || !std::isfinite(valid_width)) {
      report.rejections.push_back(PortalRejection{
          candidate.id, PortalRejectionReason::kInvalidChain, ErrorCode::kPortalValidationFailed,
          "boundary chain does not match any shared edge between the two regions"});
      continue;
    }

    Portal portal = candidate;
    portal.valid_width = valid_width;
    portal.min_clearance = best_clearance;
    portal.max_step = max_step;
    portal.mean_slope = weightedMeanSlope(mesh, touched);
    portal.traversal_cost =
        1.0f + 0.5f * std::max(0.0f, 1.0f - portal.valid_width / robot.required_width());
    portal.is_critical = false;  // T18 computes criticality.

    // Every violated criterion is reported, not just the first, so callers can
    // see the full structured reason set for a candidate.
    std::vector<PortalRejection> failures;
    const auto reject = [&](PortalRejectionReason reason, ErrorCode code, std::string detail) {
      failures.push_back(PortalRejection{candidate.id, reason, code, std::move(detail)});
    };
    const float width_threshold = robot.required_width() + params.width_margin;
    if (!(portal.valid_width > width_threshold)) {
      reject(PortalRejectionReason::kTooNarrow, ErrorCode::kPortalTooNarrow,
             "valid_width=" + std::to_string(portal.valid_width) +
                 " is not > required_width+margin=" + std::to_string(width_threshold));
    }
    if (!(portal.mean_slope <= robot.max_slope_rad())) {
      reject(PortalRejectionReason::kSlope, ErrorCode::kPortalValidationFailed,
             "mean_slope=" + std::to_string(portal.mean_slope) +
                 " exceeds " + std::to_string(robot.max_slope_rad()));
    }
    if (!(portal.max_step <= robot.max_step_height)) {
      reject(PortalRejectionReason::kStep, ErrorCode::kPortalValidationFailed,
             "max_step=" + std::to_string(portal.max_step) +
                 " exceeds " + std::to_string(robot.max_step_height));
    }
    if (!(portal.min_clearance >= robot.required_clearance())) {
      reject(PortalRejectionReason::kClearance, ErrorCode::kPortalValidationFailed,
             "min_clearance=" + std::to_string(portal.min_clearance) +
                 " is below required_clearance=" + std::to_string(robot.required_clearance()));
    }
    if (failures.empty()) {
      report.portals.push_back(std::move(portal));
    } else {
      for (PortalRejection& failure : failures) {
        report.rejections.push_back(std::move(failure));
      }
    }
  }
  return report;
}

Result<std::vector<Portal>> PortalValidator::validate(const TriangleMesh& mesh,
                                                      const std::vector<SurfaceRegion>& regions,
                                                      std::vector<Portal> candidates,
                                                      const RobotParams& robot,
                                                      const PortalParams& params) const {
  auto report = validateDetailed(mesh, regions, candidates, robot, params);
  if (!report.ok()) return report.status();
  if (report.value().portals.empty()) {
    std::string message = "all portal candidates were rejected:";
    for (const PortalRejection& rejection : report.value().rejections) {
      message += " [" + std::string(toString(rejection.reason)) + "]";
    }
    return Status::Error(ErrorCode::kPortalValidationFailed, message);
  }
  std::vector<Portal> accepted = std::move(report.value().portals);
  return accepted;
}

}  // namespace topmap
