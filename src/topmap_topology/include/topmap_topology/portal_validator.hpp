#pragma once

#include <string>
#include <vector>

#include "topmap_core/params.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_topology/region_types.hpp"

namespace topmap {

// R8: free width around `seed`, measured with regionBoundaryDistance restricted
// to the region-union subgraph. This is the local width field primitive; a
// Portal's `valid_width` is the best (widest) crossing over its boundary chain.
float portalLocalWidth(const TriangleMesh& mesh, const std::vector<TriangleId>& region_union,
                       TriangleId seed);

// Why a candidate Portal was dropped. The codes mirror the four robot-aware
// criteria plus structural problems.
enum class PortalRejectionReason {
  kNone = 0,
  kInvalidRegion,
  kInvalidChain,
  kTooNarrow,
  kSlope,
  kStep,
  kClearance,
};

const char* toString(PortalRejectionReason reason);

struct PortalRejection {
  PortalId id = kInvalidId;
  PortalRejectionReason reason = PortalRejectionReason::kNone;
  ErrorCode code = ErrorCode::kOk;
  std::string detail;
};

struct PortalValidationReport {
  std::vector<Portal> portals;
  std::vector<PortalRejection> rejections;
};

// T16: robot-aware Portal validation. Candidate geometry is re-checked against
// the mesh and the region partition:
//   * valid_width is recomputed from the region-union boundary-distance field
//     at the portal crossing (never carried over from the raw boundary-chain
//     length, never TriangleNode.min_clearance). For each crossing edge the
//     narrower of the two sides sets the local width; the portal takes the
//     widest crossing (bottleneck). The rule
//     `valid_width > robot.required_width() + params.width_margin` is strict;
//   * mean_slope <= robot.max_slope_rad();
//   * max_step <= robot.max_step_height;
//   * min_clearance >= robot.required_clearance() (the footprint erosion).
// Accepted portals get traversal_cost/is_critical filled and parallel portals
// of the same Region pair are all preserved.
class PortalValidator {
 public:
  Result<std::vector<Portal>> validate(const TriangleMesh& mesh,
                                       const std::vector<SurfaceRegion>& regions,
                                       std::vector<Portal> candidates, const RobotParams& robot,
                                       const PortalParams& params) const;

  // Same rules as validate(), but returns the structured per-candidate
  // rejection reasons instead of collapsing them into a single Status.
  Result<PortalValidationReport> validateDetailed(const TriangleMesh& mesh,
                                                  const std::vector<SurfaceRegion>& regions,
                                                  const std::vector<Portal>& candidates,
                                                  const RobotParams& robot,
                                                  const PortalParams& params) const;
};

}  // namespace topmap
