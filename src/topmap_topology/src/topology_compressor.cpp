#include "topmap_topology/topology_compressor.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "topmap_topology/topology_invariants.hpp"

namespace topmap {
namespace {

constexpr float kEps = 1e-9f;

Status Invalid(std::string message) {
  return Status::Error(ErrorCode::kInvalidArgument, std::move(message));
}

bool FiniteNonNegative(float value) { return std::isfinite(value) && value >= 0.0f; }

template <typename T>
void SortUnique(std::vector<T>& values) {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
}

// Rejects malformed thresholds/weights up front so a caller cannot silently get
// a compression that is disabled or numerically undefined.
Status ValidateParams(const CompressionParams& params, const RegionParams& region_params) {
  if (!std::isfinite(params.max_merge_cost) || params.max_merge_cost < 0.0f)
    return Invalid("max_merge_cost must be finite and non-negative");
  if (!FiniteNonNegative(region_params.normal_weight) ||
      !FiniteNonNegative(region_params.slope_weight) ||
      !FiniteNonNegative(region_params.roughness_weight))
    return Invalid("merge cost weights must be finite and non-negative");
  if (!std::isfinite(region_params.ramp_slope_deg) || region_params.ramp_slope_deg < 0.0f)
    return Invalid("ramp_slope_deg must be finite and non-negative");
  return Status::Ok();
}

// T19 merge cost, dimensionless:
//   w_n * |dnormal| / pi + w_s * |dmean_slope| / max_slope
//   + w_r * |droughness| / max_roughness
//   + |centroid_a - centroid_b| / sqrt(max(area_a, area_b))
// SurfaceRegion carries no normal, so the normal term is exactly zero. The
// slope/roughness normalisers are the frozen A4.4 traversability defaults, the
// same normalisation R1's regionJoinScore uses.
float MergeCost(const SurfaceRegion& a, const SurfaceRegion& b, const RegionParams& params) {
  const TraversabilityParams traversability;
  const float slope_term = std::fabs(a.mean_slope - b.mean_slope) /
                           std::max(traversability.max_slope_diff_rad(), kEps);
  const float roughness_term =
      std::fabs(a.roughness - b.roughness) / std::max(traversability.max_roughness, kEps);
  const float max_area = std::max(a.area, b.area);
  const float distance_term = (a.centroid - b.centroid).norm() / std::sqrt(std::max(max_area, kEps));
  // normal_weight is kept in the contract even though no normal is available.
  return params.normal_weight * 0.0f + params.slope_weight * slope_term +
         params.roughness_weight * roughness_term + distance_term;
}

struct MergeOption {
  float cost;
  RegionId region;  // absorbed region R
  RegionId target;  // surviving region T
};

bool OptionLess(const MergeOption& a, const MergeOption& b) {
  if (a.cost != b.cost) return a.cost < b.cost;
  if (a.region != b.region) return a.region < b.region;
  return a.target < b.target;
}

// Contracts `region` into `target`: folds the two SurfaceRegions (union of
// triangle/portal ids, area-weighted statistics, min/max, OR-ed flags), removes
// the (region,target) portal and reattaches every (region,other) portal to
// (target,other) while keeping parallel portals. Origins are merged so the R11
// reachability signature survives. When `preserve_topology` is set the
// reachability signature is compared and the whole contraction is rolled back
// exactly on failure (returns false; the caller records the rejection).
bool TryMerge(TopologyGraph& graph, RegionId region, RegionId target, bool preserve_topology,
              CompressionStats* stats) {
  const TopologyGraph before = graph;
  GraphSignature before_sig;
  if (preserve_topology) before_sig = TopologyInvariants::capture(graph);

  const SurfaceRegion* absorbed = graph.region(region);
  const SurfaceRegion* survivor = graph.region(target);
  if (absorbed == nullptr || survivor == nullptr) return false;

  std::vector<SurfaceRegion> regions;
  regions.reserve(graph.regions().size() - 1);
  for (const SurfaceRegion& existing : graph.regions()) {
    if (existing.id == region) continue;
    if (existing.id != target) {
      regions.push_back(existing);
      continue;
    }

    SurfaceRegion merged = existing;
    const float total_area = survivor->area + absorbed->area;
    merged.triangle_ids = survivor->triangle_ids;
    merged.triangle_ids.insert(merged.triangle_ids.end(), absorbed->triangle_ids.begin(),
                               absorbed->triangle_ids.end());
    SortUnique(merged.triangle_ids);
    if (total_area > 0.0f) {
      merged.centroid =
          (survivor->centroid * survivor->area + absorbed->centroid * absorbed->area) / total_area;
      merged.mean_slope =
          (survivor->mean_slope * survivor->area + absorbed->mean_slope * absorbed->area) /
          total_area;
      merged.slope_variance =
          (survivor->slope_variance * survivor->area + absorbed->slope_variance * absorbed->area) /
          total_area;
      merged.roughness =
          (survivor->roughness * survivor->area + absorbed->roughness * absorbed->area) / total_area;
    } else {
      merged.centroid = (survivor->centroid + absorbed->centroid) / 2.0f;
      merged.mean_slope = (survivor->mean_slope + absorbed->mean_slope) / 2.0f;
      merged.slope_variance = (survivor->slope_variance + absorbed->slope_variance) / 2.0f;
      merged.roughness = (survivor->roughness + absorbed->roughness) / 2.0f;
    }
    merged.area = total_area;
    merged.min_z = std::min(survivor->min_z, absorbed->min_z);
    merged.max_z = std::max(survivor->max_z, absorbed->max_z);
    merged.max_slope = std::max(survivor->max_slope, absorbed->max_slope);
    merged.is_ramp = survivor->is_ramp || absorbed->is_ramp;
    merged.is_topology_critical = survivor->is_topology_critical || absorbed->is_topology_critical;
    merged.portal_ids.clear();
    regions.push_back(std::move(merged));
  }

  std::vector<Portal> portals;
  portals.reserve(graph.portals().size());
  for (const Portal& portal : graph.portals()) {
    const bool touches_region = portal.region_a == region || portal.region_b == region;
    const bool touches_target = portal.region_a == target || portal.region_b == target;
    if (touches_region && touches_target) continue;  // the (region,target) link is removed
    Portal reattached = portal;
    if (reattached.region_a == region) reattached.region_a = target;
    if (reattached.region_b == region) reattached.region_b = target;
    if (reattached.region_a == reattached.region_b) continue;  // never leave a self loop
    portals.push_back(std::move(reattached));
  }

  // Keep the stored SurfaceRegion portal_ids consistent with the graph index.
  for (SurfaceRegion& stored : regions) stored.portal_ids.clear();
  for (const Portal& portal : portals) {
    for (SurfaceRegion& stored : regions) {
      if (stored.id == portal.region_a || stored.id == portal.region_b)
        stored.portal_ids.push_back(portal.id);
    }
  }
  for (SurfaceRegion& stored : regions) SortUnique(stored.portal_ids);

  std::vector<RegionId> merged_origins = graph.originRegions(target);
  const std::vector<RegionId>& absorbed_origins = graph.originRegions(region);
  merged_origins.insert(merged_origins.end(), absorbed_origins.begin(), absorbed_origins.end());

  graph.replaceRegionsPortals(std::move(regions), std::move(portals));
  // Defensive: a rejected replacement leaves the graph unchanged; do not report
  // a merge that did not happen.
  if (graph.regions().size() != before.regions().size() - 1 || graph.hasRegion(region) ||
      !graph.hasRegion(target)) {
    graph = before;
    return false;
  }
  graph.setOriginRegions(target, std::move(merged_origins));

  if (preserve_topology) {
    std::string reason;
    if (!TopologyInvariants::sameConnectivity(before_sig, TopologyInvariants::capture(graph),
                                              &reason)) {
      graph = before;  // exact rollback
      return false;
    }
  }
  ++stats->merges_applied;
  return true;
}

}  // namespace

Result<std::pair<TopologyGraph, CompressionStats>> TopologyCompressor::compress(
    const TopologyGraph& graph, const CompressionParams& params,
    const RegionParams& region_params) const {
  const Status valid = ValidateParams(params, region_params);
  if (!valid.ok()) return valid;

  CompressionStats stats;
  stats.regions_before = static_cast<int>(graph.regions().size());
  stats.portals_before = static_cast<int>(graph.portals().size());

  TopologyGraph current = graph;
  // Each region is rejected for a given cause at most once, which both keeps the
  // counters exact and guarantees the loop cannot spin on the same state.
  std::unordered_set<RegionId> invariant_rejected;
  std::unordered_set<RegionId> regime_rejected;
  std::unordered_set<RegionId> cost_rejected;

  while (true) {
    // Candidates are recomputed from the current graph every round: a degree-2
    // region with two distinct neighbours.
    std::vector<RegionId> candidates;
    for (const SurfaceRegion& region : current.regions()) {
      const std::vector<PortalId> incident = current.portalsOf(region.id);
      const std::vector<RegionId> neighbors = current.neighborRegions(region.id);
      if (incident.size() == 2 && neighbors.size() == 2) candidates.push_back(region.id);
    }
    std::sort(candidates.begin(), candidates.end());
    if (candidates.empty()) break;

    // Unique cross-layer connections are protected structurally: contracting a
    // region that carries one would erase the only link between two height bands.
    std::unordered_set<RegionId> protected_regions;
    if (params.preserve_topology) {
      for (const PortalId portal_id : TopologyInvariants::uniqueCrossLayerPortals(current)) {
        const Portal* portal = current.portal(portal_id);
        protected_regions.insert(portal->region_a);
        protected_regions.insert(portal->region_b);
      }
    }

    std::vector<MergeOption> options;
    for (const RegionId region : candidates) {
      if (invariant_rejected.count(region) != 0) continue;
      if (protected_regions.count(region) != 0) {
        if (invariant_rejected.insert(region).second) {
          ++stats.merges_rejected;
          ++stats.rejected_by_invariant;
        }
        continue;
      }

      const SurfaceRegion& source = *current.region(region);
      const std::vector<RegionId> neighbors = current.neighborRegions(region);
      bool have_best = false;
      bool any_regime_allowed = false;
      MergeOption best{0.0f, region, kInvalidId};
      for (const RegionId target : neighbors) {
        const SurfaceRegion& destination = *current.region(target);
        if (params.forbid_cross_regime_merge &&
            !sameSlopeRegime(source, destination, region_params)) {
          continue;  // structurally forbidden, not merely expensive
        }
        any_regime_allowed = true;
        const float cost = MergeCost(source, destination, region_params);
        if (!std::isfinite(cost) || cost > params.max_merge_cost) continue;
        if (!have_best || cost < best.cost || (cost == best.cost && target < best.target)) {
          best = MergeOption{cost, region, target};
          have_best = true;
        }
      }

      if (have_best) {
        options.push_back(best);
      } else if (params.forbid_cross_regime_merge && !any_regime_allowed) {
        // Both directions change slope regime: no feasible target for this region.
        if (regime_rejected.insert(region).second) {
          ++stats.merges_rejected;
          ++stats.rejected_by_regime;
        }
      } else {
        // Only the cost gate blocked the merge; sub-counters stay zero.
        if (cost_rejected.insert(region).second) ++stats.merges_rejected;
      }
    }

    std::sort(options.begin(), options.end(), OptionLess);

    bool merged = false;
    for (const MergeOption& option : options) {
      if (TryMerge(current, option.region, option.target, params.preserve_topology, &stats)) {
        merged = true;
        break;
      }
      // The contraction changed the reachability signature and was rolled back.
      if (invariant_rejected.insert(option.region).second) {
        ++stats.merges_rejected;
        ++stats.rejected_by_invariant;
      }
    }
    if (merged) continue;  // degrees changed -> recompute candidates
    break;                 // nothing applied: stop, the remaining rejects are counted
  }

  stats.regions_after = static_cast<int>(current.regions().size());
  stats.portals_after = static_cast<int>(current.portals().size());
  return std::make_pair(std::move(current), stats);
}

}  // namespace topmap
