#include "topmap_planner/topology_astar.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace topmap {
namespace {

constexpr int kRegionType = 0;
constexpr int kPortalType = 1;

// A bipartite search state. `id` is looked up in the appropriate side of the
// graph, so Region 0 and Portal 0 never collide.
struct NodeKey {
  int type = kRegionType;
  int id = kInvalidId;
  bool operator==(const NodeKey& other) const {
    return type == other.type && id == other.id;
  }
  bool operator<(const NodeKey& other) const {
    return type != other.type ? type < other.type : id < other.id;
  }
};

struct QueueNode {
  float f = 0.0f;
  float g = 0.0f;
  int type = kRegionType;
  int id = kInvalidId;
};

// Strict total order: smallest f, then largest g (prefer the state closer to
// the goal on an f tie), then Region before Portal, then smallest id. The
// priority_queue is a max-heap, so every comparison is inverted.
struct QueueCompare {
  bool operator()(const QueueNode& a, const QueueNode& b) const {
    if (a.f != b.f) return a.f > b.f;
    if (a.g != b.g) return a.g < b.g;
    if (a.type != b.type) return a.type > b.type;
    return a.id > b.id;
  }
};

Eigen::Vector3f NodePosition(const TopologyGraph& graph, int type, int id) {
  if (type == kRegionType) {
    const SurfaceRegion* region = graph.region(id);
    return region != nullptr ? region->centroid : Eigen::Vector3f::Zero();
  }
  const Portal* portal = graph.portal(id);
  return portal != nullptr ? portal->center : Eigen::Vector3f::Zero();
}

}  // namespace

Result<TopologyPath> TopologyAStar::search(const TopologyGraph& graph, RegionId start,
                                           RegionId goal, float weight,
                                           int* expanded_nodes) const {
  if (expanded_nodes != nullptr) *expanded_nodes = 0;

  if (!std::isfinite(weight) || !(weight > 0.0f)) {
    return Status::Error(ErrorCode::kInvalidArgument, "topology weight must be finite and positive");
  }
  if (!graph.hasRegion(start)) {
    return Status::Error(ErrorCode::kInvalidArgument, "start region does not exist in the graph");
  }
  if (!graph.hasRegion(goal)) {
    return Status::Error(ErrorCode::kInvalidArgument, "goal region does not exist in the graph");
  }

  if (start == goal) {
    TopologyPath path;
    path.regions = {start};
    path.cost = 0.0f;
    return path;
  }

  const Eigen::Vector3f goal_position = graph.region(goal)->centroid;
  const auto heuristic = [&graph, &goal_position, weight](int type, int id) {
    return (NodePosition(graph, type, id) - goal_position).norm() * weight;
  };

  std::map<NodeKey, float> best_g;
  std::map<NodeKey, NodeKey> parent;
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;

  const NodeKey start_key{kRegionType, start};
  const NodeKey goal_key{kRegionType, goal};
  best_g[start_key] = 0.0f;
  open.push(QueueNode{heuristic(kRegionType, start), 0.0f, kRegionType, start});

  const auto relax = [&](const NodeKey& from, int type, int id, float g) {
    const NodeKey to{type, id};
    const auto known = best_g.find(to);
    if (known != best_g.end() && !(g < known->second)) return;
    best_g[to] = g;
    parent[to] = from;
    open.push(QueueNode{g + heuristic(type, id), g, type, id});
  };

  while (!open.empty()) {
    const QueueNode current = open.top();
    open.pop();

    const NodeKey key{current.type, current.id};
    const auto known = best_g.find(key);
    if (known == best_g.end() || current.g > known->second) continue;  // stale entry
    if (expanded_nodes != nullptr) ++(*expanded_nodes);

    if (key == goal_key) {
      std::vector<NodeKey> chain;
      for (NodeKey node = goal_key;;) {
        chain.push_back(node);
        if (node == start_key) break;
        node = parent.at(node);
      }
      std::reverse(chain.begin(), chain.end());

      TopologyPath path;
      for (const NodeKey& node : chain) {
        if (node.type == kRegionType) {
          path.regions.push_back(node.id);
        } else {
          path.portals.push_back(node.id);
        }
      }
      path.cost = best_g.at(goal_key);
      return path;
    }

    if (current.type == kRegionType) {
      const SurfaceRegion* region = graph.region(current.id);
      for (const PortalId portal_id : graph.portalsOf(current.id)) {
        const Portal* portal = graph.portal(portal_id);
        const float edge = (region->centroid - portal->center).norm();
        relax(key, kPortalType, portal_id, current.g + edge);
      }
    } else {
      const Portal* portal = graph.portal(current.id);
      for (const RegionId region_id : {portal->region_a, portal->region_b}) {
        const SurfaceRegion* region = graph.region(region_id);
        const float edge = portal->traversal_cost + (portal->center - region->centroid).norm();
        relax(key, kRegionType, region_id, current.g + edge);
      }
    }
  }

  return Status::Error(ErrorCode::kTopologyUnreachable,
                       "no topology path connects the start and goal regions");
}

}  // namespace topmap
