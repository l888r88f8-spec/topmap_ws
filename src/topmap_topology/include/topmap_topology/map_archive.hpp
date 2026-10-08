#pragma once

#include <cstdint>
#include <string>

#include "topmap_core/binary_io.hpp"
#include "topmap_core/error.hpp"
#include "topmap_core/params.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_topology/topology_graph.hpp"

namespace topmap {

// A4.7 MapMetadata. R14: carries every MapStats.msg field so T29 can fill the
// ROS message without re-deriving values from the graph.
struct MapMetadata {
  std::string frame_id = "map";
  RobotParams robot;
  MapBuildParams map;
  std::uint32_t point_count = 0;
  std::uint32_t triangle_count = 0, region_count = 0, portal_count = 0;
  std::uint32_t connected_components = 0;
  std::uint32_t regions_before_compression = 0;
  double build_time_sec = 0.0;
  double memory_mb = 0.0;
};

struct MapArchive {
  std::uint32_t format_version = kTopmapFormatVersion;
  MapMetadata metadata;
  TriangleMesh mesh;
  TopologyGraph graph;
};

// T20 versioned container. Layout (all little-endian):
//   [magic u32][version u32][endian u32][section_count u32]
//   section_count * [name_len u32][name][byte_len u64][payload]
// The 7 sections are written in the fixed order
//   meta / vertices / faces / triangle_attrs / regions / portals / graph.
// Loading requires all 7, skips unknown sections by byte_len and ignores any
// trailing bytes. Any structural defect is a structured serialization error.
Status saveMapArchive(const std::string& path, const MapArchive& archive);
Result<MapArchive> loadMapArchive(const std::string& path);

}  // namespace topmap
