#include "topmap_topology/map_archive.hpp"

#include <unistd.h>

#include <cstdio>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "topmap_surface/mesh_serialization.hpp"

namespace topmap {
namespace {

constexpr const char* kSectionMeta = "meta";
constexpr const char* kSectionVertices = "vertices";
constexpr const char* kSectionFaces = "faces";
constexpr const char* kSectionTriangleAttrs = "triangle_attrs";
constexpr const char* kSectionRegions = "regions";
constexpr const char* kSectionPortals = "portals";
constexpr const char* kSectionGraph = "graph";

constexpr std::uint32_t kSectionCount = 7u;
// A section name is always one of the fixed ASCII names above; bounding the
// length keeps a corrupt file from allocating an attacker-sized string.
constexpr std::uint32_t kMaxSectionNameLength = 256u;
constexpr std::size_t kVertexBytes = 3u * 4u;
constexpr std::size_t kFaceBytes = 3u * 4u;
// Per triangle: centroid + normal (6 f32) + 5 attribute f32 + 1 bool.
constexpr std::size_t kTriangleAttrBytes = 6u * 4u + 5u * 4u + 1u;
// Lower bounds used to reject absurd counts before reserving.
constexpr std::size_t kMinRegionBytes = 4u + 4u + 12u + 7u * 4u + 1u + 1u + 4u;
constexpr std::size_t kMinPortalBytes = 3u * 4u + 4u + 3u * 12u + 6u * 4u + 1u;

Status IoError(const std::string& message) {
  return Status::Error(ErrorCode::kSerializationIoError, message);
}

// --- raw little-endian framing helpers -------------------------------------

bool Take(const std::vector<std::uint8_t>& data, std::size_t* offset, std::size_t count,
          const std::uint8_t** out) {
  if (count > data.size() - *offset) return false;
  if (out != nullptr) *out = data.data() + *offset;
  *offset += count;
  return true;
}

std::uint32_t LoadU32(const std::uint8_t* bytes) {
  return static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8u) |
         (static_cast<std::uint32_t>(bytes[2]) << 16u) | (static_cast<std::uint32_t>(bytes[3]) << 24u);
}

std::uint64_t LoadU64(const std::uint8_t* bytes) {
  std::uint64_t value = 0;
  for (int i = 0; i < 8; ++i)
    value |= static_cast<std::uint64_t>(bytes[i]) << (8u * static_cast<unsigned>(i));
  return value;
}

void WriteSection(BinaryWriter& writer, const std::string& name,
                  const std::vector<std::uint8_t>& payload) {
  writer.writeU32(static_cast<std::uint32_t>(name.size()));
  for (char c : name) writer.writeU8(static_cast<std::uint8_t>(c));
  writer.writeU64(static_cast<std::uint64_t>(payload.size()));
  for (std::uint8_t b : payload) writer.writeU8(b);
}

// --- fixed-record helpers ---------------------------------------------------

void WriteRegion(BinaryWriter& writer, const SurfaceRegion& region) {
  writer.writeI32(region.id);
  writer.writeVecI32(region.triangle_ids);
  writer.writeVector3f(region.centroid);
  writer.writeF32(region.area);
  writer.writeF32(region.min_z);
  writer.writeF32(region.max_z);
  writer.writeF32(region.mean_slope);
  writer.writeF32(region.max_slope);
  writer.writeF32(region.slope_variance);
  writer.writeF32(region.roughness);
  writer.writeBool(region.is_ramp);
  writer.writeBool(region.is_topology_critical);
  writer.writeVecI32(region.portal_ids);
}

bool ReadRegion(BinaryReader& reader, SurfaceRegion* region) {
  if (!reader.readI32(region->id)) return false;
  if (!reader.readVecI32(region->triangle_ids)) return false;
  if (!reader.readVector3f(region->centroid)) return false;
  if (!reader.readF32(region->area) || !reader.readF32(region->min_z) ||
      !reader.readF32(region->max_z) || !reader.readF32(region->mean_slope) ||
      !reader.readF32(region->max_slope) || !reader.readF32(region->slope_variance) ||
      !reader.readF32(region->roughness)) {
    return false;
  }
  if (!reader.readBool(region->is_ramp) || !reader.readBool(region->is_topology_critical))
    return false;
  return reader.readVecI32(region->portal_ids);
}

void WritePortal(BinaryWriter& writer, const Portal& portal) {
  writer.writeI32(portal.id);
  writer.writeI32(portal.region_a);
  writer.writeI32(portal.region_b);
  writer.writeVecI32(portal.boundary_chain);
  writer.writeVector3f(portal.left);
  writer.writeVector3f(portal.right);
  writer.writeVector3f(portal.center);
  writer.writeF32(portal.raw_width);
  writer.writeF32(portal.valid_width);
  writer.writeF32(portal.mean_slope);
  writer.writeF32(portal.max_step);
  writer.writeF32(portal.min_clearance);
  writer.writeF32(portal.traversal_cost);
  writer.writeBool(portal.is_critical);
}

bool ReadPortal(BinaryReader& reader, Portal* portal) {
  if (!reader.readI32(portal->id) || !reader.readI32(portal->region_a) ||
      !reader.readI32(portal->region_b)) {
    return false;
  }
  if (!reader.readVecI32(portal->boundary_chain)) return false;
  if (!reader.readVector3f(portal->left) || !reader.readVector3f(portal->right) ||
      !reader.readVector3f(portal->center)) {
    return false;
  }
  if (!reader.readF32(portal->raw_width) || !reader.readF32(portal->valid_width) ||
      !reader.readF32(portal->mean_slope) || !reader.readF32(portal->max_step) ||
      !reader.readF32(portal->min_clearance) || !reader.readF32(portal->traversal_cost)) {
    return false;
  }
  return reader.readBool(portal->is_critical);
}

// Splits the T11 combined mesh payload into its three archive sections.
Status WriteMeshSections(BinaryWriter& vertices_writer, BinaryWriter& faces_writer,
                         BinaryWriter& attrs_writer, const TriangleMesh& mesh) {
  if (mesh.vertices().size() > 0xffffffffull || mesh.size() > 0xffffffffull)
    return IoError("mesh is too large for the serialized count fields");

  vertices_writer.writeU32(static_cast<std::uint32_t>(mesh.vertices().size()));
  for (const Eigen::Vector3f& vertex : mesh.vertices()) vertices_writer.writeVector3f(vertex);

  faces_writer.writeU32(static_cast<std::uint32_t>(mesh.size()));
  attrs_writer.writeU32(static_cast<std::uint32_t>(mesh.size()));
  for (const TriangleNode& triangle : mesh.triangles()) {
    for (const PointId vertex_id : triangle.vertex_ids) faces_writer.writeI32(vertex_id);
    attrs_writer.writeVector3f(triangle.centroid);
    attrs_writer.writeVector3f(triangle.normal);
    attrs_writer.writeF32(triangle.slope);
    attrs_writer.writeF32(triangle.roughness);
    attrs_writer.writeF32(triangle.curvature);
    attrs_writer.writeF32(triangle.area);
    attrs_writer.writeF32(triangle.min_clearance);
    attrs_writer.writeBool(triangle.traversable);
  }
  return Status::Ok();
}

// Reads the three mesh sections back, reassembles the exact T11 payload and
// rebuilds through readTriangleMesh so the archive and T11 share one contract.
Result<TriangleMesh> ReadMeshSections(const std::vector<std::uint8_t>& vertices_payload,
                                      const std::vector<std::uint8_t>& faces_payload,
                                      const std::vector<std::uint8_t>& attrs_payload) {
  BinaryReader vertices_reader(vertices_payload);
  std::uint32_t vertex_count = 0;
  if (!vertices_reader.readU32(vertex_count)) return IoError("short read on vertex count");
  if (vertex_count > vertices_reader.remaining() / kVertexBytes)
    return IoError("vertex count exceeds the vertices section");
  std::vector<Eigen::Vector3f> vertices;
  vertices.reserve(vertex_count);
  for (std::uint32_t i = 0; i < vertex_count; ++i) {
    Eigen::Vector3f vertex;
    if (!vertices_reader.readVector3f(vertex)) return IoError("short read on vertex data");
    vertices.push_back(vertex);
  }

  BinaryReader faces_reader(faces_payload);
  std::uint32_t face_count = 0;
  if (!faces_reader.readU32(face_count)) return IoError("short read on face count");
  if (face_count > faces_reader.remaining() / kFaceBytes)
    return IoError("face count exceeds the faces section");
  std::vector<std::array<PointId, 3>> faces;
  faces.reserve(face_count);
  for (std::uint32_t i = 0; i < face_count; ++i) {
    std::array<PointId, 3> face{};
    for (int k = 0; k < 3; ++k) {
      if (!faces_reader.readI32(face[k])) return IoError("short read on face vertex ids");
    }
    faces.push_back(face);
  }

  BinaryReader attrs_reader(attrs_payload);
  std::uint32_t attr_count = 0;
  if (!attrs_reader.readU32(attr_count)) return IoError("short read on triangle attribute count");
  if (attr_count > attrs_reader.remaining() / kTriangleAttrBytes)
    return IoError("triangle attribute count exceeds the triangle_attrs section");
  if (attr_count != face_count) return IoError("face and triangle attribute counts differ");
  std::vector<TriangleNode> attrs;
  attrs.reserve(attr_count);
  for (std::uint32_t i = 0; i < attr_count; ++i) {
    TriangleNode node;
    if (!attrs_reader.readVector3f(node.centroid) || !attrs_reader.readVector3f(node.normal) ||
        !attrs_reader.readF32(node.slope) || !attrs_reader.readF32(node.roughness) ||
        !attrs_reader.readF32(node.curvature) || !attrs_reader.readF32(node.area) ||
        !attrs_reader.readF32(node.min_clearance) || !attrs_reader.readBool(node.traversable)) {
      return IoError("short read on triangle attributes");
    }
    attrs.push_back(node);
  }

  // Reassemble the T11 payload (vertices + per-triangle ids and attributes) and
  // let readTriangleMesh validate and derive adjacency/components.
  BinaryWriter combined;
  combined.writeU32(vertex_count);
  for (const Eigen::Vector3f& vertex : vertices) combined.writeVector3f(vertex);
  combined.writeU32(face_count);
  for (std::uint32_t i = 0; i < face_count; ++i) {
    for (const PointId vertex_id : faces[i]) combined.writeI32(vertex_id);
    combined.writeVector3f(attrs[i].centroid);
    combined.writeVector3f(attrs[i].normal);
    combined.writeF32(attrs[i].slope);
    combined.writeF32(attrs[i].roughness);
    combined.writeF32(attrs[i].curvature);
    combined.writeF32(attrs[i].area);
    combined.writeF32(attrs[i].min_clearance);
    combined.writeBool(attrs[i].traversable);
  }

  BinaryReader reader(combined.bytes());
  Result<TriangleMesh> mesh = readTriangleMesh(reader);
  if (!mesh.ok()) return IoError("invalid mesh payload: " + mesh.status().message);
  return mesh;
}

}  // namespace

Status saveMapArchive(const std::string& path, const MapArchive& archive) {
  BinaryWriter writer;
  writer.writeU32(kTopmapMagic);
  writer.writeU32(archive.format_version);
  writer.writeU32(kTopmapEndianMarker);
  writer.writeU32(kSectionCount);

  {
    BinaryWriter payload;
    payload.writeString(archive.metadata.frame_id);
    const RobotParams& robot = archive.metadata.robot;
    payload.writeF32(robot.width);
    payload.writeF32(robot.length);
    payload.writeF32(robot.max_slope_deg);
    payload.writeF32(robot.max_step_height);
    payload.writeF32(robot.safety_distance);
    payload.writeF32(robot.max_path_curvature);
    const MapBuildParams& map = archive.metadata.map;
    payload.writeF32(map.voxel_size);
    payload.writeI32(map.outlier_k);
    payload.writeF32(map.outlier_std_ratio);
    payload.writeF32(map.normal_search_radius);
    payload.writeF32(map.max_edge_length);
    payload.writeF32(map.max_normal_deviation_deg);
    payload.writeBool(map.prefer_layered_triangulation);
    payload.writeBool(map.keep_raw_cloud);
    payload.writeU32(archive.metadata.point_count);
    payload.writeU32(archive.metadata.triangle_count);
    payload.writeU32(archive.metadata.region_count);
    payload.writeU32(archive.metadata.portal_count);
    payload.writeU32(archive.metadata.connected_components);
    payload.writeU32(archive.metadata.regions_before_compression);
    payload.writeF64(archive.metadata.build_time_sec);
    payload.writeF64(archive.metadata.memory_mb);
    WriteSection(writer, kSectionMeta, payload.bytes());
  }

  {
    BinaryWriter vertices;
    BinaryWriter faces;
    BinaryWriter attrs;
    const Status status = WriteMeshSections(vertices, faces, attrs, archive.mesh);
    if (!status.ok()) return status;
    WriteSection(writer, kSectionVertices, vertices.bytes());
    WriteSection(writer, kSectionFaces, faces.bytes());
    WriteSection(writer, kSectionTriangleAttrs, attrs.bytes());
  }

  {
    BinaryWriter payload;
    payload.writeU32(static_cast<std::uint32_t>(archive.graph.regions().size()));
    for (const SurfaceRegion& region : archive.graph.regions()) WriteRegion(payload, region);
    WriteSection(writer, kSectionRegions, payload.bytes());
  }

  {
    BinaryWriter payload;
    payload.writeU32(static_cast<std::uint32_t>(archive.graph.portals().size()));
    for (const Portal& portal : archive.graph.portals()) WritePortal(payload, portal);
    WriteSection(writer, kSectionPortals, payload.bytes());
  }

  {
    BinaryWriter payload;
    payload.writeU32(static_cast<std::uint32_t>(archive.graph.regions().size()));
    for (const SurfaceRegion& region : archive.graph.regions()) {
      payload.writeI32(region.id);
      const std::vector<RegionId>& origins = archive.graph.originRegions(region.id);
      payload.writeU32(static_cast<std::uint32_t>(origins.size()));
      for (const RegionId origin : origins) payload.writeI32(origin);
    }
    payload.writeU32(static_cast<std::uint32_t>(archive.graph.portals().size()));
    for (const Portal& portal : archive.graph.portals()) {
      payload.writeI32(portal.id);
      payload.writeI32(portal.region_a);
      payload.writeI32(portal.region_b);
    }
    WriteSection(writer, kSectionGraph, payload.bytes());
  }

  // Atomic replace: write a same-directory temporary, flush it, then rename so
  // an existing target is never left half-written. A failed write removes it.
  const std::string temp =
      path + ".topmap.tmp." + std::to_string(static_cast<long>(::getpid()));
  {
    std::ofstream out(temp, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) return IoError("cannot open " + temp + " for writing");
    out.write(reinterpret_cast<const char*>(writer.bytes().data()),
              static_cast<std::streamsize>(writer.bytes().size()));
    out.flush();
    if (!out) {
      out.close();
      std::remove(temp.c_str());
      return IoError("failed to write map archive to " + temp);
    }
    out.close();
    if (out.fail()) {
      std::remove(temp.c_str());
      return IoError("failed to flush map archive to " + temp);
    }
  }
  if (std::rename(temp.c_str(), path.c_str()) != 0) {
    std::remove(temp.c_str());
    return IoError("failed to move map archive into place at " + path);
  }
  return Status::Ok();
}

Result<MapArchive> loadMapArchive(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) return IoError("cannot open map archive " + path);
  std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                                  std::istreambuf_iterator<char>());
  if (in.bad()) return IoError("failed to read map archive " + path);

  std::size_t offset = 0;
  const std::uint8_t* cursor = nullptr;
  if (!Take(bytes, &offset, 4, &cursor)) return IoError("archive is shorter than its header");
  const std::uint32_t magic = LoadU32(cursor);
  if (magic != kTopmapMagic)
    return Status::Error(ErrorCode::kSerializationMagicMismatch,
                         "map archive magic does not match");
  if (!Take(bytes, &offset, 4, &cursor)) return IoError("archive is truncated after magic");
  const std::uint32_t version = LoadU32(cursor);
  if (version > kTopmapFormatVersion) {
    return Status::Error(ErrorCode::kSerializationVersionUnsupported,
                         "map archive version " + std::to_string(version) + " is newer than " +
                             std::to_string(kTopmapFormatVersion));
  }
  if (!Take(bytes, &offset, 4, &cursor)) return IoError("archive is truncated after version");
  if (LoadU32(cursor) != kTopmapEndianMarker)
    return IoError("map archive endian marker does not match");
  if (!Take(bytes, &offset, 4, &cursor))
    return IoError("archive is truncated after endian marker");
  const std::uint32_t section_count = LoadU32(cursor);

  std::unordered_map<std::string, std::vector<std::uint8_t>> sections;
  for (std::uint32_t i = 0; i < section_count; ++i) {
    if (!Take(bytes, &offset, 4, &cursor)) return IoError("short read on section name length");
    const std::uint32_t name_length = LoadU32(cursor);
    if (name_length > kMaxSectionNameLength) return IoError("section name is unreasonably long");
    if (!Take(bytes, &offset, name_length, &cursor))
      return IoError("short read on section name");
    const std::string name(reinterpret_cast<const char*>(cursor), name_length);
    if (!Take(bytes, &offset, 8, &cursor)) return IoError("short read on section length");
    const std::uint64_t byte_length = LoadU64(cursor);
    if (byte_length > bytes.size() - offset)
      return IoError("section " + name + " length exceeds the remaining file");
    if (!Take(bytes, &offset, static_cast<std::size_t>(byte_length), &cursor))
      return IoError("short read on section " + name + " payload");
    std::vector<std::uint8_t> payload(cursor, cursor + static_cast<std::size_t>(byte_length));
    if (!sections.emplace(name, std::move(payload)).second)
      return IoError("duplicate section " + name);
  }

  const char* const required[] = {kSectionMeta,     kSectionVertices, kSectionFaces,
                                  kSectionTriangleAttrs, kSectionRegions, kSectionPortals,
                                  kSectionGraph};
  for (const char* name : required) {
    if (sections.find(name) == sections.end())
      return IoError(std::string("missing required section ") + name);
  }

  MapMetadata metadata;
  {
    BinaryReader reader(sections.at(kSectionMeta));
    if (!reader.readString(metadata.frame_id)) return IoError("short read on frame_id");
    RobotParams& robot = metadata.robot;
    if (!reader.readF32(robot.width) || !reader.readF32(robot.length) ||
        !reader.readF32(robot.max_slope_deg) || !reader.readF32(robot.max_step_height) ||
        !reader.readF32(robot.safety_distance) || !reader.readF32(robot.max_path_curvature)) {
      return IoError("short read on robot parameters");
    }
    MapBuildParams& map = metadata.map;
    if (!reader.readF32(map.voxel_size) || !reader.readI32(map.outlier_k) ||
        !reader.readF32(map.outlier_std_ratio) || !reader.readF32(map.normal_search_radius) ||
        !reader.readF32(map.max_edge_length) || !reader.readF32(map.max_normal_deviation_deg) ||
        !reader.readBool(map.prefer_layered_triangulation) ||
        !reader.readBool(map.keep_raw_cloud)) {
      return IoError("short read on map build parameters");
    }
    if (!reader.readU32(metadata.point_count) || !reader.readU32(metadata.triangle_count) ||
        !reader.readU32(metadata.region_count) || !reader.readU32(metadata.portal_count) ||
        !reader.readU32(metadata.connected_components) ||
        !reader.readU32(metadata.regions_before_compression) ||
        !reader.readF64(metadata.build_time_sec) || !reader.readF64(metadata.memory_mb)) {
      return IoError("short read on map metadata counters");
    }
  }

  Result<TriangleMesh> mesh = ReadMeshSections(sections.at(kSectionVertices),
                                               sections.at(kSectionFaces),
                                               sections.at(kSectionTriangleAttrs));
  if (!mesh.ok()) return mesh.status();

  std::vector<SurfaceRegion> regions;
  {
    BinaryReader reader(sections.at(kSectionRegions));
    std::uint32_t count = 0;
    if (!reader.readU32(count)) return IoError("short read on region count");
    if (count > reader.remaining() / kMinRegionBytes)
      return IoError("region count exceeds the regions section");
    regions.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
      SurfaceRegion region;
      if (!ReadRegion(reader, &region)) return IoError("short read on region record");
      regions.push_back(std::move(region));
    }
  }

  std::vector<Portal> portals;
  {
    BinaryReader reader(sections.at(kSectionPortals));
    std::uint32_t count = 0;
    if (!reader.readU32(count)) return IoError("short read on portal count");
    if (count > reader.remaining() / kMinPortalBytes)
      return IoError("portal count exceeds the portals section");
    portals.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
      Portal portal;
      if (!ReadPortal(reader, &portal)) return IoError("short read on portal record");
      portals.push_back(std::move(portal));
    }
  }

  Result<TopologyGraph> built = TopologyGraph::build(regions, portals);
  if (!built.ok()) return IoError("corrupt graph: " + built.status().message);
  TopologyGraph graph = built.take();

  {
    BinaryReader reader(sections.at(kSectionGraph));
    std::uint32_t region_records = 0;
    if (!reader.readU32(region_records)) return IoError("short read on graph region count");
    if (region_records > graph.regions().size())
      return IoError("graph section describes more regions than the regions section");

    std::vector<std::pair<RegionId, std::vector<RegionId>>> origin_entries;
    std::unordered_map<RegionId, RegionId> origin_owner;
    std::unordered_set<RegionId> seen;
    for (std::uint32_t i = 0; i < region_records; ++i) {
      RegionId region_id = kInvalidId;
      if (!reader.readI32(region_id)) return IoError("short read on graph region id");
      if (!graph.hasRegion(region_id)) return IoError("graph references a missing region");
      if (!seen.insert(region_id).second)
        return IoError("duplicate region record in the graph section");
      std::uint32_t origin_count = 0;
      if (!reader.readU32(origin_count)) return IoError("short read on origin count");
      if (origin_count > reader.remaining() / 4u)
        return IoError("origin count exceeds the graph section");
      std::vector<RegionId> origins;
      origins.reserve(origin_count);
      for (std::uint32_t k = 0; k < origin_count; ++k) {
        RegionId origin = kInvalidId;
        if (!reader.readI32(origin)) return IoError("short read on origin id");
        if (origin < 0) return IoError("negative origin id");
        const auto inserted = origin_owner.emplace(origin, region_id);
        if (!inserted.second && inserted.first->second != region_id)
          return IoError("origin id belongs to more than one region");
        origins.push_back(origin);
      }
      origin_entries.push_back({region_id, std::move(origins)});
    }
    if (seen.size() != graph.regions().size())
      return IoError("graph section does not describe every region");

    std::uint32_t edge_count = 0;
    if (!reader.readU32(edge_count)) return IoError("short read on graph edge count");
    if (edge_count > reader.remaining() / (3u * 4u))
      return IoError("edge count exceeds the graph section");
    for (std::uint32_t i = 0; i < edge_count; ++i) {
      PortalId portal_id = kInvalidId;
      RegionId region_a = kInvalidId;
      RegionId region_b = kInvalidId;
      if (!reader.readI32(portal_id) || !reader.readI32(region_a) || !reader.readI32(region_b))
        return IoError("short read on graph edge");
      const Portal* portal = graph.portal(portal_id);
      if (portal == nullptr) return IoError("graph edge references a missing portal");
      if (!graph.hasRegion(region_a) || !graph.hasRegion(region_b))
        return IoError("graph edge references a missing region");
      const bool matches = (portal->region_a == region_a && portal->region_b == region_b) ||
                           (portal->region_a == region_b && portal->region_b == region_a);
      if (!matches) return IoError("graph edge disagrees with the portal endpoints");
    }

    // Apply provenance only after the whole graph section validated. Clearing
    // first lets a renumbered super Region absorb an origin whose numeric id is
    // also some other current region's id.
    for (const SurfaceRegion& region : graph.regions()) graph.setOriginRegions(region.id, {});
    for (const auto& entry : origin_entries) graph.setOriginRegions(entry.first, entry.second);
  }

  MapArchive archive;
  archive.format_version = version;
  archive.metadata = std::move(metadata);
  archive.mesh = mesh.take();
  archive.graph = std::move(graph);
  return archive;
}

}  // namespace topmap
