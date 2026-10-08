#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "topmap_core/binary_io.hpp"
#include "topmap_topology/map_archive.hpp"
#include "topmap_topology/topology_compressor.hpp"
#include "topmap_topology/topology_invariants.hpp"

namespace {

using topmap::BinaryWriter;
using topmap::MapArchive;
using topmap::MapMetadata;
using topmap::MeshData;
using topmap::Portal;
using topmap::PortalId;
using topmap::RegionId;
using topmap::SurfaceRegion;
using topmap::TopologyCompressor;
using topmap::TopologyGraph;
using topmap::TopologyInvariants;
using topmap::TriangleMesh;

int g_counter = 0;

std::string TempPath(const std::string& tag) {
  return "/tmp/topmap_archive_" + tag + "_" + std::to_string(static_cast<long>(::getpid())) + "_" +
         std::to_string(g_counter++) + ".bin";
}

void RemoveFile(const std::string& path) { std::remove(path.c_str()); }

std::vector<std::uint8_t> ReadFile(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(file),
                                   std::istreambuf_iterator<char>());
}

void WriteFile(const std::string& path, const std::vector<std::uint8_t>& bytes) {
  std::ofstream file(path, std::ios::binary);
  file.write(reinterpret_cast<const char*>(bytes.data()),
             static_cast<std::streamsize>(bytes.size()));
}

SurfaceRegion R(int id, float z = 0.0f) {
  SurfaceRegion r;
  r.id = id;
  r.area = 10.0f;
  r.min_z = z;
  r.max_z = z + 0.2f;
  r.centroid = Eigen::Vector3f(static_cast<float>(id), 0.0f, z);
  return r;
}

Portal P(int id, int a, int b) {
  Portal p;
  p.id = id;
  p.region_a = a;
  p.region_b = b;
  p.valid_width = 1.5f;
  return p;
}

TriangleMesh MakeMesh() {
  MeshData data;
  data.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  data.faces = {{0, 1, 2}, {0, 2, 3}};
  return TriangleMesh::fromMeshData(data, {}).take();
}

MapArchive MakeArchive() {
  MapArchive archive;
  archive.metadata.frame_id = "odom";
  archive.metadata.robot.width = 0.72f;
  archive.metadata.robot.length = 0.95f;
  archive.metadata.robot.max_slope_deg = 22.5f;
  archive.metadata.robot.max_step_height = 0.08f;
  archive.metadata.robot.safety_distance = 0.20f;
  archive.metadata.robot.max_path_curvature = 0.30f;
  archive.metadata.map.voxel_size = 0.07f;
  archive.metadata.map.outlier_k = 17;
  archive.metadata.map.outlier_std_ratio = 1.3f;
  archive.metadata.map.normal_search_radius = 0.40f;
  archive.metadata.map.max_edge_length = 0.28f;
  archive.metadata.map.max_normal_deviation_deg = 31.0f;
  archive.metadata.map.prefer_layered_triangulation = true;
  archive.metadata.map.keep_raw_cloud = false;
  archive.metadata.point_count = 1234;
  archive.metadata.triangle_count = 2;
  archive.metadata.region_count = 2;
  archive.metadata.portal_count = 1;
  archive.metadata.connected_components = 1;
  archive.metadata.regions_before_compression = 4;
  archive.metadata.build_time_sec = 1.25;
  archive.metadata.memory_mb = 8.5;

  archive.mesh = MakeMesh();

  SurfaceRegion r0 = R(0);
  r0.triangle_ids = {0};
  r0.centroid = Eigen::Vector3f(0.25f, 0.25f, 0.0f);
  r0.mean_slope = 0.11f;
  r0.max_slope = 0.22f;
  r0.slope_variance = 0.01f;
  r0.roughness = 0.02f;
  r0.is_topology_critical = true;
  r0.portal_ids = {0};
  SurfaceRegion r1 = R(1);
  r1.triangle_ids = {1};
  r1.centroid = Eigen::Vector3f(0.75f, 0.75f, 0.0f);
  r1.is_ramp = true;
  r1.mean_slope = 0.30f;
  r1.portal_ids = {0};
  Portal p = P(0, 0, 1);
  p.boundary_chain = {0, 1};
  p.left = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  p.right = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  p.center = Eigen::Vector3f(0.5f, 0.0f, 0.0f);
  p.raw_width = 1.0f;
  p.valid_width = 0.9f;
  p.mean_slope = 0.05f;
  p.max_step = 0.01f;
  p.min_clearance = 0.5f;
  p.traversal_cost = 0.7f;
  p.is_critical = true;
  archive.graph = TopologyGraph::build({r0, r1}, {p}).take();
  return archive;
}

std::vector<std::uint8_t> SaveToBytes(const MapArchive& archive) {
  const std::string path = TempPath("save");
  EXPECT_TRUE(topmap::saveMapArchive(path, archive).ok());
  std::vector<std::uint8_t> bytes = ReadFile(path);
  RemoveFile(path);
  return bytes;
}

struct Section {
  std::string name;
  std::vector<std::uint8_t> payload;
};

std::vector<Section> ParseSections(const std::vector<std::uint8_t>& bytes) {
  topmap::BinaryReader reader(bytes);
  std::uint32_t magic = 0, version = 0, endian = 0, count = 0;
  reader.readU32(magic);
  reader.readU32(version);
  reader.readU32(endian);
  reader.readU32(count);
  std::vector<Section> sections;
  for (std::uint32_t i = 0; i < count; ++i) {
    std::uint32_t name_len = 0;
    reader.readU32(name_len);
    Section section;
    section.name.resize(name_len);
    for (std::uint32_t k = 0; k < name_len; ++k) {
      std::uint8_t b = 0;
      reader.readU8(b);
      section.name[k] = static_cast<char>(b);
    }
    std::uint64_t byte_len = 0;
    reader.readU64(byte_len);
    section.payload.resize(byte_len);
    for (std::uint64_t k = 0; k < byte_len; ++k) {
      std::uint8_t b = 0;
      reader.readU8(b);
      section.payload[k] = b;
    }
    sections.push_back(std::move(section));
  }
  return sections;
}

std::vector<std::uint8_t> BuildFile(const std::vector<Section>& sections,
                                    std::uint32_t magic = topmap::kTopmapMagic,
                                    std::uint32_t version = topmap::kTopmapFormatVersion,
                                    std::uint32_t endian = topmap::kTopmapEndianMarker) {
  BinaryWriter writer;
  writer.writeU32(magic);
  writer.writeU32(version);
  writer.writeU32(endian);
  writer.writeU32(static_cast<std::uint32_t>(sections.size()));
  for (const Section& section : sections) {
    writer.writeU32(static_cast<std::uint32_t>(section.name.size()));
    for (char c : section.name) writer.writeU8(static_cast<std::uint8_t>(c));
    writer.writeU64(static_cast<std::uint64_t>(section.payload.size()));
    for (std::uint8_t b : section.payload) writer.writeU8(b);
  }
  return writer.bytes();
}

Section* FindSection(std::vector<Section>& sections, const std::string& name) {
  for (Section& section : sections)
    if (section.name == name) return &section;
  return nullptr;
}

void PutU64LE(std::vector<std::uint8_t>* bytes, std::size_t offset, std::uint64_t value) {
  for (int i = 0; i < 8; ++i)
    (*bytes)[offset + static_cast<std::size_t>(i)] =
        static_cast<std::uint8_t>((value >> (8 * i)) & 0xffu);
}

void PutI32LE(std::vector<std::uint8_t>* bytes, std::size_t offset, std::int32_t value) {
  const std::uint32_t u = static_cast<std::uint32_t>(value);
  for (int i = 0; i < 4; ++i)
    (*bytes)[offset + static_cast<std::size_t>(i)] =
        static_cast<std::uint8_t>((u >> (8 * i)) & 0xffu);
}

void WriteRegionRecord(BinaryWriter& writer, const SurfaceRegion& region) {
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

struct EdgeRecord {
  RegionId portal;
  RegionId region_a;
  RegionId region_b;
};

// Rebuilds the graph section payload so a test can express exactly which origin
// records and edge records appear (including deliberate defects).
std::vector<std::uint8_t> GraphPayload(
    const std::vector<std::pair<RegionId, std::vector<RegionId>>>& origin_records,
    const std::vector<EdgeRecord>& edges) {
  BinaryWriter writer;
  writer.writeU32(static_cast<std::uint32_t>(origin_records.size()));
  for (const auto& record : origin_records) {
    writer.writeI32(record.first);
    writer.writeU32(static_cast<std::uint32_t>(record.second.size()));
    for (const RegionId origin : record.second) writer.writeI32(origin);
  }
  writer.writeU32(static_cast<std::uint32_t>(edges.size()));
  for (const EdgeRecord& edge : edges) {
    writer.writeI32(edge.portal);
    writer.writeI32(edge.region_a);
    writer.writeI32(edge.region_b);
  }
  return writer.bytes();
}

// A valid archive whose two regions are joined by two parallel portals, so the
// edge table has two records and a duplicate can be injected.
MapArchive MakeParallelPortalArchive() {
  MapArchive archive = MakeArchive();
  SurfaceRegion r0 = R(0);
  r0.triangle_ids = {0};
  r0.portal_ids = {0, 1};
  SurfaceRegion r1 = R(1);
  r1.triangle_ids = {1};
  r1.portal_ids = {0, 1};
  archive.graph = TopologyGraph::build({r0, r1}, {P(0, 0, 1), P(1, 0, 1)}).take();
  archive.metadata.portal_count = 2;
  return archive;
}

}  // namespace

TEST(MapArchiveTest, RoundTripPreservesEveryField) {
  const MapArchive original = MakeArchive();
  const std::string path = TempPath("roundtrip");
  ASSERT_TRUE(topmap::saveMapArchive(path, original).ok());
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_TRUE(loaded.ok()) << loaded.status().message;
  RemoveFile(path);
  const MapArchive& back = loaded.value();

  EXPECT_EQ(back.format_version, topmap::kTopmapFormatVersion);

  // MapMetadata: every field, including the R14 additions.
  EXPECT_EQ(back.metadata.frame_id, original.metadata.frame_id);
  EXPECT_FLOAT_EQ(back.metadata.robot.width, original.metadata.robot.width);
  EXPECT_FLOAT_EQ(back.metadata.robot.length, original.metadata.robot.length);
  EXPECT_FLOAT_EQ(back.metadata.robot.max_slope_deg, original.metadata.robot.max_slope_deg);
  EXPECT_FLOAT_EQ(back.metadata.robot.max_step_height, original.metadata.robot.max_step_height);
  EXPECT_FLOAT_EQ(back.metadata.robot.safety_distance, original.metadata.robot.safety_distance);
  EXPECT_FLOAT_EQ(back.metadata.robot.max_path_curvature,
                  original.metadata.robot.max_path_curvature);
  EXPECT_FLOAT_EQ(back.metadata.map.voxel_size, original.metadata.map.voxel_size);
  EXPECT_EQ(back.metadata.map.outlier_k, original.metadata.map.outlier_k);
  EXPECT_FLOAT_EQ(back.metadata.map.outlier_std_ratio, original.metadata.map.outlier_std_ratio);
  EXPECT_FLOAT_EQ(back.metadata.map.normal_search_radius,
                  original.metadata.map.normal_search_radius);
  EXPECT_FLOAT_EQ(back.metadata.map.max_edge_length, original.metadata.map.max_edge_length);
  EXPECT_FLOAT_EQ(back.metadata.map.max_normal_deviation_deg,
                  original.metadata.map.max_normal_deviation_deg);
  EXPECT_EQ(back.metadata.map.prefer_layered_triangulation,
            original.metadata.map.prefer_layered_triangulation);
  EXPECT_EQ(back.metadata.map.keep_raw_cloud, original.metadata.map.keep_raw_cloud);
  EXPECT_EQ(back.metadata.point_count, original.metadata.point_count);
  EXPECT_EQ(back.metadata.triangle_count, original.metadata.triangle_count);
  EXPECT_EQ(back.metadata.region_count, original.metadata.region_count);
  EXPECT_EQ(back.metadata.portal_count, original.metadata.portal_count);
  EXPECT_EQ(back.metadata.connected_components, original.metadata.connected_components);
  EXPECT_EQ(back.metadata.regions_before_compression,
            original.metadata.regions_before_compression);
  EXPECT_DOUBLE_EQ(back.metadata.build_time_sec, original.metadata.build_time_sec);
  EXPECT_DOUBLE_EQ(back.metadata.memory_mb, original.metadata.memory_mb);

  // TriangleMesh: vertices and every serialized TriangleNode attribute.
  ASSERT_EQ(back.mesh.size(), original.mesh.size());
  ASSERT_EQ(back.mesh.vertices().size(), original.mesh.vertices().size());
  for (std::size_t i = 0; i < original.mesh.vertices().size(); ++i)
    EXPECT_TRUE(back.mesh.vertices()[i].isApprox(original.mesh.vertices()[i]));
  for (std::size_t i = 0; i < original.mesh.size(); ++i) {
    const auto& a = original.mesh.triangle(static_cast<topmap::TriangleId>(i));
    const auto& b = back.mesh.triangle(static_cast<topmap::TriangleId>(i));
    EXPECT_EQ(b.vertex_ids, a.vertex_ids);
    EXPECT_TRUE(b.centroid.isApprox(a.centroid));
    EXPECT_TRUE(b.normal.isApprox(a.normal));
    EXPECT_FLOAT_EQ(b.slope, a.slope);
    EXPECT_FLOAT_EQ(b.roughness, a.roughness);
    EXPECT_FLOAT_EQ(b.curvature, a.curvature);
    EXPECT_FLOAT_EQ(b.area, a.area);
    EXPECT_FLOAT_EQ(b.min_clearance, a.min_clearance);
    EXPECT_EQ(b.traversable, a.traversable);
  }
  EXPECT_EQ(back.mesh.neighbors(0), original.mesh.neighbors(0));

  // SurfaceRegions: all fields, triangle ids, portal ids and flags.
  ASSERT_EQ(back.graph.regions().size(), original.graph.regions().size());
  for (std::size_t i = 0; i < original.graph.regions().size(); ++i) {
    const SurfaceRegion& a = original.graph.regions()[i];
    const SurfaceRegion& b = back.graph.regions()[i];
    EXPECT_EQ(b.id, a.id);
    EXPECT_EQ(b.triangle_ids, a.triangle_ids);
    EXPECT_TRUE(b.centroid.isApprox(a.centroid));
    EXPECT_FLOAT_EQ(b.area, a.area);
    EXPECT_FLOAT_EQ(b.min_z, a.min_z);
    EXPECT_FLOAT_EQ(b.max_z, a.max_z);
    EXPECT_FLOAT_EQ(b.mean_slope, a.mean_slope);
    EXPECT_FLOAT_EQ(b.max_slope, a.max_slope);
    EXPECT_FLOAT_EQ(b.slope_variance, a.slope_variance);
    EXPECT_FLOAT_EQ(b.roughness, a.roughness);
    EXPECT_EQ(b.is_ramp, a.is_ramp);
    EXPECT_EQ(b.is_topology_critical, a.is_topology_critical);
    EXPECT_EQ(b.portal_ids, a.portal_ids);
  }

  // Portals: all fields, boundary chain and flags.
  ASSERT_EQ(back.graph.portals().size(), original.graph.portals().size());
  for (std::size_t i = 0; i < original.graph.portals().size(); ++i) {
    const Portal& a = original.graph.portals()[i];
    const Portal& b = back.graph.portals()[i];
    EXPECT_EQ(b.id, a.id);
    EXPECT_EQ(b.region_a, a.region_a);
    EXPECT_EQ(b.region_b, a.region_b);
    EXPECT_EQ(b.boundary_chain, a.boundary_chain);
    EXPECT_TRUE(b.left.isApprox(a.left));
    EXPECT_TRUE(b.right.isApprox(a.right));
    EXPECT_TRUE(b.center.isApprox(a.center));
    EXPECT_FLOAT_EQ(b.raw_width, a.raw_width);
    EXPECT_FLOAT_EQ(b.valid_width, a.valid_width);
    EXPECT_FLOAT_EQ(b.mean_slope, a.mean_slope);
    EXPECT_FLOAT_EQ(b.max_step, a.max_step);
    EXPECT_FLOAT_EQ(b.min_clearance, a.min_clearance);
    EXPECT_FLOAT_EQ(b.traversal_cost, a.traversal_cost);
    EXPECT_EQ(b.is_critical, a.is_critical);
  }

  EXPECT_EQ(back.graph.connectedComponents(), original.graph.connectedComponents());
  EXPECT_EQ(back.graph.portalCountBetween(0, 1), original.graph.portalCountBetween(0, 1));
  EXPECT_EQ(back.graph.originRegions(0), original.graph.originRegions(0));
}

TEST(MapArchiveTest, CompressedOriginsAndCriticalFlagsRoundTrip) {
  std::vector<SurfaceRegion> regions;
  std::vector<Portal> portals;
  for (int i = 0; i < 4; ++i) regions.push_back(R(i));
  for (int i = 0; i + 1 < 4; ++i) portals.push_back(P(i, i, i + 1));
  const TopologyGraph graph = TopologyGraph::build(regions, portals).take();

  topmap::CompressionParams params;
  params.max_merge_cost = 1.0f;
  TopologyGraph compressed =
      TopologyCompressor().compress(graph, params, topmap::RegionParams{}).take().first;
  compressed.setCriticalFlags(TopologyInvariants::capture(compressed));
  ASSERT_LT(compressed.regions().size(), graph.regions().size());
  ASSERT_GT(compressed.allOriginIds().size(), compressed.regions().size());

  MapArchive archive;
  archive.mesh = MakeMesh();
  archive.graph = compressed;
  archive.metadata.region_count = static_cast<std::uint32_t>(compressed.regions().size());
  archive.metadata.portal_count = static_cast<std::uint32_t>(compressed.portals().size());
  archive.metadata.regions_before_compression = 4;
  archive.metadata.connected_components =
      static_cast<std::uint32_t>(compressed.connectedComponents());

  const std::string path = TempPath("origins");
  ASSERT_TRUE(topmap::saveMapArchive(path, archive).ok());
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_TRUE(loaded.ok()) << loaded.status().message;
  RemoveFile(path);
  const TopologyGraph& back = loaded.value().graph;

  // Compressed provenance must not be lost: every original id resolves to the
  // same current region, and each region keeps its full origin list.
  for (const RegionId origin : compressed.allOriginIds()) {
    auto before = compressed.currentRegionOfOrigin(origin);
    auto after = back.currentRegionOfOrigin(origin);
    ASSERT_TRUE(before.ok()) << origin;
    ASSERT_TRUE(after.ok()) << origin;
    EXPECT_EQ(before.value(), after.value());
  }
  for (const SurfaceRegion& region : compressed.regions()) {
    EXPECT_EQ(back.originRegions(region.id), compressed.originRegions(region.id));
    const SurfaceRegion* restored = back.region(region.id);
    ASSERT_NE(restored, nullptr);
    EXPECT_EQ(restored->is_topology_critical, region.is_topology_critical);
  }
  for (const Portal& portal : compressed.portals()) {
    const Portal* restored = back.portal(portal.id);
    ASSERT_NE(restored, nullptr);
    EXPECT_EQ(restored->is_critical, portal.is_critical);
  }
}

TEST(MapArchiveTest, MagicAndVersionRejected) {
  const std::string path = TempPath("magic");
  const std::vector<Section> sections = ParseSections(SaveToBytes(MakeArchive()));

  WriteFile(path, BuildFile(sections, 0xdeadbeefu));
  auto bad_magic = topmap::loadMapArchive(path);
  ASSERT_FALSE(bad_magic.ok());
  EXPECT_EQ(bad_magic.status().code, topmap::ErrorCode::kSerializationMagicMismatch);

  WriteFile(path, BuildFile(sections, topmap::kTopmapMagic, topmap::kTopmapFormatVersion + 1));
  auto bad_version = topmap::loadMapArchive(path);
  ASSERT_FALSE(bad_version.ok());
  EXPECT_EQ(bad_version.status().code, topmap::ErrorCode::kSerializationVersionUnsupported);
  RemoveFile(path);
}

TEST(MapArchiveTest, EndianMismatchRejected) {
  const std::string path = TempPath("endian");
  WriteFile(path, BuildFile(ParseSections(SaveToBytes(MakeArchive())), topmap::kTopmapMagic,
                            topmap::kTopmapFormatVersion, 0x0badf00du));
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.status().code, topmap::ErrorCode::kSerializationIoError);
  RemoveFile(path);
}

TEST(MapArchiveTest, UnknownSectionAndTrailingBytesAllowed) {
  const MapArchive original = MakeArchive();
  std::vector<Section> sections = ParseSections(SaveToBytes(original));
  ASSERT_GE(sections.size(), 4u);
  sections.insert(sections.begin() + 3,
                  Section{"vendor_extension", std::vector<std::uint8_t>{1, 2, 3, 4, 5}});
  std::vector<std::uint8_t> bytes = BuildFile(sections);
  for (std::uint8_t b : {0xde, 0xad, 0xbe, 0xef, 0x00}) bytes.push_back(b);

  const std::string path = TempPath("unknown");
  WriteFile(path, bytes);
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_TRUE(loaded.ok()) << loaded.status().message;
  RemoveFile(path);
  EXPECT_EQ(loaded.value().mesh.size(), original.mesh.size());
  EXPECT_EQ(loaded.value().graph.regions().size(), original.graph.regions().size());
  EXPECT_EQ(loaded.value().metadata.frame_id, original.metadata.frame_id);
}

TEST(MapArchiveTest, MissingRequiredSectionRejected) {
  std::vector<Section> sections = ParseSections(SaveToBytes(MakeArchive()));
  sections.erase(std::remove_if(sections.begin(), sections.end(),
                                [](const Section& s) { return s.name == "regions"; }),
                 sections.end());
  const std::string path = TempPath("missing");
  WriteFile(path, BuildFile(sections));
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.status().code, topmap::ErrorCode::kSerializationIoError);
  RemoveFile(path);
}

TEST(MapArchiveTest, DuplicateRequiredSectionRejected) {
  std::vector<Section> sections = ParseSections(SaveToBytes(MakeArchive()));
  Section* meta = FindSection(sections, "meta");
  ASSERT_NE(meta, nullptr);
  sections.push_back(*meta);
  const std::string path = TempPath("duplicate");
  WriteFile(path, BuildFile(sections));
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.status().code, topmap::ErrorCode::kSerializationIoError);
  RemoveFile(path);
}

TEST(MapArchiveTest, SectionLengthBeyondPayloadRejected) {
  std::vector<std::uint8_t> bytes = BuildFile(ParseSections(SaveToBytes(MakeArchive())));
  // First section is "meta": name_len at 16, 4-byte name, then the u64 byte_len.
  PutU64LE(&bytes, 16 + 4 + 4, 0xffffffffffffffffull);
  const std::string path = TempPath("badlen");
  WriteFile(path, bytes);
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.status().code, topmap::ErrorCode::kSerializationIoError);
  RemoveFile(path);
}

TEST(MapArchiveTest, TruncatedAtEveryOffsetFails) {
  const std::vector<std::uint8_t> bytes = SaveToBytes(MakeArchive());
  ASSERT_GT(bytes.size(), 32u);
  for (std::size_t length = 0; length < bytes.size(); ++length) {
    const std::string path = TempPath("trunc");
    WriteFile(path, std::vector<std::uint8_t>(bytes.begin(), bytes.begin() + length));
    auto loaded = topmap::loadMapArchive(path);
    EXPECT_FALSE(loaded.ok()) << "archive truncated to " << length << " bytes loaded";
    RemoveFile(path);
  }
}

TEST(MapArchiveTest, InvalidWritePathAndMissingFileRejected) {
  const MapArchive archive = MakeArchive();
  const std::string missing_dir =
      "/tmp/topmap_no_such_dir_" + std::to_string(static_cast<long>(::getpid())) + "/map.bin";
  const topmap::Status saved = topmap::saveMapArchive(missing_dir, archive);
  EXPECT_FALSE(saved.ok());
  EXPECT_EQ(saved.code, topmap::ErrorCode::kSerializationIoError);

  const std::string missing_file =
      "/tmp/topmap_no_such_file_" + std::to_string(static_cast<long>(::getpid())) + ".bin";
  auto loaded = topmap::loadMapArchive(missing_file);
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.status().code, topmap::ErrorCode::kSerializationIoError);
}

TEST(MapArchiveTest, CorruptGraphReferenceRejected) {
  std::vector<Section> sections = ParseSections(SaveToBytes(MakeArchive()));
  Section* portals = FindSection(sections, "portals");
  ASSERT_NE(portals, nullptr);
  // Portal payload: u32 count, then i32 id, i32 region_a, i32 region_b, ...
  PutI32LE(&portals->payload, 8, 999);
  const std::string path = TempPath("corruptref");
  WriteFile(path, BuildFile(sections));
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.status().code, topmap::ErrorCode::kSerializationIoError);
  RemoveFile(path);
}

TEST(MapArchiveTest, SelfLoopPortalRejected) {
  std::vector<Section> sections = ParseSections(SaveToBytes(MakeArchive()));
  Section* portals = FindSection(sections, "portals");
  ASSERT_NE(portals, nullptr);
  // Make region_b equal region_a (0), a self loop.
  PutI32LE(&portals->payload, 12, 0);
  const std::string path = TempPath("selfloop");
  WriteFile(path, BuildFile(sections));
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.status().code, topmap::ErrorCode::kSerializationIoError);
  RemoveFile(path);
}

TEST(MapArchiveTest, DuplicateRegionIdsRejected) {
  std::vector<Section> sections = ParseSections(SaveToBytes(MakeArchive()));
  Section* regions = FindSection(sections, "regions");
  ASSERT_NE(regions, nullptr);

  SurfaceRegion a = R(7);
  a.triangle_ids = {0};
  SurfaceRegion b = R(7);  // duplicate id
  b.triangle_ids = {1};
  BinaryWriter writer;
  writer.writeU32(2);
  WriteRegionRecord(writer, a);
  WriteRegionRecord(writer, b);
  regions->payload = writer.bytes();

  const std::string path = TempPath("dupid");
  WriteFile(path, BuildFile(sections));
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.status().code, topmap::ErrorCode::kSerializationIoError);
  RemoveFile(path);
}

TEST(MapArchiveTest, MissingEdgeRecordRejected) {
  // The archive has one portal, but the graph edge table is empty: a defect that
  // the per-record endpoint check alone cannot see.
  std::vector<Section> sections = ParseSections(SaveToBytes(MakeArchive()));
  Section* graph = FindSection(sections, "graph");
  ASSERT_NE(graph, nullptr);
  graph->payload = GraphPayload({{0, {0}}, {1, {1}}}, {});
  const std::string path = TempPath("missingedge");
  WriteFile(path, BuildFile(sections));
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.status().code, topmap::ErrorCode::kSerializationIoError);
  RemoveFile(path);
}

TEST(MapArchiveTest, DuplicateEdgeRecordRejected) {
  // Two real portals, but the edge table repeats portal 0 and omits portal 1.
  std::vector<Section> sections = ParseSections(SaveToBytes(MakeParallelPortalArchive()));
  Section* graph = FindSection(sections, "graph");
  ASSERT_NE(graph, nullptr);
  graph->payload = GraphPayload({{0, {0}}, {1, {1}}}, {{0, 0, 1}, {0, 0, 1}});
  const std::string path = TempPath("dupedge");
  WriteFile(path, BuildFile(sections));
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.status().code, topmap::ErrorCode::kSerializationIoError);
  RemoveFile(path);
}

TEST(MapArchiveTest, DuplicateOriginWithinRegionRejected) {
  // An origin repeated inside one record must be rejected, not silently folded by
  // setOriginRegions' SortUnique.
  std::vector<Section> sections = ParseSections(SaveToBytes(MakeArchive()));
  Section* graph = FindSection(sections, "graph");
  ASSERT_NE(graph, nullptr);
  graph->payload = GraphPayload({{0, {0, 0}}, {1, {1}}}, {{0, 0, 1}});
  const std::string path = TempPath("duporigin");
  WriteFile(path, BuildFile(sections));
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.status().code, topmap::ErrorCode::kSerializationIoError);
  RemoveFile(path);
}

TEST(MapArchiveTest, KnownSectionTrailingGarbageRejected) {
  // Every known section must be consumed exactly: same-version garbage may not
  // ride along inside a parsed payload. Top-level trailing bytes stay allowed.
  const std::vector<std::uint8_t> bytes = SaveToBytes(MakeArchive());
  for (const char* name :
       {"meta", "vertices", "faces", "triangle_attrs", "regions", "portals", "graph"}) {
    std::vector<Section> sections = ParseSections(bytes);
    Section* section = FindSection(sections, name);
    ASSERT_NE(section, nullptr) << name;
    section->payload.push_back(0xab);
    const std::string path = TempPath("trailing");
    WriteFile(path, BuildFile(sections));
    auto loaded = topmap::loadMapArchive(path);
    EXPECT_FALSE(loaded.ok()) << "trailing byte accepted in section " << name;
    if (!loaded.ok()) {
      EXPECT_EQ(loaded.status().code, topmap::ErrorCode::kSerializationIoError);
    }
    RemoveFile(path);
  }
}

TEST(MapArchiveTest, FutureVersionSaveDoesNotClobberExistingTarget) {
  const std::string path = TempPath("futuresave");
  ASSERT_TRUE(topmap::saveMapArchive(path, MakeArchive()).ok());
  const std::vector<std::uint8_t> before = ReadFile(path);

  MapArchive future = MakeArchive();
  future.format_version = topmap::kTopmapFormatVersion + 1;
  const topmap::Status saved = topmap::saveMapArchive(path, future);
  ASSERT_FALSE(saved.ok());
  EXPECT_EQ(saved.code, topmap::ErrorCode::kSerializationVersionUnsupported);

  // The old, readable archive must be untouched.
  EXPECT_EQ(ReadFile(path), before);
  auto loaded = topmap::loadMapArchive(path);
  ASSERT_TRUE(loaded.ok()) << loaded.status().message;
  EXPECT_EQ(loaded.value().format_version, topmap::kTopmapFormatVersion);
  RemoveFile(path);
}
