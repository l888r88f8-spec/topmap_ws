#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "topmap_planner/benchmark_runner.hpp"

namespace {

using topmap::BenchmarkRecord;
using topmap::ErrorCode;
using topmap::PlanningMethod;
using topmap::SyntheticScene;
using topmap::TopmapConfig;

constexpr const char* kHeader =
    "scene,method,success,expected_unreachable,expanded_nodes,planning_time_sec,path_length,"
    "min_clearance,triangle_count,region_count,portal_count,invalid_cross_layer_edges,"
    "reachability_preserved,portal_order_valid,regions_before_compression,"
    "regions_after_compression,error_code";

std::size_t CountFields(const std::string& line) {
  std::size_t count = 1;
  for (const char c : line) {
    if (c == ',') ++count;
  }
  return count;
}

void WriteFile(const std::string& path, const std::string& content) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << content;
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

// Final-plan example (R33): the hierarchical advantage is only defined for
// multi-region scenes, so the comparison uses kLoop (8 Regions) rather than
// kRamp (3 Regions, corridor == full mesh).
TEST(BenchmarkRunnerTest, HierarchicalExpandsFewerNodesThanFullMesh) {
  const TopmapConfig cfg;
  auto records = topmap::runBenchmark({SyntheticScene::kLoop},
                                      {PlanningMethod::kMeshAStar, PlanningMethod::kHierarchical},
                                      cfg, 0.25f);
  ASSERT_TRUE(records.ok()) << records.status().message;
  ASSERT_EQ(records.value().size(), 2u);
  const BenchmarkRecord& full = records.value()[0];
  const BenchmarkRecord& hier = records.value()[1];
  ASSERT_TRUE(full.success) << full.error_code;
  ASSERT_TRUE(hier.success) << hier.error_code;
  EXPECT_LT(hier.expanded_nodes, full.expanded_nodes);

  // Accurate pre/post compression counts. The production default
  // (max_merge_cost = 0.25) is strict enough that this scene is unchanged, so
  // the strict A-10 decrease is checked with a permissive threshold below.
  EXPECT_GE(hier.regions_before_compression, 3);
  EXPECT_EQ(hier.regions_before_compression, hier.region_count);
  EXPECT_LE(hier.regions_after_compression, hier.regions_before_compression);
  EXPECT_TRUE(hier.reachability_preserved);
  EXPECT_EQ(hier.invalid_cross_layer_edges, 0);
  EXPECT_EQ(full.reachability_preserved, hier.reachability_preserved);
}

// A-10: when the cost gate actually admits merges, Method C plans on the
// compressed graph and the reported Region counts and reachability flag follow
// the real TopologyCompressor result.
TEST(BenchmarkRunnerTest, CompressionMetricsTrackActualMerges) {
  TopmapConfig cfg;
  cfg.compression.max_merge_cost = 20.0f;
  auto records = topmap::runBenchmark({SyntheticScene::kLoop},
                                      {PlanningMethod::kHierarchical}, cfg, 0.25f);
  ASSERT_TRUE(records.ok()) << records.status().message;
  ASSERT_EQ(records.value().size(), 1u);
  const BenchmarkRecord& record = records.value()[0];
  ASSERT_TRUE(record.success) << record.error_code;
  EXPECT_GE(record.regions_before_compression, 3);
  EXPECT_LT(record.regions_after_compression, record.regions_before_compression);
  EXPECT_TRUE(record.reachability_preserved);
}

// Final-plan example: CSV round-trip plus the per-method metrics that the T33
// acceptance script consumes.
TEST(BenchmarkRunnerTest, HallCsvRoundTripAndThreeMethods) {
  const TopmapConfig cfg;
  auto records = topmap::runBenchmark(
      {SyntheticScene::kHallNarrowHall},
      {PlanningMethod::kPointCloudAStar, PlanningMethod::kMeshAStar, PlanningMethod::kHierarchical},
      cfg, 0.25f);
  ASSERT_TRUE(records.ok()) << records.status().message;
  ASSERT_EQ(records.value().size(), 3u);
  EXPECT_EQ(records.value()[0].method, "point_cloud");
  EXPECT_EQ(records.value()[1].method, "mesh");
  EXPECT_EQ(records.value()[2].method, "hierarchical");
  for (const BenchmarkRecord& record : records.value()) {
    EXPECT_TRUE(record.success) << record.method << ": " << record.error_code;
    EXPECT_FALSE(record.expected_unreachable);
    EXPECT_GT(record.expanded_nodes, 0) << record.method;
    EXPECT_GT(record.path_length, 0.0f) << record.method;
    EXPECT_GE(record.min_clearance, 0.45f) << record.method;
    EXPECT_EQ(record.invalid_cross_layer_edges, 0) << record.method;
  }
  // A-03: the hierarchical path follows the topology Portal order.
  EXPECT_TRUE(records.value()[2].portal_order_valid);

  const std::string csv = "test_benchmark_runner_roundtrip.csv";
  ASSERT_TRUE(topmap::writeBenchmarkCsv(csv, records.value()).ok());
  auto read = topmap::readBenchmarkCsv(csv);
  ASSERT_TRUE(read.ok()) << read.status().message;
  ASSERT_EQ(read.value().size(), 3u);
  EXPECT_EQ(read.value()[2].method, "hierarchical");
  EXPECT_EQ(read.value()[2].scene, "hall_narrow_hall");
  EXPECT_EQ(read.value()[1].expanded_nodes, records.value()[1].expanded_nodes);
  EXPECT_FLOAT_EQ(read.value()[2].min_clearance, records.value()[2].min_clearance);
  std::remove(csv.c_str());
}

TEST(BenchmarkRunnerTest, CsvHeaderIsExactAndRowWellFormed) {
  EXPECT_EQ(topmap::benchmarkCsvHeader(), kHeader);
  BenchmarkRecord record;
  record.scene = "loop";
  record.method = "hierarchical";
  record.success = true;
  record.expected_unreachable = false;
  record.expanded_nodes = 7;
  const std::string row = topmap::benchmarkCsvRow(record);
  EXPECT_EQ(CountFields(row), CountFields(kHeader));
  EXPECT_EQ(row.rfind("loop,hierarchical,1,0,7,", 0), 0u);
}

TEST(BenchmarkRunnerTest, ReadCsvRejectsMalformedInput) {
  const std::string path = "test_benchmark_runner_malformed.csv";
  const std::string good_row =
      "flat_plane,mesh,1,0,10,0.5,1.5,0.5,20,1,0,0,1,1,1,1,0\n";

  WriteFile(path, std::string("wrong,header\n") + good_row);
  EXPECT_FALSE(topmap::readBenchmarkCsv(path).ok());

  WriteFile(path, std::string(kHeader) + "\n" + "flat_plane,mesh,1\n");
  EXPECT_FALSE(topmap::readBenchmarkCsv(path).ok());

  WriteFile(path, std::string(kHeader) + "\n" +
                     "flat_plane,mesh,1,0,not_an_int,0.5,1.5,0.5,20,1,0,0,1,1,1,1,0\n");
  EXPECT_FALSE(topmap::readBenchmarkCsv(path).ok());

  WriteFile(path, std::string(kHeader) + "\n" +
                     "flat_plane,mesh,1,0,10,0.5junk,1.5,0.5,20,1,0,0,1,1,1,1,0\n");
  EXPECT_FALSE(topmap::readBenchmarkCsv(path).ok());

  WriteFile(path, std::string(kHeader) + "\n" + good_row + "trailing,junk\n");
  EXPECT_FALSE(topmap::readBenchmarkCsv(path).ok());

  WriteFile(path, std::string(kHeader) + "\n" + good_row +
                     "flat_plane,mesh,1,0,10,0.5,1.5,0.5,20,1,0,0,1,1,1,1,0,extra\n");
  EXPECT_FALSE(topmap::readBenchmarkCsv(path).ok());

  // Header-only is a valid empty result, and a trailing newline is harmless.
  WriteFile(path, std::string(kHeader) + "\n");
  auto empty = topmap::readBenchmarkCsv(path);
  ASSERT_TRUE(empty.ok()) << empty.status().message;
  EXPECT_TRUE(empty.value().empty());

  std::remove(path.c_str());
}

TEST(BenchmarkRunnerTest, RunBenchmarkRejectsInvalidArguments) {
  const TopmapConfig cfg;
  const float nan = std::numeric_limits<float>::quiet_NaN();

  auto no_methods =
      topmap::runBenchmark({SyntheticScene::kFlatPlane}, {}, cfg, 0.25f);
  ASSERT_FALSE(no_methods.ok());
  EXPECT_EQ(no_methods.status().code, ErrorCode::kInvalidArgument);

  auto no_scenes = topmap::runBenchmark({}, {PlanningMethod::kMeshAStar}, cfg, 0.25f);
  ASSERT_FALSE(no_scenes.ok());
  EXPECT_EQ(no_scenes.status().code, ErrorCode::kInvalidArgument);

  for (const float radius : {0.0f, -0.25f, nan}) {
    auto bad = topmap::runBenchmark({SyntheticScene::kFlatPlane},
                                    {PlanningMethod::kMeshAStar}, cfg, radius);
    ASSERT_FALSE(bad.ok()) << radius;
    EXPECT_EQ(bad.status().code, ErrorCode::kInvalidArgument) << radius;
  }
}

TEST(BenchmarkRunnerTest, OverlappingFloorsExpectedUnreachableSemantics) {
  const TopmapConfig cfg;
  auto records = topmap::runBenchmark(
      {SyntheticScene::kOverlappingFloors},
      {PlanningMethod::kPointCloudAStar, PlanningMethod::kMeshAStar, PlanningMethod::kHierarchical},
      cfg, 0.25f);
  ASSERT_TRUE(records.ok()) << records.status().message;
  ASSERT_EQ(records.value().size(), 3u);
  for (const BenchmarkRecord& record : records.value()) {
    EXPECT_TRUE(record.expected_unreachable) << record.method;
    EXPECT_TRUE(record.success) << record.method;
    EXPECT_NE(record.error_code, 0) << record.method;
    // A-01: there must be no passable cross-floor connection.
    EXPECT_EQ(record.invalid_cross_layer_edges, 0) << record.method;
    EXPECT_EQ(record.path_length, 0.0f) << record.method;
  }
  // Method C rejects the scene before geometry via the topology graph.
  EXPECT_EQ(records.value()[2].error_code,
            static_cast<int>(ErrorCode::kTopologyUnreachable));
  // The geometric methods must fail with their own genuine no-path code.
  EXPECT_NE(records.value()[0].error_code,
            static_cast<int>(ErrorCode::kTopologyUnreachable));
  EXPECT_NE(records.value()[1].error_code,
            static_cast<int>(ErrorCode::kTopologyUnreachable));
}

TEST(BenchmarkRunnerTest, DeterministicRecordOrdering) {
  const TopmapConfig cfg;
  auto records = topmap::runBenchmark(
      {SyntheticScene::kFlatPlane, SyntheticScene::kOverlappingFloors},
      {PlanningMethod::kMeshAStar, PlanningMethod::kHierarchical}, cfg, 0.25f);
  ASSERT_TRUE(records.ok()) << records.status().message;
  ASSERT_EQ(records.value().size(), 4u);
  EXPECT_EQ(records.value()[0].scene, "flat_plane");
  EXPECT_EQ(records.value()[0].method, "mesh");
  EXPECT_EQ(records.value()[1].scene, "flat_plane");
  EXPECT_EQ(records.value()[1].method, "hierarchical");
  EXPECT_EQ(records.value()[2].scene, "overlapping_floors");
  EXPECT_EQ(records.value()[2].method, "mesh");
  EXPECT_EQ(records.value()[3].scene, "overlapping_floors");
  EXPECT_EQ(records.value()[3].method, "hierarchical");
}

// The write path renders to a temp file and renames, so a failed rename must
// leave the existing target untouched.
TEST(BenchmarkRunnerTest, WriteCsvFailureLeavesExistingDataIntact) {
  const std::string dir = "test_benchmark_runner_clobber_dir";
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  ASSERT_TRUE(std::filesystem::create_directory(dir, ec));
  const std::string sentinel = dir + "/sentinel.txt";
  WriteFile(sentinel, "ORIGINAL");

  BenchmarkRecord record;
  record.scene = "flat_plane";
  record.method = "mesh";
  const auto status = topmap::writeBenchmarkCsv(dir, {record});
  ASSERT_FALSE(status.ok());
  EXPECT_EQ(status.code, ErrorCode::kSerializationIoError);
  EXPECT_EQ(ReadFile(sentinel), "ORIGINAL");

  EXPECT_EQ(std::filesystem::remove_all(dir, ec), 2u);
}

}  // namespace
