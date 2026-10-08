#include <charconv>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "topmap_planner/benchmark_runner.hpp"

namespace {

constexpr const char* kDefaultConfig = "src/topmap_bringup/config/topmap.yaml";
constexpr const char* kDefaultOutput = "benchmark/results/benchmark.csv";

void PrintUsage() {
  std::cout
      << "Usage: benchmark_runner [options]\n"
      << "  --config <path>        Topmap YAML config (default " << kDefaultConfig << ")\n"
      << "  --output <path>        Output CSV path (default " << kDefaultOutput << ")\n"
      << "  --scenes <list>        Comma-separated scene names (default: all ten)\n"
      << "  --methods <list>       Comma-separated: point_cloud,mesh,hierarchical\n"
      << "  --point-radius <r>     Method A neighbour radius, metres (default 0.25)\n"
      << "  --ablation <name|all|none>\n"
      << "                         none,no_compression,pure_geometric,no_bottleneck_split,\n"
      << "                         no_topology_preservation,b_vs_c or all (default none)\n"
      << "  --help                 Print this message\n";
}

std::vector<std::string> SplitComma(const std::string& text) {
  std::vector<std::string> items;
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = text.find(',', start);
    if (comma == std::string::npos) {
      if (start < text.size()) items.push_back(text.substr(start));
      break;
    }
    if (comma > start) items.push_back(text.substr(start, comma - start));
    start = comma + 1;
  }
  return items;
}

bool ParseScenes(const std::string& text, std::vector<topmap::SyntheticScene>* scenes,
                 std::string* error) {
  scenes->clear();
  for (const std::string& name : SplitComma(text)) {
    auto parsed = topmap::parseSyntheticScene(name);
    if (!parsed.ok()) {
      *error = "unknown scene '" + name + "'";
      return false;
    }
    scenes->push_back(parsed.value());
  }
  if (scenes->empty()) {
    *error = "--scenes must not be empty";
    return false;
  }
  return true;
}

bool ParseMethods(const std::string& text, std::vector<topmap::PlanningMethod>* methods,
                  std::string* error) {
  methods->clear();
  for (const std::string& name : SplitComma(text)) {
    if (name == "point_cloud") {
      methods->push_back(topmap::PlanningMethod::kPointCloudAStar);
    } else if (name == "mesh") {
      methods->push_back(topmap::PlanningMethod::kMeshAStar);
    } else if (name == "hierarchical") {
      methods->push_back(topmap::PlanningMethod::kHierarchical);
    } else {
      *error = "unknown method '" + name + "'";
      return false;
    }
  }
  if (methods->empty()) {
    *error = "--methods must not be empty";
    return false;
  }
  return true;
}

bool ParsePositiveFloat(const std::string& text, float* value) {
  if (text.empty()) return false;
  const char* begin = text.data();
  const char* end = text.data() + text.size();
  const auto result = std::from_chars(begin, end, *value);
  if (result.ec != std::errc() || result.ptr != end) return false;
  return std::isfinite(*value) && *value > 0.0f;
}

// The ablation catalogue. Ablation5 is the plan's B-vs-C comparison; it keeps
// the production config and only restricts the method set, so its method suffix
// stays meaningful.
constexpr const char* kAblationNames[] = {
    "no_compression",
    "pure_geometric",
    "no_bottleneck_split",
    "no_topology_preservation",
    "b_vs_c",
};

std::vector<std::string> ParseAblations(const std::string& text, std::string* error) {
  std::vector<std::string> names;
  if (text == "all") {
    names.push_back("none");
    for (const char* name : kAblationNames) names.push_back(name);
    return names;
  }
  if (text == "none" || text.empty()) return {"none"};
  for (const std::string& name : SplitComma(text)) {
    if (name == "none") {
      names.push_back(name);
      continue;
    }
    bool known = false;
    for (const char* candidate : kAblationNames) {
      if (name == candidate) {
        known = true;
        break;
      }
    }
    if (!known) {
      *error = "unknown ablation '" + name + "'";
      return {};
    }
    names.push_back(name);
  }
  if (names.empty()) *error = "--ablation must not be empty";
  return names;
}

bool ApplyAblation(const std::string& name, topmap::TopmapConfig* config,
                   std::vector<topmap::PlanningMethod>* methods) {
  if (name == "none") return true;
  if (name == "no_compression") {
    config->compression.max_merge_cost = 0.0f;
    return true;
  }
  if (name == "pure_geometric") {
    config->region.normal_weight = 1.0f;
    config->region.slope_weight = 0.0f;
    config->region.roughness_weight = 0.0f;
    config->region.height_weight = 0.0f;
    config->region.region_score_threshold = 1.0f;
    return true;
  }
  if (name == "no_bottleneck_split") {
    config->region.bottleneck_ratio = 0.001f;
    return true;
  }
  if (name == "no_topology_preservation") {
    config->compression.preserve_topology = false;
    return true;
  }
  if (name == "b_vs_c") {
    std::vector<topmap::PlanningMethod> filtered;
    for (const topmap::PlanningMethod method : *methods) {
      if (method == topmap::PlanningMethod::kMeshAStar ||
          method == topmap::PlanningMethod::kHierarchical) {
        filtered.push_back(method);
      }
    }
    if (filtered.empty()) return false;
    *methods = std::move(filtered);
    return true;
  }
  return false;
}

std::vector<topmap::SyntheticScene> AllScenes() {
  return {topmap::SyntheticScene::kFlatPlane,
          topmap::SyntheticScene::kRamp,
          topmap::SyntheticScene::kOverlappingFloors,
          topmap::SyntheticScene::kMultiLevelRamps,
          topmap::SyntheticScene::kHallNarrowHall,
          topmap::SyntheticScene::kUShape,
          topmap::SyntheticScene::kTShape,
          topmap::SyntheticScene::kSingleBridge,
          topmap::SyntheticScene::kParallelPortals,
          topmap::SyntheticScene::kLoop};
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = kDefaultConfig;
  std::string output_path = kDefaultOutput;
  std::string scenes_arg;
  std::string methods_arg = "point_cloud,mesh,hierarchical";
  std::string ablation_arg = "none";
  float point_radius = 0.25f;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      PrintUsage();
      return 0;
    }
    const auto take_value = [&](std::string* out) {
      if (i + 1 >= argc) return false;
      *out = argv[++i];
      return true;
    };
    if (arg == "--config") {
      if (!take_value(&config_path)) {
        std::cerr << "missing value for --config\n";
        return 2;
      }
    } else if (arg == "--output") {
      if (!take_value(&output_path)) {
        std::cerr << "missing value for --output\n";
        return 2;
      }
    } else if (arg == "--scenes") {
      if (!take_value(&scenes_arg)) {
        std::cerr << "missing value for --scenes\n";
        return 2;
      }
    } else if (arg == "--methods") {
      if (!take_value(&methods_arg)) {
        std::cerr << "missing value for --methods\n";
        return 2;
      }
    } else if (arg == "--ablation") {
      if (!take_value(&ablation_arg)) {
        std::cerr << "missing value for --ablation\n";
        return 2;
      }
    } else if (arg == "--point-radius") {
      std::string value;
      if (!take_value(&value) || !ParsePositiveFloat(value, &point_radius)) {
        std::cerr << "--point-radius must be a finite positive number\n";
        return 2;
      }
    } else {
      std::cerr << "unknown option '" << arg << "'\n";
      PrintUsage();
      return 2;
    }
  }

  std::vector<topmap::SyntheticScene> scenes;
  std::string error;
  if (scenes_arg.empty()) {
    scenes = AllScenes();
  } else if (!ParseScenes(scenes_arg, &scenes, &error)) {
    std::cerr << error << "\n";
    return 2;
  }
  std::vector<topmap::PlanningMethod> methods;
  if (!ParseMethods(methods_arg, &methods, &error)) {
    std::cerr << error << "\n";
    return 2;
  }
  const std::vector<std::string> ablations = ParseAblations(ablation_arg, &error);
  if (!error.empty()) {
    std::cerr << error << "\n";
    return 2;
  }

  auto config = topmap::loadTopmapConfig(config_path);
  if (!config.ok()) {
    std::cerr << "cannot load config '" << config_path << "': " << config.status().message
              << "\n";
    return 1;
  }

  std::vector<topmap::BenchmarkRecord> all_records;
  for (const std::string& ablation : ablations) {
    topmap::TopmapConfig variant = config.value();
    std::vector<topmap::PlanningMethod> variant_methods = methods;
    if (!ApplyAblation(ablation, &variant, &variant_methods)) {
      std::cerr << "ablation '" << ablation << "' selected no methods\n";
      return 2;
    }
    auto records = topmap::runBenchmark(scenes, variant_methods, variant, point_radius);
    if (!records.ok()) {
      std::cerr << "benchmark failed: " << records.status().message << "\n";
      return 1;
    }
    const std::string suffix = (ablation == "none") ? std::string() : "/" + ablation;
    for (topmap::BenchmarkRecord record : records.value()) {
      record.method += suffix;
      all_records.push_back(std::move(record));
    }
  }

  const std::filesystem::path output(output_path);
  if (output.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(output.parent_path(), ec);
  }
  const topmap::Status written = topmap::writeBenchmarkCsv(output_path, all_records);
  if (!written.ok()) {
    std::cerr << "cannot write '" << output_path << "': " << written.message << "\n";
    return 1;
  }
  std::cout << "wrote " << all_records.size() << " benchmark rows to " << output_path << "\n";
  return 0;
}
