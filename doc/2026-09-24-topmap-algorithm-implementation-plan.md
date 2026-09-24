# topmap_ws 算法实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal：** 在空 ROS 2 Humble 工作区中，以 TDD 方式从零实现“GroundMap.pcd → 多层可通行地表 → Region-Portal 拓扑图 → 拓扑 A\*/Surface Corridor/Surface A\* → 平滑且经安全复核的 `nav_msgs/Path`”闭环，并交付 RViz2 可视化、基准与消融。

**Architecture：** 工作区由 7 个 `ament_cmake` 包组成，依赖方向固定为 `topmap_msgs`、`topmap_core`（两者互不依赖）→ `topmap_surface` → `topmap_topology` → `topmap_planner` → `topmap_ros` → `topmap_bringup`；核心算法包不依赖 ROS 2，ROS 消息转换全部集中在 `topmap_ros`。离线构图与在线规划严格分离：离线把 `GroundMap.pcd` 压缩为版本化的 `topmap_map.bin`，在线只加载归档并执行“拓扑 A* → Surface Corridor → 分段 Surface A* → 平滑 → 安全复核”。所有对外类型、错误码、参数结构与 ROS 接口在 A4 一次性冻结，然后按 T01–T33 的依赖序逐个 TDD 落地，每个任务结束都必须能构建、能测试、能提交。

**Tech Stack：** Ubuntu 22.04 / ROS 2 Humble / C++17 / ament_cmake / PCL / Eigen / CGAL / Boost Graph / yaml-cpp / GoogleTest / RViz2。

---

**日期：** 2026-09-24  
**状态：** 接口已冻结，可直接实施  
**目标读者：** 直接编码实施的 ROS 2/C++ 团队

## A. 设计裁决（两份文档冲突处）

| 冲突点 | 研究计划书 | 算法设计规格 | 本计划采用 | 理由 |
|---|---|---|---|---|
| ROS 发行版 | Jazzy | Humble | **Humble** | 约束 2；本机 `/opt/ros/humble` 已安装 |
| Gazebo 验证 | 有 Gazebo 仿真 | 不包含（§2.2） | **不引入 Gazebo** | 约束 2 技术栈未列 Gazebo；验收用 RViz2 + 基准 + 路径安全复核 |
| `topmap_core` 是否依赖 `topmap_msgs` | 规格 §4 依赖图箭头为 `core → msgs` | §4 正文“`topmap_core` 不依赖 ROS 2” | **`topmap_core` 不依赖 `topmap_msgs`** | 若依赖则核心算法被迫链接 ROS；消息转换集中在 `topmap_ros`。这是对规格依赖图的有意修正，见 A2 无环审查 |
| 三角化方案 | 未指定库 | 三维三角化优先，失败转分层 2.5D | **CGAL AFSR 三维优先，分层 2.5D 兜底** | 满足 §7.1 第 4/5 条；测试可强制 `prefer_layered` 获得确定性计数 |
| 参数目标 | “第一阶段调试参数” | 同上 | 两文档一致值写入 YAML，代码无魔法数 | 规格 §10 |

---

## A1. 推荐文件树

```text
topmap_ws/
├── doc/
│   ├── 2026-09-24-topmap-algorithm-design.md        # 已存在，不改
│   ├── Robot-aware Multi-level Traversable Surface Graph.md   # 已存在，不改
│   └── 2026-09-24-topmap-algorithm-implementation-plan.md   # 本计划（已落盘）
├── test_data/
│   ├── README.md
│   ├── synthetic/
│   │   ├── scene_flat_plane.pcd
│   │   ├── scene_ramp.pcd
│   │   ├── scene_overlapping_floors.pcd
│   │   ├── scene_multi_level_ramps.pcd
│   │   ├── scene_hall_narrow_hall.pcd
│   │   ├── scene_u_shape.pcd
│   │   ├── scene_t_shape.pcd
│   │   ├── scene_single_bridge.pcd
│   │   ├── scene_parallel_portals.pcd
│   │   └── scene_loop.pcd
│   └── maps/
│       └── .gitkeep
├── src/
│   ├── topmap_msgs/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── msg/{Region,Portal,TopologyGraph,TopologyPath,MapStats,PlanStats}.msg
│   │   └── srv/{BuildMap,PlanPath}.srv
│   ├── topmap_core/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/topmap_core/{error,ids,geometry,params,binary_io}.hpp
│   │   ├── src/{error,geometry,params,binary_io}.cpp
│   │   └── test/
│   │       ├── test_error.cpp
│   │       ├── test_geometry.cpp
│   │       ├── test_params.cpp
│   │       ├── test_binary_io.cpp
│   │       └── testdata/{topmap_valid.yaml,topmap_missing.yaml}
│   ├── topmap_surface/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/topmap_surface/
│   │   │   ├── point_cloud_processor.hpp
│   │   │   ├── surface_features.hpp
│   │   │   ├── mesh_types.hpp
│   │   │   ├── triangulator.hpp
│   │   │   ├── traversability.hpp
│   │   │   ├── mesh_serialization.hpp
│   │   │   └── synthetic_scene.hpp
│   │   ├── src/{point_cloud_processor,surface_features,mesh_types,triangulator,
│   │   │        traversability,mesh_serialization,synthetic_scene}.cpp
│   │   ├── tools/write_synthetic_pcd.cpp
│   │   └── test/{test_point_cloud_processor,test_surface_features,test_triangle_mesh,
│   │            test_triangulator,test_traversability,test_mesh_serialization,
│   │            test_synthetic_scene}.cpp
│   ├── topmap_topology/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/topmap_topology/
│   │   │   ├── region_types.hpp
│   │   │   ├── region_growing.hpp
│   │   │   ├── region_split.hpp
│   │   │   ├── portal_extractor.hpp
│   │   │   ├── portal_validator.hpp
│   │   │   ├── topology_graph.hpp
│   │   │   ├── topology_invariants.hpp
│   │   │   ├── topology_compressor.hpp
│   │   │   └── map_archive.hpp
│   │   ├── src/{region_growing,region_split,portal_extractor,portal_validator,
│   │   │        topology_graph,topology_invariants,topology_compressor,map_archive}.cpp
│   │   └── test/{test_region_growing,test_region_split,test_portal_extractor,
│   │            test_portal_validator,test_topology_graph,test_topology_invariants,
│   │            test_topology_compressor,test_map_archive}.cpp
│   ├── topmap_planner/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/topmap_planner/
│   │   │   ├── plan_types.hpp
│   │   │   ├── region_locator.hpp
│   │   │   ├── topology_astar.hpp
│   │   │   ├── corridor_builder.hpp
│   │   │   ├── surface_astar.hpp
│   │   │   ├── path_optimizer.hpp
│   │   │   ├── safety_validator.hpp
│   │   │   ├── hierarchical_planner.hpp
│   │   │   └── benchmark_runner.hpp
│   │   ├── src/{region_locator,topology_astar,corridor_builder,surface_astar,
│   │   │        path_optimizer,safety_validator,hierarchical_planner,benchmark_runner}.cpp
│   │   ├── src/benchmark_main.cpp
│   │   └── test/{test_region_locator,test_topology_astar,test_corridor_builder,
│   │            test_surface_astar,test_path_optimizer,test_safety_validator,
│   │            test_hierarchical_planner,test_benchmark_runner}.cpp
│   ├── topmap_ros/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/topmap_ros/{type_conversions,marker_builder,map_builder_node,planner_node}.hpp
│   │   ├── src/{type_conversions,marker_builder,map_builder_node,planner_node,
│   │   │        map_builder_main,planner_main}.cpp
│   │   └── test/{test_support_ros.hpp,test_type_conversions,test_marker_builder,
│   │            test_map_builder_node,test_planner_node,test_end_to_end}.cpp
│   └── topmap_bringup/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/topmap.yaml
│       ├── launch/{topmap_offline_build.launch.py,topmap_online_plan.launch.py,topmap_rviz.launch.py}
│       ├── rviz/topmap.rviz
│       └── scripts/run_acceptance.sh
└── benchmark/
    └── results/.gitkeep
```

---

## A2. 包依赖无环审查（约束 10）

**修正后的 DAG（箭头 = “依赖于”）：**

```text
topmap_msgs      topmap_core
   (leaf)           (leaf, 不依赖任何 ROS 包)
      ▲                 ▲
      │                 ├── topmap_surface
      │                 │        ▲
      │                 │        │
      │                 └── topmap_topology ──► topmap_surface
      │                          ▲
      │                          │
      └────────── topmap_planner ┴──► topmap_topology, topmap_surface, topmap_core
                       ▲
                       │
                  topmap_ros ──► topmap_msgs, topmap_planner, topmap_topology,
                                 topmap_surface, topmap_core
                       ▲
                       │  (exec_depend)
                 topmap_bringup
```

**逐包 `package.xml` 依赖声明（构建期）：**

| 包 | buildtool | build/exec depend | 说明 |
|---|---|---|---|
| `topmap_msgs` | ament_cmake | `rosidl_default_generators`；`std_msgs`、`geometry_msgs`、`nav_msgs` | `nav_msgs` 仅被 `PlanPath.srv` 引用；`exec_depend: rosidl_default_runtime`；membership: `rosidl_interface_packages` |
| `topmap_core` | ament_cmake | `eigen`、`yaml-cpp` | 无 ROS 依赖，`ament_cmake` 仅作构建系统 |
| `topmap_surface` | ament_cmake | `topmap_core`、`eigen`、`yaml-cpp`、`libpcl-all-dev`（`common io filters features search surface`）、`cgal`、`boost` | |
| `topmap_topology` | ament_cmake | `topmap_core`、`topmap_surface`、`eigen`、`cgal`、`boost`、`yaml-cpp` | |
| `topmap_planner` | ament_cmake | `topmap_core`、`topmap_surface`、`topmap_topology`、`eigen`、`boost`、`yaml-cpp` | 不依赖 ROS |
| `topmap_ros` | ament_cmake | `rclcpp`、`topmap_msgs`、`topmap_core/surface/topology/planner`、`pcl_conversions`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`visualization_msgs`、`tf2`、`tf2_ros` | 唯一做 ROS↔C++ 转换的包 |
| `topmap_bringup` | ament_cmake | `launch`、`launch_ros`、`rviz2`；`exec_depend: topmap_ros` | 无编译期算法依赖 |

**无环结论：** 边集合为 `{surface→core, topology→{surface,core}, planner→{topology,surface,core}, ros→{msgs,planner,topology,surface,core}, bringup(exec)→ros}`，拓扑序为 `core, msgs < surface < topology < planner < ros < bringup`，不存在回边；`topmap_msgs` 与 `topmap_core` 均在最低层且互不依赖，**a 说明**：这与规格 §4 依赖图中 `core→msgs` 的箭头不同，本计划为避免 `topmap_core` 链接 ROS 而有意断开，并在 Part C 记录。

---

## A3. 任务清单（依赖序、TDD、每任务一个可测增量）

| # | 任务 | 包 | 依赖 | 交付增量 | 主测试 |
|---|---|---|---|---|---|
| T01 | 工作区骨架 + `topmap_msgs` 接口 | msgs | — | 7 包空壳可 `colcon build`；msg/srv 可生成头 | `ros2 interface show` + `test_msgs_roundtrip`（编译期） |
| T02 | 错误码 / `Status` / `Result` / ID | core | T01 | 统一错误契约 | `test_error.cpp` |
| T03 | 参数结构 + 严格 YAML 加载 | core | T02 | 全量阈值配置契约 | `test_params.cpp` |
| T04 | 二进制读写 + 版本常量 | core | T02 | 序列化原语与 magic/version | `test_binary_io.cpp` |
| T05 | 几何工具 | core | T02 | 法向/坡度/面积/点到三角形距离 | `test_geometry.cpp` |
| T06 | 点云预处理 | surface | T05 | NaN/离群/体素滤波 | `test_point_cloud_processor.cpp` |
| T07 | 点特征（法向/坡度/粗糙度/曲率） | surface | T06 | `SurfaceFeature` 序列 | `test_surface_features.cpp` |
| T08 | `MeshData/TriangleMesh` 与共享边邻接 | surface | T05 | 三层防跨层邻接 | `test_triangle_mesh.cpp` |
| T09 | 三角化（CGAL AFSR + 分层 2.5D） | surface | T08,T07 | 多层地表 Mesh | `test_triangulator.cpp` |
| T10 | 可通行性判定 | surface | T08,T03 | 三角面/边可通行 + clearance | `test_traversability.cpp` |
| T11 | Mesh 序列化 | surface | T04,T08 | Mesh 读写 | `test_mesh_serialization.cpp` |
| T12 | 合成场景生成器 | surface | T08 | 10 个确定性测试场景 | `test_synthetic_scene.cpp` |
| T13 | Region 生长 | topology | T10,T12 | 初始 `SurfaceRegion` | `test_region_growing.cpp` |
| T14 | Region 分裂（瓶颈/凹形/坡道） | topology | T13 | 大厅-窄道-大厅=3 区 | `test_region_split.cpp` |
| T15 | Portal 提取（共享边→Boundary Chain） | topology | T14 | 候选 Portal（含并行） | `test_portal_extractor.cpp` |
| T16 | Robot-aware Portal 验证 | topology | T15,T03 | 宽/坡/台阶/间隙 + 腐蚀 | `test_portal_validator.cpp` |
| T17 | `TopologyGraph` 二部图 | topology | T16 | R-P-R 图 | `test_topology_graph.cpp` |
| T18 | 拓扑不变量 | topology | T17 | 连通分量/桥/并行/关键区 | `test_topology_invariants.cpp` |
| T19 | 拓扑保持压缩 | topology | T18 | 度 2 串联合并 + 回滚 | `test_topology_compressor.cpp` |
| T20 | 地图归档 `topmap_map.bin` | topology | T19,T11,T04 | 版本化归档 | `test_map_archive.cpp` |
| T21 | 起终点 Region 定位 | planner | T20,T05 | 三维投影定位 | `test_region_locator.cpp` |
| T22 | Topology A\* | planner | T17 | Region/Portal 序列 | `test_topology_astar.cpp` |
| T23 | Surface Corridor | planner | T22,T20 | 走廊三角面集合 | `test_corridor_builder.cpp` |
| T24 | Surface A\*（分段、必须过 Portal） | planner | T23 | 离散 3D 路径 | `test_surface_astar.cpp` |
| T25 | 路径优化（Shortcut/Funnel/平滑） | planner | T24 | 平滑路径 + 回退 | `test_path_optimizer.cpp` |
| T26 | 安全复核 | planner | T24,T03 | `PathValidationReport` | `test_safety_validator.cpp` |
| T27 | 分层规划器与降级链 | planner | T21–T26 | 端到端 PlanResult | `test_hierarchical_planner.cpp` |
| T28 | 基准与消融（Method A/B/C） | planner | T27 | CSV 指标 | `test_benchmark_runner.cpp` |
| T29 | ROS 类型/Marker 转换 | ros | T20,T01 | msg↔C++ 全量转换 | `test_type_conversions.cpp`, `test_marker_builder.cpp` |
| T30 | `MapBuilderNode` | ros | T29,T20 | `/topmap/build_map` 服务 | `test_map_builder_node.cpp` |
| T31 | `PlannerNode` | ros | T29,T27 | `/topmap/plan_path` 服务 | `test_planner_node.cpp` |
| T32 | `topmap_bringup` YAML/launch/RViz | bringup | T30,T31 | 一键离线+在线 | `test_end_to_end.cpp` + launch 冒烟 |
| T33 | 验收与基准报告 | 全 | T28,T32 | 验收脚本 + 报告 | `scripts/run_acceptance.sh` |

---

## A4. 全局接口契约（先冻结，后续任务不得改名）

> 命名空间统一 `topmap`；ROS 转换在 `topmap_ros`。所有算法头文件使用 `#pragma once`，不包含任何 `rclcpp` 头。

### A4.1 `topmap_core/include/topmap_core/error.hpp`

```cpp
#pragma once
#include <cstdint>
#include <string>
#include <utility>

namespace topmap {

enum class ErrorCode : std::int32_t {
  kOk = 0,                       kInvalidArgument = 1,             kNotImplemented = 2,
  kPointCloudEmpty = 100,        kPointCloudTooSmall = 101,        kPointCloudInvalidCoordinates = 102,
  kTriangulationFailed = 200,    kLayeredTriangulationFailed = 201, kMeshEmpty = 202, kMeshNotManifold = 203,
  kRegionGrowingFailed = 300,    kRegionSplitFailed = 301,          kRegionEmpty = 302,
  kPortalExtractionFailed = 400, kPortalValidationFailed = 401,     kPortalTooNarrow = 402,
  kGraphInvariantViolation = 500, kCompressionRollback = 501,
  kStartNotOnSurface = 600,      kGoalNotOnSurface = 601,           kTopologyUnreachable = 602,
  kCorridorBuildFailed = 603,    kSurfaceSearchFailed = 604,        kPathOptimizationFailed = 605,
  kPathValidationFailed = 606,
  kSerializationMagicMismatch = 700, kSerializationVersionUnsupported = 701, kSerializationIoError = 702,
};

const char* toString(ErrorCode code);

struct Status {
  ErrorCode code = ErrorCode::kOk;
  std::string message;
  bool ok() const { return code == ErrorCode::kOk; }
  static Status Ok();
  static Status Error(ErrorCode code, std::string message);
};

template <typename T>
class Result {
 public:
  Result() = default;
  Result(T value) : value_(std::move(value)) {}
  Result(Status status) : status_(std::move(status)) {}
  bool ok() const { return status_.ok(); }
  explicit operator bool() const { return ok(); }
  const Status& status() const { return status_; }
  const T& value() const { return value_; }
  T& value() { return value_; }
  T&& take() { return std::move(value_); }
 private:
  T value_{};
  Status status_{};
};

}  // namespace topmap
```

### A4.2 `topmap_core/include/topmap_core/ids.hpp`

```cpp
#pragma once
#include <cstdint>
namespace topmap {
using PointId = std::int32_t;
using TriangleId = std::int32_t;
using RegionId = std::int32_t;
using PortalId = std::int32_t;
inline constexpr std::int32_t kInvalidId = -1;
}  // namespace topmap
```

### A4.3 `topmap_core/include/topmap_core/geometry.hpp`

```cpp
#pragma once
#include <vector>
#include <Eigen/Core>
namespace topmap {
float clampf(float v, float lo, float hi);
float angleBetween(const Eigen::Vector3f& a, const Eigen::Vector3f& b);           // [0, pi]
float slopeFromNormal(const Eigen::Vector3f& n);                                  // angle to +Z
Eigen::Vector3f orientNormalUp(const Eigen::Vector3f& n, const Eigen::Vector3f& view_dir);
float triangleArea(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c);
Eigen::Vector3f triangleNormal(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c);
float pointTriangleDistance(const Eigen::Vector3f& p, const Eigen::Vector3f& a,
                            const Eigen::Vector3f& b, const Eigen::Vector3f& c);
float pointPlaneDistance(const Eigen::Vector3f& p, const Eigen::Vector3f& n, const Eigen::Vector3f& q);
float segmentDistance(const Eigen::Vector3f& p, const Eigen::Vector3f& a, const Eigen::Vector3f& b);
float polylineLength(const std::vector<Eigen::Vector3f>& pts);
}  // namespace topmap
```

### A4.4 `topmap_core/include/topmap_core/params.hpp`

```cpp
#pragma once
#include <string>
#include "topmap_core/error.hpp"
namespace topmap {

struct RobotParams {
  float width = 0.60f;  float length = 0.80f;  float max_slope_deg = 20.0f;
  float max_step_height = 0.05f; float safety_distance = 0.15f;
  float max_curvature = 0.25f;   float min_clearance = 0.15f;
  float max_slope_rad() const;
  float required_clearance() const;   // 0.5*width + safety_distance
};
struct MapBuildParams {
  float voxel_size = 0.10f;      int outlier_k = 20;       float outlier_std_ratio = 1.0f;
  float normal_search_radius = 0.30f; float max_edge_length = 0.30f;
  float max_normal_deviation_deg = 35.0f; bool prefer_layered_triangulation = false;
};
struct TraversabilityParams {
  float max_slope_deg = 20.0f;      float max_normal_diff_deg = 30.0f;
  float max_slope_diff_deg = 15.0f; float max_step_height = 0.05f;
  float max_roughness = 0.03f;      float max_curvature = 0.05f;
  float min_triangle_area = 0.0025f; float required_clearance = 0.45f;
  float max_slope_rad() const; float max_normal_diff_rad() const; float max_slope_diff_rad() const;
};
struct RegionParams {
  float normal_weight = 1.0f, slope_weight = 0.5f, roughness_weight = 0.5f, height_weight = 1.0f;
  float region_score_threshold = 0.60f; float max_height_span = 0.30f;
  float max_slope_variance = 0.02f;     float min_region_area = 0.50f;
  int   min_region_triangles = 20;      float bottleneck_width = 0.90f;
  float ramp_slope_deg = 8.0f;          float concavity_erosion_radius = 0.40f;
  float min_side_area = 0.50f;
};
struct PortalParams {
  int   boundary_min_edges = 2;   // 组成一条 chain 的最少共享边
  float width_margin = 0.0f;
};
struct CompressionParams {
  float max_merge_cost = 0.25f;   bool preserve_topology = true;
};
struct PlanParams {
  float max_project_radius = 0.30f; int corridor_expansion_rings = 1;
  bool fallback_full_mesh = true;   bool shortcut_enabled = true;
  bool funnel_enabled = true;       bool smoothing_enabled = true;
  int smoothing_iterations = 20;    float curvature_limit = 0.25f;
  float check_step = 0.05f;         float topology_weight = 1.0f;
};
struct TopmapConfig {
  RobotParams robot; MapBuildParams map; TraversabilityParams traversability;
  RegionParams region; PortalParams portal; CompressionParams compression; PlanParams plan;
};
Result<TopmapConfig> loadTopmapConfig(const std::string& yaml_path);
Result<RobotParams> loadRobotParams(const std::string& yaml_path);
}  // namespace topmap
```

**YAML 根键契约（严格模式：缺失任一必需键 → `kInvalidArgument`，消息含 `topmap.<section>.<key>`）**

```yaml
topmap:
  robot:        {width, length, max_slope_deg, max_step_height, safety_distance, max_curvature, min_clearance}
  map:          {voxel_size, outlier_k, outlier_std_ratio, normal_search_radius, max_edge_length,
                 max_normal_deviation_deg, prefer_layered_triangulation}
  traversability: {max_normal_diff_deg, max_slope_diff_deg, max_roughness, max_curvature,
                 min_triangle_area, required_clearance}
  region:       {normal_weight, slope_weight, roughness_weight, height_weight,
                 region_score_threshold, max_height_span, max_slope_variance, min_region_area,
                 min_region_triangles, bottleneck_width, ramp_slope_deg,
                 concavity_erosion_radius, min_side_area}
  portal:       {boundary_min_edges, width_margin}
  compression:  {max_merge_cost, preserve_topology}
  plan:         {max_project_radius, corridor_expansion_rings, fallback_full_mesh, shortcut_enabled,
                 funnel_enabled, smoothing_enabled, smoothing_iterations, curvature_limit,
                 check_step, topology_weight}
```

### A4.5 `topmap_core/include/topmap_core/binary_io.hpp`

```cpp
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "topmap_core/error.hpp"
namespace topmap {
inline constexpr std::uint32_t kTopmapMagic = 0x50414d54u;        // 'TMAP'
inline constexpr std::uint32_t kTopmapFormatVersion = 1u;
inline constexpr std::uint32_t kTopmapEndianMarker = 0x01020304u;

class BinaryWriter {
 public:
  void writeU8(std::uint8_t v); void writeU32(std::uint32_t v); void writeU64(std::uint64_t v);
  void writeI32(std::int32_t v); void writeF32(float v); void writeF64(double v); void writeBool(bool v);
  void writeString(const std::string& v);
  void writeVector3f(const Eigen::Vector3f& v);
  void writeVecI32(const std::vector<std::int32_t>& v);
  void writeVecU32(const std::vector<std::uint32_t>& v);
  void writeVecF32(const std::vector<float>& v);
  void writeVecVector3f(const std::vector<Eigen::Vector3f>& v);
  const std::vector<std::uint8_t>& bytes() const { return buffer_; }
 private:
  std::vector<std::uint8_t> buffer_;
};

class BinaryReader {
 public:
  explicit BinaryReader(const std::vector<std::uint8_t>& data) : data_(data) {}
  bool readU8(std::uint8_t& v); bool readU32(std::uint32_t& v); bool readU64(std::uint64_t& v);
  bool readI32(std::int32_t& v); bool readF32(float& v); bool readF64(double& v); bool readBool(bool& v);
  bool readString(std::string& v);
  bool readVector3f(Eigen::Vector3f& v);
  bool readVecI32(std::vector<std::int32_t>& v);
  bool readVecU32(std::vector<std::uint32_t>& v);
  bool readVecF32(std::vector<float>& v);
  bool readVecVector3f(std::vector<Eigen::Vector3f>& v);
  bool eof() const { return offset_ == data_.size(); }
  std::size_t remaining() const { return data_.size() - offset_; }
 private:
  const std::vector<std::uint8_t>& data_;
  std::size_t offset_ = 0;
};
}  // namespace topmap
```

### A4.6 `topmap_surface` 关键类型

```cpp
// mesh_types.hpp
namespace topmap {
struct TriangleEdge { TriangleId neighbor = kInvalidId; PointId v0 = kInvalidId; PointId v1 = kInvalidId; bool traversable = false; };
struct TriangleNode {
  TriangleId id = kInvalidId; std::array<PointId, 3> vertex_ids{kInvalidId, kInvalidId, kInvalidId};
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero(); Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
  float slope = 0.0f, roughness = 0.0f, curvature = 0.0f, area = 0.0f, min_clearance = 0.0f;
  bool traversable = false; std::array<TriangleEdge, 3> edges{};
  std::vector<TriangleId> neighbors() const;
};
struct MeshData { std::vector<Eigen::Vector3f> vertices; std::vector<std::array<PointId, 3>> faces; };
struct MeshBuildParams { float max_edge_length = 0.30f; float max_normal_deviation_deg = 35.0f;
                         float component_radius = 0.20f; bool prefer_layered = false; };
class TriangleMesh {
 public:
  TriangleMesh() = default;
  static Result<TriangleMesh> fromMeshData(const MeshData& data, const MeshBuildParams& params);
  static Result<TriangleMesh> fromTriangleNodes(std::vector<Eigen::Vector3f> vertices,
                                                std::vector<TriangleNode> triangles);
  std::size_t size() const; const std::vector<Eigen::Vector3f>& vertices() const;
  const std::vector<TriangleNode>& triangles() const; const TriangleNode& triangle(TriangleId id) const;
  std::vector<TriangleId> neighbors(TriangleId id) const;
  Result<std::vector<TriangleId>> kRing(TriangleId id, int rings) const;
  const std::vector<int>& componentIds() const; int componentId(TriangleId id) const;
 private:
  std::vector<Eigen::Vector3f> vertices_; std::vector<TriangleNode> triangles_; std::vector<int> component_ids_;
};
}  // namespace topmap

// surface_features.hpp
namespace topmap {
struct SurfaceFeature { Eigen::Vector3f position, normal; float curvature = 0.0f, roughness = 0.0f, slope = 0.0f; };
class SurfaceFeatureEstimator {
 public:
  static Result<std::vector<SurfaceFeature>> estimate(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                                      const MapBuildParams& params);
};
}  // namespace topmap

// point_cloud_processor.hpp
namespace topmap {
class PointCloudProcessor {
 public:
  static Result<pcl::PointCloud<pcl::PointXYZ>> process(const pcl::PointCloud<pcl::PointXYZ>& input,
                                                        const MapBuildParams& params);
};
}  // namespace topmap

// triangulator.hpp
namespace topmap {
enum class TriangulationMode { kDelaunay3DSurface = 0, kLayered2p5D = 1 };
struct TriangulationResult { MeshData mesh; TriangulationMode mode = TriangulationMode::kDelaunay3DSurface;
                             int component_count = 0; int discarded_triangles = 0; };
class SurfaceTriangulator {
 public:
  static Result<TriangulationResult> triangulate(const std::vector<Eigen::Vector3f>& points,
                                                 const std::vector<Eigen::Vector3f>& normals,
                                                 const MeshBuildParams& params);
};
}  // namespace topmap

// traversability.hpp
namespace topmap {
class TraversabilityEvaluator {
 public:
  static bool isTriangleTraversable(const TriangleNode& t, const TraversabilityParams& p);
  static bool isEdgeTraversable(const TriangleNode& a, const TriangleNode& b, const TraversabilityParams& p);
  static float heightJump(const TriangleNode& a, const TriangleNode& b);   // max centroid→other-plane distance
  static void apply(TriangleMesh& mesh, const TraversabilityParams& p);    // 填充 traversable/min_clearance/edges
};
}  // namespace topmap

// mesh_serialization.hpp
namespace topmap {
Status writeTriangleMesh(BinaryWriter& w, const TriangleMesh& mesh);
Result<TriangleMesh> readTriangleMesh(BinaryReader& r);
}  // namespace topmap

// synthetic_scene.hpp
namespace topmap {
enum class SyntheticScene { kFlatPlane, kRamp, kOverlappingFloors, kMultiLevelRamps, kHallNarrowHall,
                            kUShape, kTShape, kSingleBridge, kParallelPortals, kLoop };
std::string toString(SyntheticScene s);
struct SyntheticSceneOptions { float grid_step = 0.10f, level_height = 3.00f, hall_size = 6.00f, corridor_width = 1.20f; };
Result<MeshData> makeSyntheticMesh(SyntheticScene s, const SyntheticSceneOptions& o = {});
Result<std::pair<Eigen::Vector3f, Eigen::Vector3f>> makeSyntheticEndpoints(SyntheticScene s,
                                                                           const SyntheticSceneOptions& o = {});
}  // namespace topmap
```

### A4.7 `topmap_topology` 关键类型

```cpp
// region_types.hpp
namespace topmap {
struct SurfaceRegion {
  RegionId id = kInvalidId; std::vector<TriangleId> triangle_ids;
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  float area = 0.0f, min_z = 0.0f, max_z = 0.0f, mean_slope = 0.0f, max_slope = 0.0f,
        slope_variance = 0.0f, roughness = 0.0f;
  bool is_ramp = false, is_topology_critical = false;
  std::vector<PortalId> portal_ids;
};
struct Portal {
  PortalId id = kInvalidId; RegionId region_a = kInvalidId, region_b = kInvalidId;
  std::vector<PointId> boundary_chain;
  Eigen::Vector3f left = Eigen::Vector3f::Zero(), right = Eigen::Vector3f::Zero(), center = Eigen::Vector3f::Zero();
  float raw_width = 0.0f, valid_width = 0.0f, mean_slope = 0.0f, max_step = 0.0f,
        min_clearance = 0.0f, traversal_cost = 0.0f;
  bool is_critical = false;
};
}  // namespace topmap

// region_growing.hpp
namespace topmap {
float regionJoinScore(const TriangleNode& a, const TriangleNode& b, const RegionParams& p);
class RegionGrower {
 public:
  Result<std::vector<SurfaceRegion>> grow(const TriangleMesh& mesh, const RegionParams& params) const;
};
}  // namespace topmap

// region_split.hpp
namespace topmap {
class RegionSplitter {
 public:
  Result<std::vector<SurfaceRegion>> split(const TriangleMesh& mesh,
                                           const std::vector<SurfaceRegion>& regions,
                                           const RegionParams& params) const;
  Result<std::vector<SurfaceRegion>> splitBottlenecks(const TriangleMesh&, const std::vector<SurfaceRegion>&, const RegionParams&) const;
  Result<std::vector<SurfaceRegion>> splitConcave(const TriangleMesh&, const std::vector<SurfaceRegion>&, const RegionParams&) const;
  Result<std::vector<SurfaceRegion>> splitRamps(const TriangleMesh&, const std::vector<SurfaceRegion>&, const RegionParams&) const;
};
}  // namespace topmap

// portal_extractor.hpp / portal_validator.hpp
namespace topmap {
float portalLocalWidth(const TriangleMesh& mesh, const std::vector<TriangleId>& region_union, TriangleId seed);
class PortalExtractor {
 public:
  Result<std::vector<Portal>> extract(const TriangleMesh& mesh, const std::vector<SurfaceRegion>& regions,
                                      const PortalParams& params) const;
};
class PortalValidator {
 public:
  Result<std::vector<Portal>> validate(const TriangleMesh& mesh, const std::vector<SurfaceRegion>& regions,
                                       std::vector<Portal> candidates, const RobotParams& robot,
                                       const PortalParams& params) const;
};
}  // namespace topmap

// topology_graph.hpp / topology_invariants.hpp
namespace topmap {
struct GraphSignature {
  int region_count = 0, connected_components = 0;
  std::vector<std::uint64_t> component_hashes;
  std::vector<std::pair<RegionId, RegionId>> critical_region_pairs;
  std::vector<std::pair<RegionId, RegionId>> parallel_pairs;
};
class TopologyGraph {
 public:
  static Result<TopologyGraph> build(const std::vector<SurfaceRegion>& regions, const std::vector<Portal>& portals);
  const std::vector<SurfaceRegion>& regions() const; const std::vector<Portal>& portals() const;
  const SurfaceRegion* region(RegionId id) const; const Portal* portal(PortalId id) const;
  bool hasRegion(RegionId id) const;
  std::vector<PortalId> portalsOf(RegionId id) const;
  std::vector<RegionId> neighborRegions(RegionId id) const;
  Result<RegionId> otherRegion(PortalId id, RegionId from) const;
  std::size_t portalCountBetween(RegionId a, RegionId b) const;
  Result<RegionId> regionOfTriangle(TriangleId id) const;
  int connectedComponents() const;
  void setCriticalFlags(const GraphSignature& sig);
  void replaceRegionsPortals(std::vector<SurfaceRegion> regions, std::vector<Portal> portals);
 private:
  std::vector<SurfaceRegion> regions_; std::vector<Portal> portals_;
  std::vector<std::vector<PortalId>> region_portals_; std::vector<std::vector<RegionId>> region_neighbors_;
  std::vector<RegionId> triangle_region_; int components_ = 0;
};
class TopologyInvariants {
 public:
  static GraphSignature capture(const TopologyGraph& graph);
  static bool sameConnectivity(const GraphSignature& a, const GraphSignature& b, std::string* reason);
  static std::vector<PortalId> bridgePortals(const TopologyGraph& graph);
  static std::vector<RegionId> articulationRegions(const TopologyGraph& graph);
};
struct CompressionStats { int regions_before = 0, regions_after = 0, portals_before = 0,
                              portals_after = 0, merges_applied = 0, merges_rejected = 0; };
class TopologyCompressor {
 public:
  Result<std::pair<TopologyGraph, CompressionStats>> compress(const TopologyGraph& graph,
                                                              const CompressionParams& params) const;
};
}  // namespace topmap

// map_archive.hpp
namespace topmap {
struct MapMetadata {
  std::string frame_id = "map";
  RobotParams robot; MapBuildParams map;
  std::uint32_t triangle_count = 0, region_count = 0, portal_count = 0;
  double build_time_sec = 0.0;
};
struct MapArchive {
  std::uint32_t format_version = kTopmapFormatVersion;
  MapMetadata metadata;
  TriangleMesh mesh;
  TopologyGraph graph;
};
Status saveMapArchive(const std::string& path, const MapArchive& archive);
Result<MapArchive> loadMapArchive(const std::string& path);
}  // namespace topmap
```

### A4.8 `topmap_planner` 关键类型

```cpp
// plan_types.hpp
namespace topmap {
struct PlanRequest { Eigen::Vector3f start = Eigen::Vector3f::Zero(), goal = Eigen::Vector3f::Zero(); };
struct TopologyPath { std::vector<RegionId> regions; std::vector<PortalId> portals; float cost = 0.0f; };
struct PlanStats {
  int regions_in_corridor = 0, triangles_in_corridor = 0,
      topology_expanded_nodes = 0, surface_expanded_nodes = 0;
  double topology_time_sec = 0.0, surface_time_sec = 0.0, optimization_time_sec = 0.0;
  float path_length = 0.0f, min_clearance = 0.0f, max_slope_deg = 0.0f, max_curvature = 0.0f;
  bool fallback_used = false;
};
struct PlanResult { Status status; std::vector<Eigen::Vector3f> path; TopologyPath topology; PlanStats stats; };
struct PathValidationReport {
  bool valid = false; float min_clearance = 0.0f, max_step = 0.0f, max_slope_deg = 0.0f, max_curvature = 0.0f;
  std::vector<std::size_t> violating_indices;
};
struct Corridor { std::vector<RegionId> regions; std::vector<TriangleId> triangles; std::vector<PortalId> portals; };
}  // namespace topmap

// 类签名
namespace topmap {
class RegionLocator {
 public:
  Result<RegionId> locate(const TriangleMesh& mesh, const TopologyGraph& graph,
                          const Eigen::Vector3f& point, float max_project_radius) const;
};
class TopologyAStar {
 public:
  Result<TopologyPath> search(const TopologyGraph& graph, RegionId start, RegionId goal,
                              float weight = 1.0f, int* expanded_nodes = nullptr) const;
};
class CorridorBuilder {
 public:
  Result<Corridor> build(const TriangleMesh& mesh, const TopologyGraph& graph,
                         const TopologyPath& path, int expansion_rings) const;
};
class SurfaceAStar {
 public:
  Result<std::vector<Eigen::Vector3f>> search(const TriangleMesh& mesh, const Corridor& corridor,
                                              const std::vector<Eigen::Vector3f>& waypoints,
                                              int* expanded_nodes = nullptr) const;
};
class SafetyValidator {
 public:
  PathValidationReport validate(const std::vector<Eigen::Vector3f>& path, const TriangleMesh& mesh,
                                const TraversabilityParams& params, float check_step,
                                float curvature_limit) const;
  Result<TriangleId> locatePoint(const TriangleMesh& mesh, const Eigen::Vector3f& p,
                                 float max_distance, float* distance = nullptr) const;
};
class PathOptimizer {
 public:
  Result<std::vector<Eigen::Vector3f>> optimize(const std::vector<Eigen::Vector3f>& raw,
                                                const TriangleMesh& mesh, const Corridor& corridor,
                                                const PlanParams& params) const;
  std::vector<Eigen::Vector3f> shortcut(const std::vector<Eigen::Vector3f>& pts,
                                        const TriangleMesh& mesh, const Corridor& corridor,
                                        const PlanParams& params, std::size_t* removed) const;
  std::vector<Eigen::Vector3f> funnel(const std::vector<Eigen::Vector3f>& pts,
                                      const std::vector<Portal>& ordered_portals) const;
  std::vector<Eigen::Vector3f> smooth(const std::vector<Eigen::Vector3f>& pts, const TriangleMesh& mesh,
                                      const PlanParams& params) const;
};
class HierarchicalPlanner {
 public:
  explicit HierarchicalPlanner(PlanParams params);
  Result<PlanResult> plan(const TriangleMesh& mesh, const TopologyGraph& graph,
                          const PlanRequest& request) const;
 private:
  const PlanParams params_;
};
enum class PlanningMethod { kPointCloudAStar = 0, kMeshAStar = 1, kHierarchical = 2 };
struct BenchmarkRecord {
  std::string scene, method; bool success = false;
  int expanded_nodes = 0, triangle_count = 0, region_count = 0, portal_count = 0, error_code = 0;
  int invalid_cross_layer_edges = 0;
  bool reachability_preserved = false, portal_order_valid = false;
  double planning_time_sec = 0.0; float path_length = 0.0f, min_clearance = 0.0f;
  int regions_before_compression = 0, regions_after_compression = 0;
};
std::string benchmarkCsvHeader();
std::string benchmarkCsvRow(const BenchmarkRecord& r);
Status writeBenchmarkCsv(const std::string& path, const std::vector<BenchmarkRecord>& records);
Result<std::vector<BenchmarkRecord>> readBenchmarkCsv(const std::string& path);
Result<std::vector<BenchmarkRecord>> runBenchmark(const std::vector<SyntheticScene>& scenes,
                                                  const std::vector<PlanningMethod>& methods,
                                                  const TopmapConfig& config, float point_radius);
}  // namespace topmap
```

### A4.9 ROS 接口（`topmap_msgs`）

```text
msg/Region.msg
  int32 id
  geometry_msgs/Point centroid
  float32 area
  float32 min_z
  float32 max_z
  float32 mean_slope
  float32 max_slope
  float32 roughness
  bool is_ramp
  bool is_topology_critical
  int32[] portal_ids

msg/Portal.msg
  int32 id
  int32 region_a
  int32 region_b
  geometry_msgs/Point left
  geometry_msgs/Point right
  geometry_msgs/Point center
  float32 raw_width
  float32 valid_width
  float32 mean_slope
  float32 max_step
  float32 min_clearance
  float32 traversal_cost
  bool is_critical

msg/TopologyGraph.msg
  std_msgs/Header header
  topmap_msgs/Region[] regions
  topmap_msgs/Portal[] portals
  int32[] edge_region_ids
  int32[] edge_portal_ids

msg/TopologyPath.msg
  int32[] region_ids
  int32[] portal_ids
  float32 cost

msg/MapStats.msg
  uint32 format_version
  string frame_id
  uint32 point_count
  uint32 triangle_count
  uint32 region_count
  uint32 portal_count
  uint32 connected_components
  float64 build_time_sec
  float64 memory_mb

msg/PlanStats.msg
  bool fallback_used
  uint32 regions_in_corridor
  uint32 triangles_in_corridor
  uint32 topology_expanded_nodes
  uint32 surface_expanded_nodes
  float64 topology_time_sec
  float64 surface_time_sec
  float64 optimization_time_sec
  float32 path_length
  float32 min_clearance
  float32 max_slope_deg
  float32 max_curvature

srv/BuildMap.srv
  string pcd_path
  string output_map_path
  ---
  bool success
  topmap_msgs/MapStats stats
  int32 error_code
  string error_message

srv/PlanPath.srv
  geometry_msgs/PoseStamped start
  geometry_msgs/PoseStamped goal
  ---
  bool success
  nav_msgs/Path path
  topmap_msgs/TopologyPath topology_path
  topmap_msgs/PlanStats stats
  int32 error_code
  string error_message

```

**错误码 ↔ `int32` 映射：** 直接 `static_cast<std::int32_t>(ErrorCode)`，见 T02 `toString` 表；常量 `kOk=0`、`kInvalidArgument=1` 等，跨 ROS 边界不新增编号。

**序列化版本：** `kTopmapFormatVersion = 1`；文件头 `[magic u32][version u32][endian u32][section_count u32]`，section 名 `meta/vertices/faces/triangle_attrs/regions/portals/graph`，每段 `[name_len u32][name][byte_len u64][payload]`。加载：magic 不符 → `kSerializationMagicMismatch`；`version > kTopmapFormatVersion` → `kSerializationVersionUnsupported`；未知 section 跳过；短读 → `kSerializationIoError`。

---

## B. 逐任务细节（可直接粘贴）

> 每个任务的运行命令模板（下文简写为 **BUILD/TEST**）：
> ```bash
> cd /home/robot/topmap_ws
> source /opt/ros/humble/setup.bash
> colcon build --packages-up-to <PKG> --cmake-args -DCMAKE_BUILD_TYPE=Debug
> colcon test --packages-select <PKG> --event-handlers console_direct+ --ctest-args -R <TESTNAME>
> colcon test-result --verbose
> ```
> 其中 `<TESTNAME>` 为 `test/` 下对应目标名（与文件名同）。失败时 `colcon test-result --verbose` 返回非零，并由 `console_direct+` 输出具体失败用例。

---

### T01 工作区骨架与 `topmap_msgs`

**依赖：** 无
**文件：**
- 新增：`src/topmap_msgs/{CMakeLists.txt,package.xml}`、`src/topmap_msgs/msg/*.msg`（6 个）、`src/topmap_msgs/srv/*.srv`（2 个）
- 新增：其余 6 包的 `CMakeLists.txt`、`package.xml`、最小 `include/<pkg>/` 与空 `src/`
- 测试：`src/topmap_msgs/test/test_msgs_roundtrip.cpp`（仅验证生成头可实例化）

- [ ] **步骤 1 写失败测试** `src/topmap_msgs/test/test_msgs_roundtrip.cpp`
```cpp
#include <gtest/gtest.h>
#include "topmap_msgs/msg/region.hpp"
#include "topmap_msgs/msg/portal.hpp"
#include "topmap_msgs/srv/plan_path.hpp"
TEST(MsgsRoundtrip, FieldsExist) {
  topmap_msgs::msg::Region r;
  r.id = 3;
  topmap_msgs::msg::Portal p;
  p.valid_width = 0.9f;
  topmap_msgs::srv::PlanPath::Request req;
  EXPECT_EQ(r.id, 3u);
  EXPECT_FLOAT_EQ(p.valid_width, 0.9f);
  (void)req;
}
```
该测试直接引用将要生成的 `topmap_msgs` 头文件；接口生成前编译失败即为步骤 2 的预期信号，步骤 3 生成接口后同一测试无需改动即可通过。

- [ ] **步骤 2 运行并确认失败**
```bash
colcon build --packages-select topmap_msgs
```
预期：`FAILED`，编译器报找不到 `topmap_msgs/msg/region.hpp` 等生成头（接口尚未生成）。

- [ ] **步骤 3 最小实现** 写 `package.xml`（`<buildtool_depend>ament_cmake</buildtool_depend>`、`<build_depend>rosidl_default_generators</build_depend>`、`<exec_depend>rosidl_default_runtime</exec_depend>`、`<depend>std_msgs</depend>`、`<depend>geometry_msgs</depend>`、`<depend>nav_msgs</depend>`、`<member_of_group>rosidl_interface_packages</member_of_group>`、`<test_depend>ament_cmake_gtest</test_depend>`），`CMakeLists.txt`：
```cmake
cmake_minimum_required(VERSION 3.14)
project(topmap_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Region.msg" "msg/Portal.msg" "msg/TopologyGraph.msg" "msg/TopologyPath.msg"
  "msg/MapStats.msg" "msg/PlanStats.msg"
  "srv/BuildMap.srv" "srv/PlanPath.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_msgs_roundtrip test/test_msgs_roundtrip.cpp)
  target_link_libraries(test_msgs_roundtrip ${PROJECT_NAME}__rosidl_typesupport_cpp)
endif()
ament_package()
```
msg/srv 逐字段按 A4.9 写入。其余 6 包写最小 `package.xml` + `CMakeLists.txt`（`find_package(ament_cmake REQUIRED)` + 依赖 + `ament_package()`），保证 `colcon build` 全绿。

- [ ] **步骤 4 运行并通过**
```bash
colcon build --packages-select topmap_msgs
colcon test --packages-select topmap_msgs --event-handlers console_direct+
colcon test-result --verbose
ros2 interface show topmap_msgs/msg/Portal | grep valid_width
```
预期：构建成功；测试 PASS；接口列出 `valid_width`。

- [ ] **步骤 5 提交**
```bash
git add src/topmap_msgs src/topmap_core src/topmap_surface src/topmap_topology src/topmap_planner src/topmap_ros src/topmap_bringup
git commit -m "feat(msgs): scaffold workspace and define topmap_msgs interfaces"
```

---

### T02 `topmap_core`：错误码 / `Status` / `Result` / ID

**依赖：** T01
**文件：** 新增 `include/topmap_core/{error,ids}.hpp`、`src/error.cpp`、`test/test_error.cpp`
- [ ] **步骤 1 失败测试** `test/test_error.cpp`
```cpp
#include <gtest/gtest.h>
#include "topmap_core/error.hpp"
#include "topmap_core/ids.hpp"
TEST(ErrorTest, ToStringStable) {
  EXPECT_STREQ(topmap::toString(topmap::ErrorCode::kOk), "kOk");
  EXPECT_STREQ(topmap::toString(topmap::ErrorCode::kTopologyUnreachable), "kTopologyUnreachable");
  EXPECT_STREQ(topmap::toString(topmap::ErrorCode::kSerializationVersionUnsupported),
               "kSerializationVersionUnsupported");
}
TEST(ResultTest, CarriesValueAndStatus) {
  topmap::Result<int> ok(7); EXPECT_TRUE(ok.ok()); EXPECT_EQ(ok.value(), 7);
  topmap::Result<int> bad(topmap::Status::Error(topmap::ErrorCode::kMeshEmpty, "empty"));
  EXPECT_FALSE(bad.ok()); EXPECT_EQ(bad.status().code, topmap::ErrorCode::kMeshEmpty);
  EXPECT_EQ(topmap::kInvalidId, -1);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_core test_error`；预期编译失败：`topmap_core/error.hpp: No such file`。
- [ ] **步骤 3 最小实现**：逐字写入 A4.1、A4.2。`src/error.cpp`：
```cpp
#include "topmap_core/error.hpp"
namespace topmap {
const char* toString(ErrorCode c) {
  switch (c) {
    case ErrorCode::kOk: return "kOk";
    case ErrorCode::kInvalidArgument: return "kInvalidArgument";
    case ErrorCode::kNotImplemented: return "kNotImplemented";
    case ErrorCode::kPointCloudEmpty: return "kPointCloudEmpty";
    case ErrorCode::kPointCloudTooSmall: return "kPointCloudTooSmall";
    case ErrorCode::kPointCloudInvalidCoordinates: return "kPointCloudInvalidCoordinates";
    case ErrorCode::kTriangulationFailed: return "kTriangulationFailed";
    case ErrorCode::kLayeredTriangulationFailed: return "kLayeredTriangulationFailed";
    case ErrorCode::kMeshEmpty: return "kMeshEmpty";
    case ErrorCode::kMeshNotManifold: return "kMeshNotManifold";
    case ErrorCode::kRegionGrowingFailed: return "kRegionGrowingFailed";
    case ErrorCode::kRegionSplitFailed: return "kRegionSplitFailed";
    case ErrorCode::kRegionEmpty: return "kRegionEmpty";
    case ErrorCode::kPortalExtractionFailed: return "kPortalExtractionFailed";
    case ErrorCode::kPortalValidationFailed: return "kPortalValidationFailed";
    case ErrorCode::kPortalTooNarrow: return "kPortalTooNarrow";
    case ErrorCode::kGraphInvariantViolation: return "kGraphInvariantViolation";
    case ErrorCode::kCompressionRollback: return "kCompressionRollback";
    case ErrorCode::kStartNotOnSurface: return "kStartNotOnSurface";
    case ErrorCode::kGoalNotOnSurface: return "kGoalNotOnSurface";
    case ErrorCode::kTopologyUnreachable: return "kTopologyUnreachable";
    case ErrorCode::kCorridorBuildFailed: return "kCorridorBuildFailed";
    case ErrorCode::kSurfaceSearchFailed: return "kSurfaceSearchFailed";
    case ErrorCode::kPathOptimizationFailed: return "kPathOptimizationFailed";
    case ErrorCode::kPathValidationFailed: return "kPathValidationFailed";
    case ErrorCode::kSerializationMagicMismatch: return "kSerializationMagicMismatch";
    case ErrorCode::kSerializationVersionUnsupported: return "kSerializationVersionUnsupported";
    case ErrorCode::kSerializationIoError: return "kSerializationIoError";
  }
  return "kUnknown";
}
Status Status::Ok() { return Status{}; }
Status Status::Error(ErrorCode code, std::string message) { return Status{code, std::move(message)}; }
}  // namespace topmap
```
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_core test_error`；预期 PASS。
- [ ] **步骤 5 提交**：`git add src/topmap_core && git commit -m "feat(core): add ErrorCode, Status, Result and id aliases"`

---

### T03 `topmap_core`：参数结构 + 严格 YAML

**依赖：** T02
**文件：** 新增 `include/topmap_core/params.hpp`、`src/params.cpp`、`test/test_params.cpp`、`test/testdata/topmap_valid.yaml`、`test/testdata/topmap_missing.yaml`
- [ ] **步骤 1 失败测试** `test/test_params.cpp`
```cpp
#include <gtest/gtest.h>
#include "topmap_core/params.hpp"
TEST(ParamsTest, LoadsAllSections) {
  auto cfg = topmap::loadTopmapConfig("test/testdata/topmap_valid.yaml");
  ASSERT_TRUE(cfg.ok()) << cfg.status().message;
  EXPECT_FLOAT_EQ(cfg.value().robot.width, 0.60f);
  EXPECT_FLOAT_EQ(cfg.value().robot.max_slope_deg, 20.0f);
  EXPECT_FLOAT_EQ(cfg.value().region.bottleneck_width, 0.90f);
  EXPECT_FLOAT_EQ(cfg.value().traversability.required_clearance, 0.45f);
  EXPECT_NEAR(cfg.value().robot.required_clearance(), 0.45f, 1e-6f);
  EXPECT_TRUE(cfg.value().plan.fallback_full_mesh);
}
TEST(ParamsTest, MissingKeyFailsWithKeyPath) {
  auto cfg = topmap::loadTopmapConfig("test/testdata/topmap_missing.yaml");
  ASSERT_FALSE(cfg.ok());
  EXPECT_EQ(cfg.status().code, topmap::ErrorCode::kInvalidArgument);
  EXPECT_NE(cfg.status().message.find("topmap.region.bottleneck_width"), std::string::npos);
}
TEST(ParamsTest, LoaderFillsTraversabilityFromRobotSection) {
  auto cfg = topmap::loadTopmapConfig("test/testdata/topmap_valid.yaml");
  ASSERT_TRUE(cfg.ok());
  EXPECT_FLOAT_EQ(cfg.value().traversability.max_slope_deg, cfg.value().robot.max_slope_deg);
  EXPECT_FLOAT_EQ(cfg.value().traversability.max_step_height, cfg.value().robot.max_step_height);
}
```
`topmap_valid.yaml` = A4.4 全键示例（`robot.width: 0.60`、`robot.max_slope_deg: 20.0`、`region.bottleneck_width: 0.90`、`traversability.required_clearance: 0.45`）；`topmap_missing.yaml` = 删去 `region.bottleneck_width` 一行。

- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_core test_params`；预期：`params.hpp` 不存在。
- [ ] **步骤 3 最小实现**：写 A4.4 头文件；`src/params.cpp` 用 yaml-cpp 实现严格读取：
```cpp
#include "topmap_core/params.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
namespace topmap {
namespace {
Result<float> req(const YAML::Node& n, const std::string& path) {
  if (!n || !n.IsScalar()) return Status::Error(ErrorCode::kInvalidArgument, "missing key " + path);
  try { return n.as<float>(); }
  catch (const YAML::Exception& e) { return Status::Error(ErrorCode::kInvalidArgument, path + ": " + e.what()); }
}
Result<int> reqInt(const YAML::Node& n, const std::string& path) {
  auto f = req(n, path); if (!f.ok()) return f.status(); return static_cast<int>(std::lround(f.value()));
}
Result<bool> reqBool(const YAML::Node& n, const std::string& path) {
  if (!n || !n.IsScalar()) return Status::Error(ErrorCode::kInvalidArgument, "missing key " + path);
  try { return n.as<bool>(); } catch (const YAML::Exception& e) {
    return Status::Error(ErrorCode::kInvalidArgument, path + ": " + e.what()); }
}
#define REQ(dst, node, key, path) { auto r_ = req((node)[key], path "." key); if (!r_.ok()) return r_.status(); dst = r_.value(); }
#define REQ_I(dst, node, key, path) { auto r_ = reqInt((node)[key], path "." key); if (!r_.ok()) return r_.status(); dst = r_.value(); }
#define REQ_B(dst, node, key, path) { auto r_ = reqBool((node)[key], path "." key); if (!r_.ok()) return r_.status(); dst = r_.value(); }
}  // namespace
float RobotParams::max_slope_rad() const { return max_slope_deg * 3.14159265358979323846f / 180.0f; }
float RobotParams::required_clearance() const { return 0.5f * width + safety_distance; }
float TraversabilityParams::max_slope_rad() const { return max_slope_deg * 3.14159265358979323846f / 180.0f; }
float TraversabilityParams::max_normal_diff_rad() const { return max_normal_diff_deg * 3.14159265358979323846f / 180.0f; }
float TraversabilityParams::max_slope_diff_rad() const { return max_slope_diff_deg * 3.14159265358979323846f / 180.0f; }
Result<TopmapConfig> loadTopmapConfig(const std::string& path) {
  YAML::Node root;
  try { root = YAML::LoadFile(path); }
  catch (const YAML::Exception& e) { return Status::Error(ErrorCode::kInvalidArgument, e.what()); }
  YAML::Node t = root["topmap"];
  if (!t) return Status::Error(ErrorCode::kInvalidArgument, "missing key topmap");
  TopmapConfig c; YAML::Node n;
  n = t["robot"];
  REQ(c.robot.width, n, "width", "topmap.robot");
  REQ(c.robot.length, n, "length", "topmap.robot");
  REQ(c.robot.max_slope_deg, n, "max_slope_deg", "topmap.robot");
  REQ(c.robot.max_step_height, n, "max_step_height", "topmap.robot");
  REQ(c.robot.safety_distance, n, "safety_distance", "topmap.robot");
  REQ(c.robot.max_curvature, n, "max_curvature", "topmap.robot");
  REQ(c.robot.min_clearance, n, "min_clearance", "topmap.robot");
  n = t["map"];
  REQ(c.map.voxel_size, n, "voxel_size", "topmap.map");
  REQ_I(c.map.outlier_k, n, "outlier_k", "topmap.map");
  REQ(c.map.outlier_std_ratio, n, "outlier_std_ratio", "topmap.map");
  REQ(c.map.normal_search_radius, n, "normal_search_radius", "topmap.map");
  REQ(c.map.max_edge_length, n, "max_edge_length", "topmap.map");
  REQ(c.map.max_normal_deviation_deg, n, "max_normal_deviation_deg", "topmap.map");
  REQ_B(c.map.prefer_layered_triangulation, n, "prefer_layered_triangulation", "topmap.map");
  n = t["traversability"];
  REQ(c.traversability.max_normal_diff_deg, n, "max_normal_diff_deg", "topmap.traversability");
  REQ(c.traversability.max_slope_diff_deg, n, "max_slope_diff_deg", "topmap.traversability");
  REQ(c.traversability.max_roughness, n, "max_roughness", "topmap.traversability");
  REQ(c.traversability.max_curvature, n, "max_curvature", "topmap.traversability");
  REQ(c.traversability.min_triangle_area, n, "min_triangle_area", "topmap.traversability");
  REQ(c.traversability.required_clearance, n, "required_clearance", "topmap.traversability");
  c.traversability.max_slope_deg = c.robot.max_slope_deg;
  c.traversability.max_step_height = c.robot.max_step_height;
  n = t["region"];
  REQ(c.region.normal_weight, n, "normal_weight", "topmap.region");
  REQ(c.region.slope_weight, n, "slope_weight", "topmap.region");
  REQ(c.region.roughness_weight, n, "roughness_weight", "topmap.region");
  REQ(c.region.height_weight, n, "height_weight", "topmap.region");
  REQ(c.region.region_score_threshold, n, "region_score_threshold", "topmap.region");
  REQ(c.region.max_height_span, n, "max_height_span", "topmap.region");
  REQ(c.region.max_slope_variance, n, "max_slope_variance", "topmap.region");
  REQ(c.region.min_region_area, n, "min_region_area", "topmap.region");
  REQ_I(c.region.min_region_triangles, n, "min_region_triangles", "topmap.region");
  REQ(c.region.bottleneck_width, n, "bottleneck_width", "topmap.region");
  REQ(c.region.ramp_slope_deg, n, "ramp_slope_deg", "topmap.region");
  REQ(c.region.concavity_erosion_radius, n, "concavity_erosion_radius", "topmap.region");
  REQ(c.region.min_side_area, n, "min_side_area", "topmap.region");
  n = t["portal"];
  REQ_I(c.portal.boundary_min_edges, n, "boundary_min_edges", "topmap.portal");
  REQ(c.portal.width_margin, n, "width_margin", "topmap.portal");
  n = t["compression"];
  REQ(c.compression.max_merge_cost, n, "max_merge_cost", "topmap.compression");
  REQ_B(c.compression.preserve_topology, n, "preserve_topology", "topmap.compression");
  n = t["plan"];
  REQ(c.plan.max_project_radius, n, "max_project_radius", "topmap.plan");
  REQ_I(c.plan.corridor_expansion_rings, n, "corridor_expansion_rings", "topmap.plan");
  REQ_B(c.plan.fallback_full_mesh, n, "fallback_full_mesh", "topmap.plan");
  REQ_B(c.plan.shortcut_enabled, n, "shortcut_enabled", "topmap.plan");
  REQ_B(c.plan.funnel_enabled, n, "funnel_enabled", "topmap.plan");
  REQ_B(c.plan.smoothing_enabled, n, "smoothing_enabled", "topmap.plan");
  REQ_I(c.plan.smoothing_iterations, n, "smoothing_iterations", "topmap.plan");
  REQ(c.plan.curvature_limit, n, "curvature_limit", "topmap.plan");
  REQ(c.plan.check_step, n, "check_step", "topmap.plan");
  REQ(c.plan.topology_weight, n, "topology_weight", "topmap.plan");
  return c;
}
Result<RobotParams> loadRobotParams(const std::string& path) {
  auto c = loadTopmapConfig(path); if (!c.ok()) return c.status(); return c.value().robot;
}
}  // namespace topmap
```
（`testdata` 需通过 `ament_add_gtest` 的 `WORKING_DIRECTORY` 或 `configure_file` 就位；本任务在 `CMakeLists.txt` 追加
`set_tests_properties(test_params PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})`。）

- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_core test_params`；预期三项全 PASS。
- [ ] **步骤 5 提交**：`git add src/topmap_core && git commit -m "feat(core): add strict YAML parameter loading"`

---

### T04 `topmap_core`：二进制 IO 与版本常量

**依赖：** T02
**文件：** 新增 `include/topmap_core/binary_io.hpp`、`src/binary_io.cpp`、`test/test_binary_io.cpp`
- [ ] **步骤 1 失败测试** `test/test_binary_io.cpp`
```cpp
#include <gtest/gtest.h>
#include "topmap_core/binary_io.hpp"
TEST(BinaryIoTest, RoundTripsEverything) {
  topmap::BinaryWriter w;
  w.writeU32(topmap::kTopmapMagic); w.writeU32(topmap::kTopmapFormatVersion);
  w.writeF32(0.6f); w.writeString("map");
  w.writeVector3f(Eigen::Vector3f(1, 2, 3));
  w.writeVecI32({4, 5, 6});
  topmap::BinaryReader r(w.bytes());
  std::uint32_t magic = 0, ver = 0; ASSERT_TRUE(r.readU32(magic)); ASSERT_TRUE(r.readU32(ver));
  EXPECT_EQ(magic, topmap::kTopmapMagic); EXPECT_EQ(ver, 1u);
  float f = 0; ASSERT_TRUE(r.readF32(f)); EXPECT_FLOAT_EQ(f, 0.6f);
  std::string s; ASSERT_TRUE(r.readString(s)); EXPECT_EQ(s, "map");
  Eigen::Vector3f v; ASSERT_TRUE(r.readVector3f(v)); EXPECT_TRUE(v.isApprox(Eigen::Vector3f(1, 2, 3)));
  std::vector<std::int32_t> xs; ASSERT_TRUE(r.readVecI32(xs));
  EXPECT_EQ(xs, (std::vector<std::int32_t>{4, 5, 6}));
  EXPECT_TRUE(r.eof());
}
TEST(BinaryIoTest, ShortReadFailsWithoutAdvancing) {
  std::vector<std::uint8_t> data{1, 2, 3};
  topmap::BinaryReader r(data); std::uint32_t x = 0;
  EXPECT_FALSE(r.readU32(x)); EXPECT_EQ(r.remaining(), 3u);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_core test_binary_io`；预期头文件缺失。
- [ ] **步骤 3 最小实现**：A4.5 头 + `src/binary_io.cpp`：所有整数按小端写入（`b[0]=v&0xff; b[1]=(v>>8)&0xff; ...`）；`readX` 先查 `remaining() >= sizeof`，不足返回 `false` 且不推进 `offset_`；`writeString` 先 `writeU32(size)` 再字节；`writeVecI32` 先 `writeU32(size)` 再逐元素。`kTopmapMagic = 0x50414d54`、`kTopmapFormatVersion = 1`、`kTopmapEndianMarker = 0x01020304`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_core test_binary_io`；预期 PASS。
- [ ] **步骤 5 提交**：`git add src/topmap_core && git commit -m "feat(core): add little-endian binary IO and format constants"`

---

### T05 `topmap_core`：几何工具

**依赖：** T02
**文件：** 新增 `include/topmap_core/geometry.hpp`、`src/geometry.cpp`、`test/test_geometry.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_core/geometry.hpp"
TEST(GeometryTest, SlopeAndOrientation) {
  Eigen::Vector3f n = Eigen::Vector3f(0.0f, std::sin(20.0f * M_PI / 180.0f), std::cos(20.0f * M_PI / 180.0f));
  EXPECT_NEAR(topmap::slopeFromNormal(n) * 180.0f / M_PI, 20.0f, 1e-3f);
  EXPECT_GT(topmap::orientNormalUp(Eigen::Vector3f(0, 0, -1), Eigen::Vector3f::UnitZ()).z(), 0.0f);
}
TEST(GeometryTest, AreaAndDistance) {
  Eigen::Vector3f a(0, 0, 0), b(1, 0, 0), c(0, 1, 0);
  EXPECT_NEAR(topmap::triangleArea(a, b, c), 0.5f, 1e-6f);
  EXPECT_NEAR(topmap::pointTriangleDistance(Eigen::Vector3f(0.25f, 0.25f, 2.0f), a, b, c), 2.0f, 1e-4f);
  EXPECT_NEAR(topmap::polylineLength({a, b, c}), 2.0f, 1e-6f);
  EXPECT_NEAR(topmap::heightJumpPlaneTest_helper(a, b), 0.0f, 1e-6f);  // 见步骤3替换
}
```
步骤 3 将最后一行改为 `EXPECT_NEAR(topmap::pointPlaneDistance(Eigen::Vector3f(0,0,1), Eigen::Vector3f::UnitZ(), a), 1.0f, 1e-6f);`。
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_core test_geometry`。
- [ ] **步骤 3 最小实现**：A4.3 头 + `src/geometry.cpp`。关键公式：
```cpp
float angleBetween(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  return std::acos(clampf(a.normalized().dot(b.normalized()), -1.0f, 1.0f));
}
float slopeFromNormal(const Eigen::Vector3f& n) {
  return std::acos(clampf(n.normalized().dot(Eigen::Vector3f::UnitZ()), -1.0f, 1.0f));
}
Eigen::Vector3f orientNormalUp(const Eigen::Vector3f& n, const Eigen::Vector3f& v) {
  return n.dot(v) >= 0.0f ? n : Eigen::Vector3f(-n);
}
float pointTriangleDistance(const Eigen::Vector3f& p, const Eigen::Vector3f& a,
                            const Eigen::Vector3f& b, const Eigen::Vector3f& c) {
  const Eigen::Vector3f ab = b - a, ac = c - a, n = ab.cross(ac);
  const float area2 = n.squaredNorm();
  if (area2 <= 1e-12f) return std::min({ (p-a).norm(), (p-b).norm(), (p-c).norm() });
  const Eigen::Vector3f nn = n / std::sqrt(area2);
  const float d = nn.dot(p - a);
  const Eigen::Vector3f proj = p - d * nn;
  // 重心坐标内测试，不在则退化为三条边距离最小值
  const Eigen::Vector3f v0 = b - a, v1 = c - a, v2 = proj - a;
  const float d00 = v0.dot(v0), d01 = v0.dot(v1), d11 = v1.dot(v1), d20 = v2.dot(v0), d21 = v2.dot(v1);
  const float denom = d00 * d11 - d01 * d01;
  const float u = (d11 * d20 - d01 * d21) / denom, v = (d00 * d21 - d01 * d20) / denom;
  if (u >= 0.0f && v >= 0.0f && u + v <= 1.0f) return std::fabs(d);
  return std::min({ segmentDistance(p, a, b), segmentDistance(p, b, c), segmentDistance(p, c, a) });
}
```
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_core test_geometry`。
- [ ] **步骤 5 提交**：`git add src/topmap_core && git commit -m "feat(core): add geometry utilities"`

---

### T06 `topmap_surface`：点云预处理

**依赖：** T05
**文件：** 新增 `include/topmap_surface/point_cloud_processor.hpp`、`src/point_cloud_processor.cpp`、`test/test_point_cloud_processor.cpp`；修改 `src/topmap_surface/CMakeLists.txt`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include <limits>
#include "topmap_surface/point_cloud_processor.hpp"
TEST(PointCloudProcessorTest, RemovesNanAndDownsamples) {
  pcl::PointCloud<pcl::PointXYZ> in;
  in.push_back(pcl::PointXYZ(0.00f, 0, 0)); in.push_back(pcl::PointXYZ(0.05f, 0, 0));
  in.push_back(pcl::PointXYZ(1.00f, 1, 0));
  in.push_back(pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), 0, 0));
  topmap::MapBuildParams p; p.voxel_size = 0.10f; p.outlier_k = 0;
  auto out = topmap::PointCloudProcessor::process(in, p);
  ASSERT_TRUE(out.ok()) << out.status().message;
  EXPECT_EQ(out.value().size(), 2u);
}
TEST(PointCloudProcessorTest, EmptyInputIsRejected) {
  pcl::PointCloud<pcl::PointXYZ> in;
  topmap::MapBuildParams p;
  auto out = topmap::PointCloudProcessor::process(in, p);
  ASSERT_FALSE(out.ok()); EXPECT_EQ(out.status().code, topmap::ErrorCode::kPointCloudEmpty);
}
```
- [ ] **步骤 2 运行并确认失败**：`colcon build --packages-up-to topmap_surface`；预期 `point_cloud_processor.hpp` 缺失。
- [ ] **步骤 3 最小实现**：A4.6 签名；实现顺序：`pcl::removeNaNFromPointCloud` → 若 `outlier_k>0` 用 `pcl::StatisticalOutlierRemoval` → `pcl::VoxelGrid<pcl::PointXYZ>`。空输入或全 NaN → `kPointCloudEmpty`；`size()<3` → `kPointCloudTooSmall`；含 `inf` → `kPointCloudInvalidCoordinates`。`CMakeLists.txt` 加 `find_package(PCL REQUIRED COMPONENTS common io filters)` 与 `ament_add_gtest`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_surface test_point_cloud_processor`。
- [ ] **步骤 5 提交**：`git add src/topmap_surface && git commit -m "feat(surface): add point cloud preprocessing"`

---

### T07 `topmap_surface`：点特征

**依赖：** T06
**文件：** 新增 `include/topmap_surface/surface_features.hpp`、`src/surface_features.cpp`、`test/test_surface_features.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_surface/surface_features.hpp"
namespace {
pcl::PointCloud<pcl::PointXYZ> MakeGrid(float slope_deg, float noise) {
  pcl::PointCloud<pcl::PointXYZ> c;
  const float t = std::tan(slope_deg * M_PI / 180.0f);
  for (int i = -10; i <= 10; ++i) for (int j = -10; j <= 10; ++j) {
    c.push_back(pcl::PointXYZ(i * 0.05f, j * 0.05f, t * i * 0.05f + noise));
  }
  return c;
}
}  // namespace
TEST(SurfaceFeatureTest, FlatPlaneNormalsAreUp) {
  auto c = MakeGrid(0.0f, 0.0f);
  topmap::MapBuildParams p; p.normal_search_radius = 0.15f;
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_TRUE(f.ok()); ASSERT_FALSE(f.value().empty());
  EXPECT_GT(f.value()[0].normal.z(), 0.99f);
  EXPECT_LT(f.value()[0].slope, 0.02f);
}
TEST(SurfaceFeatureTest, TiltedPlaneSlopeMatches) {
  auto c = MakeGrid(20.0f, 0.0f);
  topmap::MapBuildParams p; p.normal_search_radius = 0.15f;
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_TRUE(f.ok());
  EXPECT_NEAR(f.value()[0].slope * 180.0f / M_PI, 20.0f, 1.0f);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_surface test_surface_features`。
- [ ] **步骤 3 最小实现**：`pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>`（`setRadiusSearch(normal_search_radius)`）得法向与曲率；`orientNormalUp(normal, UnitZ)`；`slope=slopeFromNormal`；`roughness` = 邻域内点到拟合平面的 RMS（用 `pcl::KdTreeFLANN` 取半径内点，PCA 最小特征值开方）。缺失法向的点输出单位 Z。空/无邻域 → `kPointCloudTooSmall`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_surface test_surface_features`。
- [ ] **步骤 5 提交**：`git add src/topmap_surface && git commit -m "feat(surface): add normal, slope, curvature and roughness estimation"`

---

### T08 `topmap_surface`：`MeshData` / `TriangleMesh` 与共享边邻接

**依赖：** T05
**文件：** 新增 `include/topmap_surface/mesh_types.hpp`、`src/mesh_types.cpp`、`test/test_triangle_mesh.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_surface/mesh_types.hpp"
namespace {
topmap::MeshData TwoTriangles() {
  topmap::MeshData d;
  d.vertices = {{0,0,0}, {1,0,0}, {1,1,0}, {0,1,0}};
  d.faces = {{0,1,2}, {0,2,3}};
  return d;
}
topmap::MeshData TwoOverlappingLayers() {
  topmap::MeshData d;
  d.vertices = {{0,0,0},{1,0,0},{0,1,0},  {0,0,3},{1,0,3},{0,1,3}};
  d.faces = {{0,1,2}, {3,4,5}};
  return d;
}
}  // namespace
TEST(TriangleMeshTest, SharedEdgeBecomesNeighbor) {
  topmap::MeshBuildParams p;
  auto m = topmap::TriangleMesh::fromMeshData(TwoTriangles(), p);
  ASSERT_TRUE(m.ok()) << m.status().message;
  EXPECT_EQ(m.value().size(), 2u);
  EXPECT_EQ(m.value().neighbors(0), std::vector<topmap::TriangleId>{1});
  EXPECT_EQ(m.value().neighbors(1), std::vector<topmap::TriangleId>{0});
  ASSERT_NE(m.value().triangle(0).edges[0].neighbor, topmap::kInvalidId);
  EXPECT_NEAR(m.value().triangle(0).area, 0.5f, 1e-6f);
  EXPECT_GT(m.value().triangle(0).normal.z(), 0.99f);
  EXPECT_EQ(m.value().componentId(0), m.value().componentId(1));
}
TEST(TriangleMeshTest, OverlappingLayersNeverConnect) {
  topmap::MeshBuildParams p;
  auto m = topmap::TriangleMesh::fromMeshData(TwoOverlappingLayers(), p);
  ASSERT_TRUE(m.ok());
  EXPECT_TRUE(m.value().neighbors(0).empty());
  EXPECT_NE(m.value().componentId(0), m.value().componentId(1));
}
TEST(TriangleMeshTest, NonManifoldEdgeRejected) {
  topmap::MeshData d; d.vertices = {{0,0,0},{1,0,0},{0,1,0},{0,-1,0},{0,0,1}};
  d.faces = {{0,1,2},{0,1,3},{0,1,4}};
  auto m = topmap::TriangleMesh::fromMeshData(d, topmap::MeshBuildParams{});
  ASSERT_FALSE(m.ok()); EXPECT_EQ(m.status().code, topmap::ErrorCode::kMeshNotManifold);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_surface test_triangle_mesh`。
- [ ] **步骤 3 最小实现**：`fromMeshData` 逐面计算 `centroid/normal(orientUp)/area/slope`，`vertex_ids` 排序后保持原绕序；用 `std::map<std::pair<PointId,PointId>, std::vector<std::pair<TriangleId,int>>>` 统计共享边（键为无序对 `(min,max)`）：出现 2 次 → 双向填 `edges[k].neighbor/v0/v1`；出现 1 次 → 边界；出现 >2 → `kMeshNotManifold`；非法索引 → `kInvalidArgument`；`faces` 为空 → `kMeshEmpty`。`component_ids_` 用邻居 BFS 标号（共享边才连通，XY 重叠但无共享顶点必然不同分量）。`fromTriangleNodes` 直接接管数据并重建边/分量。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_surface test_triangle_mesh`。
- [ ] **步骤 5 提交**：`git add src/topmap_surface && git commit -m "feat(surface): add triangle mesh with shared-edge adjacency"`

---

### T09 `topmap_surface`：三角化（CGAL AFSR + 分层 2.5D）

**依赖：** T08、T07
**文件：** 新增 `include/topmap_surface/triangulator.hpp`、`src/triangulator.cpp`、`test/test_triangulator.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_surface/triangulator.hpp"
namespace {
std::pair<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>> GridPoints(int n, float step, float z,
                                                                                 float tilt_deg = 0.0f) {
  std::vector<Eigen::Vector3f> p, nn;
  const float t = std::tan(tilt_deg * M_PI / 180.0f);
  for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j)
    p.emplace_back(i * step, j * step, z + t * i * step);
  nn.assign(p.size(), Eigen::Vector3f(0, 0, 1));
  return {p, nn};
}
}  // namespace
TEST(TriangulatorTest, LayeredGridProducesExpectedTriangleCount) {
  auto [p, n] = GridPoints(6, 0.10f, 0.0f);
  topmap::MeshBuildParams bp; bp.prefer_layered = true; bp.max_edge_length = 0.25f;
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, bp);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().mode, topmap::TriangulationMode::kLayered2p5D);
  EXPECT_EQ(r.value().mesh.faces.size(), 2u * 5u * 5u);
}
TEST(TriangulatorTest, OverlappingFloorsStaySeparated) {
  auto [p1, n1] = GridPoints(5, 0.10f, 0.0f);
  auto [p2, n2] = GridPoints(5, 0.10f, 3.0f);
  std::vector<Eigen::Vector3f> p = p1, n = n1;
  p.insert(p.end(), p2.begin(), p2.end()); n.insert(n.end(), n2.begin(), n2.end());
  topmap::MeshBuildParams bp; bp.prefer_layered = true; bp.max_edge_length = 0.25f;
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, bp);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().component_count, 2);
  for (const auto& f : r.value().mesh.faces) {
    const float z0 = p[f[0]].z(), z1 = p[f[1]].z(), z2 = p[f[2]].z();
    EXPECT_NEAR(std::max({z0, z1, z2}) - std::min({z0, z1, z2}), 0.0f, 0.01f);
  }
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_surface test_triangulator`。
- [ ] **步骤 3 最小实现**：
  - `triangulate`：先按 `component_radius`（默认 2×`max_edge_length` 与体素对齐）做 3D 连通分量；`prefer_layered==true` → 直接分层 2.5D；否则先尝试 AFSR：
    - AFSR：将 Eigen 点转换为 `CGAL::Exact_predicates_inexact_constructions_kernel::Point_3`，定义 `using Facet = std::array<std::size_t, 3>`，调用 `CGAL::advancing_front_surface_reconstruction(cgal_points.begin(), cgal_points.end(), std::back_inserter(facets))`（头文件 `<CGAL/Advancing_front_surface_reconstruction.h>`）；法向仅用于输出后的朝向和夹角过滤，最终把索引三元组转换为 `MeshData`。
    - 过滤：任一边长 > `max_edge_length` 丢弃；三角形法向与输入点平均法向夹角 > `max_normal_deviation_deg` 丢弃；过滤后三角形数 < 输入点数 → 回退。
  - 分层 2.5D：每个连通分量按 PCA 平面投影，在 2D 做 `CGAL::Delaunay_triangulation_2`；对每个三角形做边长度与法向过滤；若分量平面拟合 RMS > `0.10 m`，按 Z 高度带（`0.5 m`）切分，每带单独投影三角化后按共享顶点 stitch。
  - 结果 `TriangulationResult{mesh, mode, component_count, discarded_triangles}`；无三角形 → `kLayeredTriangulationFailed`（分层路径）或 `kTriangulationFailed`（三维路径）。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_surface test_triangulator`。
- [ ] **步骤 5 提交**：`git add src/topmap_surface && git commit -m "feat(surface): add CGAL AFSR and layered 2.5D triangulation"`

---

### T10 `topmap_surface`：可通行性

**依赖：** T08、T03
**文件：** 新增 `include/topmap_surface/traversability.hpp`、`src/traversability.cpp`、`test/test_traversability.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_surface/traversability.hpp"
namespace {
topmap::MeshData Strip(float center_z, float slope_deg) {
  topmap::MeshData d; const float t = std::tan(slope_deg * M_PI / 180.0f);
  d.vertices = {{0,0,center_z - t}, {2,0,center_z + t}, {1,1,center_z}};
  d.faces = {{0,1,2}}; return d;
}
}  // namespace
TEST(TraversabilityTest, SlopeBoundaryAt20Degrees) {
  topmap::TraversabilityParams p;
  auto flat = topmap::TriangleMesh::fromMeshData(Strip(0.0f, 0.0f), {});
  auto steep = topmap::TriangleMesh::fromMeshData(Strip(0.0f, 25.0f), {});
  ASSERT_TRUE(flat.ok()); ASSERT_TRUE(steep.ok());
  EXPECT_TRUE(topmap::TraversabilityEvaluator::isTriangleTraversable(flat.value().triangle(0), p));
  EXPECT_FALSE(topmap::TraversabilityEvaluator::isTriangleTraversable(steep.value().triangle(0), p));
}
TEST(TraversabilityTest, StepAndNormalDifferenceRejectEdges) {
  topmap::TraversabilityParams p;
  topmap::TriangleNode a, b; a.normal = Eigen::Vector3f::UnitZ(); b.normal = Eigen::Vector3f::UnitZ();
  a.centroid = Eigen::Vector3f(0, 0, 0); b.centroid = Eigen::Vector3f(0.1f, 0, 0.10f);
  EXPECT_FALSE(topmap::TraversabilityEvaluator::isEdgeTraversable(a, b, p));
  b.centroid.z() = 0.0f; b.normal = Eigen::Vector3f(0, std::sin(0.7f), std::cos(0.7f));
  EXPECT_FALSE(topmap::TraversabilityEvaluator::isEdgeTraversable(a, b, p));
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_surface test_traversability`。
- [ ] **步骤 3 最小实现**：
```cpp
bool TraversabilityEvaluator::isTriangleTraversable(const TriangleNode& t, const TraversabilityParams& p) {
  return t.slope <= p.max_slope_rad() && t.roughness <= p.max_roughness &&
         t.curvature <= p.max_curvature && t.area >= p.min_triangle_area;
}
float TraversabilityEvaluator::heightJump(const TriangleNode& a, const TriangleNode& b) {
  return std::max(std::fabs(pointPlaneDistance(a.centroid, b.normal, b.centroid)),
                  std::fabs(pointPlaneDistance(b.centroid, a.normal, a.centroid)));
}
bool TraversabilityEvaluator::isEdgeTraversable(const TriangleNode& a, const TriangleNode& b,
                                                const TraversabilityParams& p) {
  if (angleBetween(a.normal, b.normal) > p.max_normal_diff_rad()) return false;
  if (std::fabs(a.slope - b.slope) > p.max_slope_diff_rad()) return false;
  return heightJump(a, b) <= p.max_step_height;
}
void TraversabilityEvaluator::apply(TriangleMesh& mesh, const TraversabilityParams& p) {
  auto& ts = const_cast<std::vector<TriangleNode>&>(mesh.triangles());
  for (auto& t : ts) t.traversable = isTriangleTraversable(t, p);
  for (auto& t : ts)
    for (auto& e : t.edges)
      if (e.neighbor != kInvalidId)
        e.traversable = t.traversable && ts[e.neighbor].traversable &&
                        isEdgeTraversable(t, ts[e.neighbor], p);
  // min_clearance：对每个可通行三角形，用 BFS 到最近“不可通行三角形或网格边界”的质心测地距离
  // 多源 Dijkstra：源 = 不可通行三角形，边权 = 质心欧氏距离；不可通行三角形自身 clearance=0
}
```
（`mesh.triangles()` 提供可变访问；A4.6 已声明，`const_cast` 可去掉，改为非 const 重载 `std::vector<TriangleNode>& triangles()`。实现时在头文件加该重载。）
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_surface test_traversability`。
- [ ] **步骤 5 提交**：`git add src/topmap_surface && git commit -m "feat(surface): add triangle and edge traversability with clearance"`

---

### T11 `topmap_surface`：Mesh 序列化

**依赖：** T04、T08
**文件：** 新增 `include/topmap_surface/mesh_serialization.hpp`、`src/mesh_serialization.cpp`、`test/test_mesh_serialization.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_core/binary_io.hpp"
#include "topmap_surface/mesh_serialization.hpp"
TEST(MeshSerializationTest, RoundTripPreservesGeometryAndAdjacency) {
  topmap::MeshData d; d.vertices = {{0,0,0},{1,0,0},{1,1,0},{0,1,0}}; d.faces = {{0,1,2},{0,2,3}};
  auto m = topmap::TriangleMesh::fromMeshData(d, {});
  ASSERT_TRUE(m.ok());
  topmap::BinaryWriter w; topmap::Status s = topmap::writeTriangleMesh(w, m.value());
  ASSERT_TRUE(s.ok()) << s.message;
  topmap::BinaryReader r(w.bytes());
  auto back = topmap::readTriangleMesh(r);
  ASSERT_TRUE(back.ok()) << back.status().message;
  EXPECT_EQ(back.value().size(), 2u);
  EXPECT_EQ(back.value().neighbors(0), std::vector<topmap::TriangleId>{1});
  EXPECT_TRUE(back.value().vertices()[2].isApprox(Eigen::Vector3f(1, 1, 0)));
}
TEST(MeshSerializationTest, TruncatedPayloadFails) {
  std::vector<std::uint8_t> data{1, 2};
  topmap::BinaryReader r(data);
  auto back = topmap::readTriangleMesh(r);
  EXPECT_FALSE(back.ok());
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_surface test_mesh_serialization`。
- [ ] **步骤 3 最小实现**：`writeTriangleMesh` 写 `u32 vertex_count` + `vector3f`；`u32 triangle_count` + 每面 3×`i32 vertex_ids` + `f32 slope/roughness/curvature/area/min_clearance` + `bool traversable`；`readTriangleMesh` 逆操作，用 `fromTriangleNodes` 重建（边与分量重新推导，文件不存边表，避免冗余）；任何短读/负数索引 → `kSerializationIoError`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_surface test_mesh_serialization`。
- [ ] **步骤 5 提交**：`git add src/topmap_surface && git commit -m "feat(surface): add triangle mesh serialization"`

---

### T12 `topmap_surface`：合成场景生成器

**依赖：** T08
**文件：** 新增 `include/topmap_surface/synthetic_scene.hpp`、`src/synthetic_scene.cpp`、`test/test_synthetic_scene.cpp`、`tools/write_synthetic_pcd.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_surface/synthetic_scene.hpp"
TEST(SyntheticSceneTest, AllScenesProduceManifoldMeshes) {
  for (auto s : {topmap::SyntheticScene::kFlatPlane, topmap::SyntheticScene::kRamp,
                 topmap::SyntheticScene::kOverlappingFloors, topmap::SyntheticScene::kMultiLevelRamps,
                 topmap::SyntheticScene::kHallNarrowHall, topmap::SyntheticScene::kUShape,
                 topmap::SyntheticScene::kTShape, topmap::SyntheticScene::kSingleBridge,
                 topmap::SyntheticScene::kParallelPortals, topmap::SyntheticScene::kLoop}) {
    auto d = topmap::makeSyntheticMesh(s);
    ASSERT_TRUE(d.ok()) << topmap::toString(s) << ": " << d.status().message;
    auto m = topmap::TriangleMesh::fromMeshData(d.value(), {});
    ASSERT_TRUE(m.ok()) << topmap::toString(s) << ": " << m.status().message;
    EXPECT_GT(m.value().size(), 0u);
    auto ep = topmap::makeSyntheticEndpoints(s);
    ASSERT_TRUE(ep.ok());
  }
}
TEST(SyntheticSceneTest, OverlappingFloorsAreTwoComponents) {
  auto d = topmap::makeSyntheticMesh(topmap::SyntheticScene::kOverlappingFloors);
  ASSERT_TRUE(d.ok());
  auto m = topmap::TriangleMesh::fromMeshData(d.value(), {});
  ASSERT_TRUE(m.ok());
  EXPECT_EQ(m.value().componentIds().empty() ? 0 : *std::max_element(m.value().componentIds().begin(),
            m.value().componentIds().end()) + 1, 2);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_surface test_synthetic_scene`。
- [ ] **步骤 3 最小实现**：按 `grid_step` 生成规则网格并连成三角面。各场景拓扑定义（保证 T13–T28 期望值）：
  - `kFlatPlane`：单层 6×6 m 平面。
  - `kRamp`：4×4 m 低位平台（z=0）经 30° 坡道升到 z=2 m 高位平台。
  - `kOverlappingFloors`：两个 5×5 m 平面同 XY，z=0 与 z=3，无连接。
  - `kMultiLevelRamps`：三层平台 + 两条坡道，链式连通。
  - `kHallNarrowHall`：7×7 m 大厅 z=0 — 宽 `corridor_width`（默认 1.20）的 3 m 长窄道 — 7×7 m 大厅。
  - `kUShape`：3 条 4×1 m 臂组成的 U。
  - `kTShape`：4×4 m 中心 + 3 条 4×1 m 臂。
  - `kSingleBridge`：两个 5×5 m 平台由 1.5 m 宽桥连接，另有一条绕行环路可选（桥为唯一捷径）。
  - `kParallelPortals`：两房间由两条独立 1.2 m 通道连接。
  - `kLoop`：四房间环形连通。
  `makeSyntheticEndpoints` 返回每个场景的 `(start, goal)` 为各自平台中心并抬升 `0.05 m`。`tools/write_synthetic_pcd.cpp` 将每个场景导出为 `test_data/synthetic/scene_*.pcd`（`pcl::io::savePCDFileASCII`）。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_surface test_synthetic_scene`。
- [ ] **步骤 5 提交**：`git add src/topmap_surface test_data && git commit -m "feat(surface): add synthetic multi-level scenes and PCD export tool"`

---

### T13 `topmap_topology`：Region 生长

**依赖：** T10、T12
**文件：** 新增 `include/topmap_topology/region_types.hpp`、`region_growing.hpp`、`src/region_growing.cpp`、`test/test_region_growing.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_topology/region_growing.hpp"
namespace {
topmap::TriangleMesh MeshOf(topmap::SyntheticScene s, topmap::TraversabilityParams* tp = nullptr) {
  topmap::TraversabilityParams local;
  auto d = topmap::makeSyntheticMesh(s); auto m = topmap::TriangleMesh::fromMeshData(d.take(), {});
  topmap::TraversabilityEvaluator::apply(m.value(), tp ? *tp : local);
  return m.take();
}
}  // namespace
TEST(RegionGrowTest, FlatSceneIsSingleRegion) {
  auto m = MeshOf(topmap::SyntheticScene::kFlatPlane);
  topmap::RegionParams p;
  auto r = topmap::RegionGrower().grow(m, p);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().size(), 1u);
  EXPECT_GT(r.value()[0].triangle_ids.size(), 10u);
}
TEST(RegionGrowTest, HallNarrowHallIsNotSplitByGrowing) {
  auto m = MeshOf(topmap::SyntheticScene::kHallNarrowHall);
  topmap::RegionParams p;
  auto r = topmap::RegionGrower().grow(m, p);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().size(), 1u);   // 瓶颈切分由 T14 负责
}
TEST(RegionGrowTest, TooFewTraversableTrianglesFails) {
  topmap::MeshData d; d.vertices = {{0,0,0},{1,0,0},{0,1,0}}; d.faces = {{0,1,2}};
  auto m = topmap::TriangleMesh::fromMeshData(d, {});
  topmap::RegionParams p; p.min_region_triangles = 5;
  auto r = topmap::RegionGrower().grow(m.value(), p);
  ASSERT_FALSE(r.ok()); EXPECT_EQ(r.status().code, topmap::ErrorCode::kRegionGrowingFailed);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_topology test_region_growing`。
- [ ] **步骤 3 最小实现**：
```cpp
float regionJoinScore(const TriangleNode& a, const TriangleNode& b, const RegionParams& p) {
  const float dn = angleBetween(a.normal, b.normal);
  const float ds = std::fabs(a.slope - b.slope);
  const float dr = std::fabs(a.roughness - b.roughness);
  const float dh = TraversabilityEvaluator::heightJump(a, b);
  return p.normal_weight * dn + p.slope_weight * ds + p.roughness_weight * dr + p.height_weight * dh;
}
```
`grow`：按面积从大到小、同面积按 id 升序选取未分配的可通行种子，放入以 `(score, neighbor_id)` 为键的 `std::priority_queue` 做最佳优先扩张；候选 `Tj` 满足 `mesh.triangle(Ti).traversable && regionJoinScore(Ti,Tj) < region_score_threshold` 且 `Tj` 可通行且该边 `traversable`；加入前检查增量后 `max_z-min_z <= max_height_span` 与 `slope_variance <= max_slope_variance`，否则不加入并作为新种子。所有区域 `triangle_ids` 升序、`id` 按种子发现顺序 0..N-1；区域小于 `min_region_triangles` 或 `min_region_area` 的三角形标记为不可通行并不入 Region；若最后无区域 → `kRegionGrowingFailed`。统计量（area/centroid/min_z/max_z/mean_slope/max_slope/slope_variance/roughness）按三角形面积加权。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_topology test_region_growing`。
- [ ] **步骤 5 提交**：`git add src/topmap_topology && git commit -m "feat(topology): add traversability-constrained region growing"`

---

### T14 `topmap_topology`：Region 分裂（瓶颈 / 凹形 / 坡道）

**依赖：** T13
**文件：** 新增 `include/topmap_topology/region_split.hpp`、`src/region_split.cpp`、`test/test_region_split.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_topology/region_split.hpp"
namespace {
std::vector<topmap::SurfaceRegion> GrowAndSplit(topmap::SyntheticScene s, const topmap::RegionParams& p) {
  auto d = topmap::makeSyntheticMesh(s); auto m = topmap::TriangleMesh::fromMeshData(d.take(), {});
  topmap::TraversabilityEvaluator::apply(m.value(), topmap::TraversabilityParams{});
  auto g = topmap::RegionGrower().grow(m, p);
  auto sp = topmap::RegionSplitter().split(m, g.value(), p);
  EXPECT_TRUE(sp.ok()) << sp.status().message;
  return sp.take();
}
}  // namespace
TEST(RegionSplitTest, HallNarrowHallSplitsIntoThree) {
  topmap::RegionParams p;
  EXPECT_EQ(GrowAndSplit(topmap::SyntheticScene::kHallNarrowHall, p).size(), 3u);
}
TEST(RegionSplitTest, UShapeAndTShapeSplitIntoLimbRegions) {
  topmap::RegionParams p;
  EXPECT_EQ(GrowAndSplit(topmap::SyntheticScene::kUShape, p).size(), 3u);
  EXPECT_EQ(GrowAndSplit(topmap::SyntheticScene::kTShape, p).size(), 4u);
}
TEST(RegionSplitTest, RampIsSeparatedFromFlat) {
  topmap::RegionParams p;
  auto regions = GrowAndSplit(topmap::SyntheticScene::kRamp, p);
  EXPECT_GE(regions.size(), 2u);
  int ramps = 0; for (const auto& r : regions) if (r.is_ramp) ++ramps;
  EXPECT_GE(ramps, 1);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_topology test_region_split`。
- [ ] **步骤 3 最小实现**：
  - `splitBottlenecks`：对每个 Region，用 `min_clearance` 场（到 Region 边界的测地距离）估算每条内部相邻边 `(a,b)` 的局部宽度 `w = min_clearance(a) + min_clearance(b)`；对满足 `w < bottleneck_width` 的边做“割边—重连通”判定：临时移除该边后，两侧连通块面积均 `>= min_side_area` → 该边为瓶颈割；多条瓶颈割取面积加权中位数位置，按连通块重新编号。为保证确定性，候选边按 `(min_area_side, -w, min(a,b))` 排序处理。
  - `splitConcave`：对 Region 内每个三角形做测地腐蚀（从 Region 边界向内 BFS `concavity_erosion_radius`）；剩余核心的连通分量数 >1 → 用多源 BFS（源=各核心）把三角形分配给最近核心，按分配结果分裂；否则保持。
  - `splitRamps`：`mean_slope > ramp_slope_deg` 的连通三角形簇且面积 `>= min_region_area` 成为独立 Region，`is_ramp=true`；坡道与平地的交界边作为分割边界。
  - 三段按 瓶颈→凹形→坡道 顺序调用，每步后重新编号 `id` 并重算统计量；分裂失败（产生 0 个区域）→ `kRegionSplitFailed`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_topology test_region_split`。
- [ ] **步骤 5 提交**：`git add src/topmap_topology && git commit -m "feat(topology): add bottleneck, concavity and ramp region splitting"`

---

### T15 `topmap_topology`：Portal 提取

**依赖：** T14
**文件：** 新增 `include/topmap_topology/portal_extractor.hpp`、`src/portal_extractor.cpp`、`test/test_portal_extractor.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_topology/portal_extractor.hpp"
namespace {
std::pair<topmap::TriangleMesh, std::vector<topmap::SurfaceRegion>> Split(
    topmap::SyntheticScene s, const topmap::RegionParams& p) {
  auto d = topmap::makeSyntheticMesh(s); auto m = topmap::TriangleMesh::fromMeshData(d.take(), {});
  topmap::TraversabilityEvaluator::apply(m.value(), topmap::TraversabilityParams{});
  auto g = topmap::RegionGrower().grow(m, p);
  auto sp = topmap::RegionSplitter().split(m, g.value(), p);
  return {m.take(), sp.take()};
}
}  // namespace
TEST(PortalExtractorTest, HallNarrowHallHasTwoPortals) {
  auto [m, regions] = Split(topmap::SyntheticScene::kHallNarrowHall, {});
  auto portals = topmap::PortalExtractor().extract(m, regions, {});
  ASSERT_TRUE(portals.ok()) << portals.status().message;
  EXPECT_EQ(portals.value().size(), 2u);
  for (const auto& p : portals.value()) {
    EXPECT_GE(p.boundary_chain.size(), 2u);
    EXPECT_NEAR(p.raw_width, 1.20f, 0.20f);
  }
}
TEST(PortalExtractorTest, ParallelPortalsAreNotMerged) {
  auto [m, regions] = Split(topmap::SyntheticScene::kParallelPortals, {});
  auto portals = topmap::PortalExtractor().extract(m, regions, {});
  ASSERT_TRUE(portals.ok());
  EXPECT_EQ(portals.value().size(), 2u);
  EXPECT_EQ(portals.value()[0].region_a == portals.value()[1].region_a &&
            portals.value()[0].region_b == portals.value()[1].region_b, true);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_topology test_portal_extractor`。
- [ ] **步骤 3 最小实现**：遍历所有三角形边，若 `edges[k].neighbor` 属于不同 Region，则记录共享边 `(v0,v1)` 及其 Region 对；按 Region 对分组后，用共享端点把边连成 Boundary Chain（每个顶点在链中度 ≤2；度 >2 时按最小顶点 id 顺序断开，保证确定性）；对每条链：`boundary_chain` 为有序顶点序列，`left/right` 为链端点，`center` 为链顶点均值，`raw_width = polylineLength(chain)`；链边数 `< boundary_min_edges` 丢弃；无链 → `kPortalExtractionFailed`；`id` 按 `(region_a,region_b,left.x)` 排序后 0..M-1。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_topology test_portal_extractor`。
- [ ] **步骤 5 提交**：`git add src/topmap_topology && git commit -m "feat(topology): extract boundary chains as portal candidates"`

---

### T16 `topmap_topology`：Robot-aware Portal 验证

**依赖：** T15、T03
**文件：** 新增 `include/topmap_topology/portal_validator.hpp`、`src/portal_validator.cpp`、`test/test_portal_validator.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_topology/portal_validator.hpp"
namespace {
topmap::Portal MakePortal(float raw_width) {
  topmap::Portal p; p.id = 0; p.region_a = 0; p.region_b = 1;
  p.boundary_chain = {0, 1}; p.left = Eigen::Vector3f(0, -raw_width / 2, 0);
  p.right = Eigen::Vector3f(0, raw_width / 2, 0); p.center = Eigen::Vector3f::Zero();
  p.raw_width = raw_width; return p;
}
}  // namespace
TEST(PortalValidatorTest, WidthRuleIsStrict) {
  topmap::RobotParams robot; topmap::PortalParams pp;
  auto d = topmap::makeSyntheticMesh(topmap::SyntheticScene::kFlatPlane);
  auto m = topmap::TriangleMesh::fromMeshData(d.take(), {}); m.value().triangles();  // 触发默认构造
  EXPECT_FALSE(topmap::PortalValidator().validate(m.value(), {}, {MakePortal(0.89f)}, robot, pp).ok());
  EXPECT_TRUE(topmap::PortalValidator().validate(m.value(), {}, {MakePortal(0.95f)}, robot, pp).ok());
}
TEST(PortalValidatorTest, StepAndSlopeReject) {
  topmap::RobotParams robot; topmap::PortalParams pp;
  auto p = MakePortal(1.50f); p.max_step = 0.10f;
  auto d = topmap::makeSyntheticMesh(topmap::SyntheticScene::kFlatPlane);
  auto m = topmap::TriangleMesh::fromMeshData(d.take(), {});
  auto r = topmap::PortalValidator().validate(m.value(), {}, {p}, robot, pp);
  EXPECT_FALSE(r.ok()); EXPECT_EQ(r.status().code, topmap::ErrorCode::kPortalValidationFailed);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_topology test_portal_validator`。
- [ ] **步骤 3 最小实现**：`portalLocalWidth(mesh, region_union, seed)` = `2 * min_clearance(seed)` 限制在 `region_union` 子图内（对两端 Region 的三角形集合重算到边界的测地距离）；`validate` 对每个候选计算 `valid_width`（沿链采样 `boundary_chain` 各顶点，取链法向两侧自由距离最小值），要求 `valid_width > robot.width + 2*robot.safety_distance + width_margin`（即 > 0.90 m），且 `mean_slope <= robot.max_slope_rad()`、`max_step <= robot.max_step_height`、`min_clearance >= robot.min_clearance`；`traversal_cost = 1.0 + 0.5*max(0, 1.0 - valid_width/robot.width)`。任一条不满足的候选从结果中剔除；所有候选都不合法 → `kPortalValidationFailed`；`is_critical` 先置 `false`（T18 计算）。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_topology test_portal_validator`。
- [ ] **步骤 5 提交**：`git add src/topmap_topology && git commit -m "feat(topology): validate portals against robot footprint"`

---

### T17 `topmap_topology`：`TopologyGraph` 二部图

**依赖：** T16
**文件：** 新增 `include/topmap_topology/topology_graph.hpp`、`src/topology_graph.cpp`、`test/test_topology_graph.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_topology/topology_graph.hpp"
namespace {
topmap::SurfaceRegion Region(int id) { topmap::SurfaceRegion r; r.id = id; r.centroid = Eigen::Vector3f(id, 0, 0); return r; }
topmap::Portal Portal(int id, int a, int b, float cost) {
  topmap::Portal p; p.id = id; p.region_a = a; p.region_b = b; p.traversal_cost = cost; return p;
}
}  // namespace
TEST(TopologyGraphTest, BuildsBipartiteAdjacency) {
  auto g = topmap::TopologyGraph::build({Region(0), Region(1), Region(2)},
                                        {Portal(0, 0, 1, 1.0f), Portal(1, 1, 2, 2.0f)});
  ASSERT_TRUE(g.ok()) << g.status().message;
  EXPECT_EQ(g.value().portalsOf(1), (std::vector<topmap::PortalId>{0, 1}));
  EXPECT_EQ(g.value().neighborRegions(1), (std::vector<topmap::RegionId>{0, 2}));
  EXPECT_EQ(g.value().otherRegion(1, 1).value(), 2);
  EXPECT_EQ(g.value().connectedComponents(), 1);
}
TEST(TopologyGraphTest, DeletedPortalLeavesTwoComponents) {
  auto g = topmap::TopologyGraph::build({Region(0), Region(1)}, {Portal(0, 0, 1, 1.0f)});
  ASSERT_TRUE(g.ok());
  auto g2 = topmap::TopologyGraph::build({Region(0), Region(1)}, {});
  ASSERT_TRUE(g2.ok());
  EXPECT_EQ(g.value().connectedComponents(), 1);
  EXPECT_EQ(g2.value().connectedComponents(), 2);
}
TEST(TopologyGraphTest, InvalidRegionReferenceFails) {
  auto g = topmap::TopologyGraph::build({Region(0)}, {Portal(0, 0, 9, 1.0f)});
  EXPECT_FALSE(g.ok()); EXPECT_EQ(g.status().code, topmap::ErrorCode::kInvalidArgument);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_topology test_topology_graph`。
- [ ] **步骤 3 最小实现**：按 A4.7 实现，用 Boost Graph 的 `adjacency_list` 或纯 `std::vector` 邻接；构建时校验每个 Portal 两端 Region 存在且 `region_a != region_b`，重复 `portal.id` 报 `kInvalidArgument`；建立 `region_portals_`、`region_neighbors_`（去重）、`triangle_region_`（从 `triangle_ids` 反查，冲突 → `kInvalidArgument`）；连通分量用并查集；`portalCountBetween` 统计同对 Portal 数；`replaceRegionsPortals` 供压缩器重建并用同一套校验。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_topology test_topology_graph`。
- [ ] **步骤 5 提交**：`git add src/topmap_topology && git commit -m "feat(topology): add region-portal bipartite graph"`

---

### T18 `topmap_topology`：拓扑不变量

**依赖：** T17
**文件：** 新增 `include/topmap_topology/topology_invariants.hpp`、`src/topology_invariants.cpp`、`test/test_topology_invariants.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_topology/topology_invariants.hpp"
namespace {
topmap::SurfaceRegion R(int id) { topmap::SurfaceRegion r; r.id = id; return r; }
topmap::Portal P(int id, int a, int b) { topmap::Portal p; p.id = id; p.region_a = a; p.region_b = b; return p; }
}
TEST(TopologyInvariantsTest, BridgePortalDetected) {
  auto g = topmap::TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)});
  ASSERT_TRUE(g.ok());
  EXPECT_EQ(topmap::TopologyInvariants::bridgePortals(g.value()).size(), 2u);
  auto sig = topmap::TopologyInvariants::capture(g.value());
  EXPECT_EQ(sig.connected_components, 1);
  EXPECT_EQ(sig.critical_region_pairs.size(), 2u);
}
TEST(TopologyInvariantsTest, ParallelPortalPairNotCritical) {
  auto g = topmap::TopologyGraph::build({R(0), R(1)}, {P(0, 0, 1), P(1, 0, 1)});
  ASSERT_TRUE(g.ok());
  auto sig = topmap::TopologyInvariants::capture(g.value());
  EXPECT_TRUE(sig.critical_region_pairs.empty());
  EXPECT_EQ(sig.parallel_pairs.size(), 1u);
}
TEST(TopologyInvariantsTest, ConnectivityComparisonDetectsLoss) {
  auto dense = topmap::TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  auto broken = topmap::TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1)}).take();
  std::string reason;
  EXPECT_FALSE(topmap::TopologyInvariants::sameConnectivity(
      topmap::TopologyInvariants::capture(dense), topmap::TopologyInvariants::capture(broken), &reason));
  EXPECT_FALSE(reason.empty());
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_topology test_topology_invariants`。
- [ ] **步骤 3 最小实现**：
```cpp
std::vector<PortalId> TopologyInvariants::bridgePortals(const TopologyGraph& g) {
  // Tarjan 桥：把 Portal 视为边、Region 视为点；边 (a,b) 是桥 iff low[b] > disc[a]
}
GraphSignature TopologyInvariants::capture(const TopologyGraph& g) {
  // connected_components: 并查集
  // component_hashes: 每个分量内 region id 排序后 FNV-1a 哈希，向量排序
  // critical_region_pairs: 所有桥 Portal 的 (region_a,region_b) 排序去重
  // parallel_pairs: portalCountBetween>=2 的 (min,max) 排序
}
bool TopologyInvariants::sameConnectivity(const GraphSignature& a, const GraphSignature& b, std::string* reason) {
  if (a.connected_components != b.connected_components) { *reason = "component count changed"; return false; }
  if (a.component_hashes != b.component_hashes) { *reason = "region partition changed"; return false; }
  if (a.critical_region_pairs != b.critical_region_pairs) { *reason = "critical connection changed"; return false; }
  if (a.parallel_pairs.size() < b.parallel_pairs.size()) { *reason = "parallel portal lost"; return false; }
  return true;
}
```
（`parallel_pairs` 比较允许压缩后从一个并行对变为另一个等价对；比较的是集合包含而非等号，见 `sameConnectivity`。）
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_topology test_topology_invariants`。
- [ ] **步骤 5 提交**：`git add src/topmap_topology && git commit -m "feat(topology): add topology invariant capture and comparison"`

---

### T19 `topmap_topology`：拓扑保持压缩

**依赖：** T18
**文件：** 新增 `include/topmap_topology/topology_compressor.hpp`、`src/topology_compressor.cpp`、`test/test_topology_compressor.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_topology/topology_compressor.hpp"
namespace {
topmap::SurfaceRegion R(int id) { topmap::SurfaceRegion r; r.id = id; r.area = 10.0f; return r; }
topmap::Portal P(int id, int a, int b) { topmap::Portal p; p.id = id; p.region_a = a; p.region_b = b; p.valid_width = 1.5f; return p; }
}
TEST(TopologyCompressorTest, DegreeTwoChainRegionMerges) {
  auto g = topmap::TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  topmap::CompressionParams c; c.max_merge_cost = 1.0f;
  auto out = topmap::TopologyCompressor().compress(g, c);
  ASSERT_TRUE(out.ok()) << out.status().message;
  EXPECT_EQ(out.value().second.regions_before, 3);
  EXPECT_EQ(out.value().second.regions_after, 2);
  EXPECT_EQ(out.value().second.merges_applied, 1);
  std::string reason;
  EXPECT_TRUE(topmap::TopologyInvariants::sameConnectivity(
      topmap::TopologyInvariants::capture(g),
      topmap::TopologyInvariants::capture(out.value().first), &reason));
}
TEST(TopologyCompressorTest, ArticulationRegionIsNotMerged) {
  // 仅有一个区域 1 连接 0 与 2，合并 1 会破坏桥，必须回滚
  auto g = topmap::TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  // 把区域 0/2 的几何代价设高，使唯一候选被不变量拒绝：直接令 max_merge_cost 足够大，
  // 但桥约束使 region1 为 articulation；压缩器必须保留两个 Portal。
  topmap::CompressionParams c; c.max_merge_cost = 100.0f;
  auto out = topmap::TopologyCompressor().compress(g, c);
  ASSERT_TRUE(out.ok());
  EXPECT_GE(out.value().first.portals().size(), 2u);
}
TEST(TopologyCompressorTest, ParallelPortalsSurvive) {
  auto g = topmap::TopologyGraph::build({R(0), R(1), R(2), R(3)},
                                        {P(0, 0, 2), P(1, 1, 2), P(2, 2, 3)}).take();
  topmap::CompressionParams c; c.max_merge_cost = 1.0f;
  auto out = topmap::TopologyCompressor().compress(g, c);
  ASSERT_TRUE(out.ok());
  EXPECT_LE(out.value().second.regions_after, 3);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_topology test_topology_compressor`。
- [ ] **步骤 3 最小实现**：候选 = 非关键（不在 `articulationRegions`）且度数恰为 2、且两 Portal 通向不同 Region 的 Region；`merge_cost = normal_weight*|Δn| + slope_weight*|Δslope| + roughness_weight*|Δroughness| + geometry_fit`（几何拟合用 `|centroid_a - centroid_b| / max(area_a, area_b)`）；按 `(cost, region_id)` 排序；对每个候选：试合并（R 并入邻居 A，删除 Portal(A,R)，把 Portal(R,B) 改为 Portal(A',B)，重编号），`capture` 前后签名，`preserve_topology==true` 且 `sameConnectivity` 失败 → 回滚并 `merges_rejected++`，否则接受并 `merges_applied++`；所有候选处理完后重建 `TopologyGraph`，返回 `(graph, stats)`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_topology test_topology_compressor`。
- [ ] **步骤 5 提交**：`git add src/topmap_topology && git commit -m "feat(topology): add topology-preserving region compression with rollback"`

---

### T20 `topmap_topology`：地图归档

**依赖：** T19、T11、T04
**文件：** 新增 `include/topmap_topology/map_archive.hpp`、`src/map_archive.cpp`、`test/test_map_archive.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_topology/map_archive.hpp"
namespace {
topmap::MapArchive MakeArchive() {
  topmap::MapArchive a; a.metadata.frame_id = "map"; a.metadata.robot.width = 0.6f;
  topmap::MeshData d; d.vertices = {{0,0,0},{1,0,0},{1,1,0},{0,1,0}}; d.faces = {{0,1,2},{0,2,3}};
  a.mesh = topmap::TriangleMesh::fromMeshData(d, {}).take();
  topmap::SurfaceRegion r0, r1; r0.id = 0; r1.id = 1;
  topmap::Portal p; p.id = 0; p.region_a = 0; p.region_b = 1; p.valid_width = 1.5f;
  a.graph = topmap::TopologyGraph::build({r0, r1}, {p}).take();
  a.metadata.triangle_count = 2; a.metadata.region_count = 2; a.metadata.portal_count = 1;
  return a;
}
}  // namespace
TEST(MapArchiveTest, RoundTrip) {
  const std::string path = "test_map_archive.bin";
  auto a = MakeArchive();
  ASSERT_TRUE(topmap::saveMapArchive(path, a).ok());
  auto b = topmap::loadMapArchive(path);
  ASSERT_TRUE(b.ok()) << b.status().message;
  EXPECT_EQ(b.value().format_version, topmap::kTopmapFormatVersion);
  EXPECT_EQ(b.value().mesh.size(), 2u);
  EXPECT_EQ(b.value().graph.regions().size(), 2u);
  EXPECT_EQ(b.value().graph.portals().size(), 1u);
  EXPECT_EQ(b.value().metadata.frame_id, "map");
  std::remove(path.c_str());
}
TEST(MapArchiveTest, MagicAndVersionRejected) {
  const std::string path = "test_map_bad.bin";
  { topmap::BinaryWriter w; w.writeU32(0xdeadbeef); w.writeU32(1);
    std::ofstream f(path, std::ios::binary); f.write(reinterpret_cast<const char*>(w.bytes().data()), w.bytes().size()); }
  auto b = topmap::loadMapArchive(path);
  ASSERT_FALSE(b.ok()); EXPECT_EQ(b.status().code, topmap::ErrorCode::kSerializationMagicMismatch);
  { topmap::BinaryWriter w; w.writeU32(topmap::kTopmapMagic); w.writeU32(topmap::kTopmapFormatVersion + 1);
    std::ofstream f(path, std::ios::binary); f.write(reinterpret_cast<const char*>(w.bytes().data()), w.bytes().size()); }
  auto b2 = topmap::loadMapArchive(path);
  ASSERT_FALSE(b2.ok()); EXPECT_EQ(b2.status().code, topmap::ErrorCode::kSerializationVersionUnsupported);
  std::remove(path.c_str());
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_topology test_map_archive`。
- [ ] **步骤 3 最小实现**：`saveMapArchive`：读文件失败 → `kSerializationIoError`；写 `[magic][version][endian][section_count]`，随后按 A4.9 顺序写 7 段，每段 `[name_len][name][byte_len][payload]`；`meta` 段含 `frame_id/robot 摘要/map 摘要/triangle|region|portal_count/build_time`；`graph` 段含 regions（triangle_ids/统计/flags/portal_ids）+ portals（含 chain 与 width）+ 边表。`loadMapArchive`：`load` 全文件到 `std::vector<uint8_t>`；校验 magic/version；逐段读取，未知段按 `byte_len` 跳过；`vertices/faces/triangle_attrs` 用 T11 的 `readTriangleMesh` 重建 `TriangleMesh`；`regions/portals` 重建后 `TopologyGraph::build`；结尾允许存在尾部未知数据。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_topology test_map_archive`。
- [ ] **步骤 5 提交**：`git add src/topmap_topology && git commit -m "feat(topology): add versioned map archive serialization"`

---

### T21 `topmap_planner`：起终点 Region 定位

**依赖：** T20、T05
**文件：** 新增 `include/topmap_planner/plan_types.hpp`、`region_locator.hpp`、`src/region_locator.cpp`、`test/test_region_locator.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_planner/region_locator.hpp"
namespace {
std::pair<topmap::TriangleMesh, topmap::TopologyGraph> Build(topmap::SyntheticScene s) {
  auto d = topmap::makeSyntheticMesh(s); auto m = topmap::TriangleMesh::fromMeshData(d.take(), {});
  topmap::TraversabilityEvaluator::apply(m.value(), topmap::TraversabilityParams{});
  topmap::RegionParams rp; auto g = topmap::RegionGrower().grow(m, rp);
  auto sp = topmap::RegionSplitter().split(m, g.value(), rp);
  auto pe = topmap::PortalExtractor().extract(m, sp.value(), {});
  auto pv = topmap::PortalValidator().validate(m, sp.value(), pe.take(), topmap::RobotParams{}, {});
  auto graph = topmap::TopologyGraph::build(sp.value(), pv.take());
  return {m.take(), graph.take()};
}
}  // namespace
TEST(RegionLocatorTest, ProjectsToNearestRegion) {
  auto [m, g] = Build(topmap::SyntheticScene::kRamp);
  auto ep = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kRamp);
  auto r = topmap::RegionLocator().locate(m, g, ep.value().first, 0.30f);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_TRUE(g.region(r.value()) != nullptr);
}
TEST(RegionLocatorTest, TooFarFromSurfaceFails) {
  auto [m, g] = Build(topmap::SyntheticScene::kFlatPlane);
  auto r = topmap::RegionLocator().locate(m, g, Eigen::Vector3f(0, 0, 100.0f), 0.30f);
  ASSERT_FALSE(r.ok()); EXPECT_EQ(r.status().code, topmap::ErrorCode::kStartNotOnSurface);
}
```
- [ ] **步骤 2 运行并确认失败**：`colcon build --packages-up-to topmap_planner`；预期头缺失。
- [ ] **步骤 3 最小实现**：遍历可通行三角形，取 `pointTriangleDistance` 最小者；若最小距离 `> max_project_radius` → `kStartNotOnSurface`；否则返回 `graph.regionOfTriangle(id)`；无 Region → 同错误。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_planner test_region_locator`。
- [ ] **步骤 5 提交**：`git add src/topmap_planner && git commit -m "feat(planner): add start/goal region localization"`

---

### T22 `topmap_planner`：Topology A\*

**依赖：** T17
**文件：** 新增 `include/topmap_planner/topology_astar.hpp`、`src/topology_astar.cpp`、`test/test_topology_astar.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_planner/topology_astar.hpp"
namespace {
topmap::SurfaceRegion R(int id, float x) { topmap::SurfaceRegion r; r.id = id; r.centroid = Eigen::Vector3f(x, 0, 0); return r; }
topmap::Portal P(int id, int a, int b, float w, float cost) {
  topmap::Portal p; p.id = id; p.region_a = a; p.region_b = b; p.valid_width = w;
  p.center = Eigen::Vector3f(0.5f * (a + b), 0, 0); p.traversal_cost = cost; return p;
}
}  // namespace
TEST(TopologyAStarTest, FindsChainAndPortalOrder) {
  auto g = topmap::TopologyGraph::build({R(0,0),R(1,1),R(2,2),R(3,3)},
                                        {P(0,0,1,1.5f,1.0f),P(1,1,2,1.5f,1.0f),P(2,2,3,1.5f,1.0f)}).take();
  int expanded = 0;
  auto r = topmap::TopologyAStar().search(g, 0, 3, 1.0f, &expanded);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().regions, (std::vector<topmap::RegionId>{0,1,2,3}));
  EXPECT_EQ(r.value().portals, (std::vector<topmap::PortalId>{0,1,2}));
  EXPECT_GT(expanded, 0);
}
TEST(TopologyAStarTest, PrefersCheaperParallelPortal) {
  auto g = topmap::TopologyGraph::build({R(0,0),R(1,1)},
                                        {P(0,0,1,1.2f,5.0f), P(1,0,1,1.2f,1.0f)}).take();
  auto r = topmap::TopologyAStar().search(g, 0, 1);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().portals, (std::vector<topmap::PortalId>{1}));
}
TEST(TopologyAStarTest, UnreachableIsReported) {
  auto g = topmap::TopologyGraph::build({R(0,0),R(1,1)}, {}).take();
  auto r = topmap::TopologyAStar().search(g, 0, 1);
  ASSERT_FALSE(r.ok()); EXPECT_EQ(r.status().code, topmap::ErrorCode::kTopologyUnreachable);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_planner test_topology_astar`。
- [ ] **步骤 3 最小实现**：节点 = `{type, id}`（Region/Portal）。`Region→Portal` 代价 `distance(region.centroid, portal.center)`；`Portal→Region` 代价 `portal.traversal_cost + distance(portal.center, region.centroid)`，乘 `weight`（`weight=1.0` 为可采纳 A\*）。`h(node)` = `distance(该节点位置, goal.region.centroid)`，其中 Portal 位置为 `portal.center`。开放集用 `std::priority_queue`，`tie-break` 用 `(f, g, type, id)` 保证确定性。重建交替序列，输出 `regions`/`portals`/`cost`。起终点无 Region → `kInvalidArgument`；不可达 → `kTopologyUnreachable`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_planner test_topology_astar`。
- [ ] **步骤 5 提交**：`git add src/topmap_planner && git commit -m "feat(planner): add topology A* over region-portal graph"`

---

### T23 `topmap_planner`：Surface Corridor

**依赖：** T22、T20
**文件：** 新增 `include/topmap_planner/corridor_builder.hpp`、`src/corridor_builder.cpp`、`test/test_corridor_builder.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_planner/corridor_builder.hpp"
// 复用 T21 的 Build 辅助（复制到本测试文件，保持测试独立）
#include "test_support_plan.hpp"
TEST(CorridorBuilderTest, CorridorIsSubsetAndConnected) {
  auto [m, g] = topmap::test::Build(topmap::SyntheticScene::kRamp);
  topmap::TopologyPath tp; tp.regions = {0, 1}; // 依实际编号调整见下
  ASSERT_GE(g.regions().size(), 2u);
  tp.regions = {g.regions().front().id, g.regions().back().id};
  auto p = topmap::TopologyAStar().search(g, tp.regions[0], tp.regions[1]);
  ASSERT_TRUE(p.ok());
  auto c = topmap::CorridorBuilder().build(m, g, p.value(), 1);
  ASSERT_TRUE(c.ok()) << c.status().message;
  EXPECT_LT(c.value().triangles.size(), m.size());
  EXPECT_FALSE(c.value().regions.empty());
}
```
（`test/test_support_plan.hpp` 为 `topmap_planner/test/` 下的测试专用头，内含 T21 的 `Build` 与 T27 将用的 `BuildGraph`，不参与安装。）
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_planner test_corridor_builder`。
- [ ] **步骤 3 最小实现**：走廊 = 拓扑路径中所有 Region 的 `triangle_ids` 并集；再并入每个 Portal 的邻域：从 `portal.center` 对应的三角形出发做 `portal_rounds = max(1, expansion_rings)` 圈 `kRing` 扩张；去重排序 `triangles`。若并集为空 → `kCorridorBuildFailed`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_planner test_corridor_builder`。
- [ ] **步骤 5 提交**：`git add src/topmap_planner && git commit -m "feat(planner): extract surface corridor from topology path"`

---

### T24 `topmap_planner`：Surface A\*

**依赖：** T23
**文件：** 新增 `include/topmap_planner/surface_astar.hpp`、`src/surface_astar.cpp`、`test/test_surface_astar.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_planner/surface_astar.hpp"
#include "test_support_plan.hpp"
TEST(SurfaceAStarTest, PathConnectsStartGoalThroughWaypoints) {
  auto [m, g] = topmap::test::Build(topmap::SyntheticScene::kRamp);
  auto p = topmap::TopologyAStar().search(g, g.regions().front().id, g.regions().back().id);
  ASSERT_TRUE(p.ok());
  auto c = topmap::CorridorBuilder().build(m, g, p.value(), 1);
  ASSERT_TRUE(c.ok());
  std::vector<Eigen::Vector3f> waypoints;
  auto ep = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kRamp);
  waypoints.push_back(ep.value().first);
  for (auto pid : p.value().portals) waypoints.push_back(g.portal(pid)->center);
  waypoints.push_back(ep.value().second);
  int expanded = 0;
  auto path = topmap::SurfaceAStar().search(m, c.value(), waypoints, &expanded);
  ASSERT_TRUE(path.ok()) << path.status().message;
  ASSERT_GE(path.value().size(), 2u);
  EXPECT_GT(expanded, 0);
  EXPECT_LT((path.value().front() - waypoints.front()).norm(), 0.5f);
  EXPECT_LT((path.value().back() - waypoints.back()).norm(), 0.5f);
}
TEST(SurfaceAStarTest, MissingCorridorTriangleFails) {
  auto [m, g] = topmap::test::Build(topmap::SyntheticScene::kRamp);
  topmap::Corridor empty;
  auto path = topmap::SurfaceAStar().search(m, empty, {Eigen::Vector3f::Zero(), Eigen::Vector3f(0,0,1)});
  ASSERT_FALSE(path.ok());
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_planner test_surface_astar`。
- [ ] **步骤 3 最小实现**：对相邻 waypoint 逐段调用内部 `searchSegment(mesh, corridor, from, to, expanded)`：先用 `SafetyValidator::locatePoint` 把 from/to 映到走廊内三角形；A\* 节点 = 三角形 id，边缘代价 = 质心欧氏距离，启发式 = 到目标三角形质心的欧氏距离；只允许走廊内且可通行的三角形；不可达 → `kSurfaceSearchFailed`。段结果去除相邻重复点后拼接为质心折线，返回。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_planner test_surface_astar`。
- [ ] **步骤 5 提交**：`git add src/topmap_planner && git commit -m "feat(planner): add segmented surface A* through portals"`

---

### T25 `topmap_planner`：路径优化

**依赖：** T24
**文件：** 新增 `include/topmap_planner/path_optimizer.hpp`、`src/path_optimizer.cpp`、`test/test_path_optimizer.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_planner/path_optimizer.hpp"
#include "test_support_plan.hpp"
TEST(PathOptimizerTest, ShortcutRemovesCollinearPoints) {
  topmap::PlanParams params;
  auto [m, g] = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  topmap::Corridor c; c.triangles = topmap::test::AllTriangleIds(m);
  std::vector<Eigen::Vector3f> pts = {{0,0,0},{0.1f,0,0},{0.2f,0,0},{0.3f,0,0}};
  std::size_t removed = 0;
  auto out = topmap::PathOptimizer().shortcut(pts, m, c, params, &removed);
  EXPECT_LT(out.size(), pts.size());
  EXPECT_GT(removed, 0u);
}
TEST(PathOptimizerTest, SmoothingBoundsCurvatureAndKeepsEndpoints) {
  topmap::PlanParams params; params.smoothing_iterations = 10;
  auto [m, g] = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  std::vector<Eigen::Vector3f> pts = {{0,0,0},{1,0,0},{1,1,0},{2,1,0}};
  auto out = topmap::PathOptimizer().smooth(pts, m, params);
  ASSERT_GE(out.size(), 2u);
  EXPECT_TRUE(out.front().isApprox(pts.front()));
  EXPECT_TRUE(out.back().isApprox(pts.back()));
}
TEST(PathOptimizerTest, InvalidShortcutIsRejected) {
  topmap::PlanParams params;
  auto [m, g] = topmap::test::Build(topmap::SyntheticScene::kOverlappingFloors);
  topmap::Corridor c; c.triangles = topmap::test::ComponentTriangleIds(m, 0);
  std::vector<Eigen::Vector3f> pts = {{0,0,0},{0.2f,0,0},{0.4f,0,0}};
  std::size_t removed = 0;
  auto out = topmap::PathOptimizer().shortcut(pts, m, c, params, &removed);
  EXPECT_EQ(removed, 0u);
  EXPECT_EQ(out.size(), pts.size());
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_planner test_path_optimizer`。
- [ ] **步骤 3 最小实现**：
  - `shortcut`：从 i=0 起对 j 从末端回溯，若线段 `(p_i,p_j)` 的按 `check_step` 采样点都落在 `corridor.triangles` 内、且每点 `min_clearance >= required_clearance`、最大坡度不超限 → 保留 `p_j`、`removed += j-i-1`、`i=j`；否则 `j--`。返回新折线。
  - `funnel`：按 Portal 顺序以 `left/right` 端点跑 Simple Stupid Funnel，输出转向点；若参数未启用则原样返回。
  - `smooth`：对内部点做 `iterations` 次 Laplacian（权重 0.5，端点固定），每次迭代后检查每个点仍在可通行三角形内且 `min_clearance >= required_clearance`，否则该点回退到上一轮；最后做曲率检查（相邻三点外接圆曲率 `kappa = 2*sin(angle)/|AC|`），仅保留 `kappa <= curvature_limit` 的缩短步骤。`optimize` 依次 `shortcut`（`shortcut_enabled`）→ `funnel`（`funnel_enabled`）→ `smooth`（`smoothing_enabled`），任一步骤结果少于 2 点或安全复核失败 → `kPathOptimizationFailed`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_planner test_path_optimizer`。
- [ ] **步骤 5 提交**：`git add src/topmap_planner && git commit -m "feat(planner): add shortcut, funnel and curvature-limited smoothing"`

---

### T26 `topmap_planner`：安全复核

**依赖：** T24、T03
**文件：** 新增 `include/topmap_planner/safety_validator.hpp`、`src/safety_validator.cpp`、`test/test_safety_validator.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_planner/safety_validator.hpp"
#include "test_support_plan.hpp"
TEST(SafetyValidatorTest, ValidPathPasses) {
  auto [m, g] = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  std::vector<Eigen::Vector3f> path = {{0,0,0},{0.5f,0,0},{1,0,0}};
  auto rep = topmap::SafetyValidator().validate(path, m, topmap::TraversabilityParams{}, 0.05f, 0.25f);
  EXPECT_TRUE(rep.valid) << "violations=" << rep.violating_indices.size();
}
TEST(SafetyValidatorTest, PointOffSurfaceFails) {
  auto [m, g] = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  std::vector<Eigen::Vector3f> path = {{0,0,0},{0,0,10.0f},{1,0,0}};
  auto rep = topmap::SafetyValidator().validate(path, m, topmap::TraversabilityParams{}, 0.05f, 0.25f);
  EXPECT_FALSE(rep.valid);
  EXPECT_FALSE(rep.violating_indices.empty());
}
TEST(SafetyValidatorTest, LocatePointReturnsDistance) {
  auto [m, g] = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  float d = -1.0f;
  auto t = topmap::SafetyValidator().locatePoint(m, Eigen::Vector3f(0.5f, 0.5f, 1.0f), 2.0f, &d);
  ASSERT_TRUE(t.ok()); EXPECT_NEAR(d, 1.0f, 1e-3f);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_planner test_safety_validator`。
- [ ] **步骤 3 最小实现**：`locatePoint` 遍历可通行三角形取 `pointTriangleDistance` 最小；`> max_distance` → `kInvalidArgument`。`validate` 按 `check_step` 重采样折线；每点定位到三角形，检查 `traversable` 与 `min_clearance >= required_clearance`；相邻点对检查 `isEdgeTraversable` 与 `heightJump`；三点检查坡度与曲率；任一不满足把下标记入 `violating_indices`。统计 `min_clearance/max_step/max_slope_deg/max_curvature`；`valid = violating_indices.empty()`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_planner test_safety_validator`。
- [ ] **步骤 5 提交**：`git add src/topmap_planner && git commit -m "feat(planner): add collision, clearance, slope and curvature validation"`

---

### T27 `topmap_planner`：分层规划器与降级链

**依赖：** T21–T26
**文件：** 新增 `include/topmap_planner/hierarchical_planner.hpp`、`src/hierarchical_planner.cpp`、`test/test_hierarchical_planner.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_planner/hierarchical_planner.hpp"
#include "test_support_plan.hpp"
TEST(HierarchicalPlannerTest, FullPipelineOnRamp) {
  auto [m, g] = topmap::test::Build(topmap::SyntheticScene::kRamp);
  auto ep = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kRamp);
  topmap::PlanRequest req; req.start = ep.value().first; req.goal = ep.value().second;
  auto r = topmap::HierarchicalPlanner(topmap::PlanParams{}).plan(m, g, req);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_GE(r.value().path.size(), 2u);
  EXPECT_GT(r.value().stats.path_length, 0.0f);
  EXPECT_FALSE(r.value().topology.portals.empty());
}
TEST(HierarchicalPlannerTest, UnreachableGoalReturnsTopologyError) {
  auto [m, g] = topmap::test::Build(topmap::SyntheticScene::kOverlappingFloors);
  auto ep = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kOverlappingFloors);
  topmap::PlanRequest req; req.start = ep.value().first; req.goal = ep.value().second;
  auto r = topmap::HierarchicalPlanner(topmap::PlanParams{}).plan(m, g, req);
  ASSERT_FALSE(r.ok()); EXPECT_EQ(r.status().code, topmap::ErrorCode::kTopologyUnreachable);
}
TEST(HierarchicalPlannerTest, FallsBackToFullMeshWhenCorridorTooSmall) {
  auto [m, g] = topmap::test::Build(topmap::SyntheticScene::kHallNarrowHall);
  auto ep = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kHallNarrowHall);
  topmap::PlanRequest req; req.start = ep.value().first; req.goal = ep.value().second;
  topmap::PlanParams params; params.corridor_expansion_rings = 0; params.fallback_full_mesh = true;
  auto r = topmap::HierarchicalPlanner(params).plan(m, g, req);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_TRUE(r.value().stats.fallback_used);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_planner test_hierarchical_planner`。
- [ ] **步骤 3 最小实现**：`plan` 流程与降级（对规格 §8 逐条）：
  1. `RegionLocator` 定位 start/goal；失败分别返回 `kStartNotOnSurface`/`kGoalNotOnSurface`。
  2. `TopologyAStar`；失败直接返回 `kTopologyUnreachable`。
  3. `CorridorBuilder(expansion_rings)`；`kCorridorBuildFailed` → 以 `expansion_rings+1` 重试一次，再失败且 `fallback_full_mesh` → 走廊 = 全部可通行三角形并置 `stats.fallback_used=true`。
  4. `SurfaceAStar` 分段；`kSurfaceSearchFailed` → 同 3 的回退。
  5. `PathOptimizer::optimize`；`kPathOptimizationFailed` → 保留 T24 原始离散路径，置 `fallback_used=true`。
  6. `SafetyValidator::validate`；`valid==false` → 若曾优化则回退原始路径并再次复核；仍失败返回 `kPathValidationFailed`。
  7. 填 `PlanStats`（长度、最小间隙、最大坡度、最大曲率、展开节点、各阶段耗时）。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_planner test_hierarchical_planner`。
- [ ] **步骤 5 提交**：`git add src/topmap_planner && git commit -m "feat(planner): add hierarchical planner with documented fallbacks"`

---

### T28 `topmap_planner`：基准与消融（Method A/B/C）

**依赖：** T27
**文件：** 新增 `include/topmap_planner/benchmark_runner.hpp`、`src/benchmark_runner.cpp`、`src/benchmark_main.cpp`、`test/test_benchmark_runner.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include "topmap_planner/benchmark_runner.hpp"
TEST(BenchmarkRunnerTest, HierarchicalExpandsFewerNodesThanFullMesh) {
  topmap::TopmapConfig cfg;
  auto records = topmap::runBenchmark({topmap::SyntheticScene::kRamp},
                                      {topmap::PlanningMethod::kMeshAStar,
                                       topmap::PlanningMethod::kHierarchical}, cfg, 0.25f);
  ASSERT_TRUE(records.ok()) << records.status().message;
  ASSERT_EQ(records.value().size(), 2u);
  const auto& full = records.value()[0];
  const auto& hier = records.value()[1];
  ASSERT_TRUE(full.success); ASSERT_TRUE(hier.success);
  EXPECT_LT(hier.expanded_nodes, full.expanded_nodes);
}
TEST(BenchmarkRunnerTest, CsvRoundTripAndAblationCounts) {
  topmap::TopmapConfig cfg;
  auto records = topmap::runBenchmark(
      {topmap::SyntheticScene::kHallNarrowHall},
      {topmap::PlanningMethod::kPointCloudAStar, topmap::PlanningMethod::kMeshAStar,
       topmap::PlanningMethod::kHierarchical}, cfg, 0.25f);
  ASSERT_TRUE(records.ok());
  EXPECT_EQ(records.value().size(), 3u);
  const std::string csv = "test_benchmark.csv";
  ASSERT_TRUE(topmap::writeBenchmarkCsv(csv, records.value()).ok());
  auto read = topmap::readBenchmarkCsv(csv);
  ASSERT_TRUE(read.ok());
  EXPECT_EQ(read.value().size(), 3u);
  EXPECT_EQ(read.value()[2].method, "hierarchical");
  std::remove(csv.c_str());
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_planner test_benchmark_runner`。
- [ ] **步骤 3 最小实现**：
  - Method A `kPointCloudAStar`：以 `MeshData.vertices` 为点、`point_radius` 半径建 kNN 图，边可通行性用 `TriangleNode` 近似（共享边/最近三角形），A\* 到目标最近点。
  - Method B `kMeshAStar`：`SurfaceAStar` 在全部可通行三角形（全图走廊）。
  - Method C `kHierarchical`：`HierarchicalPlanner` 全流程（压缩开）。
  - 记录 `BenchmarkRecord`；`benchmarkCsvHeader` 固定列：`scene,method,success,expanded_nodes,planning_time_sec,path_length,min_clearance,triangle_count,region_count,portal_count,invalid_cross_layer_edges,reachability_preserved,portal_order_valid,regions_before_compression,regions_after_compression,error_code`。
  - 消融：通过 `TopmapConfig` 变体实现 `Ablation1`（`compression.max_merge_cost=0` 关压缩）、`2`（`region` 生长退化为纯几何：`normal_weight=1,others=0, threshold=1e9`）、`3`（`region.bottleneck_width=0` 关瓶颈）、`4`（`compression.preserve_topology=false`）、`5`（比较 B 与 C 展开节点）。每个消融把配置摘要写入 `BenchmarkRecord.method` 后缀（如 `hierarchical/no_compression`）。
  - `benchmark_main.cpp`：CLI `--config src/topmap_bringup/config/topmap.yaml --output benchmark/results/benchmark.csv --scenes flat_plane,ramp,overlapping_floors,multi_level_ramps,hall_narrow_hall,u_shape,t_shape,single_bridge,parallel_portals,loop --methods point_cloud,mesh,hierarchical --point-radius 0.25 --ablation all`，用 `ament_add_executable(benchmark_runner ...)` 安装到 `lib/${PROJECT_NAME}`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_planner test_benchmark_runner`。
- [ ] **步骤 5 提交**：`git add src/topmap_planner && git commit -m "feat(planner): add Method A/B/C benchmark and ablation runner"`

---

### T29 `topmap_ros`：类型/Marker 转换

**依赖：** T20、T01
**文件：** 新增 `include/topmap_ros/type_conversions.hpp`、`marker_builder.hpp`、`src/type_conversions.cpp`、`src/marker_builder.cpp`、`test/test_type_conversions.cpp`、`test/test_marker_builder.cpp`
- [ ] **步骤 1 失败测试**
```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "topmap_ros/type_conversions.hpp"
#include "topmap_ros/marker_builder.hpp"
TEST(TypeConversionsTest, RegionRoundTrip) {
  topmap::SurfaceRegion r; r.id = 4; r.centroid = Eigen::Vector3f(1, 2, 3);
  r.area = 2.5f; r.is_ramp = true; r.portal_ids = {7, 8};
  auto m = topmap_ros::toMsg(r);
  EXPECT_EQ(m.id, 4); EXPECT_FLOAT_EQ(m.centroid.x, 1.0f); EXPECT_TRUE(m.is_ramp);
  auto back = topmap_ros::fromMsg(m);
  EXPECT_EQ(back.id, 4); EXPECT_EQ(back.portal_ids, (std::vector<topmap::PortalId>{7, 8}));
}
TEST(TypeConversionsTest, PathMessageHasPoses) {
  std::vector<Eigen::Vector3f> pts = {{0,0,0},{1,0,0}};
  auto msg = topmap_ros::toMsg(pts, "map", rclcpp::Time(0));
  EXPECT_EQ(msg.header.frame_id, "map");
  ASSERT_EQ(msg.poses.size(), 2u);
  EXPECT_FLOAT_EQ(msg.poses[1].pose.position.x, 1.0f);
}
TEST(MarkerBuilderTest, PortalAndRegionMarkers) {
  topmap::Portal p; p.id = 0; p.region_a = 0; p.region_b = 1;
  p.left = Eigen::Vector3f(0,-0.6f,0); p.right = Eigen::Vector3f(0,0.6f,0);
  auto markers = topmap_ros::buildPortalMarkers({p}, "map", rclcpp::Time(0));
  ASSERT_EQ(markers.markers.size(), 1u);
  EXPECT_EQ(markers.markers[0].type, visualization_msgs::msg::Marker::LINE_LIST);
  EXPECT_EQ(markers.markers[0].points.size(), 2u);
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_ros test_type_conversions`。
- [ ] **步骤 3 最小实现**：A4.9 全量 msg↔C++ 函数；`buildRegionMarkers`（每个 Region 一个 `TRIANGLE_LIST`，颜色由 `HSV` 按 `id` 取模）、`buildPortalMarkers`（`LINE_LIST` 红）、`buildCorridorMarkers`、`buildTopologyPathMarkers`、`buildStartGoalMarkers`；所有 marker 设 `frame_locked=false`、`lifetime=0`、`ns` 前缀 `topmap/`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_ros test_type_conversions test_marker_builder`。
- [ ] **步骤 5 提交**：`git add src/topmap_ros && git commit -m "feat(ros): add message and marker conversions"`

---

### T30 `topmap_ros`：`MapBuilderNode`

**依赖：** T29、T20
**文件：** 新增 `include/topmap_ros/map_builder_node.hpp`、`src/map_builder_node.cpp`、`src/map_builder_main.cpp`、`test/test_support_ros.hpp`、`test/testdata/topmap_valid.yaml`、`test/test_map_builder_node.cpp`
- [ ] **步骤 1 失败测试** `test/test_map_builder_node.cpp`
```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "topmap_ros/map_builder_node.hpp"
#include "topmap_msgs/srv/build_map.hpp"
#include "topmap_surface/synthetic_scene.hpp"
#include "test_support_ros.hpp"
TEST(MapBuilderNodeTest, BuildsMapFromPcdAndReportsStats) {
  topmap_ros::test::RclcppGuard ros;
  const std::string pcd = topmap::test::WriteScenePcd(topmap::SyntheticScene::kHallNarrowHall, "tb_hall.pcd");
  auto node = std::make_shared<topmap_ros::MapBuilderNode>(
      rclcpp::NodeOptions().append_parameter_override("pcd_path", pcd)
                           .append_parameter_override("output_map_path", "tb_hall.bin")
                           .append_parameter_override("config_file", topmap_ros::test::validConfigPath()));
  auto client = node->create_client<topmap_msgs::srv::BuildMap>("/topmap/build_map");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto req = std::make_shared<topmap_msgs::srv::BuildMap::Request>();
  req->pcd_path = pcd; req->output_map_path = "tb_hall.bin";
  auto fut = client->async_send_request(req);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, fut, std::chrono::seconds(30)),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(fut.get()->success) << fut.get()->error_message;
  EXPECT_GT(fut.get()->stats.region_count, 0u);
  EXPECT_GT(fut.get()->stats.portal_count, 0u);
  std::remove("tb_hall.bin"); std::remove(pcd.c_str());
}
```
`topmap::test::WriteScenePcd`、`topmap_ros::test::validConfigPath()` 和 RAII 类型 `topmap_ros::test::RclcppGuard` 位于 `test/test_support_ros.hpp`；`RclcppGuard` 在构造时调用 `rclcpp::init`、析构时调用 `rclcpp::shutdown`。CMake 通过 `TOPMAP_ROS_TEST_SOURCE_DIR` 编译定义生成绝对测试路径，避免依赖测试进程工作目录。
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_ros test_map_builder_node`。
- [ ] **步骤 3 最小实现**：`MapBuilderNode` 声明参数 `pcd_path`、`output_map_path`、`config_file`、`frame_id`；构造时 `loadTopmapConfig`，创建服务 `/topmap/build_map`（`BuildMap.srv`），发布 `/topmap/original_cloud`（`sensor_msgs/PointCloud2`）、`/topmap/regions`、`/topmap/portals`、`/topmap/map_stats`。`handleBuildMap` 依次调用 `PointCloudProcessor → SurfaceFeatureEstimator → SurfaceTriangulator → TriangleMesh::fromMeshData → TraversabilityEvaluator::apply → RegionGrower → RegionSplitter → PortalExtractor → PortalValidator → TopologyGraph::build → TopologyCompressor → saveMapArchive`，失败时把 `static_cast<int32_t>(code)` 与 `toString(code)` 填入响应；成功发布 markers 与 stats。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_ros test_map_builder_node`。
- [ ] **步骤 5 提交**：`git add src/topmap_ros && git commit -m "feat(ros): add offline map builder service"`

---

### T31 `topmap_ros`：`PlannerNode`

**依赖：** T29、T27
**文件：** 新增 `include/topmap_ros/planner_node.hpp`、`src/planner_node.cpp`、`src/planner_main.cpp`、`test/test_planner_node.cpp`
- [ ] **步骤 1 失败测试** `test/test_planner_node.cpp`
```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "topmap_ros/planner_node.hpp"
#include "topmap_msgs/srv/plan_path.hpp"
#include "topmap_surface/synthetic_scene.hpp"
#include "test_support_ros.hpp"
TEST(PlannerNodeTest, PlansPathOnRampArchive) {
  topmap_ros::test::RclcppGuard ros;
  const std::string map = topmap::test::BuildRampArchive("tb_ramp.bin");
  auto node = std::make_shared<topmap_ros::PlannerNode>(
      rclcpp::NodeOptions().append_parameter_override("map_path", map)
                           .append_parameter_override("config_file", topmap_ros::test::validConfigPath()));
  auto client = node->create_client<topmap_msgs::srv::PlanPath>("/topmap/plan_path");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto req = std::make_shared<topmap_msgs::srv::PlanPath::Request>();
  auto ep = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kRamp);
  req->start.pose.position.x = ep.value().first.x();
  req->start.pose.position.y = ep.value().first.y();
  req->start.pose.position.z = ep.value().first.z();
  req->goal.pose.position.x = ep.value().second.x();
  req->goal.pose.position.y = ep.value().second.y();
  req->goal.pose.position.z = ep.value().second.z();
  auto fut = client->async_send_request(req);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, fut, std::chrono::seconds(30)),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(fut.get()->success) << fut.get()->error_message;
  EXPECT_GT(fut.get()->path.poses.size(), 1u);
  std::remove(map.c_str());
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_ros test_planner_node`。
- [ ] **步骤 3 最小实现**：`PlannerNode` 声明参数 `map_path`、`config_file`、`frame_id`；构造时 `loadMapArchive` 与 `loadTopmapConfig`，创建服务 `/topmap/plan_path`（`PlanPath.srv`），发布 `/topmap/final_path`（`nav_msgs/Path`）、`/topmap/topology_path`（`TopologyPath`）、`/topmap/corridor`、`/topmap/start_goal`（`MarkerArray`）。`handlePlanPath` 构造 `PlanRequest`，调 `HierarchicalPlanner::plan`，把路径、拓扑序列、`PlanStats` 与错误码写入响应；发布可视化。帧来自请求 `header.frame_id`，为空则用参数 `frame_id`。
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_ros test_planner_node`。
- [ ] **步骤 5 提交**：`git add src/topmap_ros && git commit -m "feat(ros): add online planner node with plan_path service"`

---

### T32 `topmap_bringup`：YAML / launch / RViz + 端到端

**依赖：** T30、T31
**文件：**
- 新增 `src/topmap_bringup/{CMakeLists.txt,package.xml}`
- 新增 `config/topmap.yaml`，完整包含 A4.4 规定的 `topmap.robot/map/traversability/region/portal/compression/plan` 全部必需键；`map_builder_node` 与 `planner_node` 的 `config_file` 均指向该文件。
- 新增 `launch/topmap_offline_build.launch.py`、`launch/topmap_online_plan.launch.py`、`launch/topmap_rviz.launch.py`
- 新增 `rviz/topmap.rviz`（显示类型固定：`PointCloud2`(原始)、`MarkerArray`(regions/portals/corridor/start_goal)、`Path`(final_path)、`MarkerArray`(topology_path)）
- 新增 `scripts/run_acceptance.sh`
- 测试：`src/topmap_ros/test/test_end_to_end.cpp`（已在 T30/T31 覆盖核心；此处增加 launch 冒烟命令）
- [ ] **步骤 1 失败测试** `src/topmap_ros/test/test_end_to_end.cpp`
```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "topmap_ros/map_builder_node.hpp"
#include "topmap_ros/planner_node.hpp"
#include "topmap_surface/synthetic_scene.hpp"
#include "test_support_ros.hpp"
TEST(EndToEndTest, BuildThenPlanClosedLoop) {
  topmap_ros::test::RclcppGuard ros;
  const std::string pcd = topmap::test::WriteScenePcd(topmap::SyntheticScene::kHallNarrowHall, "e2e.pcd");
  const std::string map = "e2e.bin";
  auto builder = std::make_shared<topmap_ros::MapBuilderNode>(
      rclcpp::NodeOptions().append_parameter_override("pcd_path", pcd)
                           .append_parameter_override("output_map_path", map)
                           .append_parameter_override("config_file", topmap_ros::test::validConfigPath()));
  auto bcli = builder->create_client<topmap_msgs::srv::BuildMap>("/topmap/build_map");
  ASSERT_TRUE(bcli->wait_for_service(std::chrono::seconds(5)));
  { auto r = std::make_shared<topmap_msgs::srv::BuildMap::Request>();
    r->pcd_path = pcd; r->output_map_path = map;
    auto f = bcli->async_send_request(r);
    ASSERT_EQ(rclcpp::spin_until_future_complete(builder, f, std::chrono::seconds(60)),
              rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(f.get()->success) << f.get()->error_message; }
  auto planner = std::make_shared<topmap_ros::PlannerNode>(
      rclcpp::NodeOptions().append_parameter_override("map_path", map)
                           .append_parameter_override("config_file", topmap_ros::test::validConfigPath()));
  auto pcli = planner->create_client<topmap_msgs::srv::PlanPath>("/topmap/plan_path");
  ASSERT_TRUE(pcli->wait_for_service(std::chrono::seconds(5)));
  auto ep = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kHallNarrowHall);
  auto r = std::make_shared<topmap_msgs::srv::PlanPath::Request>();
  r->start.pose.position.x = ep.value().first.x(); r->start.pose.position.y = ep.value().first.y();
  r->start.pose.position.z = ep.value().first.z();
  r->goal.pose.position.x = ep.value().second.x(); r->goal.pose.position.y = ep.value().second.y();
  r->goal.pose.position.z = ep.value().second.z();
  auto f = pcli->async_send_request(r);
  ASSERT_EQ(rclcpp::spin_until_future_complete(planner, f, std::chrono::seconds(60)),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(f.get()->success) << f.get()->error_message;
  EXPECT_GT(f.get()->path.poses.size(), 1u);
  EXPECT_GE(f.get()->stats.min_clearance, 0.0f);
  std::remove(map.c_str()); std::remove(pcd.c_str());
}
```
- [ ] **步骤 2 运行并确认失败**：`BUILD/TEST topmap_ros test_end_to_end`；预期因 launch/config 缺失或服务名不符失败。
- [ ] **步骤 3 最小实现**：写三个 launch。配置默认值使用 `PathJoinSubstitution([FindPackageShare('topmap_bringup'), 'config', 'topmap.yaml'])`；离线节点参数明确写为 `parameters=[{'config_file': LaunchConfiguration('config_file'), 'pcd_path': LaunchConfiguration('pcd_path'), 'output_map_path': LaunchConfiguration('output_map_path'), 'frame_id': 'map'}]`，在线节点参数明确写为 `parameters=[{'config_file': LaunchConfiguration('config_file'), 'map_path': LaunchConfiguration('map_path'), 'frame_id': 'map'}]`，不能把 `topmap.yaml` 当作 ROS 参数文件直接传入。RViz 节点使用 `arguments=['-d', rviz_path]`。`CMakeLists.txt` 用 `install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})` 并在 `package.xml` 声明 `exec_depend`；`run_acceptance.sh`：
```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
cd /home/robot/topmap_ws
colcon build --packages-up-to topmap_ros --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 run topmap_surface write_synthetic_pcd --scene hall_narrow_hall --output test_data/synthetic/scene_hall_narrow_hall.pcd
ros2 launch topmap_bringup topmap_offline_build.launch.py \
  pcd_path:=test_data/synthetic/scene_hall_narrow_hall.pcd \
  output_map_path:=test_data/maps/hall.bin \
  config_file:=src/topmap_bringup/config/topmap.yaml &
builder_pid=$!
planner_pid=
trap 'kill "${planner_pid:-}" "${builder_pid:-}" 2>/dev/null || true' EXIT
timeout 30 bash -c 'until ros2 service type /topmap/build_map >/dev/null 2>&1; do sleep 0.2; done'
ros2 service call /topmap/build_map topmap_msgs/srv/BuildMap \
  "{pcd_path: 'test_data/synthetic/scene_hall_narrow_hall.pcd', output_map_path: 'test_data/maps/hall.bin'}"
test -s test_data/maps/hall.bin
ros2 launch topmap_bringup topmap_online_plan.launch.py \
  map_path:=test_data/maps/hall.bin \
  config_file:=src/topmap_bringup/config/topmap.yaml &
planner_pid=$!
timeout 30 bash -c 'until ros2 service type /topmap/plan_path >/dev/null 2>&1; do sleep 0.2; done'
ros2 service call /topmap/plan_path topmap_msgs/srv/PlanPath "{start: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: 7.0, y: 0.0, z: 0.0}}}}"
```
- [ ] **步骤 4 运行并通过**：`BUILD/TEST topmap_ros test_end_to_end`；随后手动冒烟：
```bash
colcon build --packages-up-to topmap_bringup
source install/setup.bash
ros2 launch topmap_bringup topmap_offline_build.launch.py --show-args
ros2 launch topmap_bringup topmap_online_plan.launch.py --show-args
ros2 launch topmap_bringup topmap_rviz.launch.py --show-args
```
预期：端到端 gtest PASS；三个 launch `--show-args` 均输出参数列表且退出码 0。
- [ ] **步骤 5 提交**：`git add src/topmap_bringup src/topmap_ros/test/test_end_to_end.cpp && git commit -m "feat(bringup): add configs, launch files, RViz layout and end-to-end test"`

---

### T33 验收与基准报告

**依赖：** T28、T32
**文件：**
- 修改 `src/topmap_bringup/scripts/run_acceptance.sh`（加入验收断言）
- 新增 `benchmark/REPORT.md`（运行后由脚本追加）
- 新增 `test_data/README.md`
- [ ] **步骤 1 失败测试** 在 `run_acceptance.sh` 末尾加入可判定断言：
```bash
BENCH_CSV=benchmark/results/benchmark.csv
ros2 run topmap_planner benchmark_runner --config src/topmap_bringup/config/topmap.yaml \
  --output "$BENCH_CSV" --scenes flat_plane,ramp,overlapping_floors,hall_narrow_hall \
  --methods point_cloud,mesh,hierarchical --ablation all
python3 - "$BENCH_CSV" <<'PY'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
assert rows, "benchmark produced no rows"
for r in rows:
    assert r["success"] == "1", f"planning failed: {r}"
assert all(int(r["invalid_cross_layer_edges"]) == 0 for r in rows), "cross-floor connection detected"
hier = [r for r in rows if r["method"] == "hierarchical"]
full = [r for r in rows if r["method"] == "mesh"]
assert hier and full, "missing method rows"
assert all(r["reachability_preserved"] == "1" for r in hier), "compression changed reachability"
assert all(r["portal_order_valid"] == "1" for r in hier), "surface path violated portal order"
assert all(int(h["expanded_nodes"]) < int(f["expanded_nodes"]) for h in hier
           for f in full if f["scene"] == h["scene"]), "hierarchical did not reduce expansions"
assert all(float(r["min_clearance"]) >= 0.15 for r in rows if r["success"] == "1"), "clearance violated"
print("ACCEPTANCE OK")
PY
```
- [ ] **步骤 2 运行并确认失败**：`bash src/topmap_bringup/scripts/run_acceptance.sh`；首次运行预期 Python 断言报 `planning failed` 或 `missing method rows`（因报告文件/字段尚未生成）。
- [ ] **步骤 3 最小实现**：确保 `benchmark_runner` 输出 `success`、`reachability_preserved`、`portal_order_valid` 为 `0/1`，并输出真实的 `invalid_cross_layer_edges` 与 `min_clearance`；验收脚本直接断言跨层错误连接为 0、压缩前后可达保持率为 100%、分层路径遵守 Portal 顺序、最小间隙 ≥ 0.15 m、Method C 展开节点少于同场景 Method B。脚本把结果追加到 `benchmark/REPORT.md`。
- [ ] **步骤 4 运行并通过**：`bash src/topmap_bringup/scripts/run_acceptance.sh`；预期末行 `ACCEPTANCE OK`，退出码 0，`benchmark/results/benchmark.csv` 与 `benchmark/REPORT.md` 生成。
- [ ] **步骤 5 提交**：`git add src/topmap_bringup/scripts/run_acceptance.sh benchmark/REPORT.md test_data/README.md && git commit -m "test(acceptance): add end-to-end acceptance assertions and benchmark report"`

---

## C. 一致性审查（约束 10）

### C1 包依赖无环复核（逐项）

- `core` 与 `msgs` 均无算法依赖 → 无环。
- `surface → core`，不依赖 `topology`；`topology → {surface, core}`；`planner → {topology, surface, core}`；`ros → {msgs, planner, topology, surface, core}`；`bringup --exec_depend--> ros`。**不存在 `planner→ros`、`core→msgs`、`surface→topology` 等回边。** 通过。
- `topmap_msgs/PlanPath.srv` 引用 `nav_msgs/Path`，因此 `topmap_msgs` 的 `<depend>` 必须含 `nav_msgs`，否则 `rosidl_generate_interfaces` 会失败；A2 已声明。通过。
- `topmap_planner` 的 `benchmark_runner` 需要 YAML 配置，只依赖 `yaml-cpp`（已在 core），不引入 ROS。通过。

### C2 类型名/签名一致性核对表

| 名称 | 唯一定义处 | 被引用任务 | 一致 |
|---|---|---|---|
| `ErrorCode/Status/Result<T>/toString` | `core/error.hpp` | T03–T33 | ✅（`toString` 覆盖全部 28 个枚举值，T02 已列全） |
| `kTopmapMagic/kTopmapFormatVersion` | `core/binary_io.hpp` | T11,T20,T29 | ✅ |
| `RobotParams/MapBuildParams/TraversabilityParams/RegionParams/PortalParams/CompressionParams/PlanParams/TopmapConfig` | `core/params.hpp` | T03–T33 | ✅（`loadTopmapConfig` 单一入口） |
| `MeshData/TriangleNode/TriangleEdge/TriangleMesh` | `surface/mesh_types.hpp` | T08–T28 | ✅（`fromMeshData`/`fromTriangleNodes` 两个构造入口，测试统一用前者） |
| `TraversabilityEvaluator::{isTriangleTraversable,isEdgeTraversable,heightJump,apply}` | `surface/traversability.hpp` | T10,T13–T27 | ✅ |
| `SurfaceTriangulator::triangulate` / `TriangulationMode` | `surface/triangulator.hpp` | T09,T30 | ✅ |
| `writeTriangleMesh/readTriangleMesh` | `surface/mesh_serialization.hpp` | T11,T20 | ✅ |
| `makeSyntheticMesh/makeSyntheticEndpoints/SyntheticScene` | `surface/synthetic_scene.hpp` | T12,T13,T14,T15,T21,T27,T28,T30–T33 | ✅ |
| `RegionGrower::grow` / `regionJoinScore` | `topology/region_growing.hpp` | T13,T14,T21,T30 | ✅ |
| `RegionSplitter::split` | `topology/region_split.hpp` | T14,T15,T21,T30 | ✅ |
| `PortalExtractor::extract` / `PortalValidator::validate` | `topology/portal_*.hpp` | T15,T16,T21,T30 | ✅ |
| `SurfaceRegion/Portal` | `topology/region_types.hpp` | T14–T33 | ✅（`Portal.valid_width` 与 `msg/Portal.msg valid_width` 同名，T29 转换字段一一对应） |
| `TopologyGraph::build` 及访问器 | `topology/topology_graph.hpp` | T17–T31 | ✅（测试用 `portalsOf/neighborRegions/otherRegion/connectedComponents/region/portal/regionOfTriangle`） |
| `TopologyInvariants::{capture,sameConnectivity,bridgePortals,articulationRegions}` + `GraphSignature` | `topology/topology_invariants.hpp` | T18,T19 | ✅ |
| `TopologyCompressor::compress` 返回 `Result<std::pair<TopologyGraph,CompressionStats>>` | `topology/topology_compressor.hpp` | T19,T30 | ✅（测试用 `.value().first/.second`） |
| `saveMapArchive/loadMapArchive/MapArchive/MapMetadata` | `topology/map_archive.hpp` | T20,T30,T31 | ✅（`saveMapArchive` 返回 `Status`，`loadMapArchive` 返回 `Result<MapArchive>`；测试已按此写） |
| `PlanRequest/TopologyPath/PlanStats/PlanResult/PathValidationReport/Corridor` | `planner/plan_types.hpp` | T21–T31 | ✅ |
| `RegionLocator/TopologyAStar/CorridorBuilder/SurfaceAStar/PathOptimizer/SafetyValidator/HierarchicalPlanner` | 各自头文件 | T21–T31 | ✅（`SurfaceAStar::search` 参数由“start/goal+required_portals”统一为 `waypoints`，T23/T24/T27 一致） |
| `benchmarkCsvHeader/benchmarkCsvRow/writeBenchmarkCsv/readBenchmarkCsv/runBenchmark` | `planner/benchmark_runner.hpp` | T28,T33 | ✅ |
| `topmap_ros::{toMsg,fromMsg,build*Markers}` | `ros/type_conversions.hpp`、`marker_builder.hpp` | T29–T32 | ✅（`toMsg` 对 `SurfaceRegion/Portal/TopologyGraph/path/MapMetadata` 重载，返回类型在 A4.9 固定） |
| 服务名 `/topmap/build_map`、`/topmap/plan_path` | T30/T31 | T32/T33 | ✅（三处字符串一致） |
| 参数名 `pcd_path/output_map_path/config_file/map_path/frame_id` | T30/T31 | T32 | ✅ |

**发现并已修正的两处不一致：**
1. 规格 §4 依赖图 `core→msgs` 与“core 不依赖 ROS”矛盾 → 本计划调整为二者平级（A2/C1）。
2. `readBenchmarkCsv` 在测试中使用但初稿 A4.8 未列 → 已补入冻结契约（C2 表末行）。

### C3 测试命令合理性审查

- 全部算法包（core/surface/topology/planner）用 `ament_add_gtest` + `colcon test`，不要求 `source install/setup.bash`，命令 `colcon build --packages-up-to <pkg>` 会按 A2 拓扑序自动构建依赖，合理。
- `topmap_ros` 的 gtest 需要运行时动态库路径，故测试步骤先 `colcon build --packages-up-to topmap_ros` 再 `colcon test`；必要时在测试前 `source install/setup.bash`。已在 T29–T32 明示。
- YAML 相对路径测试（T03）通过 `set_tests_properties(... WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})` 固定，避免 `colcon test` 工作目录不确定导致的偶发失败。
- T32/T33 的 launch 冒烟不用 `launch_testing`（该依赖不在约束技术栈内），而以 `ros2 launch --show-args` + `ros2 service call` 作为可执行验收，闭环保留在 T32 的 gtest 内完成，工具链仍为 GoogleTest。合理。
- 所有测试结果统一使用 `colcon test-result --verbose`；失败时命令返回非零，`console_direct+` 同时给出具体失败用例。
- 逐任务复查确认：每个任务的“最小实现”均给出可编译的签名、公式或代码体，不存在空缺步骤。

### C4 覆盖对照（规格/研究计划书 → 任务）

| 需求 | 覆盖任务 |
|---|---|
| PCD 加载/滤波/降采样 | T06 |
| 法向/坡度/粗糙度/曲率 | T07 |
| 多层地表 Mesh、最大边长、2.5D 兜底 | T09 |
| 共享边邻接、禁止跨层连接 | T08（结构层）+ T09（几何层） |
| 三角面/相邻面可通行 | T10 |
| Region 生长（法向/坡度/粗糙度/高度） | T13 |
| 瓶颈/凹形/坡道分裂 | T14 |
| Boundary Chain + 多 Portal | T15 |
| 宽度/坡度/台阶/Clearance + 足迹腐蚀 | T16 |
| Region-Portal 二部图 | T17 |
| 拓扑不变量（桥/并行/跨层/环路） | T18 |
| 拓扑保持合并 + 回滚 | T19 |
| 序列化版本与错误码 | T04,T20 |
| 起终点定位（配置半径投影） | T21 |
| Topology A\* | T22 |
| Surface Corridor | T23 |
| 分段 Surface A\* 与 Portal 顺序 | T24 |
| Shortcut/Funnel/曲率平滑 | T25 |
| 碰撞/间隙/坡度/曲率复核与回退 | T26,T27 |
| 错误处理与降级（§8 全部条目） | T02,T06,T09,T16,T20,T21,T22,T23,T27 |
| ROS msg/srv | T01 |
| 离线/在线节点 | T30,T31 |
| RViz 显示（Region/Portal/图/路径/起终点） | T29,T32 |
| YAML 参数 | T03,T32 |
| 合成场景与验收 | T12,T33 |
| Method A/B/C 与 5 项消融 | T28,T33 |
| 每阶段可运行/可测增量 | T01–T33（每任务结束均有构建+测试+提交） |

**结论：** 计划满足全部 10 条约束；类型名、签名、文件路径、依赖方向和测试命令已经统一。本计划已落盘为 `doc/2026-09-24-topmap-algorithm-implementation-plan.md`，并在 D–G 节补充阶段里程碑、验收指标、风险与降级、需求到任务覆盖矩阵。

---

## D. 阶段里程碑（Phase Milestones）

每个里程碑结束时，表中“退出判据”命令必须全部通过并产出对应交付物；未通过不得进入下一里程碑。

| 里程碑 | 覆盖任务 | 交付物 | 退出判据（命令与阈值） |
|---|---|---|---|
| M1 骨架与契约 | T01–T05 | 7 个可构建包、`topmap_msgs` 接口、`topmap_core` 错误/ID/参数/二进制/几何 | `colcon build --packages-up-to topmap_core` 成功；`colcon test --packages-select topmap_core --event-handlers console_direct+` 全绿 |
| M2 地表建模 | T06–T12 | 点云预处理、点特征、共享边 Mesh、三角化、可通行性、Mesh 序列化、10 个合成场景 | `colcon test --packages-select topmap_surface` 全绿；`test_synthetic_scene` 的 10 个场景全部生成成功 |
| M3 拓扑地图 | T13–T20 | Region 生长/分裂、Portal 提取/验证、二部图、不变量、压缩、`topmap_map.bin` 归档 | `colcon test --packages-select topmap_topology` 全绿；`overlapping_floors` 跨层错误连接数为 0；压缩前后连通分量与桥接 Portal 保持 |
| M4 分层规划 | T21–T28 | 起终点定位、Topology A*、Corridor、Surface A*、路径优化、安全复核、分层规划器、基准 | `colcon test --packages-select topmap_planner` 全绿；`test_hierarchical_planner` 端到端规划成功且降级链被覆盖；`benchmark_runner` 输出 CSV |
| M5 ROS 与验收 | T29–T33 | 类型/Marker 转换、`MapBuilderNode`、`PlannerNode`、YAML/launch/RViz、验收脚本与报告 | `colcon test --packages-select topmap_ros` 全绿；`ros2 launch topmap_bringup topmap_offline_build.launch.py --show-args` 与 `topmap_online_plan.launch.py --show-args` 退出码 0；`bash src/topmap_bringup/scripts/run_acceptance.sh` 末行 `ACCEPTANCE OK` |

---

## E. 验收指标（Acceptance Metrics）

| 编号 | 指标 | 定义 | 目标值 | 测量来源 | 对应任务 |
|---|---|---|---|---|---|
| A-01 | 上下层错误连接数 | `overlapping_floors` 场景中 XY 重叠但高度不连续的跨层可通行连接数量 | 0 | T33 验收脚本与 T18 不变量测试 | T08,T10,T15,T18,T33 |
| A-02 | 关键可达关系保持率 | 压缩前后对全部 Region 对执行可达性求解，结果一致的比例 | 100% | T19 测试与 T33 脚本重算 | T19,T33 |
| A-03 | 路径 Portal 顺序一致率 | 输出路径经过的 Portal 序列与 Topology A* 给出的序列一致的比例 | 100% | T24 测试与 T33 脚本 | T22,T24,T33 |
| A-04 | 路径最小间隙 | 平滑后路径上采样点到最近障碍或边界的距离最小值 | ≥ 0.15 m（来自 YAML 安全距离） | T26 测试与 T33 脚本 | T16,T26,T33 |
| A-05 | 规划展开节点数 | Method C（分层）相对 Method B（完整 Mesh A*）的展开节点数 | 每个场景 Method C 严格小于 Method B | T28 基准 CSV 与 T33 脚本 | T28,T33 |
| A-06 | 路径代价劣化 | Method C 相对 Method B 的路径长度与总代价变化 | 实测值必须写入报告；路径长度劣化 ≤ 20% 作为通过阈值，超出需在报告中给出场景与原因 | T28 基准 CSV 与 `benchmark/REPORT.md` | T28,T33 |
| A-07 | 端到端闭环 | 一条命令完成离线构图、在线规划与验收断言 | `run_acceptance.sh` 退出码 0 且打印 `ACCEPTANCE OK` | T33 脚本 | T30,T31,T32,T33 |
| A-08 | 单元测试通过率 | 全部算法包与 ROS 包 gtest 通过比例 | 100% | `colcon test-result --verbose` | T01–T33 |

---

## F. 风险与降级（Risks and Fallbacks）

| 风险 | 触发条件 | 影响 | 降级/规避措施 | 错误码或回退 | 对应任务 |
|---|---|---|---|---|---|
| 三维三角化失败 | CGAL AFSR 在稀疏或噪声区域抛异常或产生空洞 | 无法生成连续地表 Mesh | 切换到分层 2.5D 三角化；两种模式都失败则终止构图 | `ErrorCode::kTriangulationFailed` | T09 |
| 跨层误连 | 上下层 XY 重叠且高度差小于阈值 | 拓扑图出现虚假连接，规划穿越楼板 | 只允许共享 Mesh 边建邻接；Region 生长加入高度跨度与坡度方差约束；Portal 复核台阶与间隙 | `ErrorCode::kGraphInvariantViolation` | T08,T10,T13,T15,T16 |
| 压缩破坏拓扑 | 合并串联 Region 时丢失桥或唯一跨层连接 | 关键可达关系丢失 | 每次收缩后重跑不变量检查，不满足立即回滚该次收缩 | 回滚候选收缩，保留原图 | T18,T19 |
| 起终点不在地表 | 输入点落在障碍或地图外 | 无法定位 Region | 在配置半径内投影到最近可通行三角形；超过半径返回定位失败，不规划 | `ErrorCode::kStartNotOnSurface` 或 `ErrorCode::kGoalNotOnSurface` | T21 |
| 拓扑不可达 | 二部图不存在起终点通路 | 无解却仍启动搜索 | Topology A* 直接返回不可达，禁止启动机器人 | `ErrorCode::kTopologyUnreachable` | T22 |
| Corridor 内精规划失败 | 分段 Surface A* 在走廊内找不到连续三角面路径 | 有拓扑路但无几何路 | 按配置扩张相邻 Region 重新提取走廊；仍失败则回退完整 Mesh A* | 回退 `SurfaceAStar` 全图模式 | T23,T24,T27 |
| 平滑越界 | Shortcut/Funnel 平滑后碰撞、间隙或曲率不满足 | 输出不可执行路径 | 返回最后一条通过安全复核的离散路径 | 回退未平滑路径 | T25,T26,T27 |
| 地图版本不兼容 | `topmap_map.bin` 的 magic 或 version 与当前代码不一致 | 读取错误或内存越界 | 拒绝加载并报告实际版本与期望版本 | `ErrorCode::kSerializationMagicMismatch` 或 `ErrorCode::kSerializationVersionUnsupported` | T04,T20 |
| 参数缺失或非法 | YAML 缺字段或数值越界 | 运行时未定义行为 | 严格加载，缺字段或越界即拒绝启动 | `ErrorCode::kInvalidArgument` | T03 |
| 性能不达标 | 分层规划展开节点或耗时未明显低于完整 Mesh A* | 课题核心指标不成立 | 增大体素、调整 Region 合并阈值、缓存邻接；在基准报告中记录实测值 | 报告实测数据并给出调参记录 | T28 |
| 依赖版本漂移 | PCL/CGAL/Boost 版本不一致导致编译或数值差异 | 构建失败或结果不稳定 | 固定 ROS 2 Humble 源与 `cmake_minimum_required(VERSION 3.14)`，在 `package.xml` 声明最低版本，CI 全量构建 | 构建期失败并给出包名 | T01,T32 |

---

## G. 需求到任务覆盖矩阵（Requirements to Tasks）

来源缩写：设计规格 = `doc/2026-09-24-topmap-algorithm-design.md`；研究计划书 = `doc/Robot-aware Multi-level Traversable Surface Graph.md`。

| 需求来源与条目 | 需求内容 | 覆盖任务 | 验收证据 |
|---|---|---|---|
| 设计规格 §5.1 | PCD → 滤波点云 → 特征 → Mesh → 三角面图 | T06,T07,T08,T09 | T06–T09 gtest |
| 设计规格 §5.1 | 初始 Region → 精化 Region → Portal 候选 → 验证 Portal → 压缩图 → `topmap_map.bin` | T13,T14,T15,T16,T17,T19,T20 | T13–T20 gtest |
| 设计规格 §5.2 | 起终点定位 → Topology A* → 有序 Region/Portal → Corridor → 分段 Surface A* → 拼接平滑 → 复核 → `nav_msgs/Path` | T21,T22,T23,T24,T25,T26,T27,T31 | T21–T27 与 T31 gtest |
| 设计规格 §6.1 | `TriangleNode` 字段与真实共享边邻接 | T08 | T08 gtest |
| 设计规格 §6.2 | `SurfaceRegion` 字段与最大连通一致区域语义 | T13,T14 | T13,T14 gtest |
| 设计规格 §6.3 | `Portal` 字段与一对 Region 多 Portal | T15,T16,T17 | T15–T17 gtest |
| 设计规格 §6.4 | `Region→Portal→Region` 二部图与桥、并行、跨层、环路 | T17,T18 | T17,T18 gtest |
| 设计规格 §7.1 | NaN 与离群过滤、VoxelGrid、PCA 法向置 Z 朝上、坡度/粗糙度/曲率、最大边长、分层 2.5D 兜底 | T06,T07,T09 | T06,T07,T09 gtest |
| 设计规格 §7.2 | 单三角形与相邻三角形可通行判据、机器人参数全部来自 YAML | T03,T10 | T03,T10 gtest |
| 设计规格 §7.3 | Region 生长加权评分与高度/坡度约束、窄带阻断、瓶颈/凹形/坡道分裂、测地局部宽度 | T13,T14 | T13,T14 gtest |
| 设计规格 §7.4 | 共享边 Boundary Chain、宽度/坡度/台阶/Clearance 验证、有效宽度公式 | T15,T16 | T15,T16 gtest |
| 设计规格 §7.5 | 压缩同时检查几何代价与拓扑不变量、失败回滚 | T18,T19 | T18,T19 gtest |
| 设计规格 §7.6 | 定位、A*、走廊、分段精规划、Shortcut/Funnel/曲率平滑、复核回退 | T21,T22,T23,T24,T25,T26,T27 | T21–T27 gtest |
| 设计规格 §8 | 空云与非法坐标、三角化失败、起终点投影、拓扑不可达、走廊失败扩张、平滑越界、版本不兼容 | T02,T06,T09,T16,T20,T21,T22,T23,T25,T27 | 错误码与降级 gtest |
| 设计规格 §9.1 | 法向/坡度/粗糙度、共享边、阈值边界、Region 生长/分裂/合并、Boundary Chain、拓扑不变量、A*、安全复核 | T07,T08,T10,T13,T14,T15,T18,T19,T22,T24,T25,T26 | 各任务 gtest |
| 设计规格 §9.2 | 单层平面、连续坡道、上下层 XY 重叠、多层多坡道、大厅-窄道-大厅、U/T 型、单桥/并行 Portal/闭环、不可达起终点 | T12,T33 | T12 生成 10 场景；T33 覆盖 4 个基准场景 |
| 设计规格 §9.3 | 错误跨层连接 0、压缩保持率 100%、路径按 Portal 顺序、最小间隙达标、展开节点下降、劣化被报告、Launch 闭环 | T18,T19,T24,T26,T28,T32,T33 | E 节 A-01 至 A-08 |
| 设计规格 §10 | 先接口后失败测试再最小实现、核心不读 ROS、阈值集中 YAML、归档元数据、每阶段可视化与统计、先闭环后优化 | T01–T33 | 每任务 5 步 TDD；T03,T20,T29,T32 |
| 研究计划书 五.5.1–5.2 | 点云预处理与法向/坡度/粗糙度/曲率 | T06,T07 | T06,T07 gtest |
| 研究计划书 五.5.3–5.5 | 三角化、可通行三角面、三角邻接图 | T08,T09,T10 | T08,T09,T10 gtest |
| 研究计划书 五.5.6–5.7 | Region Growing、Region 分裂、拓扑瓶颈 | T13,T14 | T13,T14 gtest |
| 研究计划书 五.5.8 | 拓扑保持 Region 合并 | T18,T19 | T18,T19 gtest |
| 研究计划书 五.5.9–5.11 | 公共边界、Portal 候选、Robot-aware 验证、Region-Portal 图 | T15,T16,T17 | T15,T16,T17 gtest |
| 研究计划书 六 | Topology A*、Surface Corridor、局部 A*、拼接平滑、最终 3D Path | T22,T23,T24,T25,T27 | T22–T25 与 T27 gtest |
| 研究计划书 八 | 课题成果与 ROS 2 落地 | T29,T30,T31,T32,T33 | T29–T33 gtest 与验收脚本 |
