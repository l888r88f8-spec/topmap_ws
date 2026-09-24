# 多层可通行拓扑图算法设计规格

**日期：** 2026-09-24  
**目标读者：** 直接编码实施的 ROS 2/C++ 开发团队  
**状态：** 已批准架构，待书面规格复核

## 1. 目标

从离线地面点云 `GroundMap.pcd` 自动构建机器人感知的多层 `Region-Portal` 拓扑地图，并在该地图上完成拓扑粗规划、地表走廊提取和三维地表精规划，最终输出可供控制器执行的 `nav_msgs/msg/Path`。

系统必须满足：

- 不依赖人工指定楼层高度；
- 保持楼层、坡道、平台和窄通道之间的真实连通关系；
- 上下层在 XY 平面重叠时不得建立错误连接；
- 所有可通行判断均受机器人尺寸、坡度、台阶和安全距离约束；
- 离线地图构建与在线路径规划分离；
- 核心算法可脱离 ROS 2 运行单元测试。

## 2. 范围

### 2.1 包含

- PCD 点云加载、滤波和降采样；
- 法向、坡度、粗糙度与曲率计算；
- 多层地表 Triangle Mesh 构建；
- 三角面和相邻三角面的可通行性判断；
- Region 生长、瓶颈分裂、凹形分裂和坡道分离；
- Robot-aware Portal 提取与验证；
- 拓扑保持的 Region 合并和图压缩；
- Region-Portal 二部图构建与序列化；
- Topology A*、Surface Corridor 和 Surface A*；
- 路径平滑、安全复核、ROS 2 接口及 RViz2 可视化。

### 2.2 不包含

- SLAM、点云配准和机器人定位；
- 动态障碍预测及局部避障；
- 底盘控制器和运动控制；
- 在线增量地图更新；
- 语义分割、房间命名和人工楼层标注；
- 完整动力学轨迹优化。

## 3. 技术栈

- Ubuntu 22.04；
- ROS 2 Humble；
- C++17 与 `ament_cmake`；
- PCL：点云滤波、邻域和特征估计；
- Eigen：向量、矩阵及几何计算；
- CGAL：三角化、边界链和几何运算；
- Boost Graph：Region-Portal 图及 A*；
- yaml-cpp：机器人和算法参数；
- GoogleTest：单元测试；
- RViz2：调试可视化。

## 4. 软件架构

```text
topmap_ws/
├── src/
│   ├── topmap_msgs/       # 消息和服务定义
│   ├── topmap_core/       # 核心数据类型、参数、错误和序列化
│   ├── topmap_surface/    # 点云、Mesh、特征和可通行性
│   ├── topmap_topology/   # Region、瓶颈、Portal 和拓扑压缩
│   ├── topmap_planner/    # 粗规划、Corridor、精规划和路径优化
│   ├── topmap_ros/        # 离线建图和在线规划节点
│   └── topmap_bringup/    # 参数、Launch 和 RViz 配置
├── test_data/             # 合成及真实测试地图
└── doc/                   # 设计、计划和实验文档
```

依赖方向固定为：

```text
topmap_msgs ───────────────────────┐
                                  ↓
topmap_core → surface → topology → planner → ros → bringup
```

`topmap_msgs` 与 `topmap_core` 互不依赖；`topmap_core` 不依赖 ROS 2。算法包只依赖核心类型和必要的几何库，ROS 消息转换集中在 `topmap_ros`。

## 5. 数据流

### 5.1 离线地图构建

```text
GroundMap.pcd
  → FilteredPointCloud
  → SurfaceFeatures
  → TriangleMesh
  → TraversableTriangleGraph
  → InitialRegions
  → RefinedRegions
  → PortalCandidates
  → ValidatedPortals
  → CompressedRegionPortalGraph
  → topmap_map.bin
```

### 5.2 在线路径规划

```text
topmap_map.bin + Start + Goal
  → Start/Goal Region 定位
  → Topology A*
  → 有序 Region/Portal 序列
  → Surface Corridor
  → 分段 Surface A*
  → 路径拼接与平滑
  → 碰撞、间隙和曲率复核
  → nav_msgs/msg/Path
```

## 6. 核心模型

### 6.1 TriangleNode

保存顶点索引、质心、法向、坡度、粗糙度、曲率、面积、间隙、可通行标志及真实共享边邻居。三角形之间仅在共享同一 Mesh 边时建立邻接。

### 6.2 SurfaceRegion

保存三角面集合、面积、高度范围、坡度统计、粗糙度、坡道标志、拓扑关键标志和关联 Portal。Region 必须是连续、可通行且地形属性近似一致的最大地表区域。

### 6.3 Portal

保存两侧 Region、边界链、中心、左右端点、原始宽度、有效宽度、坡度、台阶、最小间隙、通行代价和关键连接标志。同一对 Region 之间允许存在多个 Portal。

### 6.4 TopologyGraph

采用 `Region → Portal → Region` 二部图。图必须保留桥接 Portal、并行 Portal、唯一跨层连接和有效环路。

## 7. 算法设计

### 7.1 地表建模

1. 对输入点云执行 NaN 删除、离群点过滤和 VoxelGrid；
2. 使用 PCA 估计法向，并统一为 Z 轴朝上；
3. 计算坡度、粗糙度和曲率；
4. 使用三维地表三角化并限制最大边长；
5. 当三维三角化不能稳定保持地表时，按连通分量或高度带执行分层 2.5D 三角化；
6. 台阶高度和 Portal 宽度返回原始分辨率点云复核。

### 7.2 可通行性

单三角形判据为：

\[
Traversable(T_i)=C_{slope}\land C_{roughness}\land C_{support}
\]

相邻三角形还需满足法向差、坡度差、高度突变和共享边连续性约束。初始机器人参数为宽 `0.60 m`、长 `0.80 m`、最大坡度 `20°`、最大台阶 `0.05 m`、安全距离 `0.15 m`，所有值必须来自 YAML 配置。

### 7.3 Region 构建

Region Growing 使用法向差、坡度差、粗糙度差和高度连续性加权评分。除局部评分外，候选三角形加入后还必须满足 Region 高度跨度和坡度方差约束，防止“一层—坡道—二层”被吞并为单一区域。

生长完成后依次执行：

1. 不可通行窄带阻断；
2. 可通行瓶颈分裂；
3. L/U/T 型凹形区域分裂；
4. 坡度突变和坡道边界分裂。

局部宽度在地表测地空间计算，不使用跨层混叠的 XY 距离场。

### 7.4 Portal

Portal 从相邻 Region 的公共 Mesh 边提取。共享边首先按端点连通性组成独立 Boundary Chain，再分别验证宽度、坡度、台阶和 Clearance。

有效宽度必须满足：

\[
W_p>W_{robot}+2d_{safe}
\]

当前初始参数对应 `W_p > 0.90 m`。Portal 的宽度采用穿越边界方向的最小通行宽度，而不是简单使用边界链长度。

### 7.5 拓扑压缩

Region 合并同时检查几何代价和拓扑不变量。允许合并的前提是：

- 连通分量数量不变；
- 桥接 Portal 和唯一跨层连接不丢失；
- 并行 Portal 数量和有效环路不被错误消除；
- 关键 Region 对的机器人可达关系不变。

压缩优先处理度为 2、非关键、几何属性一致的串联 Region。每次图收缩后重新验证不变量，不满足条件立即回滚本次候选收缩。

### 7.6 分层规划

1. 起点和终点依据三维距离、法向一致性及地表连通分量定位到 Region；
2. 在 Region-Portal 二部图运行 A*；
3. 根据拓扑结果提取 Region 集合及 Portal 邻域形成 Surface Corridor；
4. 按拓扑确定的 Portal 顺序执行分段 Surface A*；
5. 路径拼接后执行 Shortcut、Funnel 和曲率受限平滑；
6. 平滑后重新执行碰撞、最小间隙、坡度和曲率检查；
7. 优化失败时返回最后一条已验证的离散路径。

## 8. 错误处理和降级

- PCD 为空、坐标非法或点数不足：地图构建失败并返回结构化错误；
- 三角化失败：切换到分层 2.5D；仍失败则终止构图；
- 起终点不在地表：允许在配置半径内投影，超过半径则返回定位失败；
- Topology A* 无路：返回拓扑不可达，不启动机器人；
- Corridor 内精规划失败：按配置扩张相邻 Region，再失败时回退完整 Mesh A*；
- 路径平滑越界：回退到未经平滑但已验证的路径；
- 地图格式或版本不兼容：拒绝加载并报告实际版本和期望版本。

## 9. 测试策略

### 9.1 单元测试

- 法向朝向、坡度和粗糙度；
- Mesh 共享边邻接；
- 台阶、坡度和宽度阈值边界；
- Region 生长、分裂和合并；
- Boundary Chain 与多 Portal；
- 桥、环路和跨层可达性保持；
- Topology A* 和 Surface A*；
- 路径安全复核及失败回退。

### 9.2 合成场景

- 单层平面；
- 连续坡道；
- 上下层 XY 重叠；
- 多层多坡道；
- 大厅—窄通道—大厅；
- U 型和 T 型区域；
- 单桥、并行 Portal 和闭环拓扑；
- 不可达起终点。

### 9.3 系统验收

- 上下层错误连接数为 0；
- 拓扑压缩前后关键可达关系保持率为 100%；
- 路径全部按拓扑 Portal 顺序通过；
- 路径最小间隙不小于配置安全距离；
- 相比完整 Mesh A*，规划展开节点数和时间显著降低；
- 路径长度和总代价的劣化量被实验明确报告；
- 所有 ROS 2 节点可通过 Launch 完成离线构图和在线规划闭环。

## 10. 实施原则

- 每个算法模块先定义接口和失败测试，再写最小实现；
- 核心算法不读取 ROS 参数、不发布 Topic；
- 所有阈值集中在带版本的 YAML 配置中；
- 离线产物包含地图版本、坐标系、机器人配置摘要和算法参数摘要；
- 每个阶段必须输出可视化结果和机器可读统计；
- 首个里程碑优先建立“PCD 输入到 Path 输出”的可运行闭环，再逐步加入瓶颈检测、拓扑压缩和性能优化。
