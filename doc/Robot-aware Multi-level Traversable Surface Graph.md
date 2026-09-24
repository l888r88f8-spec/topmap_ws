# 复杂三维环境下多层可通行拓扑地图与全局规划课题实施计划书

## 一、课题名称

**面向复杂三维环境的多层可通行拓扑地图构建与地面机器人快速全局规划方法研究**

---

## 二、研究背景与问题

在厂区、地下车库、多层建筑、坡道及室外起伏道路等复杂三维环境中，地面机器人虽然主要沿地表运动，但其可通行空间具有明显的三维特征，例如：

* 不同高度地面重叠；
* 坡道连接不同高度区域；
* 多层平台；
* 狭窄通道；
* 局部高程变化；
* 台阶、陡坡等不可通行区域。

直接在原始三维地面点云上进行全局路径规划时，通常存在以下问题：

1. 点云规模较大，搜索节点数量多；
2. 点云缺少天然的拓扑邻接关系，需要额外进行邻域搜索；
3. 三维欧氏距离无法准确描述多层环境中的真实可达关系；
4. 搜索过程中需要频繁进行坡度、连续性以及可通行性判断；
5. 大量地面点表达相同连续地表，存在较大的数据冗余；
6. 多层结构容易出现错误跨层连接；
7. 大规模环境下全局规划时间随点云规模明显增加。

因此，本课题拟研究一种面向地面机器人导航的**多层可通行区域拓扑地图表达方法**，将密集地面点云转换为具有明确区域和连接关系的 `Region-Portal` 拓扑图。

在保持坡道、多层地面以及关键通行结构的同时压缩搜索空间，并进一步研究基于**拓扑粗规划 + 地表精规划**的分层全局路径规划方法。

---

# 三、总体研究目标

本课题最终实现以下目标。

## 3.1 多层地面结构自动建模

从已有三维地面点云：

```text
GroundMap.pcd
```

中自动提取机器人可通行地表。

不依赖人工设置：

```text
第一层高度
第二层高度
第三层高度
```

而根据地表连续性自动保持：

```text
一层
  ↓
坡道
  ↓
二层
  ↓
坡道
  ↓
三层
```

之间的真实连通关系。

---

## 3.2 自动生成 Region

将大量地面点或三角面自动划分为具有导航意义的可通行区域，例如：

```text
R1：一层大厅
R2：走廊
R3：坡道
R4：二层平台
R5：狭窄通道
```

Region 内部应满足：

* 地表连续；
* 地形特征近似；
* 可通行属性近似；
* 不包含明显拓扑瓶颈。

---

## 3.3 自动生成 Portal

从不同 Region 的公共边界中自动提取机器人能够通过的 Portal。

例如：

```text
Region A
   ↓
 Portal
   ↓
Region B
```

Portal 需要满足：

* 足够的通行宽度；
* 坡度满足机器人能力；
* 台阶高度满足机器人越障能力；
* 足迹不会碰撞；
* 满足安全距离要求。

---

## 3.4 构建多层 Region-Portal 拓扑图

构建：

$$
G_T=(R\cup P,E)
$$

其中：

* \(R\)：Region；
* \(P\)：Portal；
* \(E\)：Region 与 Portal 之间的连接。

最终形成：

```text
R1
 │
P1
 │
R2
 │
P2
 │
R3
 ├──── P3 ──── R4
 │
P4
 │
R5
```

从而显式表达复杂三维环境中的真实地面连通关系。

---

## 3.5 实现拓扑保持压缩

减少大量冗余 Region，同时保持：

* 楼层连通关系；
* 坡道连接关系；
* 狭窄通道；
* 路口结构；
* 关键 Portal；
* 原始地图的机器人可达关系。

目标为：

$$
G_{dense}
\rightarrow
G_{compressed}
$$

并尽可能满足：

$$
Connectivity(G_{dense})
=
Connectivity(G_{compressed})
$$

---

## 3.6 实现分层全局路径规划

采用：

```text
拓扑粗规划
    ↓
提取 Surface Corridor
    ↓
局部地表精规划
    ↓
路径优化
    ↓
最终 3D Path
```

避免直接在整个密集三维地面地图上搜索。

最终目标是：

> 在保证复杂三维地面真实连通关系和路径可执行性的前提下，显著降低全局路径规划节点数量和规划时间。

---

# 四、总体技术路线

```text
GroundMap.pcd
      │
      ▼
点云预处理
      │
      ▼
法向 / 坡度 / 粗糙度计算
      │
      ▼
地表三角化
      │
      ▼
Traversable Triangle
      │
      ▼
Triangle Adjacency Graph
      │
      ▼
Traversability-Constrained Region Growing
      │
      ▼
初始 Region
      │
      ▼
Region 分裂
      │
      ├── 凹区域检测
      ├── 狭窄通道检测
      └── 拓扑瓶颈检测
      │
      ▼
Region 拓扑保持合并
      │
      ▼
Region 公共边界提取
      │
      ▼
Portal Candidate
      │
      ▼
Robot-aware Portal Validation
      │
      ▼
Region-Portal Graph
      │
      ▼
Topology A*
      │
      ▼
Surface Corridor
      │
      ▼
局部 Mesh / 地面图 A*
      │
      ▼
路径拼接和平滑
      │
      ▼
最终三维全局路径
```

---

# 五、主要研究内容

# 5.1 地面点云预处理

输入：

```text
GroundMap.pcd
```

进行：

* NaN 点去除；
* 离群点过滤；
* VoxelGrid 降采样；
* 坐标统一。

第一阶段可设置：

```yaml
voxel_size: 0.10
```

后续实验测试：

```text
0.05 m
0.10 m
0.20 m
```

三种分辨率。

---

# 5.2 地表特征计算

对于每个点计算：

* 法向量；
* 曲率；
* 粗糙度；
* 局部坡度；
* 局部高度变化。

点特征可表示为：

$$
F_i=
\{
p_i,n_i,c_i,r_i,\theta_i
\}
$$

其中：

* \(p_i\)：三维位置；
* \(n_i\)：法向量；
* \(c_i\)：曲率；
* \(r_i\)：粗糙度；
* \(\theta_i\)：局部坡度。

---

# 5.3 地表三角化

将地面点云转换为 Triangle Mesh。

得到：

$$
M=(V_M,F_M)
$$

其中：

* \(V_M\)：Mesh 顶点；
* \(F_M\)：三角面。

每个三角形：

$$
T_i
$$

保存：

```cpp
struct TriangleNode
{
    int id;

    Eigen::Vector3f centroid;
    Eigen::Vector3f normal;

    float slope;
    float roughness;
    float curvature;
    float area;

    bool traversable;

    std::vector<int> neighbors;
};
```

---

# 5.4 三角形可通行性判断

根据机器人能力设置：

```yaml
robot_width:       0.60
robot_length:      0.80
max_slope:         20.0
max_step_height:   0.05
safety_distance:   0.15
```

以上参数仅作为第一阶段调试参数，最终应根据真实机器人确定。

三角形坡度：

$$
\theta_i
=
\arccos
\left(
n_i\cdot
\begin{bmatrix}
0\\
0\\
1
\end{bmatrix}
\right)
$$

定义：

$$
Traversable(T_i)=
C_{slope}
\land
C_{roughness}
\land
C_{support}
$$

对于相邻三角形还需要判断：

$$
Traversable(T_i,T_j)
$$

避免两个三角面虽然相邻，但存在机器人不能通过的高度突变。

---

# 六、Region 生成算法

## 6.1 建立 Triangle Adjacency Graph

构建：

$$
G_M=(V_M,E_M)
$$

其中：

```text
Triangle = Node
Shared Mesh Edge = Edge
```

只有真正共享 Mesh 边的两个三角形才建立邻接关系。

这一机制可以避免：

```text
二层
────────────

────────────
一层
```

在 XY 方向重叠时发生错误连接。

---

## 6.2 Traversability-Constrained Region Growing

选择未分配的可通行三角形作为 Seed。

对于当前三角形 \(T_i\) 的相邻三角形 \(T_j\)，计算：

### 法向差异

$$
D_n=
\arccos(n_i\cdot n_j)
$$

### 坡度差异

$$
D_s=
|\theta_i-\theta_j|
$$

### 粗糙度差异

$$
D_r=
|r_i-r_j|
$$

### 高度连续性

$$
D_h=h_{ij}
$$

构建综合评价：

$$
Score(T_i,T_j)
=
w_nD_n+
w_sD_s+
w_rD_r+
w_hD_h
$$

如果：

$$
Score<T_{region}
$$

并且：

$$
Traversable(T_i,T_j)=true
$$

则：

```text
Tj → 当前 Region
```

否则建立新的 Region。

---

## 6.3 Region 定义

Region 定义为：

> 内部机器人通行性质近似一致、地表连续，并且不存在明显导航拓扑瓶颈的最大连通地表区域。

数学形式：

$$
R_i=
\max
\{
S\subset M
\mid
Connected(S)
\land
Homogeneous(S)
\land
Traversable(S)
\}
$$

---

# 七、Region 分裂

单纯 Region Growing 可能出现：

```text
大厅
██████████████
       ||
       ||
       ||
██████████████
大厅
```

整个结构因为：

* 高度一样；
* 坡度一样；
* 法向一样；

被识别成同一个 Region。

但是从导航角度：

```text
大厅 → 窄通道 → 大厅
```

应该表示为：

```text
R1 → P1 → R2
```

因此需要增加导航拓扑分析。

---

## 7.1 拓扑瓶颈检测

计算 Region 内部局部通行宽度：

$$
W(x)
$$

如果出现：

$$
W(x)<W_{bottleneck}
$$

并且该区域连接两个明显较大的自由空间，则将其识别为拓扑瓶颈。

在瓶颈位置主动切分 Region。

---

## 7.2 Region 凸性检测

如果 Region 呈现明显：

```text
L 型
U 型
T 型
```

结构，则考虑继续分割。

避免一个 Region 内出现两个点属于同一区域，但无法直接连线的问题。

---

# 八、Region 合并与拓扑保持压缩

初始 Region Growing 可能产生：

```text
R0
R1
R2
...
R500
```

需要进一步进行压缩。

---

## 8.1 Region Merge Cost

对于相邻：

$$
R_i,R_j
$$

定义：

$$
C_{merge}
=
w_nC_n+
w_sC_s+
w_rC_r+
w_gC_g
$$

其中：

* \(C_n\)：法向差；
* \(C_s\)：坡度差；
* \(C_r\)：粗糙度差；
* \(C_g\)：几何拟合误差。

---

## 8.2 拓扑保持约束

不能仅根据几何相似度合并。

合并还需要满足：

```text
关键 Portal 不消失
关键狭窄通道不消失
跨层关系不发生变化
机器人可达关系不发生变化
```

即：

$$
Connectivity(G_{before})
=
Connectivity(G_{after})
$$

如果合并导致：

```text
R1 → P1 → R2
```

这样的关键结构消失，则禁止合并。

---

# 九、Portal 自动提取

## 9.1 公共 Mesh 边提取

对于相邻 Region：

$$
R_i,R_j
$$

寻找：

$$
T_a\in R_i
$$

和：

$$
T_b\in R_j
$$

满足：

$$
T_a\cap T_b=e
$$

将所有公共 Mesh Edge 收集：

$$
B_{ij}=
\{e_1,e_2,\dots,e_n\}
$$

---

## 9.2 Boundary Chain

将连续共享边连接为：

$$
e_1
\rightarrow
e_2
\rightarrow
...
\rightarrow
e_k
$$

得到 Portal Candidate：

$$
P_{ij}^{candidate}
$$

同一个 Region 对之间允许存在多个 Portal：

```text
       P1
R1 ───────── R2
       P2
R1 ───────── R2
```

不能简单合并成一个连接。

---

# 十、Robot-aware Portal Validation

候选 Portal 需要经过机器人通行能力验证。

---

## 10.1 Portal 宽度

Portal 有效宽度：

$$
W_p
$$

要求：

$$
W_p>
W_{robot}+2d_{safe}
$$

例如：

```text
机器人宽度 = 0.60 m
安全距离   = 0.15 m
```

则要求：

$$
W_p>0.90m
$$

---

## 10.2 坡度

要求：

$$
\theta_p<\theta_{max}
$$

---

## 10.3 台阶

要求：

$$
h_p<h_{max}
$$

---

## 10.4 Clearance

要求机器人 Footprint 通过 Portal 时：

```text
不会碰撞
+
满足安全距离
```

最终：

$$
Valid(P)=
C_{width}
\land
C_{slope}
\land
C_{step}
\land
C_{clearance}
$$

---

# 十一、Portal 机器人足迹腐蚀

对于原始 Portal：

```text
|------------------------------|
```

根据：

$$
\frac{W_{robot}}{2}+d_{safe}
$$

向左右两侧收缩。

形成：

```text
原始 Portal

|------------------------------|

机器人中心有效区域

       |----------------|
```

相当于：

$$
P_{valid}
=
P_{raw}\ominus Footprint
$$

从而保证机器人中心经过 Portal 时机器人本体不会发生碰撞。

---

# 十二、Region-Portal Graph

采用二部图结构：

$$
G_T=(R\cup P,E)
$$

而不是简单：

```text
Region → Region
```

推荐：

```text
Region → Portal → Region
```

例如：

```text
R0
 │
P0
 │
R1
 │
P1
 │
R2
 ├──── P3 ──── R4
 │
P2
 │
R3
```

---

## 12.1 Region 数据结构

```cpp
struct SurfaceRegion
{
    int id;

    std::vector<int> triangle_ids;

    Eigen::Vector3f centroid;

    float area;

    float min_z;
    float max_z;

    float mean_slope;
    float max_slope;

    float roughness;

    std::vector<int> portal_ids;
};
```

---

## 12.2 Portal 数据结构

```cpp
struct Portal
{
    int id;

    int region_a;
    int region_b;

    Eigen::Vector3f left;
    Eigen::Vector3f right;
    Eigen::Vector3f center;

    float width;

    float mean_slope;
    float max_step;
    float min_clearance;

    float traversal_cost;
};
```

---

# 十三、拓扑全局规划

## 13.1 起终点定位

将起点：

$$
S
$$

定位到：

$$
R_s
$$

终点：

$$
G
$$

定位到：

$$
R_g
$$

---

## 13.2 Topology A*

在 Region-Portal Graph 上进行 A* 搜索。

例如：

```text
Rs
 ↓
P1
 ↓
R3
 ↓
P4
 ↓
R6
 ↓
P8
 ↓
Rg
```

得到拓扑通路。

---

# 十四、Surface Corridor

根据拓扑路径：

$$
R_1\rightarrow
P_1\rightarrow
R_2\rightarrow
P_2\rightarrow
R_3
$$

提取：

$$
C=
R_1\cup R_2\cup R_3
$$

形成：

```text
Surface Corridor
```

第二阶段只允许在这些 Region 对应的 Mesh 上进行搜索。

例如：

```text
完整 Mesh：
300000 triangles

Topology Corridor：
8000 triangles
```

从而显著降低精细规划搜索规模。

---

# 十五、局部精细规划

在 Surface Corridor 内运行：

```text
A*
```

或：

```text
Dijkstra
```

搜索最终三维地表路径。

路径必须经过拓扑规划确定的 Portal 顺序。

---

# 十六、最终路径优化

对原始搜索路径进行：

* Shortcut；
* Funnel；
* 曲线平滑；
* 曲率约束；
* 安全距离约束。

最终输出：

```text
nav_msgs/Path
```

供机器人控制器使用。

---

# 十七、实验设计

需要实现三种方法。

## Method A

```text
Ground Point Cloud Graph
+
A*
```

---

## Method B

```text
Triangle Mesh Graph
+
A*
```

---

## Method C

```text
Region-Portal Topology
+
Topology A*
+
Local Surface A*
```

即本课题方法。

---

# 十八、实验场景

## Scene 1：单层平面

验证基本地图表达与规划效率。

---

## Scene 2：坡道环境

```text
平地
   \
    \ 坡道
     \
      高层平台
```

验证坡道连续性。

---

## Scene 3：上下层重叠

```text
二层
────────────

────────────
一层
```

验证算法不会产生错误跨层连接。

---

## Scene 4：多层 + 多坡道

验证复杂多层环境拓扑关系。

---

## Scene 5：狭窄通道

```text
大厅 → 门 → 走廊 → 门 → 大厅
```

验证 Bottleneck 和 Portal 提取。

---

## Scene 6：工业园区复杂环境

最终验证算法在大规模真实环境中的规划性能。

---

# 十九、主要评价指标

## 19.1 地图构建性能

记录：

```text
Build Time
Memory Usage
```

---

## 19.2 图规模

记录：

```text
Node Number
Edge Number
Region Number
Portal Number
```

---

## 19.3 规划速度

记录：

$$
T_{planning}
$$

以及：

$$
N_{expanded}
$$

---

## 19.4 路径长度

$$
L=
\sum_{i=0}^{N-1}
\|p_{i+1}-p_i\|_2
$$

---

## 19.5 安全距离

记录路径执行过程中的：

$$
C_{min}
$$

即机器人到障碍物的最小实际安全距离。

---

## 19.6 规划成功率

$$
R_{plan}
=
\frac{N_{success}}
{N_{tasks}}
$$

---

## 19.7 实际执行成功率

$$
R_{exec}
=
\frac{N_{arrived}}
{N_{attempted}}
$$

---

# 二十、消融实验

为了证明各模块的有效性，需要设计以下实验。

## Ablation 1

```text
无 Region 压缩
vs
Region 压缩
```

观察：

```text
节点数量
规划时间
路径长度
```

---

## Ablation 2

```text
普通 Region Growing
vs
Traversability Region Growing
```

验证机器人可通行约束的作用。

---

## Ablation 3

```text
无 Bottleneck Detection
vs
Bottleneck Detection
```

验证窄通道自动分区效果。

---

## Ablation 4

```text
普通 Region Merge
vs
Topology-preserving Merge
```

验证拓扑保持压缩的作用。

---

## Ablation 5

```text
直接 Mesh A*
vs
Topology + Local A*
```

验证分层规划对于搜索效率的提升。

---

# 二十一、计划实施步骤

## 第一阶段：基础地图处理

### 工作内容

* GroundMap.pcd 读取；
* 点云降采样；
* 法向计算；
* 坡度计算；
* 粗糙度计算；
* RViz 可视化。

### 阶段成果

实现：

```text
ground_map_processor
```

---

# 第二阶段：Mesh 地表构建

### 工作内容

* 地面三角化；
* Triangle adjacency；
* 避免上下层错误连接；
* 三角形属性计算；
* 可通行性分类。

### 阶段成果

实现：

```text
traversable_surface_mesh
```

RViz 显示：

```text
绿色：可通行
红色：不可通行
```

---

# 第三阶段：Region 生成

### 工作内容

实现：

```text
Traversability-Constrained Region Growing
```

加入：

* 法向；
* 坡度；
* 粗糙度；
* 高度连续性。

### 阶段成果

实现：

```text
surface_region_generator
```

RViz 中每个 Region 使用不同颜色。

---

# 第四阶段：Bottleneck 与 Region Refinement

### 工作内容

实现：

```text
Bottleneck Detection
Concavity Detection
Region Split
```

### 阶段成果

能够正确形成：

```text
大厅
 ↓
Portal
 ↓
走廊
```

---

# 第五阶段：Portal 提取

### 工作内容

实现：

```text
Shared Boundary Detection
Boundary Chain
Portal Candidate
Portal Validation
```

### 阶段成果

RViz：

```text
Region：彩色面
Portal：红色线段
```

---

# 第六阶段：拓扑保持 Region 压缩

### 工作内容

设计：

```text
Region Merge Cost
+
Topology Preservation Constraint
```

比较：

```text
压缩前 Region 数量
压缩后 Region 数量
```

同时验证：

```text
Connectivity 是否保持
```

---

# 第七阶段：拓扑规划器

实现：

```text
Topology A*
```

输入：

```text
Start
Goal
Region-Portal Graph
```

输出：

```text
Region / Portal Sequence
```

---

# 第八阶段：Surface Corridor

根据拓扑结果提取：

```text
Relevant Regions
```

建立：

```text
Local Surface Graph
```

---

# 第九阶段：精细全局路径规划

在局部 Surface Corridor 内运行：

```text
A*
```

输出：

```text
3D Global Path
```

---

# 第十阶段：路径优化

加入：

```text
Shortcut
Funnel
Curve Smoothing
```

并加入：

```text
Robot Motion Constraint
```

---

# 第十一阶段：ROS 2 集成

建议环境：

```text
Ubuntu
ROS 2 Jazzy
C++17
PCL
Eigen
RViz2
Gazebo
```

ROS 2 节点可以设计为：

```text
ground_map_processor
        ↓
surface_mesh_node
        ↓
region_graph_node
        ↓
topology_planner
        ↓
surface_refinement_planner
        ↓
nav_msgs/Path
```

---

# 第十二阶段：Gazebo 验证

将最终路径交给统一局部控制器。

验证：

* 平地；
* 坡道；
* 多层；
* 狭窄通道；
* 大规模工业园区。

记录机器人实际运动结果。

---

# 二十二、建议开发周期

## 第 1～2 周

完成：

```text
PCD读取
降采样
法向
坡度
粗糙度
```

---

## 第 3～4 周

完成：

```text
Ground Mesh
Triangle Adjacency
Traversability
```

---

## 第 5～7 周

完成：

```text
Region Growing
Region Visualization
```

---

## 第 8～10 周

完成：

```text
Bottleneck Detection
Region Split
Portal Extraction
```

---

## 第 11～13 周

完成：

```text
Topology-preserving Region Merge
Region-Portal Graph
```

这一阶段为主要算法研究阶段。

---

## 第 14～15 周

完成：

```text
Topology A*
Surface Corridor
```

---

## 第 16～17 周

完成：

```text
Local Surface A*
Path Reconstruction
```

---

## 第 18～19 周

完成：

```text
Path Smoothing
ROS2 Integration
```

---

## 第 20～22 周

进行：

```text
Gazebo Test
Benchmark
Ablation Study
```

---

## 第 23～24 周

完成：

```text
数据统计
绘图
论文实验章节
```

---

# 二十三、建议的软件工程目录

```text
multi_layer_topology_planner/
│
├── src/
│   ├── map/
│   │   ├── pointcloud_processor.cpp
│   │   ├── surface_mesh.cpp
│   │   └── traversability.cpp
│   │
│   ├── region/
│   │   ├── region_growing.cpp
│   │   ├── bottleneck_detector.cpp
│   │   ├── region_split.cpp
│   │   └── region_merge.cpp
│   │
│   ├── portal/
│   │   ├── portal_extractor.cpp
│   │   └── portal_validator.cpp
│   │
│   ├── graph/
│   │   └── topology_graph.cpp
│   │
│   ├── planner/
│   │   ├── topology_astar.cpp
│   │   ├── surface_astar.cpp
│   │   └── path_smoothing.cpp
│   │
│   └── ros/
│       ├── map_node.cpp
│       └── planner_node.cpp
│
├── include/
│
├── config/
│   ├── robot.yaml
│   └── planner.yaml
│
├── launch/
│
├── rviz/
│
├── maps/
│
├── benchmark/
│
└── README.md
```

---

# 二十四、拟解决的关键科学与技术问题

本课题重点解决三个问题。

### 问题一

**如何从复杂三维地面点云中自动建立能够正确表达多层、坡道和地表连续性的导航区域？**

对应：

```text
Traversability-aware Region Construction
```

---

### 问题二

**如何在压缩地面地图规模的同时保持机器人真实的拓扑可达关系？**

对应：

```text
Topology-preserving Region Compression
```

这是本课题最值得重点研究的问题之一。

---

### 问题三

**如何利用稀疏拓扑图指导密集地表路径搜索，在保持路径质量的同时降低全局规划计算量？**

对应：

```text
Topology-guided Hierarchical Planning
```

---

# 二十五、拟形成的创新点

## 创新点 1

提出一种**面向复杂多层地面环境的可通行地表区域自适应构建方法**。

区别于按照固定 Z 高度进行楼层划分，通过：

```text
地表连续性
+
几何特征
+
机器人可通行约束
```

自动形成多层 Region。

---

## 创新点 2

提出一种**拓扑保持的 Region-Portal 地图压缩方法**。

在降低地图节点规模的同时保持：

```text
坡道
狭窄通道
关键连接
多层可达关系
```

不被破坏。

---

## 创新点 3

提出一种**基于拓扑走廊约束的分层三维全局规划方法**。

通过：

```text
Topology A*
        ↓
Surface Corridor
        ↓
Local Surface A*
```

减少密集地表搜索范围，提高复杂大规模三维环境中的规划效率。

---

# 二十六、最终系统目标

最终系统输入：

```text
GroundMap.pcd

Start:
x y z yaw

Goal:
x y z yaw
```

输出：

```text
Region Map
Portal Map
Topology Graph
Global 3D Path
```

RViz 最终应能够同时显示：

```text
原始 GroundMap
        +
不同颜色 Region
        +
Portal
        +
Region-Portal Graph
        +
Topology Path
        +
Final 3D Path
        +
Start / Goal
```

最终形成完整流程：

```text
GroundMap.pcd
       ↓
Multi-level Traversable Surface
       ↓
Region-Portal Topological Map
       ↓
Topology-guided Hierarchical Planning
       ↓
Executable 3D Ground Path
```

课题最终验证的核心并不是简单证明“拓扑图比点云快”，而是验证：

> **在保持复杂多层环境中真实地面可达关系、坡道连接关系及机器人通行约束的情况下，通过导航区域抽象和拓扑保持压缩显著降低全局规划搜索空间，并利用分层规划提高复杂三维环境中的全局路径规划效率。**
