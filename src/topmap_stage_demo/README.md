# topmap_stage_demo

ROS 2 RViz demo for the synthetic H-ground / ramp / upper-platform scene. Two
views share the same `map` frame, original point cloud coordinates and verified
window/camera layout:

1. `stages.launch.py` — the existing four-stage overlay visualization.
2. `planning.launch.py` — **interactive planning by default**: set start/goal in
   RViz and the real `topmap::HierarchicalPlanner` replans.

The demo runs the production surface-constrained `topmap_ros planner_node`
against its own archive and remaps its
`PlanPath` service to `/topmap_stage_demo/plan_path`, so a real navigation stack
is untouched.

## Build

```bash
source /opt/ros/humble/setup.bash
cd /home/robot/topmap_ws
colcon build --packages-select topmap_stage_demo --symlink-install
source /home/robot/topmap_ws/install/setup.bash
```

Runtime dependencies include `rclpy`, `rviz2`, `tf2_ros`, `topmap_ros` and
`topmap_msgs`.

## Interactive planning (default one-command)

```bash
source /home/robot/topmap_ws/install/setup.bash
ros2 launch topmap_stage_demo planning.launch.py
```

在 RViz 中操作：

1. 点击顶部 **2D Pose Estimate**，在地面内部按住鼠标左键并拖动，松开后设置
   绿色 START 起点。
2. 点击 **2D Goal Pose**，在目标地面上按住左键并拖动，松开后设置红色 GOAL。
3. 修改任意一个端点都会重新规划；状态显示 **SUCCESS** 时，橙色粗线就是本次
   规划服务返回的路径。无效点击或规划失败会清除旧路径，并显示错误状态。

默认接近俯视，适合使用这两个二维工具选点。选好点后可以切回 Move Camera，
旋转视角截图。若已旋转到斜视角，建议恢复俯视后再选点：这两个工具在水平面上
取点，斜视时点击抬高的地面可能产生 XY 偏差。网格投影随后恢复实际地面高度，
因此在高处平台选点时无需手填 z=2。选点请避开地面边缘和没有地面的黑色缺口。

The tools publish `/topmap_stage_demo/initialpose` (`PoseWithCovarianceStamped`) and `/topmap_stage_demo/goal_pose`
(`PoseStamped`), respectively. Only the newest request may publish a result.

The default safe start `[2, 3.5, 0]` and goal `[28.4, 3.5, 2]` are planned once
after the service becomes ready, so the view is useful before the first click.
The bridge never blocks the UI waiting for the server; the status marker says
when the service is not ready.

### 2D click → real surface Z

RViz's planar tools publish `z = 0`. The bridge loads `data/stages.json` once and
resolves `(x, y)` against the **actual mesh triangles**:

- XY triangle containment, then barycentric Z interpolation;
- upper platform clicks become `z = 2.0`, ramp clicks use the actual ramp face;
- holes, outside-mesh points, non-finite input and non-`map` frames are rejected;
- no nearest-3D-point snapping and no hand-drawn/interpolated planning path;
- shared edges/overlapping surfaces use a deterministic highest-Z then
  lowest-triangle-id rule (the current scene has one surface layer).

The planner archive `data/stages.topmap.bin` is exported from the same mesh and
compressed topology as `stages.json`; `test/test_archive_consistency.py` checks
this. It is loaded once by `planner_node`, never rebuilt per click. Start yaw is
preserved as a finite normalized planar quaternion.

### Path and status topics

| topic `/topmap_stage_demo/...` | message | default | meaning |
| --- | --- | --- | --- |
| `source_cloud` | PointCloud2 | on | original synthetic cloud, untouched XYZ |
| `stage3_final_regions` | MarkerArray | on | final 5 regions |
| `stage4_portal_chords` | MarkerArray | on | red portal chords |
| `planning_start` | MarkerArray | on | START sphere/label + yaw arrow |
| `planning_goal` | MarkerArray | on | GOAL sphere/label |
| `final_path` | MarkerArray | on | **service-returned path**, orange line |
| `planned_path` | nav_msgs/Path | off | the same service path, no Z lift |
| `planned_topology_sequence` | MarkerArray | off | optional service topology chain |
| `planning_status` | MarkerArray | on | visible SUCCESS / PENDING / FAILED / INVALID text |

Marker paths get only a small rendering Z lift (`0.005 m`); the planning view's
region fill uses a `0.001 m` lift so it cannot hide the line. `nav_msgs/Path` uses
the exact service coordinates. Failed requests and invalid clicks publish a
`DELETEALL` marker plus an empty path, so an old successful path cannot remain
visible. The isolated backend service is `/topmap_stage_demo/plan_path`; its
publishers are remapped under `/topmap_stage_demo/planner/...`.

## Fixed replay (optional, independent)

```bash
ros2 launch topmap_stage_demo planning.launch.py interactive:=false
# or
ros2 launch topmap_stage_demo replay.launch.py
```

The fixed replay never runs together with the interactive bridge. It publishes
the checked-in `data/planning.json` exactly as before. `stages.launch.py`
remains unchanged.

## Headless and live interaction check

```bash
ros2 launch topmap_stage_demo planning.launch.py rviz:=false
# another terminal:
source /home/robot/topmap_ws/install/setup.bash
python3 src/topmap_stage_demo/tools/verify_interactive.py
```

The script sends the same messages as the RViz tools and checks the default
plan, an upper-platform goal, reversed direction, an outside click clearing the
old path, and a rapid goal change where the superseded result must not win. It
also compares the bridge's `planned_path` with the planner node's service-result
topic to prove the displayed path is not fabricated.

## Tests / offline check

```bash
source /home/robot/topmap_ws/install/setup.bash
cd /home/robot/topmap_ws/src/topmap_stage_demo
python3 -m pytest -q test
ros2 run topmap_stage_demo interactive_planner --check
ros2 run topmap_stage_demo planning_publisher --check
ros2 run topmap_stage_demo stage_publisher --check
```

Projection tests cover ground, upper platform, ramp, shared edge, holes, outside
and NaN. State tests cover stale accepted/failed results and failure clearing.
Archive tests cover the exact mesh, region triangle sets and portal endpoints.

此前的交互版本已通过实际 ROS 测试，验证高平台和坡道的高度恢复、反向规划、连续修改终点、缺口和
地图外点击的清除，以及无效点击取消待处理结果。实际 RViz 鼠标操作已验证两个
工具均能更新端点并触发规划；全局状态为 OK。
验证记录位于 `/home/robot/topmap_ws/log/interactive_planning_review_result.json`，
窗口截图位于 `/home/robot/topmap_ws/log/interactive_planning_gui_clicks.png`。
这些记录来自贴地约束修改之前；本次修改只做编译和离线验证，新的 RViz 视觉效果由用户自行测试。

## Regenerating data

`tools/build_data.py` runs the real pipeline and writes both JSON exports plus
the planner archive:

```bash
python3 tools/build_data.py --force \
  --archive-out data/stages.topmap.bin
```

### Offline planner timing

`tools/measure_planning.py` compiles `tools/measure_planning.cpp` with the same
flags/link line as the existing `test_hierarchical_planner` target, loads the
demo archive and config, re-derives edge traversability exactly like
`planner_node`, and times one warm-up plus five production
`HierarchicalPlanner::plan` calls. Archive/config loading and map setup are
outside every timer. The JSON contains path, `PlanStats`, and independent
`SafetyValidator` results. Default request: start `[2, 3.5, 0]`, goal
`[28.4, 3.5, 2]`.

```bash
# Link a saved baseline library when comparing before/after in one session.
python3 tools/measure_planning.py \
  --planner-lib /path/to/libtopmap_planner.before.a \
  --output ../../../log/surface_query_performance/before_timing.json
python3 tools/measure_planning.py \
  --output ../../../log/surface_query_performance/after_timing.json
```

### 沿网格表面的路径约束

生产规划器使用独立的 `plan.path_surface_tolerance`（默认 `0.005 m`）检查返回
路径到真实网格的距离。`max_project_radius` 仍用于起终点和候选点定位搜索，
其 `0.30 m` 半径不再作为最终路径允许悬空的误差。路径优化后会沿合法网格恢复
地面高度，并重新校验线段内部、通道顺序及通行条件。

对于平地、坡道和平台组成的地面，规划器会从实际网格的公共折边计算路径交点，
保留坡脚、坡顶等必要转折点，再简化各段中的冗余点；没有写死演示场景坐标。
候选路径仍接受细采样的地面覆盖、连通性和贴地检查；复杂或歧义情况保留原有
密集贴地流程作为后备。导出回归同时检查坡脚、坡顶顶点和完整线段的贴地误差。

坡脚、坡顶的俯仰变化与地面左右转向曲率分开处理。RViz 的 5 毫米显示抬升仅用于
避免地面遮挡，真实 `nav_msgs/Path` 不包含该偏移。四阶段叠加视图的显示参数保持
原设置。最终视觉效果由用户重启 launch 后自行检查。

Only the synthetic scene inside this package is used for the demonstration.
No private real point clouds, robot commands or commits.
