# RViz interactive planning implementation plan

Goal: use 2D Pose Estimate and 2D Goal Pose to change endpoints and recompute a
real HierarchicalPlanner path on the existing synthetic H-ground/ramp scene.

Architecture: retain the synthetic stage visualization, load the same mesh and
topology in the existing ROS planner service, and add a small interactive client.
Resolve XY clicks to a traversable mesh triangle using XY barycentric containment
and interpolate height. Do not treat RViz's default z=0 as the high platform's
height. Reject unsupported frames, holes, outside points and non-finite input.
Keep original planner output separate from any ground-projection reference.

Scope: src/topmap_stage_demo, plus necessary build/install/log artifacts.
Preserve stages.launch.py and the production region/planning algorithms. No
robot commands, no private real point clouds, no commits.

Steps:
- Inspect the existing ROS planner loader/service and map archive API; reuse
  them rather than create another planning algorithm or repeatedly triangulate.
- Add a real exported map archive matching stages.json and a minimal client
  subscribing to /topmap_stage_demo/initialpose and /topmap_stage_demo/goal_pose. Use isolated demo service names.
- Test XY-to-surface projection, including the upper platform and ramp, before
  implementation. Test changed endpoints and failed/stale result handling.
- Make planning.launch.py interactive by default, preserving a replay option.
  Add RViz start/goal toolbar tools, retaining the verified map reference frame
  and complete-scene camera/window layout.
- Build relevant packages, run targeted tests, and independently exercise actual
  ROS input topics and service responses with multiple endpoint selections,
  invalid clicks, reversed directions, and a goal changed during planning.
- Open actual RViz to verify the tools, endpoint markers and updated path.
  Document one-command startup and click/drag interaction.

Validation is complete only when published path endpoints match the newest
valid clicks (including interpolated Z), failures do not leave a stale success
path, and the unchanged stage visualization still works.
