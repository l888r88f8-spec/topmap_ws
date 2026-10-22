# Surface-constrained planner correction

Goal: the planner's actual returned path must follow the traversable triangle
mesh, including the flat/ramp/upper-platform transitions. The RViz user will
perform the final visual test after the fix; run compilation and automated
checks here, without opening RViz or manipulating the desktop.

Known cause: the optimizer and final validator accept nearest-surface matching
within the 0.30 m endpoint projection radius. Sparse shortcut/funnel/smoothing
segments can therefore bridge a change of ground slope. The existing 0.03 m
Marker rendering lift is separate and must not conceal the algorithm error.

Required implementation:
- Add failing optimizer/planner/validator regressions for floating chords at
  ramp entry and exit, including reverse travel and negative cases over holes
  and between disconnected or overlapping surfaces.
- Keep endpoint localization radius independent from a strict path-to-surface
  tolerance. Constrain optimization to valid mesh corridors, retain/intersect
  slope-change edges, and validate complete output segments, not only vertices.
- Do not replace only the RViz coordinates or present a render-only projection
  as a planner result. Preserve clearance, step, traversability, portal order
  and connectivity checks. Do not permit nearest-surface snapping across holes.
- Account explicitly for the distinction between ground pitch changes and
  vehicle steering curvature. Strict surface following at a mesh crease must
  not be declared infeasible just because a 3D polyline has a pitch corner;
  do not globally disable robot curvature checks or increase physical limits.
- Rebuild the affected planner and ROS backend so the existing interactive
  planning.launch.py uses the corrected production planner. Update any fixed
  demonstration data/tests/docs whose old assertions require a floating path.
- Run targeted and package regression checks; report measured adherence of
  actual planner output on the synthetic H/ramp scene with marker lift excluded.

Allowed scope: relevant topmap_core/IO configuration, topmap_planner, necessary
topmap_ros integration, topmap_stage_demo data/tests/docs and build/install/log.
No region splitting changes, private real cloud input, robot commands, commits
or unrelated user-document changes.
