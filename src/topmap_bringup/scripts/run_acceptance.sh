#!/usr/bin/env bash
#
# T32/T33 acceptance: offline build -> online plan -> benchmark -> report, one
# command.
#
# This is the runnable T32/T33 script from the implementation plan. It:
#   1. builds the workspace up to the bringup package (the launch files live
#      there, so topmap_ros alone is not enough);
#   2. writes the synthetic Hall PCD with the installed topmap_surface tool;
#   3. launches the offline builder and calls /topmap/build_map;
#   4. asserts the archive is a non-empty file;
#   5. launches the online planner and calls /topmap/plan_path;
#   6. runs the full T33 benchmark, enforces the acceptance assertions and
#      regenerates benchmark/REPORT.md from the validated CSV.
#
# The script creates every directory it writes into and always tears the
# launched nodes down on exit.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${TOPMAP_WS:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
CONFIG_FILE="${SCRIPT_DIR}/../config/topmap.yaml"
SCENE_PCD="${WS_ROOT}/test_data/synthetic/scene_hall_narrow_hall.pcd"
MAP_BIN="${WS_ROOT}/test_data/maps/hall.bin"

# ROS setup scripts probe optional variables that may be unset. Keep nounset
# strictness for this script, but suspend it only while sourcing the vendor
# environment.
set +u
source /opt/ros/humble/setup.bash
set -u
cd "${WS_ROOT}"

builder_pid=""
planner_pid=""

cleanup() {
  local status=$?
  # Never leave a launched node behind, on success or failure.
  for pid in "${planner_pid}" "${builder_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  exit "${status}"
}
trap cleanup EXIT INT TERM

mkdir -p "${WS_ROOT}/test_data/synthetic" "${WS_ROOT}/test_data/maps"

colcon build --packages-up-to topmap_bringup --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
set +u
source install/setup.bash
set -u

ros2 run topmap_surface write_synthetic_pcd \
  --scene hall_narrow_hall --output "${SCENE_PCD}"

ros2 launch topmap_bringup topmap_offline_build.launch.py \
  pcd_path:="${SCENE_PCD}" \
  output_map_path:="${MAP_BIN}" \
  config_file:="${CONFIG_FILE}" &
builder_pid=$!

timeout 30 bash -c 'until ros2 service type /topmap/build_map >/dev/null 2>&1; do sleep 0.2; done'
builder_response="$(ros2 service call /topmap/build_map topmap_msgs/srv/BuildMap \
  "{pcd_path: '${SCENE_PCD}', output_map_path: '${MAP_BIN}'}")"
printf '%s\n' "${builder_response}"
[[ "${builder_response}" == *"success=True"* ]]
test -s "${MAP_BIN}"

ros2 launch topmap_bringup topmap_online_plan.launch.py \
  map_path:="${MAP_BIN}" \
  config_file:="${CONFIG_FILE}" &
planner_pid=$!

timeout 30 bash -c 'until ros2 service type /topmap/plan_path >/dev/null 2>&1; do sleep 0.2; done'
planner_response="$(ros2 service call /topmap/plan_path topmap_msgs/srv/PlanPath \
  "{start: {header: {frame_id: map}, pose: {position: {x: 3.5, y: 3.5, z: 0.05}}}, goal: {header: {frame_id: map}, pose: {position: {x: 13.5, y: 3.5, z: 0.05}}}}")"
printf '%s\n' "${planner_response}"
[[ "${planner_response}" == *"success=True"* ]]

# ---------------------------------------------------------------------------
# T33 acceptance: run the full benchmark, enforce the revised plan's assertions
# (R32/R33/R36/R37), then regenerate benchmark/REPORT.md from the validated CSV.
# The report generator replaces its marked section on every run, so re-running
# the script never appends duplicate reports.
# ---------------------------------------------------------------------------
BENCH_CSV="${WS_ROOT}/benchmark/results/benchmark.csv"
REPORT_MD="${WS_ROOT}/benchmark/REPORT.md"
mkdir -p "$(dirname "${BENCH_CSV}")"

# R37: eight scenes (the four originals plus the topology-preservation scenes
# single_bridge/parallel_portals/loop and t_shape); R32/R33 need the unreachable
# overlapping_floors scene and the multi-region scenes. `--ablation all` adds
# every ablation variant on top of the three base methods.
ros2 run topmap_planner benchmark_runner \
  --config "${CONFIG_FILE}" \
  --output "${BENCH_CSV}" \
  --scenes flat_plane,ramp,overlapping_floors,hall_narrow_hall,single_bridge,parallel_portals,loop,t_shape \
  --methods point_cloud,mesh,hierarchical --ablation all

python3 - "${BENCH_CSV}" "${REPORT_MD}" <<'PY'
#!/usr/bin/env python3
"""T33 benchmark acceptance checks and deterministic report generation.

Thresholds are the frozen values from the T33 implementation plan:
  A-01 invalid_cross_layer_edges == 0 for every record;
  A-02 reachability_preserved == 1 for base hierarchical records;
  A-03 portal_order_valid == 1 for base hierarchical records;
  A-04 min_clearance >= 0.45 m for successful reachable records;
  A-05 hierarchical expansions strictly below mesh when region_count >= 3 and
       not worse on single-region scenes;
  A-06 hierarchical/mesh path-length ratio <= 1.20;
  A-09 unreachable scenes must report kTopologyUnreachable (602).

Every failing row is recorded instead of stopping at the first assert, so the
report tells the whole truth when acceptance does not pass.
"""
import csv
import hashlib
import io
import os
import sys

BEGIN_MARKER = "<!-- BEGIN AUTO-GENERATED T33 REPORT -->"
END_MARKER = "<!-- END AUTO-GENERATED T33 REPORT -->"
UNREACHABLE = 602
MIN_CLEARANCE = 0.45
MAX_PATH_RATIO = 1.20
BENCHMARK_SCENES = [
    "flat_plane",
    "ramp",
    "overlapping_floors",
    "hall_narrow_hall",
    "single_bridge",
    "parallel_portals",
    "loop",
    "t_shape",
]


def row_tag(row):
    return "{}/{}".format(row["scene"], row["method"])


class Check:
    def __init__(self, name, threshold, detail):
        self.name = name
        self.threshold = threshold
        self.detail = detail
        self.failures = []

    @property
    def ok(self):
        return not self.failures

    def require(self, condition, message):
        if not condition:
            self.failures.append(message)


def write_marked_block(path, block):
    """Insert or replace the marked report block, never duplicating it."""
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as handle:
            old = handle.read()
    else:
        old = ""

    if BEGIN_MARKER in old and END_MARKER in old:
        pre = old[: old.index(BEGIN_MARKER)]
        post = old[old.index(END_MARKER) + len(END_MARKER):]
        result = pre
        if result:
            result = result.rstrip("\n") + "\n\n"
        result += block + "\n"
        if post.strip():
            result += "\n" + post.lstrip("\n")
    elif old.strip():
        result = old.rstrip("\n") + "\n\n" + block + "\n"
    else:
        result = block + "\n"

    with open(path, "w", encoding="utf-8") as handle:
        handle.write(result)


def cell(value):
    return "-" if value is None else str(value)


def main():
    csv_path = sys.argv[1]
    report_path = sys.argv[2]

    with open(csv_path, "rb") as handle:
        raw = handle.read()
    digest = hashlib.sha256(raw).hexdigest()
    rows = list(csv.DictReader(io.StringIO(raw.decode("utf-8"))))

    reachable = [r for r in rows if r["expected_unreachable"] != "1"]
    unreachable = [r for r in rows if r["expected_unreachable"] == "1"]
    hier = [r for r in reachable if r["method"] == "hierarchical"]
    full = [r for r in reachable if r["method"] == "mesh"]
    by_scene_full = {r["scene"]: r for r in full}

    checks = []

    check = Check("A-07/A-09 reachable scenes plan successfully", "success == 1",
                  "Every reachable record (expected_unreachable == 0) must find a path.")
    for r in reachable:
        check.require(r["success"] == "1",
                      "{}: success={} error_code={}".format(row_tag(r), r["success"], r["error_code"]))
    checks.append(check)

    check = Check("A-09 unreachable scene reports kTopologyUnreachable", "error_code == 602",
                  "overlapping_floors must be rejected with the planning-stage unreachable code.")
    check.require(bool(unreachable), "no unreachable scene exercised (overlapping_floors missing?)")
    for r in unreachable:
        check.require(int(r["error_code"]) == UNREACHABLE,
                      "{}: error_code={} expected={}".format(row_tag(r), r["error_code"], UNREACHABLE))
    checks.append(check)

    check = Check("A-01 invalid cross-layer edges", "== 0",
                  "No Portal may join Regions with disjoint height bands.")
    for r in rows:
        check.require(int(r["invalid_cross_layer_edges"]) == 0,
                      "{}: invalid_cross_layer_edges={}".format(row_tag(r), r["invalid_cross_layer_edges"]))
    checks.append(check)

    check = Check("method rows present", "hierarchical and mesh non-empty",
                  "A-02/A-03/A-05/A-06 compare the two base methods.")
    check.require(bool(hier), "no base hierarchical rows")
    check.require(bool(full), "no base mesh rows")
    checks.append(check)

    check = Check("A-02 reachability preserved by compression", "reachability_preserved == 1",
                  "Base hierarchical records must keep the pre/post-compression reachability relation.")
    for r in hier:
        check.require(r["reachability_preserved"] == "1",
                      "{}: reachability_preserved={}".format(row_tag(r), r["reachability_preserved"]))
    checks.append(check)

    check = Check("A-03 surface path obeys portal order", "portal_order_valid == 1",
                  "Base hierarchical paths must cross Portals in topology order.")
    for r in hier:
        check.require(r["portal_order_valid"] == "1",
                      "{}: portal_order_valid={}".format(row_tag(r), r["portal_order_valid"]))
    checks.append(check)

    check = Check("A-05 hierarchical expansion reduces vs mesh",
                  "region_count >= 3: strict <; else <=",
                  "Single-region scenes cannot improve (corridor == full graph).")
    for h in hier:
        f = by_scene_full.get(h["scene"])
        if f is None:
            continue
        hn, fn = int(h["expanded_nodes"]), int(f["expanded_nodes"])
        if int(h["region_count"]) >= 3:
            check.require(hn < fn, "{}: hierarchical={} mesh={} region_count={}".format(
                h["scene"], hn, fn, h["region_count"]))
        else:
            check.require(hn <= fn, "{}: hierarchical={} mesh={} region_count={}".format(
                h["scene"], hn, fn, h["region_count"]))
    checks.append(check)

    check = Check("A-04 minimum clearance", ">= 0.45 m",
                  "Successful reachable paths must keep required_clearance().")
    for r in reachable:
        if r["success"] == "1":
            check.require(float(r["min_clearance"]) >= MIN_CLEARANCE,
                          "{}: min_clearance={} < {}".format(row_tag(r), r["min_clearance"], MIN_CLEARANCE))
    checks.append(check)

    check = Check("A-06 path-length degradation (C/B)", "<= 1.20",
                  "Hierarchical path length may not exceed mesh by more than 20%.")
    for h in hier:
        f = by_scene_full.get(h["scene"])
        if f is None or float(f["path_length"]) <= 0.0:
            continue
        ratio = float(h["path_length"]) / float(f["path_length"])
        check.require(ratio <= MAX_PATH_RATIO,
                      "{}: ratio={:.4f} C={} B={}".format(h["scene"], ratio, h["path_length"], f["path_length"]))
    checks.append(check)

    total_failures = sum(len(c.failures) for c in checks)
    failing_checks = sum(1 for c in checks if c.failures)
    passed = total_failures == 0

    scene_rows = {}
    for r in rows:
        if "/" in r["method"]:
            continue
        scene_rows.setdefault(r["scene"], {})[r["method"]] = r
    discovered = list(scene_rows.keys())
    scene_order = [s for s in BENCHMARK_SCENES if s in discovered]
    scene_order += [s for s in discovered if s not in scene_order]

    method_counts = {}
    for r in rows:
        method_counts[r["method"]] = method_counts.get(r["method"], 0) + 1

    lines = []
    lines.append("# T33 Benchmark Acceptance Report")
    lines.append("")
    lines.append("_Auto-generated from the benchmark CSV by "
                 "`src/topmap_bringup/scripts/run_acceptance.sh`; the marked section is "
                 "replaced on every run._")
    lines.append("")
    lines.append("## Source")
    lines.append("")
    lines.append("- CSV: `benchmark/results/benchmark.csv`")
    lines.append("- Records: {} (+1 header)".format(len(rows)))
    lines.append("- CSV SHA-256: `{}`".format(digest))
    lines.append("- Scenes: {}".format(", ".join("`{}`".format(s) for s in scene_order)))
    lines.append("- Overall acceptance: **{}** ({} failing row assertions across {} of {} checks)".format(
        "PASSED" if passed else "FAILED", total_failures, failing_checks, len(checks)))
    lines.append("")
    lines.append("## Acceptance checks")
    lines.append("")
    lines.append("| Check | Threshold | Result | Failing |")
    lines.append("| --- | --- | --- | --- |")
    for check in checks:
        lines.append("| {} | {} | {} | {} |".format(
            check.name, check.threshold, "PASS" if check.ok else "FAIL", len(check.failures)))
    lines.append("")

    if total_failures:
        lines.append("## Failing rows")
        lines.append("")
        for check in checks:
            if not check.failures:
                continue
            lines.append("### {} ({})".format(check.name, check.threshold))
            lines.append("")
            for failure in check.failures:
                lines.append("- `{}`".format(failure))
            lines.append("")

    lines.append("## Per-scene summary (base methods)")
    lines.append("")
    lines.append("| Scene | Regions | Mesh exp | Hier exp | A-05 | Mesh len | Hier len | C/B | Min clear (base) |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for scene in scene_order:
        methods = scene_rows.get(scene, {})
        mesh = methods.get("mesh")
        hier_row = methods.get("hierarchical")
        region_count = None
        if hier_row is not None:
            region_count = int(hier_row["region_count"])
        elif mesh is not None:
            region_count = int(mesh["region_count"])

        mesh_exp = hier_exp = None
        if mesh is not None and hier_row is not None:
            mesh_exp = int(mesh["expanded_nodes"])
            hier_exp = int(hier_row["expanded_nodes"])
            if region_count is not None and region_count >= 3:
                a05 = "PASS" if hier_exp < mesh_exp else "FAIL"
            else:
                a05 = "PASS" if hier_exp <= mesh_exp else "FAIL"
        else:
            a05 = "n/a"

        ratio = None
        if mesh is not None and hier_row is not None and float(mesh["path_length"]) > 0.0:
            ratio = "{:.4f}".format(float(hier_row["path_length"]) / float(mesh["path_length"]))

        clears = [float(r["min_clearance"]) for r in methods.values()
                  if r["success"] == "1" and r["expected_unreachable"] != "1"]
        lines.append("| {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
            scene,
            cell(region_count),
            cell(mesh_exp),
            cell(hier_exp),
            a05,
            cell(None if mesh is None else mesh["path_length"]),
            cell(None if hier_row is None else hier_row["path_length"]),
            cell(ratio),
            cell(None if not clears else min(clears)),
        ))
    lines.append("")
    lines.append("## Method coverage")
    lines.append("")
    lines.append("| Method | Rows |")
    lines.append("| --- | --- |")
    for method in sorted(method_counts):
        lines.append("| `{}` | {} |".format(method, method_counts[method]))
    lines.append("")

    block = BEGIN_MARKER + "\n" + "\n".join(lines).strip("\n") + "\n" + END_MARKER
    write_marked_block(report_path, block)

    if passed:
        print("ACCEPTANCE OK")
    else:
        print("ACCEPTANCE FAILED: {} failing row assertions across {} of {} checks".format(
            total_failures, failing_checks, len(checks)))
        for check in checks:
            if check.failures:
                print("  - {}: {} failing row(s)".format(check.name, len(check.failures)))
                for failure in check.failures:
                    print("      {}".format(failure))
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
PY
