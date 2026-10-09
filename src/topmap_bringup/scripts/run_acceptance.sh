#!/usr/bin/env bash
#
# T32 acceptance skeleton: offline build -> online plan, one command.
#
# This is the runnable T32 skeleton from the implementation plan. It:
#   1. builds the workspace up to the bringup package (the launch files live
#      there, so topmap_ros alone is not enough);
#   2. writes the synthetic Hall PCD with the installed topmap_surface tool;
#   3. launches the offline builder and calls /topmap/build_map;
#   4. asserts the archive is a non-empty file;
#   5. launches the online planner and calls /topmap/plan_path.
#
# T33 will append the benchmark/acceptance assertions and report generation; they
# are intentionally not part of this task. The script creates every directory it
# writes into and always tears the launched nodes down on exit.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${TOPMAP_WS:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
CONFIG_FILE="${SCRIPT_DIR}/../config/topmap.yaml"
SCENE_PCD="${WS_ROOT}/test_data/synthetic/scene_hall_narrow_hall.pcd"
MAP_BIN="${WS_ROOT}/test_data/maps/hall.bin"

source /opt/ros/humble/setup.bash
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
source install/setup.bash

ros2 run topmap_surface write_synthetic_pcd \
  --scene hall_narrow_hall --output "${SCENE_PCD}"

ros2 launch topmap_bringup topmap_offline_build.launch.py \
  pcd_path:="${SCENE_PCD}" \
  output_map_path:="${MAP_BIN}" \
  config_file:="${CONFIG_FILE}" &
builder_pid=$!

timeout 30 bash -c 'until ros2 service type /topmap/build_map >/dev/null 2>&1; do sleep 0.2; done'
ros2 service call /topmap/build_map topmap_msgs/srv/BuildMap \
  "{pcd_path: '${SCENE_PCD}', output_map_path: '${MAP_BIN}'}"
test -s "${MAP_BIN}"

ros2 launch topmap_bringup topmap_online_plan.launch.py \
  map_path:="${MAP_BIN}" \
  config_file:="${CONFIG_FILE}" &
planner_pid=$!

timeout 30 bash -c 'until ros2 service type /topmap/plan_path >/dev/null 2>&1; do sleep 0.2; done'
ros2 service call /topmap/plan_path topmap_msgs/srv/PlanPath \
  "{start: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: 7.0, y: 0.0, z: 0.0}}}}"
