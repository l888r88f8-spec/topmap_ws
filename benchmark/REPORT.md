<!-- BEGIN AUTO-GENERATED T33 REPORT -->
# T33 Benchmark Acceptance Report

_Auto-generated from the benchmark CSV by `src/topmap_bringup/scripts/run_acceptance.sh`; the marked section is replaced on every run._

## Source

- CSV: `benchmark/results/benchmark.csv`
- Records: 136 (+1 header)
- CSV SHA-256: `1c3c83fe8d8fb4d1c7085e97ded675888c79efbf28bc9b8efad2c5f9df6882cd`
- Scenes: `flat_plane`, `ramp`, `overlapping_floors`, `hall_narrow_hall`, `single_bridge`, `parallel_portals`, `loop`, `t_shape`
- Overall acceptance: **PASSED** (0 failing row assertions across 0 of 9 checks)

## Acceptance checks

| Check | Threshold | Result | Failing |
| --- | --- | --- | --- |
| A-07/A-09 reachable scenes plan successfully | success == 1 | PASS | 0 |
| A-09 unreachable scene reports kTopologyUnreachable | error_code == 602 | PASS | 0 |
| A-01 invalid cross-layer edges | == 0 | PASS | 0 |
| method rows present | hierarchical and mesh non-empty | PASS | 0 |
| A-02 reachability preserved by compression | reachability_preserved == 1 | PASS | 0 |
| A-03 surface path obeys portal order | portal_order_valid == 1 | PASS | 0 |
| A-05 hierarchical expansion reduces vs mesh | region_count >= 3: strict <; else <= | PASS | 0 |
| A-04 minimum clearance | >= 0.45 m | PASS | 0 |
| A-06 path-length degradation (C/B) | <= 1.20 | PASS | 0 |

## Per-scene summary (base methods)

| Scene | Regions | Mesh exp | Hier exp | A-05 | Mesh len | Hier len | C/B | Min clear (base) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| flat_plane | 1 | 2555 | 2555 | PASS | 7.07106781 | 7.07106781 | 1.0000 | 0.520037651 |
| ramp | 3 | 3140 | 3136 | PASS | 13.5775051 | 13.5775051 | 1.0000 | 0.64604646 |
| overlapping_floors | 2 | 3528 | 0 | PASS | 0 | 0 | - | - |
| hall_narrow_hall | 3 | 8722 | 6671 | PASS | 10 | 10 | 1.0000 | 0.641710639 |
| single_bridge | 3 | 4576 | 2866 | PASS | 8.12071228 | 8.12071228 | 1.0000 | 0.520035267 |
| parallel_portals | 4 | 8334 | 4997 | PASS | 9.94436073 | 9.92575455 | 0.9981 | 0.480546236 |
| loop | 8 | 8317 | 5255 | PASS | 14.7756233 | 14.8158932 | 1.0027 | 0.520035267 |
| t_shape | 4 | 5203 | 2850 | PASS | 8.0535059 | 8.0535059 | 1.0000 | 0.480546892 |

## Method coverage

| Method | Rows |
| --- | --- |
| `hierarchical` | 8 |
| `hierarchical/b_vs_c` | 8 |
| `hierarchical/no_bottleneck_split` | 8 |
| `hierarchical/no_compression` | 8 |
| `hierarchical/no_topology_preservation` | 8 |
| `hierarchical/pure_geometric` | 8 |
| `mesh` | 8 |
| `mesh/b_vs_c` | 8 |
| `mesh/no_bottleneck_split` | 8 |
| `mesh/no_compression` | 8 |
| `mesh/no_topology_preservation` | 8 |
| `mesh/pure_geometric` | 8 |
| `point_cloud` | 8 |
| `point_cloud/no_bottleneck_split` | 8 |
| `point_cloud/no_compression` | 8 |
| `point_cloud/no_topology_preservation` | 8 |
| `point_cloud/pure_geometric` | 8 |
<!-- END AUTO-GENERATED T33 REPORT -->
