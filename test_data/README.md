# test_data

Generated inputs for the T32/T33 acceptance flow. Everything here is produced
deterministically from code in this workspace; nothing is downloaded and nothing
needs to be hand-edited.

## Layout

| Path | Produced by | Tracked? |
| --- | --- | --- |
| `synthetic/*.pcd` | `ros2 run topmap_surface write_synthetic_pcd` | No (regenerated) |
| `maps/*.bin` | `/topmap/build_map` (offline builder node) | No (regenerated) |
| `README.md` | this file | Yes |

`run_acceptance.sh` creates both directories on demand.

## Synthetic scenes

`topmap_surface` implements ten scenes (`synthetic_scene.hpp`): `flat_plane`,
`ramp`, `overlapping_floors`, `multi_level_ramps`, `hall_narrow_hall`,
`u_shape`, `t_shape`, `single_bridge`, `parallel_portals`, `loop`. The geometry
is fixed by `SyntheticSceneOptions` (`grid_step = 0.10 m`,
`level_height = 3.00 m`, `hall_size = 6.00 m`, `corridor_width = 1.20 m`), so a
given scene always produces the same point set. The T33 benchmark uses eight of
them: `flat_plane`, `ramp`, `overlapping_floors`, `hall_narrow_hall`,
`single_bridge`, `parallel_portals`, `loop`, `t_shape`.

## Regenerate the synthetic PCD

From a sourced workspace (`source install/setup.bash`):

```bash
mkdir -p test_data/synthetic
ros2 run topmap_surface write_synthetic_pcd \
  --scene hall_narrow_hall \
  --output test_data/synthetic/scene_hall_narrow_hall.pcd
```

`--scene` accepts any of the ten names above; `--output` writes an ASCII PCD.

## Regenerate the map archive

The offline builder compresses a PCD into the versioned `topmap_map.bin`:

```bash
ros2 launch topmap_bringup topmap_offline_build.launch.py \
  pcd_path:=test_data/synthetic/scene_hall_narrow_hall.pcd \
  output_map_path:=test_data/maps/hall.bin \
  config_file:=src/topmap_bringup/config/topmap.yaml

ros2 service call /topmap/build_map topmap_msgs/srv/BuildMap \
  "{pcd_path: 'test_data/synthetic/scene_hall_narrow_hall.pcd', output_map_path: 'test_data/maps/hall.bin'}"
```

The online planner consumes the archive:

```bash
ros2 launch topmap_bringup topmap_online_plan.launch.py \
  map_path:=test_data/maps/hall.bin \
  config_file:=src/topmap_bringup/config/topmap.yaml
```

## One-shot reproduction

```bash
bash src/topmap_bringup/scripts/run_acceptance.sh
```

performs the build, PCD generation, offline/online ROS checks, the full
eight-scene benchmark and the `benchmark/REPORT.md` regeneration.

## Git hygiene

- `test_data/maps/*.bin` is ignored (`.gitignore`): archives are large binaries
  that depend only on the checked-in generator and config.
- `test_data/synthetic/*.pcd` is ignored: the PCD regenerates from
  `write_synthetic_pcd` in seconds.
- `benchmark/results/*.csv` is ignored: benchmark runs overwrite it.
- Never commit a generated artifact just to make a test pass. Regenerate it with
  the commands above and commit the config/code that produced it instead.
