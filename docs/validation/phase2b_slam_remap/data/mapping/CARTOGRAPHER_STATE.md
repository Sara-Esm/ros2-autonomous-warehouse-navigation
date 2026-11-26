# Cartographer state for `warehouse_map_v2`

| File | SHA-256 | Size |
|---|---|---|
| `warehouse_map_v2.pbstream` | `efef8cff19de33bc7ad135eccb40ef63b7ffe2176e64c4f75d89f7f2a880cbdd` | 7945410 bytes |

## How it was generated

This is mapping attempt 2 of Phase 2b.

1. `scripts/run_mapping_session.sh` started Gazebo with the robot.
2. It started the stock TurtleBot3 Cartographer (`turtlebot3_lds_2d.lua`, unmodified, `use_sim_time`).
3. `scripts/coverage_drive.py` drove the route in `scripts/coverage_route.py`: 130.9 m with 10 spins,
   collision-aware.
4. Afterwards the session called `/finish_trajectory` (trajectory 0) and `/write_state`
   (`include_unfinished_submaps: true`).
5. `map_saver_cli` (`save_map_timeout:=30.0`, default trinary thresholds 0.65/0.25) exported
   `warehouse_map_v2.pgm/.yaml` from the **live** `/map` in the same session.

## Caveat: reloading does not reproduce the exported map

Loading this file into `cartographer_node -load_state_filename ... -load_frozen_state true` with
`cartographer_occupancy_grid_node` (see `scripts/capture_raw_grid.sh`) renders a **different** grid:

- 284×593 cells instead of 258×428;
- about 13,000 observed cells outside the building walls;
- only 31.8% occupied-cell agreement with the exported map (`origin_recovery_attempt2.txt`).

The cause was not determined. The live-exported `warehouse_map_v2.pgm` is the authoritative map. This state
file is preserved for provenance and future analysis only. Do not regenerate the map from it without first
resolving this discrepancy.

The attempt-1 state (4.8 MB, kept locally in the git-ignored `diagnostics_phase2b/mapping_attempt1/`) did
reload consistently (96% geometric agreement), but its map was superseded by attempt 2.
