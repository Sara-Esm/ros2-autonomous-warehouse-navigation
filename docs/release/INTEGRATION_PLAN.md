# v1.1 integration plan (stacked pull requests)

Generated from git on the development machine; `main` is at `01ce6a5` (v1.0.1). Nothing has been merged.

## Rules

- **Merge in the order below, one PR at a time, with a merge commit** (no squash or rebase: the stack shares history,
  and squashing would make every later PR show already-merged commits). No force-push, no history rewrite.
- **CI runs only for pull requests whose base is `main`** (`.github/workflows/warehouse-humble.yml`: `pull_request` to
  `main`, plus manual `workflow_dispatch`). So each PR gets its checks when it is retargeted to `main`.
- **Required checks per PR:** `warehouse-tests` and `gazebo-smoke`, both green on the PR's latest head commit.
  `main` currently requires **no** status checks (branch protection reports `required_status_checks` enforcement
  off), so GitHub would allow merging with failing or pending checks. Recommended before starting: in
  Settings → Branches, require `warehouse-tests` and `gazebo-smoke` for `main`.
- **Retarget after each merge:** open the next PR, click *Edit* next to its title, and change the base to `main`.
  Its diff then shrinks to that phase's own commits and CI starts. (GitHub retargets automatically only when the
  merged branch is deleted; branches are not deleted without approval.)
- Phase 2a (`feat/map-unknown-space`) changes the workflow itself (installs `nav2_map_server`); Phase 6 adds
  `ros-humble-vision-msgs`. A `pull_request` run uses the workflow from the PR's merge ref, so each of those PRs is
  tested with its own workflow change.

## Order

| # | Head | Initial base | Retarget to `main` after | Head commit | Commits | Open / review |
|---|---|---|---|---|---|---|
| 0 | `docs/diagnostic-investigation` | `main` | — | `6dad6f5` | 1 | PR #7 (open) |
| 1 | `feat/station-frame-correction` | `docs/diagnostic-investigation` | #0 merged | `69befa2` | 1 | [open](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/compare/docs/diagnostic-investigation...feat/station-frame-correction?expand=1) |
| 2 | `feat/map-unknown-space` | `feat/station-frame-correction` | #1 merged | `8b048f6` | 2 | [open](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/compare/feat/station-frame-correction...feat/map-unknown-space?expand=1) |
| 3 | `feat/slam-remap` | `feat/map-unknown-space` | #2 merged | `c638b55` | 1 | [open](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/compare/feat/map-unknown-space...feat/slam-remap?expand=1) |
| 4 | `feat/timing-sync` | `feat/slam-remap` | #3 merged | `3817e8b` | 1 | [open](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/compare/feat/slam-remap...feat/timing-sync?expand=1) |
| 5 | `feat/perception-performance` | `feat/timing-sync` | #4 merged | `1b1f48d` | 1 | [open](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/compare/feat/timing-sync...feat/perception-performance?expand=1) |
| 6 | `feat/headless-operation` | `feat/perception-performance` | #5 merged | `6730dc7` | 1 | [open](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/compare/feat/perception-performance...feat/headless-operation?expand=1) |
| 7 | `feat/motion-control` | `feat/headless-operation` | #6 merged | `fe93537` | 1 | [open](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/compare/feat/headless-operation...feat/motion-control?expand=1) |
| 8 | `feat/mission-verification` | `feat/motion-control` | #7 merged | `4cba097` | 1 | [open](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/compare/feat/motion-control...feat/mission-verification?expand=1) |
| 9 | `feat/integrated-validation` | `feat/mission-verification` | #8 merged | branch tip (the Phase 7 commit; `git log origin/feat/mission-verification..origin/feat/integrated-validation`) | 1 | [open](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/compare/feat/mission-verification...feat/integrated-validation?expand=1) |

## Expected commits per PR

### `docs/diagnostic-investigation`: Diagnostic investigation of v1.0.1

- `6dad6f5` docs: add diagnostic investigation of v1.0.1 navigation behaviour

### `feat/station-frame-correction`: Phase 1: station frame correction

- `69befa2` fix: convert world-frame station poses to map-frame Nav2 goals

### `feat/map-unknown-space`: Phase 2a: unknown-space map classification (+ CI map_server)

- `3dc18e7` fix: load never-observed map cells as unknown (Phase 2a)
- `8b048f6` ci: install nav2_map_server so the map-classification test runs

### `feat/slam-remap`: Phase 2b: SLAM re-map and surveyed goals

- `c638b55` feat: re-map the warehouse and survey station goals in the map frame (Phase 2b)

### `feat/timing-sync`: Phase 3: timing and synchronization

- `3817e8b` fix: acknowledgement timeout, node clocks and keep-static global clears (Phase 3)

### `feat/perception-performance`: Phase 4: pinhole camera model

- `1b1f48d` feat: pinhole camera model and optional headless Gazebo (Phase 4)

### `feat/headless-operation`: Phase 4b: headless launch options

- `6730dc7` feat: independent headless launch options (Phase 4b)

### `feat/motion-control`: Phase 5: motion control

- `fe93537` feat: path-alignment distance and yaw tolerance for smoother motion (Phase 5)

### `feat/mission-verification`: Phase 6: mission verification and shutdown

- `4cba097` feat: settled-state and fresh-evidence mission verification, signal and shutdown fixes (Phase 6)

### `feat/integrated-validation`: Phase 7: integrated validation and v1.1 release preparation

- one commit: "docs: integrated validation, portfolio README and v1.1 release preparation (Phase 7)" (documentation and evidence only; no code or parameter changes)

## After the last merge

- `main` then contains v1.1. Tag and release only after approval (`docs/release/RELEASE_NOTES_v1.1.md`).
- Verify on `main`: CI green on the final merge, and the local build and test commands in the README.
