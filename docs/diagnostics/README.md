# Diagnostics: v1.0.1 navigation case study

The full report is [`REPORT.md`](REPORT.md). This folder holds the compact, versioned evidence. The raw
recordings stay outside version control in `diagnostics_20261125/`, which is git-ignored: an 86 MB rosbag,
launch log, CPU and Gazebo logs.

## Contents

| Path | What |
|---|---|
| `REPORT.md` | Findings, evidence labels, limitations, proposed (untested) improvements |
| `figures/` | Map vs true geometry, map free-space connectivity, localization error, planned vs executed trajectories, mission time series, timing/sensors, controller dynamics |
| `data/static/` | Map vs world comparison (`map_vs_world.txt`, `map_regional.txt`, `map_connectivity.txt`, `world_collision_inventory.txt`) |
| `data/run/report.txt` | All run metrics: timing, rates, CPU, localization, planning/control per leg, ArUco, stops |
| `data/run/rigid_alignment.txt` | Rigid map↔world fits and the station-miss decomposition |
| `data/run/ack_timeout_phase.txt` | Timeout-vs-`/clock` phase test (which clock the BT timeout uses) |
| `data/run/controller_dynamics_results.txt`, `controller_dynamics_windows.csv` | Acceleration, lags, feasibility, oscillation association |
| `data/run/events.txt`, `arrival_stop.txt`, `rotation_and_optimal.txt` | BT failure/recovery timeline, motion around arrivals, path-length and rotation optimality |
| `data/run/*.csv` | AMCL vs truth (per update), rigid-fit residuals, plan-to-plan changes, RTF |
| `data/run/meta.txt`, `mission.log`, `params/`, `qos_topic_info.txt`, `nodes.txt` | Run metadata, mission output, runtime parameter dumps, QoS snapshot |
| `scripts/` | Everything needed to regenerate the above |

## Reproducing the analysis

The analysis needs ROS 2 Humble (`rosbag2_py`), Python 3 with numpy, OpenCV, matplotlib and scipy, and
the raw run directory. No simulation is needed. From the workspace root:

```bash
source /opt/ros/humble/setup.bash
S=docs/diagnostics/scripts
R=diagnostics_20261125              # raw data: static/ (created below) and run/ (recording)

# 1. Map vs world (repository files only; no recording needed)
python3 $S/map_vs_world.py . $R/static          # ground-truth raster, overlay, station clearances
python3 $S/map_regional.py . $R/static          # pixel classes, regional offsets (identity frame)
python3 $S/map_connectivity.py . $R/static      # free-space connectivity to the image border

# 2. Recorded run
python3 $S/analyze_run.py $R/run .              # -> $R/run/analysis/report.txt + CSVs + figures
python3 $S/rigid_alignment.py . $R/static $R/run
python3 $S/events.py $R/run > $R/run/analysis/events.txt
python3 $S/rotation_and_optimal.py $R/run
python3 $S/arrival_stop.py $R/run
python3 $S/ack_timeout_phase.py $R/run
python3 $S/controller_dynamics.py $R/run        # -> $R/run/analysis/controller_dynamics/
python3 $S/figures.py $R/run                    # localization_error.png, timing_sensors.png
```

Step 1 only needs the repository. Step 2 needs the raw recording, which is not versioned. Without it,
use the outputs in `data/`.

`analyze_run.py` must run before `rigid_alignment.py` and `figures.py`, because they read its CSVs.
`rigid_alignment.py` must run before `figures.py`.

## Recording a new run (not done again for this study)

`scripts/run_instrumented_mission.sh` performs the one instrumented mission used here. It uses the same
launch file and mission command as `scripts/run_mission_trial.sh`, plus passive recorders, and writes only
to `diagnostics_20261125/run/`. Before reusing it, read the limitations in `REPORT.md` §9:

- Its `gz model -p` polling probably triggered the Gazebo crash.
- It does not record hidden action-status topics; add `--include-hidden-topics`.
