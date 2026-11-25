# Legacy trial runners (Trials 02–10)

These scripts produced the historical logs in `audit_logs/` before
`scripts/run_mission_trial.sh` existed. They are kept unchanged so those trials
remain reproducible and auditable; each pins the commit it was written for.

| Script | Trials | Notes |
|---|---|---|
| `run_trial.sh` | 02–04 | Repeatability baseline at `3674bf1`; protocol in `docs/trial_baseline_protocol_v1.md` |
| `run_gui_off_trial.sh` | 05–06 | Headless (gzclient off) experiment |
| `run_validation_trial.sh` | 08 | Reliability-hardening validation |
| `run_tuning_trial.sh` | 10 | Nav2 tuning validation |
| `summarize_trials.sh` | — | Old side-by-side summary; superseded by `scripts/trial_metrics.py` |

They expect the simulation to be launched separately in another terminal.
Use `scripts/run_mission_trial.sh` for new trials.
