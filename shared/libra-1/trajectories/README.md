# LIBRA-I Trajectories

Loaded via "Trajectory" menu -> "Load trajectory...".

- Each file is a CSV with one pose per line.
- Columns are found by header name, in any order. All joint columns are required; an empty value in an optional column keeps the default.
- Lines starting with `#` are comments. The first non-comment line is the header.

## Columns

| Header | Unit | Required | Meaning |
| --- | --- | --- | --- |
| `roll_deg` | deg | Yes | 2-DoF joint roll (+ = left) |
| `pitch_deg` | deg | Yes | 2-DoF joint pitch (+ = up) |
| `j1_deg` | deg | Yes | J1, arm yaw |
| `j2_deg` | deg | Yes | J2, arm yaw |
| `j3_deg` | deg | Yes | J3, arm pitch |
| `hold_s` | s | No | Settling/Hold time after a pose is reached. |
| `cw` | – | No | Counterweight mode at step start: `auto` (Reactive Only mode), or `fill`, `drain`, or `idle` (Manual mode). |

> ***NOTE:*** A manual `fill` stops when a CW is estimated full, and a manual `drain` stops when one is estimated empty (the operator may resume it).

## Files

### Main

| File | Use |
| --- | --- |
| `deploy.csv` | Arm hanging straight down -> ready pose (0, 0, 0, 0, 170). |
| `retract.csv` | Ready pose -> arm hanging straight down (reverse of `deploy.csv`). |

### Experiment-specific

| File | Use |
| --- | --- |
| `sdc_24_poses.csv` | Trajectory used in the Slam Data Collection (SDC) runs, formally established in "SLAM Data Collection 2" (2026-06-08). |
