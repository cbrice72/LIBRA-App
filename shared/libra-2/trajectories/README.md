# LIBRA-II Trajectories

Loaded via "Trajectory" menu -> "Load trajectory...".

- Each file is a CSV with one pose per line.
- Columns are found by header name, in any order. All joint columns are required; an empty value in an optional column keeps the default.
- Lines starting with `#` are comments. The first non-comment line is the header.

## Columns

| Header | Unit | Required | Meaning |
| --- | --- | --- | --- |
| `yaw_deg` | deg | Yes | Yaw joint (+ = left) |
| `pitch_deg` | deg | Yes | Pitch joint (+ = up) |
| `hold_s` | s | No | Settling/Hold time after a pose is reached. |
| `cw` | – | No | Counterweight mode at step start: `auto` (Reactive Only mode), or `fill`, `drain`, or `idle` (Manual mode). |

> ***NOTE:*** A manual `fill` stops when the CW is estimated full, and a manual `drain` stops when it is estimated empty (the operator may resume it).

## Files

None yet.
