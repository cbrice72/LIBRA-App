# LIBRA-II Trajectories

Loaded via "Trajectory" menu -> "Load trajectory...".

- Each file is a CSV with one pose per line.
- All columns following the "last" joint are optional. An empty value keeps the default.
- Lines starting with `#` are comments. The first non-comment line is the header.

## Columns

| Header | Unit | Required | Meaning |
| --- | --- | --- | --- |
| `pitch_deg` | deg | Yes | Pitch joint (+ = up) |
| `yaw_deg` | deg | Yes | Yaw joint (+ = left) |
| `hold_s` | s | No | Settling/Hold time after a pose is reached. |
| `cw` | – | No | Counterweight mode at step start: `auto` (Reactive Only mode), or `fill`, `drain`, or `idle` (Manual mode). |

> ***NOTE:*** A manual `fill` stops when the CW is estimated full, and a manual `drain` stops when it is estimated empty (the operator may resume it).

## Files

None yet.
