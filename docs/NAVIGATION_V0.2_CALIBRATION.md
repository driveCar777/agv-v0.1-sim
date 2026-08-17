# Navigation V0.2 Calibration Items

## Status
`CALIBRATION_REQUIRED`

This document records physical values that must be measured on the real AGV.
V0.2 M1 keeps these items explicit instead of inventing numbers.

## Vehicle

1. `base_link_offset_x_m`
2. `base_link_offset_y_m`
3. `front_overhang_m`
4. `rear_overhang_m`
5. `track_width_m`
6. `wheelbase_m`

## Limits

1. `max_vx_mps`
2. `max_accel_mps2`
3. `max_decel_mps2`
4. `max_omega_rad_s`
5. `max_alpha_rad_s2`
6. minimum turning radius over speed

## Latency / Braking

1. perception latency
2. controller latency
3. command transport latency
4. actuator response latency
5. stopping distance at `0.10 / 0.20 / 0.30 m/s`

## LiDAR Extrinsics

For each LiDAR:

1. `frame_id`
2. `x_m`
3. `y_m`
4. `yaw_deg`

## Field Procedure

1. Park AGV against measured reference lines.
2. Measure body rectangle and base link reference offsets.
3. Mark LiDAR optical center and measure offsets from base link.
4. Command constant forward speed and constant stop from multiple initial speeds.
5. Command constant yaw turn and derive `max_omega_rad_s`.
6. Repeat under loaded and unloaded conditions if payload materially changes dynamics.

## M3 Braking Calibration Procedure

Do **not** use a single trial. Record at least 3 runs per speed per payload class.

### Speeds (if field safety allows)

| Trial | Target speed (m/s) | Payload |
|-------|-------------------|---------|
| B1 | 0.10 | empty |
| B2 | 0.15 | empty |
| B3 | 0.20 | empty |
| B4 | 0.25 | empty |
| B5 | 0.30 | empty |
| B6+ | repeat loaded if applicable | loaded |

### Record per trial

```text
initial_speed_mps
command_stop_timestamp
actual_motion_stop_timestamp
distance_travelled_after_stop_command_m
surface
payload
battery_pct
```

### Compute (post-process)

```text
effective_decel = v0^2 / (2 * stop_distance)
reaction_latency = time(vx drop) - command_stop_timestamp
required_stop_distance = v0 * latency + v0^2 / (2 * decel) + margin
```

### Tooling

```bash
python scripts/_record_braking_calibration.py --speed 0.20 --payload empty
```

Output: `logs/braking_calibration/braking_*.jsonl`

Until measured values are approved, keep:

```yaml
max_decel_mps2: null
calibration_status: CALIBRATION_REQUIRED
```
