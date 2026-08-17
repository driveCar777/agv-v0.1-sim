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
