# AjouNice MORAI GNSS integration changes

This file documents the local changes made on 2026-07-16 to the forked
`fast_lio_multi_sensor_fusion` submodule. These changes are intentionally kept
inside the submodule so that its commit can be reviewed, committed, and pushed
separately from the parent repository.

## Purpose

Use MORAI LiDAR, IMU, and GPS without wheel odometry, while preserving the
project interface:

- `/localization/odom` (`camera_init -> body`)
- TF `camera_init -> body`
- body-frame linear and angular twist
- GNSS correction aligned to the project's `morai_map` start pose

The current implementation is experimental. The 2026-07-16 `raw1.bag` test
showed strong global-position correction, but its velocity and yaw estimates
are not yet good enough to replace the existing FAST-LIO provider without
additional tuning and regression tests.

## Modified source files

### `src/GNSS_Processing.hpp`

- Added optional GeographicLib UTM projection.
- Kept the upstream `LocalCartesian` projection as the default for backward
  compatibility.
- MORAI mode uses UTM easting/northing deltas because MORAI's map coordinates
  follow grid north. Local true-north ENU introduced about 8 m of lateral
  disagreement over the recorded route because of meridian convergence.

### `src/IMU_Processing.hpp`

- Added `get_angular_velocity()` so odometry can publish the bias-corrected
  angular velocity already maintained by the IMU process.
- Added `set_gnss_heading_initialization()`.
- Added `set_defer_gnss_update()` so the MORAI external-alignment mode can
  consume GNSS after the LiDAR update. The original in-IMU-loop GNSS path is
  unchanged when this option is disabled.
- Fixed the legacy timestamp split logic so a GNSS sample whose timestamp is
  exactly equal to an IMU timestamp is not skipped.

### `src/laserMapping.cpp`

- Added configurable input/output topics and frame names.
- Added a `/localization/initialpose` subscriber for one-time external start
  alignment.
- Added GNSS validity, covariance, timestamp, and initialization checks.
- Added UTM-based MORAI coordinate handling and a fixed IMU-to-GNSS lever arm.
- Disabled the upstream 5 m displacement heading reset in external-alignment
  mode. The TF tree must not rotate in the middle of a run.
- Deferred external GNSS updates until after the LiDAR measurement update and
  before inserting the corrected scan into the incremental map.
- Added `position_only_update`. In this mode, a single GNSS antenna corrects
  only position using a 3D Kalman gain. FAST-LIO attitude, velocity, and IMU
  biases are not directly overwritten by the GNSS measurement.
- Added a configurable position-covariance floor. Without it, FAST-LIO became
  overconfident and assigned almost no gain to GNSS.
- Published `/localization/odom` with:
  - pose covariance copied from the correct state covariance indices;
  - body-frame linear velocity from `R_world_body^T * state.vel`;
  - bias-corrected IMU angular velocity;
  - configurable `camera_init` and `body` frames.

## MORAI configuration used by the parent repository

The parent repository provides
`configs/localization/fast_lio_msf_morai_velodyne.yaml`.

Important values:

- wheel input disabled;
- LiDAR to IMU translation: `[0.522, 0.0, 1.173]`;
- IMU to GPS translation: `[-1.528, 0.0, 1.030]`;
- UTM projection enabled;
- external initial-pose alignment enabled;
- position-only GNSS correction enabled;
- GNSS horizontal test standard deviation: 0.5 m in the parent adapter;
- position covariance floor: 0.25 m².

The upstream parameter name `extrinT_Gnss2IMU` is retained for compatibility,
but the measurement model uses its value as the vector from IMU to GNSS.

## Test result on `raw1.bag`

Full bag: 103.78 s, 316.815 m, 2,014 matched output samples.

- XY mean error: 0.194 m
- XY RMSE: 0.253 m
- XY maximum error: 0.722 m
- XY final error: 0.323 m
- final error / path length: 0.102%
- position-error segments above 1 m: 0
- speed MAE: 0.747 m/s
- yaw mean absolute error: 1.083 deg

The earlier FAST-LIO-only evaluation had about 7.48 m final error over the
same route. The GNSS position correction is therefore effective, but the
speed and yaw regressions mean this provider remains experimental.

## Known limitations

- Covariance values are test assumptions, not competition-calibrated values.
- Only a noise-free MORAI bag has been exercised.
- GNSS outage, outlier, reacquisition, and map-jump behavior are not validated.
- Frequent global position changes feed back into later LiDAR map matching;
  the local map is not independently rebased.
- Single-antenna GNSS provides no direct heading measurement.
- Wheel input is unsupported by the competition and disabled here.
- PCL emits upstream deprecation warnings during compilation; these are not
  introduced by this integration.

## Build

```bash
cd ~/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
catkin build fast_lio_msf localization_manager bringup
source devel/setup.bash
```

