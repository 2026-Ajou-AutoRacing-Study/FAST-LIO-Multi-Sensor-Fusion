# AjouNice MORAI GNSS integration changes

This file documents the local changes made on 2026-07-16 and 2026-07-17 to the forked
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
- Added capture of the predicted GNSS antenna position at the GNSS
  measurement timestamp while propagating the IMU state. This prediction is
  retained until the post-LiDAR external GNSS correction is applied.
- Deferred GNSS measurements that are older than the available propagation
  interval are dropped. Applying such an out-of-sequence position directly to
  the current state would introduce a speed-dependent longitudinal error.

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
- Changed the position-only external GNSS innovation from
  `GNSS(t_gnss) - predicted_antenna(t_lidar_end)` to the time-consistent form
  `GNSS(t_gnss) - predicted_antenna(t_gnss)`. The resulting innovation is
  still applied after the LiDAR update, preserving the existing map-update
  ordering while avoiding the use of an older GNSS position as if it were a
  current-time measurement.
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

## 2026-07-17 GNSS timestamp-consistency correction

### Problem

The initial external GNSS implementation selected the latest GNSS sample at
each LiDAR scan end. On `raw1.bag`, that GNSS measurement was on average about
40.2 ms older than the LiDAR state, but the position innovation compared it
against the antenna position predicted at the LiDAR scan end. The mismatch
behaved like a delay along the direction of travel:

```text
innovation_old = GNSS_position(t_gnss)
               - predicted_antenna_position(t_lidar_end)
```

The measured longitudinal error was strongly speed-dependent. The inferred
`-velocity * GNSS_age` term had a correlation of about 0.956 with the observed
longitudinal position error.

### Correction

During IMU propagation, the estimator now saves the predicted antenna
position when propagation reaches the actual GNSS timestamp. After the LiDAR
measurement update, the position-only GNSS correction uses the co-timed
innovation:

```text
innovation_new = GNSS_position(t_gnss)
               - predicted_antenna_position(t_gnss)
```

No fixed 40 ms delay, vehicle speed, MORAI Ego GT, or bag-specific constant is
used by the implementation. This is a general timestamp-consistency fix for
the current external position-only correction path.

This is not a full out-of-sequence-measurement rewind implementation. A GNSS
measurement older than the state-history interval available to the current
IMU/LiDAR propagation is discarded instead of being applied at the wrong
time. Correct sensor timestamps and a common clock remain prerequisites.

### Runtime validation

The complete 104 s `raw1_beta_drive.bag` route was replayed with the modified
estimator. With the existing position covariance floor of 0.25 m², the result
changed as follows:

| Metric | Before | After |
|---|---:|---:|
| XY mean error | 0.194 m | 0.104 m |
| XY RMSE | 0.253 m | 0.130 m |
| XY maximum error | 0.722 m | 0.524 m |
| XY final error | 0.323 m | 0.169 m |
| Final error / path length | 0.102% | 0.053% |
| Longitudinal bias | -0.182 m | -0.048 m |
| Longitudinal MAE | 0.182 m | 0.062 m |
| Longitudinal RMSE | 0.244 m | 0.082 m |

The previous speed-dependent relationship was largely removed: the fitted
effective delay decreased from roughly 45.6 ms to about 1.8 ms, with an
R-squared value of about 0.005 after correction.

A separate test reduced the covariance floor from 0.25 m² to 0.05 m². It
worsened XY RMSE to about 0.401 m and longitudinal/lateral errors also grew,
so the parent configuration retains 0.25 m².

### Remaining limitation observed after the correction

The timestamp correction improves absolute position and removes most of the
longitudinal lag, but it exposes velocity-state error more clearly during
stops. FAST-LIO can retain nonzero velocity while repeated GNSS position
updates pull the pose back, producing position jitter and excessive
accumulated path length. In this validation, the estimated path-length error
increased from about +5.40 m to +15.11 m despite the lower absolute position
error.

This should be addressed separately through velocity-state validation and a
robust stationary constraint such as ZUPT. It should not be hidden by
reintroducing the timestamp mismatch or by over-trusting GNSS covariance.

## Known limitations

- Covariance values are test assumptions, not competition-calibrated values.
- Only a noise-free MORAI bag has been exercised.
- GNSS outage, outlier, reacquisition, and map-jump behavior are not validated.
- Frequent global position changes feed back into later LiDAR map matching;
  the local map is not independently rebased.
- Single-antenna GNSS provides no direct heading measurement.
- The timestamp-consistency correction does not rewind the full filter state
  for arbitrarily delayed GNSS measurements; measurements older than the
  available propagation interval are dropped.
- Position-only GNSS correction does not directly correct attitude, velocity,
  or IMU biases. Stationary velocity drift and pose jitter remain unresolved.
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
