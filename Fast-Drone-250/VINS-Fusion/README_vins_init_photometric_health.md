# VINS Init Gate And Photometric Health Notes

This branch is an engineering stabilization branch for the Fast-Drone-250 VINS-Fusion fork. The main idea is:

- Do not let VINS enter `NON_LINEAR` from a visually clean but motion-degenerate initialization window.
- Keep bad initialization candidates isolated from the normal sliding-window estimator.
- Add low-risk visual scene diagnostics first, before letting them change optimization weights.

## Problem We Are Solving

The original failure mode was not only "few features". Some bad runs had plenty of tracks and very low reprojection error, but the vehicle had not accumulated enough useful motion. That means the visual geometry could look internally consistent while the VIO state was still poorly constrained.

For this platform, the dangerous cases are:

- Static or near-static startup.
- Low-parallax windows.
- Pure rotation or violent motion during the first accepted window.
- Stereo initialization that technically has depth but still produces unstable velocity/bias states.
- Frontend feature streams that look dense but are polluted by stale or geometrically inconsistent points.

So the branch moves initialization from "enough frames, then commit" toward "candidate window must pass explicit observability and sanity checks before commit".

## Initialization Gate

The stereo initialization path now checks a candidate window before running the final commit:

- Enough motion frames are required.
- Average parallax must not be too low.
- Excessive parallax is treated as suspicious only when IMU motion does not support it.
- Violent gyro/acceleration windows are rejected.
- IMU acceleration excitation must be above the static noise floor unless the escape condition is active.
- Track count, long-track count, stereo observations, and valid depth count must be healthy.

The gate logs detailed reasons on reject and detailed metrics on accept. This makes initialization diagnosable instead of mysterious.

Example accept log:

```text
VINS stereo init gate accept ... motion_frames=10 ... tracks=180 long=179 stereo_obs=1447 valid_depth=193 par=7.16 ... preint_max=0.02255
```

## Candidate Window Sliding

A key design choice is that ready-gate rejection does not use the normal `slideWindow()` path.

During initialization, low-parallax frames often set `MARGIN_SECOND_NEW`. Calling the normal sliding logic there can repeatedly merge IMU preintegration intervals into the newest slot. In static startup, this can inflate preintegration motion (`preint_max`) even though the vehicle is not really moving.

This branch uses a dedicated `slideInitializationCandidate()` path:

- Drop the oldest candidate frame.
- Shift the initialization window by one frame.
- Keep `frame_count == WINDOW_SIZE` so the gate can retry every frame.
- Keep each preintegration interval at roughly one-frame duration.
- Keep `all_image_frame` and `FeatureManager` synchronized with the candidate window.

Bad solved states still use full candidate reset. The distinction is:

- Ready-gate reject: slide one candidate frame and retry.
- Gyro/sanity failure: reset the candidate window because the solved state is already suspicious.

## Sanity And Quality Checks

After optimization but before switching to `NON_LINEAR`, the branch logs and checks:

- Median and P90 reprojection error in pixel-equivalent units.
- IMU rotation, velocity, and position residual summaries.
- Max speed, max pose step, window motion, and bias norms.

These are not meant to replace the pre-init gate. They are a final safety check that prevents obviously bad solved states from being published as odometry.

## Frontend Fixes Kept In This Branch

The branch also keeps two important frontend safety fixes:

- Clear `n_pts` at the start of `trackImage()` so stale new points are not reinserted as fresh features.
- Restore frontend fundamental-matrix RANSAC rejection so initialization is not fed geometrically inconsistent tracks.

These are especially important before initialization, because a dense but stale feature stream can satisfy count thresholds while still being unusable for VIO.

## Photometric Health Is Observe-Only

This branch adds per-frame photometric diagnostics:

- `brightness_mean`
- `dark_ratio`
- `saturated_ratio`
- `contrast_std`
- `blur_score`
- `photometric_health`

The metrics are computed in the feature tracker and copied into the visual health snapshot. They are logged once per second:

```text
VINS photometric health: photo=0.96 mean=94.0 dark=0.000 sat=0.087 contrast=62.1 blur=2169.3
```

Important: these metrics currently do not change visual weights, initialization gates, or optimizer behavior. This is intentional. We first want to collect normal ranges across bags, then decide whether exposure or blur should affect `VisualHealthMonitor`.

## Test Results On This Branch

Build:

```bash
source /opt/ros/noetic/setup.bash
cd /home/jun/springflyer3/Fast-Drone-250
catkin_make --source VINS-Fusion --pkg vins -DCMAKE_BUILD_TYPE=Release
```

Smoke tests were run by playing only the input topics:

- `/camera/infra1/image_rect_raw`
- `/camera/infra2/image_rect_raw`
- `/mavros/imu/data_raw`

Results:

- `2026-06-29-16-19-34.bag`: initialization finished at `1782721205.192`, `reproj_med_px=1.412`, sanity accepted.
- `2026-06-27-17-16-27.bag`: initialization finished at `1782551790.109`, `reproj_med_px=0.128`, sanity accepted.

The photometric logs appeared normally on both bags. The metric is therefore wired correctly and is safe as an observe-only signal.

## Current Caution

The `17-16-27` bag can still accept a weak-motion window:

```text
motion_frames=3 lowdyn=8/11 window_motion=0.000
```

That is not caused by the photometric code. It means the current gate may still be a bit permissive for very low-motion stereo windows. If we want stricter startup behavior, the next change should raise the motion requirement or add a stronger minimum window-motion condition. The tradeoff is later initialization.

## Recommended Next Step

Keep this branch as the stable diagnostic baseline. Before making photometric health affect weights, collect ranges on:

- Healthy bright textured flights.
- Low-light flights.
- Motion-blur or aggressive yaw flights.
- White-wall or low-texture flights.

Then connect the metrics to `VisualHealthMonitor` gradually, probably first as a soft visual-weight modifier rather than a hard reject.
