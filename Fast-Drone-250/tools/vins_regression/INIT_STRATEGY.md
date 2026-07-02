# VINS initialization simplification notes

This branch treats `/home/jun/record/*.bag` as the regression set and moves the
estimator away from stacked runtime heuristics.

## Current engineering strategy

Initialization is intentionally reduced toward a small, inspectable pipeline:

1. Wait for motion: pure-static or low-parallax windows are not candidate
   failures. They only maintain the bounded initialization buffer and log a
   throttled INFO message.
2. Cheap prefilter: reject only obviously unusable windows before paying for
   stereo initialization.
3. Candidate slide retry: if prefilter, solve, or commit sanity fails, restore
   the candidate state and slide one frame instead of clearing all accumulated
   history.
4. Solve: run the normal VINS stereo + IMU initialization/optimization path.
5. Commit sanity: accept only if reprojection and solved state checks are safe.

Runtime low-flow handling is no longer allowed to hard-write estimator state.
Low-flow may still be logged and may optionally add a soft velocity ZUPT factor,
but the default weight is `0.0`.

## Current active mechanisms

- Stereo initialization cheap prefilter.
- Candidate slide retry after true candidate prefilter, gyro-bias solve,
  reprojection sanity, or state sanity failure.
- WAIT_MOTION state for static/low-parallax windows. This does not increment the
  retry counter and is not logged as a WARN.
- Post-solve reprojection and state sanity checks.
- Runtime nominal bias prior remains enabled by default.
- Low-flow stationary cue remains enabled, but only logs by default because
  `low_flow_zupt_weight` is `0.0`.

## Removed, disabled, or retired mechanisms

- Init escape approval path.
- Normal-flight violent-motion veto based on strict max IMU spikes; this is
  replaced by a much wider extreme-IMU prefilter.
- Low-acceleration excitation veto as a hard reject.
- Excessive-parallax veto as a hard reject.
- Low-flow hard position lock.
- Low-flow direct velocity zeroing.
- Low-flow bias anchor / bias subset freeze.
- Home-loop pseudo closure.
- `.orig` backup files in the estimator source tree.

These were useful debugging experiments, but together they created hidden
couplings: a window could pass because of escape, fail because of an unrelated
IMU spike, or get initialized then pinned by low-flow state writes.

## Feasibility of replacement options

| Option | Scope | Feasibility | Notes |
| --- | --- | --- | --- |
| Minimal VINS gate + sanity | Small | Current short-term path | Best short-term path. It is easy to regress with bags and avoids hidden locks, but the current branch still keeps candidate slide retry and nominal bias prior. |
| Active candidate scoring | Medium | Practical next step | Keep several recent candidate windows, score solved results, commit the best one instead of waiting for a naturally perfect window. |
| ORB-SLAM3-style inertial-only MAP initializer | Large | Feasible but not a patch | Better architecture, but it means a new initializer module and multi-stage vision-only/inertial-only/full-BA flow. Expect week-level work. |
| Full ORB-SLAM3 replacement | Very large | Not recommended for this branch | Too much frontend/map/runtime surface area changes at once. |

Recommended next step after this simplification is active candidate scoring, not
more hard gates. It preserves the current VINS pipeline while changing the
initializer from "reject bad windows until one survives" into "solve candidates
and commit the safest solved state".

## Regression expectations

- Flight bags in `/home/jun/record` should initialize and avoid failure logs.
- `2026-07-01-16-43-33.bag` should remain uninitialized because it produced an
  unsafe low-excitation solved state.
- `imu_static.bag` should not initialize.
- `/vins_fusion/imu_propagate` may publish before initialization, but
  `pose.covariance[0]` must be `1e6` until VINS is trusted.
