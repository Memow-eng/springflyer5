# Stable VINS Init Gate Branch, 2026-06-30

This branch is the stable afternoon baseline we decided to preserve before
continuing the low-flow and accelerometer-bias experiments.

Branch name:

```text
vins-stable-init-gate-20260630
```

Base commit:

```text
811fa0b Relax VINS init violent-motion gate
```

## What This Branch Keeps

- Robust stereo initialization readiness checks before committing to
  `NON_LINEAR`.
- Dedicated initialization-candidate sliding instead of using the normal
  `slideWindow()` during ready-gate rejection.
- Initialization sanity checks for speed, pose step, window motion, bias norm,
  and reprojection quality.
- A more permissive but safer violent-motion gate using robust motion evidence
  instead of rejecting good takeoff windows from a single strict spike.
- Frontend fixes that prevent stale or geometrically bad tracks from polluting
  initialization.
- Observe-only photometric health logging for brightness, saturation, contrast,
  and blur.

## Why We Kept It

The original failure mode was that a visually clean static window could have
very low reprojection error while still being bad for VIO initialization. The
branch therefore gates on motion, parallax, track quality, stereo depth, IMU
excitation, and final solved-state sanity instead of relying on reprojection
residual alone.

The dedicated initialization slide is important because the normal
`slideWindow()` path can merge low-parallax IMU preintegration intervals during
startup. That can make `preint_max` grow even when the vehicle is static. The
candidate slide keeps preintegration intervals at one-frame scale and retries
the gate every frame.

## What Is Not Included

This branch does not include the later low-flow stationary-lock experiments
from the evening session:

- no low-flow position reanchor rejection;
- no low-flow bias anchor hold extension;
- no Ceres subset freeze for low-flow `Ba/Bg`;
- no `low_flow_bias_hold_frames = 90` experiment.

Those changes remain in the working tree for the next iteration and were not
pushed here.

## Test Notes

The afternoon baseline was selected because it initialized the important bags
cleanly and avoided the earlier bad static-window initialization behavior.

Representative checks:

- `2026-06-29-16-19-34.bag`: initialized successfully with sane stereo init
  quality and no static-window commit.
- `2026-06-27-17-10-48.bag`: recovered after relaxing the overly strict
  violent-motion gate.
- `2026-06-27-17-16-27.bag`: initialized successfully and stayed within the
  expected final-position range for the afternoon baseline.

Build command:

```bash
source /opt/ros/noetic/setup.bash
cd /home/jun/springflyer3/Fast-Drone-250
catkin_make --source VINS-Fusion --pkg vins -DCMAKE_BUILD_TYPE=Release
```

Smoke-test command pattern:

```bash
bash /tmp/sf3_photo_smoke_direct.sh /path/to/bag 11490
```

## Next Work

Continue low-flow/static handling from the current dirty working tree, not from
this pushed branch. The next open question is whether the low-flow stationary
lock should keep a short bias hold, a longer bias hold, or a softer release
strategy without hurting the other two regression bags.
