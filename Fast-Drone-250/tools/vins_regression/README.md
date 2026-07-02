# VINS rosbag regression

This directory turns `/home/jun/record/*.bag` into a repeatable VINS regression set.

See `INIT_STRATEGY.md` for the current simplified initialization/runtime strategy
and the feasibility notes for larger replacements.

## Run

```bash
cd ~/springflyer3/Fast-Drone-250
RUNS=1 tools/vins_regression/run_regression.sh
```

The output directory defaults to `/tmp/vins_regression_<timestamp>`.
Each run also writes `metadata.txt` beside `summary.csv`. Keep it with the
summary because it records the exact git state, YAML hash, and compiled
`vins_node` hash used for that regression run.

For repeated runs:

```bash
RUNS=3 tools/vins_regression/run_regression.sh
```

## Manifest

`bags.csv` lists each bag and whether initialization is expected:

- `expect_init=1`: the bag should initialize and publish `/vins_fusion/odometry`.
- `expect_init=0`: the bag should not initialize; `/vins_fusion/imu_propagate` may still publish the pre-init preview.
- `expect_init=any`: only failure logs are treated as failures.

## Summary columns

- `init_success`: `1` if `Initialization finish!` or `/vins_fusion/odometry` appears.
- `pass`: matches `expect_init` and has no failure logs.
- `end_norm`: displacement from the first to last odometry sample. This is a loop-closure residual proxy, not true ATE.
- `max_rel`: max displacement from the first odometry sample.
- `max_step`: largest adjacent odometry jump.
- `max_speed`: largest odometry speed norm.
- `imu_first_cov0` / `imu_last_cov0`: `/imu_propagate` quality marker from `pose.covariance[0]`.

This is not a replacement for ATE with ground truth. It is a lightweight regression gate for the bags currently in `/home/jun/record`.

## Traceability

Before comparing two summaries, first compare their `metadata.txt` files:

- `git_head` and `git_status` identify the source tree, including dirty changes.
- `fast_drone_250.yaml` hash identifies the runtime parameter set.
- `devel/lib/vins/vins_node` hash identifies the binary that actually ran.

Do not draw algorithm conclusions from two runs unless these fields are known.
