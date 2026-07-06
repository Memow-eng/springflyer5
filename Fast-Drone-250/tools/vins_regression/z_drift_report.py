#!/usr/bin/env python3
import csv
import glob
import math
import os
import sys


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def to_float(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def pearson(xs, ys):
    pairs = [(x, y) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 3:
        return float("nan")
    mx = sum(x for x, _ in pairs) / len(pairs)
    my = sum(y for _, y in pairs) / len(pairs)
    vx = sum((x - mx) ** 2 for x, _ in pairs)
    vy = sum((y - my) ** 2 for _, y in pairs)
    if vx <= 0.0 or vy <= 0.0:
        return float("nan")
    cov = sum((x - mx) * (y - my) for x, y in pairs)
    return cov / math.sqrt(vx * vy)


def interp_state(states, stamp):
    if not states:
        return None
    best = min(states, key=lambda r: abs(to_float(r, "time") - stamp))
    return best


def summarize_run(run_dir):
    odom = read_csv(os.path.join(run_dir, "records", "vins_fusion__odometry.csv"))
    state = read_csv(os.path.join(run_dir, "vins_state_log.csv"))
    bag = os.path.basename(run_dir).replace("_run1", ".bag")
    if not odom:
        return {
            "bag": bag,
            "odom_count": 0,
            "first_z": "",
            "final_z": "",
            "min_z": "",
            "max_z": "",
            "dz": "",
            "max_abs_z": "",
            "last_x": "",
            "last_y": "",
            "last_z": "",
            "last_ba_norm": "",
            "last_baz": "",
            "last_gz": "",
            "corr_z_baz": "",
            "corr_z_ba_norm": "",
        }

    zs = [to_float(r, "z") for r in odom]
    first = odom[0]
    last = odom[-1]
    state_by_odom = []
    for r in odom:
        s = interp_state(state, to_float(r, "stamp"))
        if s is not None:
            state_by_odom.append((to_float(r, "z"), to_float(s, "baz"), to_float(s, "ba_norm")))
    last_state = interp_state(state, to_float(last, "stamp"))

    return {
        "bag": bag,
        "odom_count": len(odom),
        "first_z": f"{zs[0]:.4f}",
        "final_z": f"{zs[-1]:.4f}",
        "min_z": f"{min(zs):.4f}",
        "max_z": f"{max(zs):.4f}",
        "dz": f"{zs[-1] - zs[0]:.4f}",
        "max_abs_z": f"{max(abs(z) for z in zs):.4f}",
        "last_x": f"{to_float(last, 'x'):.4f}",
        "last_y": f"{to_float(last, 'y'):.4f}",
        "last_z": f"{to_float(last, 'z'):.4f}",
        "last_ba_norm": "" if last_state is None else f"{to_float(last_state, 'ba_norm'):.4f}",
        "last_baz": "" if last_state is None else f"{to_float(last_state, 'baz'):.4f}",
        "last_gz": "" if last_state is None else f"{to_float(last_state, 'gz'):.4f}",
        "corr_z_baz": f"{pearson([x[0] for x in state_by_odom], [x[1] for x in state_by_odom]):.4f}" if state_by_odom else "",
        "corr_z_ba_norm": f"{pearson([x[0] for x in state_by_odom], [x[2] for x in state_by_odom]):.4f}" if state_by_odom else "",
    }


def main():
    if len(sys.argv) != 2:
        print("usage: z_drift_report.py REGRESSION_OUT_DIR", file=sys.stderr)
        return 2
    base = sys.argv[1]
    rows = [summarize_run(p) for p in sorted(glob.glob(os.path.join(base, "*_run*")))]
    fields = [
        "bag", "odom_count", "first_z", "final_z", "min_z", "max_z", "dz", "max_abs_z",
        "last_x", "last_y", "last_z", "last_ba_norm", "last_baz", "last_gz",
        "corr_z_baz", "corr_z_ba_norm",
    ]
    writer = csv.DictWriter(sys.stdout, fieldnames=fields)
    writer.writeheader()
    for row in rows:
        writer.writerow(row)
    return 0


if __name__ == "__main__":
    sys.exit(main())
