#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MANIFEST="${1:-$ROOT/tools/vins_regression/bags.csv}"
OUT="${2:-/tmp/vins_regression_$(date +%Y%m%d_%H%M%S)}"
RUNS="${RUNS:-1}"
PORT_BASE="${PORT_BASE:-11700}"
BAG_RATE="${BAG_RATE:-3.0}"

mkdir -p "$OUT"
SUMMARY="$OUT/summary.csv"
METADATA="$OUT/metadata.txt"
echo "bag,expect_init,init_success,pass,odom_count,imu_count,first_stamp,last_stamp,duration,end_norm,max_rel,max_step,max_speed,imu_first_cov0,imu_last_cov0,init_finish_count,gate_accept_count,sanity_reject_count,failure_count,run_dir" > "$SUMMARY"

write_metadata() {
    local config="$ROOT/VINS-Fusion/config/fast_drone_250.yaml"
    local vins_node="$ROOT/devel/lib/vins/vins_node"
    {
        echo "timestamp=$(date -Is)"
        echo "root=$ROOT"
        echo "manifest=$MANIFEST"
        echo "runs=$RUNS"
        echo "bag_rate=$BAG_RATE"
        echo "port_base=$PORT_BASE"
        echo "git_branch=$(git -C "$ROOT" branch --show-current 2>/dev/null || echo unknown)"
        echo "git_head=$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
        echo "git_status_begin"
        git -C "$ROOT" status --short 2>/dev/null || true
        echo "git_status_end"
        if [[ -f "$config" ]]; then
            sha256sum "$config"
            stat -c "config_mtime=%y %n" "$config"
        else
            echo "missing_config=$config"
        fi
        if [[ -f "$vins_node" ]]; then
            sha256sum "$vins_node"
            stat -c "vins_node_mtime=%y %n" "$vins_node"
        else
            echo "missing_vins_node=$vins_node"
        fi
        if [[ -f "$MANIFEST" ]]; then
            sha256sum "$MANIFEST"
            stat -c "manifest_mtime=%y %n" "$MANIFEST"
        fi
    } > "$METADATA"
}

write_metadata

run_one() {
    local bag="$1"
    local expect_init="$2"
    local run_idx="$3"
    local port="$4"
    local stem
    stem="$(basename "$bag" .bag)"
    local run_dir="$OUT/${stem}_run${run_idx}"
    mkdir -p "$run_dir/ros_home" "$run_dir/ros_log" "$run_dir/records"
    rm -f /tmp/vins_output/backend_selector_stats.csv

    source /opt/ros/noetic/setup.bash
    source "$ROOT/devel/setup.bash"

    export ROS_MASTER_URI="http://127.0.0.1:${port}"
    export ROS_IP="127.0.0.1"
    export ROS_HOSTNAME="127.0.0.1"
    export ROS_HOME="$run_dir/ros_home"
    export ROS_LOG_DIR="$run_dir/ros_log"
    export LD_LIBRARY_PATH="$ROOT/devel/lib:${LD_LIBRARY_PATH:-}"

    local core_pid="" launch_pid="" rec_pid="" play_pid=""
    cleanup_run() {
        set +e
        for pid in "$rec_pid" "$play_pid" "$launch_pid" "$core_pid"; do
            if [[ -n "$pid" ]]; then kill "$pid" 2>/dev/null || true; fi
        done
        sleep 1
        for pid in "$rec_pid" "$play_pid" "$launch_pid" "$core_pid"; do
            if [[ -n "$pid" ]]; then kill -9 "$pid" 2>/dev/null || true; fi
        done
        set -e
    }
    trap cleanup_run RETURN

    roscore -p "$port" > "$run_dir/roscore.log" 2>&1 &
    core_pid=$!
    sleep 2

    roslaunch vins fast_drone_250.launch > "$run_dir/vins_launch.log" 2>&1 &
    launch_pid=$!
    sleep 6

    /usr/bin/python3 "$ROOT/tools/vins_regression/record_odom_topics.py" "$run_dir/records" \
        /vins_fusion/odometry /vins_fusion/imu_propagate \
        > "$run_dir/recorder.log" 2>&1 &
    rec_pid=$!
    sleep 1

    rosbag play "$bag" --quiet --rate "$BAG_RATE" --topics \
        /camera/infra1/image_rect_raw \
        /camera/infra2/image_rect_raw \
        /mavros/imu/data_raw < /dev/null > "$run_dir/rosbag.log" 2>&1 &
    play_pid=$!
    wait "$play_pid" || true
    play_pid=""
    sleep 3

    if [[ -n "$rec_pid" ]]; then
        kill "$rec_pid" 2>/dev/null || true
        wait "$rec_pid" 2>/dev/null || true
        rec_pid=""
    fi
    rostopic list > "$run_dir/topics_after.txt" || true
    if [[ -f /tmp/vins_output/backend_selector_stats.csv ]]; then
        cp /tmp/vins_output/backend_selector_stats.csv "$run_dir/backend_selector_stats.csv"
    fi
    /usr/bin/python3 "$ROOT/tools/vins_regression/summarize_run.py" "$run_dir" "$bag" "$expect_init" >> "$SUMMARY"
}

port="$PORT_BASE"
tail -n +2 "$MANIFEST" | while IFS=, read -r bag expect_init notes; do
    [[ -z "${bag:-}" ]] && continue
    if [[ ! -f "$bag" ]]; then
        echo "missing bag: $bag" >&2
        continue
    fi
    for run_idx in $(seq 1 "$RUNS"); do
        echo "[regression] bag=$(basename "$bag") run=$run_idx port=$port"
        run_one "$bag" "$expect_init" "$run_idx" "$port"
        port=$((port + 1))
    done
done

echo "summary: $SUMMARY"
echo "metadata: $METADATA"
column -s, -t "$SUMMARY" || cat "$SUMMARY"
