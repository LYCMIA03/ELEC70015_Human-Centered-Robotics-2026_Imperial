#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/yuchen/work/ELEC70015_Human-Centered-Robotics-2026_Imperial"
OUTDIR="$ROOT/Log/sim_retest_20260318"
LOGDIR="$OUTDIR/logs"
METRICDIR="$OUTDIR/metrics"
mkdir -p "$LOGDIR" "$METRICDIR"

source /opt/ros/noetic/setup.bash
if [ -f "$ROOT/catkin_ws/devel/setup.bash" ]; then
  source "$ROOT/catkin_ws/devel/setup.bash"
fi
export ROS_PACKAGE_PATH="$ROOT/catkin_ws/src:${ROS_PACKAGE_PATH:-}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"

wait_topic() {
  local topic="$1"
  local timeout_s="$2"
  local t0
  t0=$(date +%s)
  while true; do
    if timeout 2 rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
      return 0
    fi
    if (( $(date +%s) - t0 > timeout_s )); then
      echo "[FAIL] timeout waiting topic: $topic" >&2
      return 1
    fi
    sleep 1
  done
}

wait_status_match() {
  local pattern="$1"
  local timeout_s="$2"
  local t0
  t0=$(date +%s)
  while true; do
    local s
    s=$(timeout 2 rostopic echo -n 1 /target_follower/status 2>/dev/null | sed -n "s/^data: //p" | tr -d '\r') || true
    if [[ -n "$s" ]] && [[ "$s" =~ $pattern ]]; then
      echo "$s"
      return 0
    fi
    if (( $(date +%s) - t0 > timeout_s )); then
      echo ""
      return 1
    fi
    sleep 1
  done
}

cleanup() {
  set +e
  for p in ${P_ECHO_STATUS:-} ${P_ECHO_RESULT:-} ${P_MAP:-} ${P_TFOL:-} ${P_TGT_STREAM1:-} ${P_TGT_STREAM2:-}; do
    [ -n "${p:-}" ] && kill "$p" >/dev/null 2>&1 || true
  done
  pkill -f "roslaunch .*mapping_unitree.launch" >/dev/null 2>&1 || true
  pkill -f "roslaunch .*target_follow_real.launch" >/dev/null 2>&1 || true
  pkill -f "target_follower.py" >/dev/null 2>&1 || true
  pkill -f "slam_gmapping" >/dev/null 2>&1 || true
  pkill -f "move_base" >/dev/null 2>&1 || true
  pkill -f "gzserver" >/dev/null 2>&1 || true
  pkill -f "gzclient" >/dev/null 2>&1 || true
  pkill -f "rosmaster --core" >/dev/null 2>&1 || true
}
trap cleanup EXIT

# 1) Mapping base stack (sim + gmapping + move_base), no legacy target_follow.launch
(
  cd "$ROOT"
  roslaunch p3at_lms_navigation mapping_unitree.launch gui:=false use_target_follower:=false
) >"$LOGDIR/11_mapping_unitree_v2.log" 2>&1 &
P_MAP=$!
echo "[INFO] mapping_unitree_v2 PID=$P_MAP"

wait_topic /clock 80
wait_topic /map 120
wait_topic /move_base/status 120
wait_topic /unitree/scan 120

# 2) TF sanity from URDF authority
timeout 6 rosrun tf tf_echo /base_link /laser >"$LOGDIR/12_tf_base_to_laser_v2.log" 2>&1 || true
timeout 6 rosrun tf tf_echo /base_link /camera_link >"$LOGDIR/13_tf_base_to_camera_v2.log" 2>&1 || true

# 3) target_follow_real overlay (real deployment logic, no embedded move_base)
(
  cd "$ROOT"
  roslaunch target_follower target_follow_real.launch \
    launch_move_base:=false \
    use_online_slam:=true \
    global_frame:=map \
    publish_robot_state_in_overlay:=false \
    enable_auto_explore:=true \
    lidar_mode:=unitree \
    standoff_distance:=0.6 \
    face_target:=true \
    target_timeout:=5.0
) >"$LOGDIR/14_target_follow_real_overlay_v2.log" 2>&1 &
P_TFOL=$!
echo "[INFO] target_follow_real_v2 PID=$P_TFOL"

wait_topic /target_follower/status 70

(rostopic echo -p /target_follower/status >"$METRICDIR/status_v2.csv") & P_ECHO_STATUS=$!
(rostopic echo -p /target_follower/result >"$METRICDIR/result_v2.csv") & P_ECHO_RESULT=$!

# 4) Auto explore check
AUTO_OK=0
if [[ -n "$(wait_status_match "EXPLORING" 90 || true)" ]]; then
  AUTO_OK=1
fi

# 5) Case A: target then refuse
(timeout 25 rostopic pub -r 4 /trash_detection/target_point geometry_msgs/PointStamped "{header: {frame_id: 'camera_link'}, point: {x: 0.05, y: 0.0, z: 2.0}}") >"$LOGDIR/15_target_stream_caseA_v2.log" 2>&1 &
P_TGT_STREAM1=$!
A_TRACK=0
if [[ -n "$(wait_status_match "TRACKING|CLOSE_APPROACH|REACHED|WAITING_ACTION" 80 || true)" ]]; then
  A_TRACK=1
fi
rostopic pub -1 /trash_action std_msgs/Bool "data: false" >/dev/null 2>&1 || true
A_RETREAT=0
if [[ -n "$(wait_status_match "RETREATING" 50 || true)" ]]; then
  A_RETREAT=1
fi

# 6) Case B: target then accept
(timeout 25 rostopic pub -r 4 /trash_detection/target_point geometry_msgs/PointStamped "{header: {frame_id: 'camera_link'}, point: {x: 0.0, y: 0.0, z: 1.8}}") >"$LOGDIR/16_target_stream_caseB_v2.log" 2>&1 &
P_TGT_STREAM2=$!
B_TRACK=0
if [[ -n "$(wait_status_match "TRACKING|CLOSE_APPROACH|REACHED|WAITING_ACTION" 80 || true)" ]]; then
  B_TRACK=1
fi
rostopic pub -1 /trash_action std_msgs/Bool "data: true" >/dev/null 2>&1 || true
B_COOL=0
if [[ -n "$(wait_status_match "POST_ACCEPT_COOLDOWN|RETREATING" 70 || true)" ]]; then
  B_COOL=1
fi
B_RETREAT=0
if [[ -n "$(wait_status_match "RETREATING" 70 || true)" ]]; then
  B_RETREAT=1
fi

python3 - "$LOGDIR" "$METRICDIR" "$AUTO_OK" "$A_TRACK" "$A_RETREAT" "$B_TRACK" "$B_COOL" "$B_RETREAT" <<'PY'
import json, os, re, sys

logdir, metdir, auto_ok, a_track, a_retreat, b_track, b_cool, b_retreat = sys.argv[1:]
auto_ok = int(auto_ok)
a_track = int(a_track)
a_retreat = int(a_retreat)
b_track = int(b_track)
b_cool = int(b_cool)
b_retreat = int(b_retreat)

def parse_tf(path):
    txt = open(path, "r", errors="ignore").read()
    m = re.search(r"Translation:\s*\[([^\]]+)\]", txt)
    q = re.search(r"Rotation:\s*in Quaternion\s*\[([^\]]+)\]", txt)
    trans = [float(v.strip()) for v in m.group(1).split(",")] if m else None
    quat = [float(v.strip()) for v in q.group(1).split(",")] if q else None
    return {"translation": trans, "quaternion": quat}

out = {
    "auto_explore_ok": bool(auto_ok),
    "caseA_tracking_or_reached_observed": bool(a_track),
    "caseA_retreat_observed_after_action_false": bool(a_retreat),
    "caseB_tracking_or_reached_observed": bool(b_track),
    "caseB_post_accept_or_retreat_observed_after_action_true": bool(b_cool),
    "caseB_retreat_observed": bool(b_retreat),
    "tf_base_to_laser": parse_tf(os.path.join(logdir, "12_tf_base_to_laser_v2.log")),
    "tf_base_to_camera_link": parse_tf(os.path.join(logdir, "13_tf_base_to_camera_v2.log")),
}
out["overall_pass"] = all([
    out["auto_explore_ok"],
    out["caseA_tracking_or_reached_observed"],
    out["caseA_retreat_observed_after_action_false"],
    out["caseB_tracking_or_reached_observed"],
    out["caseB_post_accept_or_retreat_observed_after_action_true"],
    out["caseB_retreat_observed"],
])

with open(os.path.join(metdir, "full_logic_retest_summary_v2.json"), "w") as f:
    json.dump(out, f, indent=2)

print(json.dumps(out, indent=2))
PY

timeout 4 rostopic echo -n 20 /target_follower/status >"$LOGDIR/17_status_tail_v2.log" 2>&1 || true
timeout 4 rostopic echo -n 20 /target_follower/result >"$LOGDIR/18_result_tail_v2.log" 2>&1 || true

echo "[DONE] $METRICDIR/full_logic_retest_summary_v2.json"
