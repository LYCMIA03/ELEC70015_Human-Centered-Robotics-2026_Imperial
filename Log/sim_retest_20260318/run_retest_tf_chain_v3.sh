#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/yuchen/work/ELEC70015_Human-Centered-Robotics-2026_Imperial"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTDIR="$ROOT/Log/sim_retest_20260318/${STAMP}_v3"
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
    s=$(timeout 2 rostopic echo -n 1 /target_follower/status 2>/dev/null | sed -n 's/^data: //p' | tr -d '\r') || true
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
  for p in ${P_ECHO_STATUS:-} ${P_ECHO_RESULT:-} ${P_ECHO_DIST:-} ${P_MAP:-} ${P_TFOL:-} ${P_TGT_STREAM1:-} ${P_TGT_STREAM2:-}; do
    [ -n "${p:-}" ] && kill "$p" >/dev/null 2>&1 || true
  done
  for name in gzserver gzclient move_base slam_gmapping rosmaster roscore; do
    pkill -x "$name" >/dev/null 2>&1 || true
  done
  for pat in "target_follower.py" "udp_target_bridge.py" "point_to_target_pose.py" "mapping_unitree.launch" "target_follow_real.launch"; do
    for pid in $(pgrep -f "$pat" || true); do
      kill "$pid" >/dev/null 2>&1 || true
    done
  done
}
trap cleanup EXIT

# hard cleanup before start
cleanup || true
sleep 2

(
  cd "$ROOT"
  roslaunch p3at_lms_navigation mapping_unitree.launch gui:=false use_target_follower:=false
) >"$LOGDIR/01_mapping_unitree.log" 2>&1 &
P_MAP=$!
echo "[INFO] mapping_unitree PID=$P_MAP"

wait_topic /clock 90
wait_topic /map 140
wait_topic /move_base/status 140
wait_topic /unitree/scan 140

rostopic list > "$METRICDIR/topic_list.txt" 2>/dev/null || true
rosparam get /move_base/local_costmap/obstacle_layer/observation_sources > "$METRICDIR/local_observation_sources.txt" 2>/dev/null || true
rosparam get /move_base/global_costmap/obstacle_layer/observation_sources > "$METRICDIR/global_observation_sources.txt" 2>/dev/null || true

timeout 6 rosrun tf tf_echo /base_link /laser >"$LOGDIR/02_tf_base_to_laser.log" 2>&1 || true
timeout 6 rosrun tf tf_echo /base_link /camera_link >"$LOGDIR/03_tf_base_to_camera.log" 2>&1 || true

(
  cd "$ROOT"
  roslaunch target_follower target_follow_real.launch \
    launch_move_base:=false \
    use_online_slam:=true \
    global_frame:=map \
    publish_robot_state_in_overlay:=false \
    enable_auto_explore:=true \
    lidar_mode:=unitree \
    standoff_distance:=2.3 \
    face_target:=true \
    target_timeout:=6.0
) >"$LOGDIR/04_target_follow_overlay.log" 2>&1 &
P_TFOL=$!
echo "[INFO] target_follow_real PID=$P_TFOL"

wait_topic /target_follower/status 90

(rostopic echo -p /target_follower/status >"$METRICDIR/status.csv") & P_ECHO_STATUS=$!
(rostopic echo -p /target_follower/result >"$METRICDIR/result.csv") & P_ECHO_RESULT=$!
(rostopic echo -p /target_follower/target_distance >"$METRICDIR/target_distance.csv") & P_ECHO_DIST=$!

AUTO_OK=0
if [[ -n "$(wait_status_match "EXPLORING" 120 || true)" ]]; then
  AUTO_OK=1
fi

# Case A: reach waiting action, then reject
(timeout 20 rostopic pub -r 6 /trash_detection/target_point geometry_msgs/PointStamped "{header: {frame_id: 'camera_link'}, point: {x: 0.05, y: 0.0, z: 2.0}}") >"$LOGDIR/05_target_stream_caseA.log" 2>&1 &
P_TGT_STREAM1=$!
A_TRACK=0
if [[ -n "$(wait_status_match "TRACKING|REACHED|WAITING_ACTION" 40 || true)" ]]; then
  A_TRACK=1
fi
A_WAIT=0
if [[ -n "$(wait_status_match "WAITING_ACTION" 45 || true)" ]]; then
  A_WAIT=1
fi
rostopic pub -1 /trash_action std_msgs/Bool "data: false" >/dev/null 2>&1 || true
A_RETREAT=0
if [[ -n "$(wait_status_match "RETREATING" 70 || true)" ]]; then
  A_RETREAT=1
fi
kill "$P_TGT_STREAM1" >/dev/null 2>&1 || true

# try wait back to idle before case B
wait_status_match "IDLE|EXPLORING" 40 >/dev/null 2>&1 || true

# Case B: reach waiting action, then accept
(timeout 20 rostopic pub -r 6 /trash_detection/target_point geometry_msgs/PointStamped "{header: {frame_id: 'camera_link'}, point: {x: 0.0, y: 0.0, z: 1.8}}") >"$LOGDIR/06_target_stream_caseB.log" 2>&1 &
P_TGT_STREAM2=$!
B_TRACK=0
if [[ -n "$(wait_status_match "TRACKING|REACHED|WAITING_ACTION" 40 || true)" ]]; then
  B_TRACK=1
fi
B_WAIT=0
if [[ -n "$(wait_status_match "WAITING_ACTION" 45 || true)" ]]; then
  B_WAIT=1
fi
rostopic pub -1 /trash_action std_msgs/Bool "data: true" >/dev/null 2>&1 || true
B_COOL=0
if [[ -n "$(wait_status_match "POST_ACCEPT_COOLDOWN" 60 || true)" ]]; then
  B_COOL=1
fi
B_RETREAT=0
if [[ -n "$(wait_status_match "RETREATING" 90 || true)" ]]; then
  B_RETREAT=1
fi
kill "$P_TGT_STREAM2" >/dev/null 2>&1 || true

sleep 2

python3 - "$LOGDIR" "$METRICDIR" "$AUTO_OK" "$A_TRACK" "$A_WAIT" "$A_RETREAT" "$B_TRACK" "$B_WAIT" "$B_COOL" "$B_RETREAT" <<'PY'
import json, os, re, sys

logdir, metdir, auto_ok, a_track, a_wait, a_retreat, b_track, b_wait, b_cool, b_retreat = sys.argv[1:]
auto_ok = int(auto_ok)
a_track = int(a_track)
a_wait = int(a_wait)
a_retreat = int(a_retreat)
b_track = int(b_track)
b_wait = int(b_wait)
b_cool = int(b_cool)
b_retreat = int(b_retreat)

def parse_tf(path):
    txt = open(path, "r", errors="ignore").read()
    m = re.search(r"Translation:\s*\[([^\]]+)\]", txt)
    q = re.search(r"Rotation:\s*in Quaternion\s*\[([^\]]+)\]", txt)
    trans = [float(v.strip()) for v in m.group(1).split(",")] if m else None
    quat = [float(v.strip()) for v in q.group(1).split(",")] if q else None
    return {"translation": trans, "quaternion": quat}

obs_local = ""
obs_global = ""
for name, target in [("local_observation_sources.txt", "obs_local"), ("global_observation_sources.txt", "obs_global")]:
    p = os.path.join(metdir, name)
    if os.path.exists(p):
        txt = open(p, "r", errors="ignore").read().strip()
        if target == "obs_local":
            obs_local = txt
        else:
            obs_global = txt

log_text = open(os.path.join(logdir, "04_target_follow_overlay.log"), "r", errors="ignore").read()
traceback_found = "Traceback (most recent call last):" in log_text

out = {
    "auto_explore_ok": bool(auto_ok),
    "caseA_tracking_or_reached_observed": bool(a_track),
    "caseA_waiting_action_observed": bool(a_wait),
    "caseA_retreat_after_action_false": bool(a_retreat),
    "caseB_tracking_or_reached_observed": bool(b_track),
    "caseB_waiting_action_observed": bool(b_wait),
    "caseB_post_accept_cooldown_observed": bool(b_cool),
    "caseB_retreat_after_action_true": bool(b_retreat),
    "tf_base_to_laser": parse_tf(os.path.join(logdir, "02_tf_base_to_laser.log")),
    "tf_base_to_camera_link": parse_tf(os.path.join(logdir, "03_tf_base_to_camera.log")),
    "local_costmap_observation_sources": obs_local,
    "global_costmap_observation_sources": obs_global,
    "traceback_found": traceback_found,
}

out["overall_pass"] = all([
    out["auto_explore_ok"],
    out["caseA_waiting_action_observed"],
    out["caseA_retreat_after_action_false"],
    out["caseB_waiting_action_observed"],
    out["caseB_post_accept_cooldown_observed"],
    out["caseB_retreat_after_action_true"],
    not out["traceback_found"],
])

with open(os.path.join(metdir, "full_logic_retest_summary_v3.json"), "w") as f:
    json.dump(out, f, indent=2)

print(json.dumps(out, indent=2))
PY

timeout 5 rostopic echo -n 20 /target_follower/status >"$LOGDIR/07_status_tail.log" 2>&1 || true
timeout 5 rostopic echo -n 20 /target_follower/result >"$LOGDIR/08_result_tail.log" 2>&1 || true

echo "[DONE] $METRICDIR/full_logic_retest_summary_v3.json"
