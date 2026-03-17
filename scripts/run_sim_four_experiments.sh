#!/usr/bin/env bash
#
# 一键运行四个 Gazebo+RViz 仿真实验并自动采集数据：
#   1) 简单障碍场景定点导航
#   2) 复杂障碍场景定点导航
#   3) 简单障碍场景 target following
#   4) Task6/7 风格自主建图 + AMCL 验证
#
# 结果输出:
#   Log/sim_experiments/<timestamp>/
#     ├── logs/*.log
#     ├── metrics/*.json
#     ├── summary.json
#     └── summary.md

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${REPO_ROOT}/catkin_ws"
NAV_DIR="${WS_DIR}/src/p3at_lms_navigation"

SIMPLE_WORLD="${REPO_ROOT}/catkin_ws/src/p3at_lms_gazebo/worlds/p3at_lms.world"
COMPLEX_WORLD="${REPO_ROOT}/catkin_ws/src/p3at_lms_gazebo/worlds/complex_maze.world"

SKIP_BUILD=false
HEADLESS=true
NAV_GOAL_TIMEOUT=120
TF_WARMUP=8
FOLLOW_DURATION=180
EXPLORE_TIMEOUT=300

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build) SKIP_BUILD=true; shift ;;
    --gui) HEADLESS=false; shift ;;
    --nav-goal-timeout) NAV_GOAL_TIMEOUT="$2"; shift 2 ;;
    --follow-duration) FOLLOW_DURATION="$2"; shift 2 ;;
    --explore-timeout) EXPLORE_TIMEOUT="$2"; shift 2 ;;
    -h|--help)
      cat <<'EOF'
Usage: ./scripts/run_sim_four_experiments.sh [options]

Options:
  --skip-build           Skip catkin_make
  --gui                  Enable Gazebo GUI (default is headless)
  --nav-goal-timeout N   Nav goal timeout in seconds (default: 120)
  --follow-duration N    Target-follow metric duration in seconds (default: 180)
  --explore-timeout N    Task6 mapping timeout in seconds (default: 300)
EOF
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown arg: $1" >&2
      exit 1
      ;;
  esac
done

if [[ "${HEADLESS}" == "true" ]]; then
  GUI_FLAG="false"
else
  GUI_FLAG="true"
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${REPO_ROOT}/Log/sim_experiments/${RUN_ID}"
LOG_DIR="${RUN_DIR}/logs"
METRIC_DIR="${RUN_DIR}/metrics"
mkdir -p "${LOG_DIR}" "${METRIC_DIR}"

ts() { date +"%Y-%m-%d %H:%M:%S"; }
info() { echo "[$(ts)] [INFO] $*"; }
warn() { echo "[$(ts)] [WARN] $*" >&2; }
err() { echo "[$(ts)] [ERROR] $*" >&2; }

source /opt/ros/noetic/setup.bash

cleanup_ros() {
  set +e
  rosnode kill -a >/dev/null 2>&1 || true
  pkill -f "roslaunch" >/dev/null 2>&1 || true
  pkill -f "rosmaster" >/dev/null 2>&1 || true
  pkill -f "roscore" >/dev/null 2>&1 || true
  pkill -f "gzserver" >/dev/null 2>&1 || true
  pkill -f "gzclient" >/dev/null 2>&1 || true
  pkill -f "rviz" >/dev/null 2>&1 || true
  sleep 2
}
trap cleanup_ros EXIT

ensure_workspace_sane() {
  local backup_pkg="${WS_DIR}/src/rplidar_ros_nested_backup_20260317"
  if [[ -d "${backup_pkg}" ]]; then
    if [[ ! -f "${backup_pkg}/CATKIN_IGNORE" ]]; then
      warn "Detected backup package with duplicate name: ${backup_pkg}"
      warn "Adding CATKIN_IGNORE so catkin_make can proceed."
      touch "${backup_pkg}/CATKIN_IGNORE"
    fi
  fi
}

wait_for_topic_msg() {
  local topic="$1"
  local timeout_s="${2:-60}"
  local started
  started="$(date +%s)"
  while true; do
    if timeout 2 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
      return 0
    fi
    local now elapsed
    now="$(date +%s)"
    elapsed=$((now - started))
    if (( elapsed > timeout_s )); then
      return 1
    fi
    sleep 1
  done
}

wait_for_node() {
  local node="$1"
  local timeout_s="${2:-60}"
  local started
  started="$(date +%s)"
  while true; do
    if rosnode list 2>/dev/null | grep -qE "(^|/)${node}$"; then
      return 0
    fi
    local now elapsed
    now="$(date +%s)"
    elapsed=$((now - started))
    if (( elapsed > timeout_s )); then
      return 1
    fi
    sleep 1
  done
}

launch_bg() {
  local log_file="$1"
  shift
  "$@" >"${log_file}" 2>&1 &
  echo $!
}

run_nav_experiment() {
  local scenario="$1"
  local world="$2"
  local launch_log="$3"
  local metric_out="$4"

  info "Launching mapping stack for ${scenario} world=${world}"
  local launch_pid
  launch_pid="$(launch_bg "${launch_log}" roslaunch p3at_lms_navigation mapping_unitree.launch \
    gui:="${GUI_FLAG}" \
    world:="${world}" \
    use_gazebo_target:=false \
    use_target_follower:=false \
    enable_rplidar_local_obstacle:=true)"

  if ! wait_for_node move_base 90; then
    err "move_base failed to start for ${scenario}. See ${launch_log}"
    kill "${launch_pid}" >/dev/null 2>&1 || true
    return 1
  fi
  wait_for_topic_msg /map 60 || warn "/map not confirmed quickly in ${scenario}"

  info "Running navigation metrics collector (${scenario})"
  if ! rosrun p3at_lms_navigation nav_experiment_runner.py \
    --scenario "${scenario}" \
    --goal-timeout "${NAV_GOAL_TIMEOUT}" \
    --warmup "${TF_WARMUP}" \
    --output "${metric_out}" \
    >"${LOG_DIR}/nav_${scenario}_runner.log" 2>&1; then
    err "nav_experiment_runner failed for ${scenario}"
    kill "${launch_pid}" >/dev/null 2>&1 || true
    return 1
  fi

  info "Stopping mapping stack (${scenario})"
  kill "${launch_pid}" >/dev/null 2>&1 || true
  wait "${launch_pid}" >/dev/null 2>&1 || true
  cleanup_ros
}

evaluate_target_follow_metric() {
  local metric_path="$1"
  python3 - "$metric_path" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)

s = data.get("summary", {})
robot_speed = float(s.get("robot_speed_mean_mps", 0.0) or 0.0)
target_speed = float(s.get("target_speed_mean_mps", 0.0) or 0.0)
distance_mean = float(s.get("distance_mean_m", 999.0) or 999.0)
within_tol = float(s.get("within_tolerance_rate", 0.0) or 0.0)
result_false = int(s.get("result_false_count", 0) or 0)

score = 0
if robot_speed >= 0.02:
    score += 1
if target_speed >= 0.08:
    score += 1
if distance_mean <= 2.0:
    score += 1
if within_tol >= 0.10:
    score += 1
if result_false <= 1:
    score += 1

print(
    "score=%d/5 robot_speed=%.4f target_speed=%.4f distance_mean=%.3f within_tol=%.3f result_false=%d"
    % (score, robot_speed, target_speed, distance_mean, within_tol, result_false)
)

acceptable = (
    score >= 4
    and robot_speed >= 0.02
    and target_speed >= 0.08
)
sys.exit(0 if acceptable else 2)
PY
}

run_target_follow_attempt() {
  local attempt_name="$1"
  local standoff="$2"
  local tolerance="$3"
  shift 3

  local attempt_launch_log="${LOG_DIR}/exp3_target_follow_launch_${attempt_name}.log"
  local attempt_metric="${METRIC_DIR}/exp3_target_follow_${attempt_name}.json"
  local attempt_metric_log="${LOG_DIR}/target_follow_metrics_${attempt_name}.log"

  info "Exp3 ${attempt_name}: launching target-follow stack"
  local launch_pid
  launch_pid="$(launch_bg "${attempt_launch_log}" roslaunch p3at_lms_navigation mapping_unitree.launch \
    gui:="${GUI_FLAG}" \
    world:="${SIMPLE_WORLD}" \
    use_gazebo_target:=true \
    use_target_follower:=true \
    enable_rplidar_local_obstacle:=true \
    "$@")"

  if ! wait_for_node target_follower 90; then
    err "Exp3 ${attempt_name}: target_follower failed to start. See ${attempt_launch_log}"
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
    cleanup_ros
    return 1
  fi

  wait_for_topic_msg /gazebo/model_states 60 || warn "Exp3 ${attempt_name}: /gazebo/model_states not confirmed quickly"
  wait_for_topic_msg /target_follower/status 60 || warn "Exp3 ${attempt_name}: /target_follower/status not confirmed quickly"

  info "Exp3 ${attempt_name}: collecting target-follow metrics (${FOLLOW_DURATION}s)"
  if ! rosrun p3at_lms_navigation target_follow_metrics.py \
    --duration "${FOLLOW_DURATION}" \
    --warmup "${TF_WARMUP}" \
    --standoff "${standoff}" \
    --tolerance "${tolerance}" \
    --output "${attempt_metric}" \
    >"${attempt_metric_log}" 2>&1; then
    err "Exp3 ${attempt_name}: target_follow_metrics collector failed"
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
    cleanup_ros
    return 1
  fi

  info "Exp3 ${attempt_name}: stopping stack"
  kill "${launch_pid}" >/dev/null 2>&1 || true
  wait "${launch_pid}" >/dev/null 2>&1 || true
  cleanup_ros

  echo "${attempt_metric}"
}

run_target_follow_experiment() {
  local launch_log="$1"
  local metric_out="$2"
  local chosen_attempt=""
  local chosen_metric=""
  local candidate_metric=""

  info "Exp3 strategy: default high-speed moving-target profile (0.36 m/s), with conservative fallback only if launch/collection fails"

  if candidate_metric="$(run_target_follow_attempt "attempt1_highspeed_stable_default" 1.2 0.35 \
      target_spawn_x:=2.6 \
      target_spawn_y:=0.0 \
      move_target:=true \
      target_speed:=0.36 \
      target_pause:=2.0 \
      target_waypoints:="[[2.6,0.0],[2.6,1.2],[1.2,1.2],[1.2,-1.2],[2.6,-1.2],[2.6,0.0]]" \
      standoff_distance:=1.2 \
      face_target:=false \
      enable_interaction_mode:=false \
      target_timeout:=2.0 \
      camera_frame:=rgbd_camera \
      enable_auto_explore:=false \
      send_rate_hz:=10.0 \
      min_update_dist:=0.08 \
      enable_high_speed_follow_tuning:=true \
      follow_max_vel_x:=0.50 \
      follow_max_vel_trans:=0.50 \
      follow_acc_lim_x:=1.20 \
      follow_max_vel_theta:=1.20 \
      follow_acc_lim_theta:=2.00 \
      follow_xy_goal_tolerance:=0.28 \
      follow_yaw_goal_tolerance:=3.14159 \
      follow_latch_xy_goal_tolerance:=true \
      follow_occdist_scale:=0.04 \
      close_approach_threshold:=1.50 \
      close_approach_timeout:=20.0 \
      close_approach_speed:=0.22 \
      close_approach_steer_gain:=0.90)"; then
    chosen_attempt="attempt1_highspeed_stable_default"
    chosen_metric="${candidate_metric}"
    info "Exp3 selected high-speed default profile"
  else
    warn "Exp3 high-speed default failed to run; retrying with conservative fallback profile."
    if candidate_metric="$(run_target_follow_attempt "attempt2_dynamic_conservative_fallback" 0.8 0.35 \
        target_spawn_x:=1.8 \
        target_spawn_y:=0.2 \
        move_target:=true \
        target_speed:=0.12 \
        target_pause:=3.0 \
        target_waypoints:="[[1.8,0.2],[1.8,1.2],[0.2,1.2],[0.2,-1.2],[1.8,-1.2],[1.8,0.2]]" \
        standoff_distance:=0.8 \
        face_target:=true \
        target_timeout:=3.0 \
        camera_frame:=rgbd_camera \
        enable_auto_explore:=false \
        close_approach_threshold:=0.95 \
        close_approach_timeout:=24.0 \
        close_approach_speed:=0.08 \
        close_approach_steer_gain:=0.40)"; then
      chosen_attempt="attempt2_dynamic_conservative_fallback"
      chosen_metric="${candidate_metric}"
      warn "Exp3 fallback profile selected (high-speed default unavailable in this run)"
    else
      err "Exp3 high-speed and fallback profiles both failed."
      return 1
    fi
  fi

  cp -f "${chosen_metric}" "${metric_out}"
  cp -f "${LOG_DIR}/exp3_target_follow_launch_${chosen_attempt}.log" "${launch_log}" 2>/dev/null || true
  cp -f "${LOG_DIR}/target_follow_metrics_${chosen_attempt}.log" "${LOG_DIR}/target_follow_metrics.log" 2>/dev/null || true
  info "Exp3 selected attempt: ${chosen_attempt}"
  info "Exp3 final metric file: ${metric_out}"
}

run_task67_pipeline() {
  local pipeline_log="$1"
  local map_name="$2"

  info "Running Task6/7 style pipeline (auto mapping + AMCL verify)"
  if ! bash "${REPO_ROOT}/run_full_pipeline_unitree.sh" \
      --headless \
      --skip-build \
      --timeout="${EXPLORE_TIMEOUT}" \
      --map-name="${map_name}" \
      >"${pipeline_log}" 2>&1; then
    err "run_full_pipeline_unitree.sh failed. See ${pipeline_log}"
    return 1
  fi

  cp -f "${NAV_DIR}/maps/${map_name}.yaml" "${METRIC_DIR}/" 2>/dev/null || true
  cp -f "${NAV_DIR}/maps/${map_name}.pgm" "${METRIC_DIR}/" 2>/dev/null || true
  cp -f "${NAV_DIR}/maps/amcl_report_unitree.txt" "${METRIC_DIR}/amcl_report_task67.txt" 2>/dev/null || true
  cp -f "${NAV_DIR}/maps/amcl_report_unitree.json" "${METRIC_DIR}/amcl_report_task67.json" 2>/dev/null || true
}

create_summary() {
  local out_json="${RUN_DIR}/summary.json"
  local out_md="${RUN_DIR}/summary.md"
  python3 - "$METRIC_DIR" "$RUN_DIR" "$out_json" "$out_md" <<'PY'
import json
import os
import sys
from datetime import datetime

metric_dir, run_dir, out_json, out_md = sys.argv[1:5]

def load_json(path):
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

simple_nav = load_json(os.path.join(metric_dir, "exp1_simple_nav.json"))
complex_nav = load_json(os.path.join(metric_dir, "exp2_complex_nav.json"))
follow = load_json(os.path.join(metric_dir, "exp3_target_follow.json"))
amcl = load_json(os.path.join(metric_dir, "amcl_report_task67.json"))

summary = {
    "generated_at": datetime.utcnow().isoformat() + "Z",
    "run_dir": run_dir,
    "experiments": {
        "exp1_simple_nav": simple_nav.get("summary") if simple_nav else None,
        "exp2_complex_nav": complex_nav.get("summary") if complex_nav else None,
        "exp3_target_follow": follow.get("summary") if follow else None,
        "exp4_task67_amcl": amcl,
    },
}

with open(out_json, "w", encoding="utf-8") as f:
    json.dump(summary, f, indent=2, ensure_ascii=False)

def val(d, k, default="N/A"):
    if not d:
        return default
    return d.get(k, default)

lines = []
lines.append("# Simulation Four-Experiment Summary")
lines.append("")
lines.append(f"- run_dir: `{run_dir}`")
lines.append("")
lines.append("## Exp1 Simple Obstacle Fixed-Point Navigation")
if simple_nav:
    s = simple_nav["summary"]
    lines.append(f"- success: {s['waypoints_succeeded']}/{s['waypoints_total']} ({s['success_rate']*100:.1f}%)")
    lines.append(f"- mean_goal_error_m: {s['mean_goal_error_m']:.4f}")
    lines.append(f"- mean_duration_s: {s['mean_duration_s']:.2f}")
    lines.append(f"- total_traveled_distance_m: {s['total_traveled_distance_m']:.2f}")
else:
    lines.append("- data: missing")
lines.append("")

lines.append("## Exp2 Complex Obstacle Fixed-Point Navigation")
if complex_nav:
    s = complex_nav["summary"]
    lines.append(f"- success: {s['waypoints_succeeded']}/{s['waypoints_total']} ({s['success_rate']*100:.1f}%)")
    lines.append(f"- mean_goal_error_m: {s['mean_goal_error_m']:.4f}")
    lines.append(f"- mean_duration_s: {s['mean_duration_s']:.2f}")
    lines.append(f"- total_traveled_distance_m: {s['total_traveled_distance_m']:.2f}")
else:
    lines.append("- data: missing")
lines.append("")

lines.append("## Exp3 Simple Obstacle Target Following")
if follow:
    s = follow["summary"]
    target_speed = s.get("target_speed_mean_unweighted_mps", s.get("target_speed_mean_mps", 0.0))
    lines.append(f"- sample_count: {s['sample_count']}")
    lines.append(f"- distance_mean_m: {s['distance_mean_m']:.4f}")
    lines.append(f"- standoff_rmse_m: {s['standoff_rmse_m']:.4f}")
    lines.append(f"- within_tolerance_rate: {s['within_tolerance_rate']*100:.1f}%")
    lines.append(f"- target_speed_mean_mps: {target_speed:.3f}")
else:
    lines.append("- data: missing")
lines.append("")

lines.append("## Exp4 Task6/7 (Autonomous Mapping + AMCL Verification)")
if amcl:
    waypoints = amcl.get("waypoint_results", [])
    nav_ok = sum(1 for r in waypoints if r.get("nav_success"))
    lines.append(f"- convergence_time_s: {amcl.get('convergence_time', 'N/A')}")
    lines.append(f"- waypoint_nav_success: {nav_ok}/{len(waypoints)}")
    stats = amcl.get("tracking_stats", {})
    if stats:
        lines.append(f"- amcl_mean_pos_err_m: {stats.get('mean_pos_err', 0):.4f}")
        lines.append(f"- amcl_max_pos_err_m: {stats.get('max_pos_err', 0):.4f}")
else:
    lines.append("- data: missing")
lines.append("")

with open(out_md, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
PY
}

main() {
  info "Run directory: ${RUN_DIR}"
  info "Logs: ${LOG_DIR}"
  info "Metrics: ${METRIC_DIR}"

  cleanup_ros

  ensure_workspace_sane

  if [[ "${SKIP_BUILD}" == "false" ]]; then
    info "Building catkin workspace..."
    if ! (
      cd "${WS_DIR}"
      catkin_make >"${LOG_DIR}/catkin_make.log" 2>&1
    ); then
      err "catkin_make failed. See ${LOG_DIR}/catkin_make.log"
      exit 1
    fi
  else
    info "Skipping build (--skip-build)"
  fi

  if [[ ! -f "${WS_DIR}/devel/setup.bash" ]]; then
    err "Missing ${WS_DIR}/devel/setup.bash after build. Cannot continue."
    exit 1
  fi
  source "${WS_DIR}/devel/setup.bash"

  info "Experiment 1/4: simple obstacle fixed-point navigation"
  run_nav_experiment "simple" "${SIMPLE_WORLD}" \
    "${LOG_DIR}/exp1_mapping_simple.log" \
    "${METRIC_DIR}/exp1_simple_nav.json" || exit 1

  info "Experiment 2/4: complex obstacle fixed-point navigation"
  run_nav_experiment "complex" "${COMPLEX_WORLD}" \
    "${LOG_DIR}/exp2_mapping_complex.log" \
    "${METRIC_DIR}/exp2_complex_nav.json" || exit 1

  info "Experiment 3/4: simple obstacle target following"
  run_target_follow_experiment \
    "${LOG_DIR}/exp3_target_follow_launch.log" \
    "${METRIC_DIR}/exp3_target_follow.json" || exit 1

  info "Experiment 4/4: task6/7 autonomous mapping + AMCL"
  run_task67_pipeline \
    "${LOG_DIR}/exp4_task67_pipeline.log" \
    "exp4_task67_map_${RUN_ID}" || exit 1

  create_summary

  info "All four experiments finished."
  info "Summary JSON: ${RUN_DIR}/summary.json"
  info "Summary MD:   ${RUN_DIR}/summary.md"
}

main "$@"
