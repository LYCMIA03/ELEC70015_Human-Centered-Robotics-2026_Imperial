#!/usr/bin/env bash
# ============================================================================
# run_full_pipeline_unitree.sh
# 使用 Unitree 4D Lidar L1 运行完整的自主建图 → AMCL 验证流水线
# 功能与 run_full_pipeline.sh 相同，但用 Unitree 雷达替代 SICK LMS200
#
# 用法:
#   ./run_full_pipeline_unitree.sh
#   ./run_full_pipeline_unitree.sh --headless
#   ./run_full_pipeline_unitree.sh --map-only
#   ./run_full_pipeline_unitree.sh --verify-only
#   ./run_full_pipeline_unitree.sh --timeout 600
#   ./run_full_pipeline_unitree.sh --map-name my_unitree_map
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="${SCRIPT_DIR}/catkin_ws"
NAV_PKG_DIR="${WS_DIR}/src/p3at_lms_navigation"

# Defaults
HEADLESS=false
SKIP_BUILD=false
MAP_ONLY=false
VERIFY_ONLY=false
EXPLORE_TIMEOUT=600
MAP_NAME="explored_map_unitree"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log() { echo -e "${CYAN}[$(date +%H:%M:%S)]${NC} $*"; }
die() { echo -e "${RED}[FATAL]${NC} $*" >&2; exit 1; }

for arg in "$@"; do
  case "$arg" in
    --headless)       HEADLESS=true ;;
    --skip-build)     SKIP_BUILD=true ;;
    --map-only)       MAP_ONLY=true ;;
    --verify-only)    VERIFY_ONLY=true ;;
    --timeout)        shift; EXPLORE_TIMEOUT="${2:-600}" ;;
    --timeout=*)      EXPLORE_TIMEOUT="${arg#*=}" ;;
    --map-name)       shift; MAP_NAME="${2:-explored_map_unitree}" ;;
    --map-name=*)     MAP_NAME="${arg#*=}" ;;
    --help|-h)
      echo "Usage: $0 [--headless] [--skip-build] [--map-only] [--verify-only] [--timeout N] [--map-name NAME]"
      exit 0 ;;
  esac
done

GUI_ARG="true"; $HEADLESS && GUI_ARG="false"

cleanup() {
  log "Cleaning up..."
  rosnode kill -a 2>/dev/null || true
  killall -q gzserver gzclient rosmaster roslaunch 2>/dev/null || true
  sleep 2
}
trap cleanup EXIT

# ===== Phase 0: Build =====
if ! $SKIP_BUILD; then
  log "${BOLD}Phase 0: Building catkin workspace...${NC}"
  cd "$WS_DIR" && catkin_make -j$(nproc) || die "catkin_make failed"
fi

source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"

MAP_FILE="${NAV_PKG_DIR}/maps/${MAP_NAME}"

# ===== Phase 1: Autonomous Mapping =====
if ! $VERIFY_ONLY; then
  log "${BOLD}Phase 1: Autonomous exploration mapping with Unitree 4D Lidar L1${NC}"
  log "  World:   complex_maze.world"
  log "  Lidar:   Unitree L1 → /unitree/scan (360° FOV)"
  log "  Timeout: ${EXPLORE_TIMEOUT}s"
  log "  Map:     ${MAP_FILE}"

  roslaunch p3at_lms_navigation auto_mapping_unitree.launch \
    gui:=$GUI_ARG \
    exploration_timeout:=$EXPLORE_TIMEOUT \
    map_save_name:=$MAP_NAME &
  MAP_PID=$!

  # Wait for exploration to finish (explorer node exits when done)
  while kill -0 $MAP_PID 2>/dev/null; do
    if ! rosnode list 2>/dev/null | grep -q autonomous_explorer; then
      log "Explorer finished, waiting 5s for map save..."
      sleep 5
      break
    fi
    sleep 5
  done

  kill $MAP_PID 2>/dev/null || true
  sleep 3
  cleanup

  if [[ -f "${MAP_FILE}.yaml" ]]; then
    log "${GREEN}✓ Map saved: ${MAP_FILE}.yaml${NC}"
  else
    die "Map file not found: ${MAP_FILE}.yaml"
  fi
fi

$MAP_ONLY && { log "${GREEN}Done (map-only mode).${NC}"; exit 0; }

# ===== Phase 2: AMCL Verification =====
log "${BOLD}Phase 2: AMCL localization verification with Unitree 4D Lidar L1${NC}"
log "  Map:   ${MAP_FILE}.yaml"
log "  Lidar: Unitree L1 → /unitree/scan"

VERIFY_MAP="${MAP_FILE}.yaml"
[[ -f "$VERIFY_MAP" ]] || VERIFY_MAP="${NAV_PKG_DIR}/maps/complex_maze_map.yaml"

roslaunch p3at_lms_navigation auto_amcl_verify_unitree.launch \
  gui:=$GUI_ARG \
  map_file:=$VERIFY_MAP &
VERIFY_PID=$!

while kill -0 $VERIFY_PID 2>/dev/null; do
  if ! rosnode list 2>/dev/null | grep -q amcl_verifier; then
    log "Verifier finished."
    sleep 3
    break
  fi
  sleep 5
done

kill $VERIFY_PID 2>/dev/null || true

REPORT="${NAV_PKG_DIR}/maps/amcl_report_unitree.txt"
if [[ -f "$REPORT" ]]; then
  log "${GREEN}✓ AMCL report generated: ${REPORT}${NC}"
  echo ""
  cat "$REPORT"
else
  log "${YELLOW}Report not found: ${REPORT}${NC}"
fi

log "${GREEN}${BOLD}Pipeline complete (Unitree 4D Lidar L1).${NC}"
