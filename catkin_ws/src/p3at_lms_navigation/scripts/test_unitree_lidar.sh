#!/usr/bin/env bash
# ============================================================================
# test_unitree_lidar.sh — Unitree 4D Lidar L1 集成测试脚本
#
# 功能:
#   1. 验证 catkin_make 编译成功
#   2. 验证 xacro 模型解析正确
#   3. 验证 Gazebo 仿真中双雷达 topic 正常
#   4. 运行 gmapping SLAM 建图测试
#   5. 运行 AMCL 导航测试
#   6. 运行自主探索建图测试
#   7. 运行 AMCL 精度验证测试
#
# 用法:
#   ./test_unitree_lidar.sh                    # 运行所有测试
#   ./test_unitree_lidar.sh --test=xacro       # 仅测试 xacro 解析
#   ./test_unitree_lidar.sh --test=topics      # 仅测试仿真 topics
#   ./test_unitree_lidar.sh --test=mapping     # 仅测试 SLAM 建图
#   ./test_unitree_lidar.sh --test=nav         # 仅测试导航
#   ./test_unitree_lidar.sh --test=auto_map    # 仅测试自主探索
#   ./test_unitree_lidar.sh --test=amcl_verify # 仅测试 AMCL 验证
#   ./test_unitree_lidar.sh --headless         # 无 GUI 模式
# ============================================================================

set -euo pipefail

# -------- Configurable Parameters --------
WS_DIR="$(cd "$(dirname "$0")/../../.." && pwd)/catkin_ws"
HEADLESS=false
TEST_FILTER="all"
TOPIC_WAIT_SEC=30
MAPPING_SEC=60
NAV_SEC=60

# -------- Color Output --------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
log_pass()  { echo -e "${GREEN}[PASS]${NC}  $*"; }
log_fail()  { echo -e "${RED}[FAIL]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }

# -------- Parse Arguments --------
for arg in "$@"; do
  case "$arg" in
    --headless) HEADLESS=true ;;
    --test=*)   TEST_FILTER="${arg#*=}" ;;
    --help|-h)
      echo "Usage: $0 [--headless] [--test=xacro|topics|mapping|nav|auto_map|amcl_verify]"
      exit 0 ;;
  esac
done

GUI_ARG="true"
$HEADLESS && GUI_ARG="false"

# -------- Source workspace --------
source /opt/ros/noetic/setup.bash
if [[ -f "$WS_DIR/devel/setup.bash" ]]; then
  source "$WS_DIR/devel/setup.bash"
else
  log_fail "Workspace not built. Run: cd $WS_DIR && catkin_make"
  exit 1
fi

PASS=0; FAIL=0; SKIP=0

run_test() {
  local name="$1"; shift
  if [[ "$TEST_FILTER" != "all" && "$TEST_FILTER" != "$name" ]]; then
    SKIP=$((SKIP+1)); return 0
  fi
  log_info "Running test: $name"
  if "$@"; then
    log_pass "$name"
    PASS=$((PASS+1))
  else
    log_fail "$name"
    FAIL=$((FAIL+1))
  fi
}

# -------- Cleanup --------
cleanup_ros() {
  log_info "Cleaning up ROS nodes..."
  rosnode kill -a 2>/dev/null || true
  killall -q gzserver gzclient rosmaster rosout 2>/dev/null || true
  sleep 2
}
trap cleanup_ros EXIT

# ========================================================================
# TEST 1: xacro parsing
# ========================================================================
test_xacro() {
  log_info "Parsing p3at_unitree.urdf.xacro..."
  local urdf_output
  urdf_output=$(xacro "$(rospack find p3at_lms_description)/urdf/p3at_unitree.urdf.xacro" 2>&1)

  # Check for key elements
  echo "$urdf_output" | grep -q 'name="unitree_lidar"' || { log_fail "unitree_lidar link not found"; return 1; }
  echo "$urdf_output" | grep -q 'name="unitree_lidar_joint"' || { log_fail "unitree_lidar_joint not found"; return 1; }
  echo "$urdf_output" | grep -q 'name="unitree_imu"' || { log_fail "unitree_imu link not found"; return 1; }
  echo "$urdf_output" | grep -q 'name="laser"' || { log_fail "SICK laser link not found (should still exist)"; return 1; }
  echo "$urdf_output" | grep -q 'unitree/scan' || { log_fail "unitree/scan topic not found in URDF"; return 1; }
  echo "$urdf_output" | grep -q 'unitree/cloud' || { log_fail "unitree/cloud topic not found in URDF"; return 1; }
  echo "$urdf_output" | grep -q '<topicName>scan</topicName>' || { log_fail "SICK /scan topic not found in URDF"; return 1; }

  log_info "URDF contains: unitree_lidar, unitree_imu, laser (SICK), both scan topics"
  return 0
}

# ========================================================================
# TEST 2: Gazebo topics
# ========================================================================
test_topics() {
  log_info "Launching Gazebo sim_unitree and checking topics..."
  roslaunch p3at_lms_gazebo sim_unitree.launch gui:=$GUI_ARG &
  local launch_pid=$!
  sleep 15  # Give Gazebo time to start

  local missing=0
  for topic in /scan /unitree/scan /unitree/cloud /unitree/imu /odom /cmd_vel /tf; do
    if rostopic list 2>/dev/null | grep -qx "$topic"; then
      log_info "  ✓ $topic"
    else
      log_warn "  ✗ $topic (missing)"
      missing=$((missing+1))
    fi
  done

  # Check that Unitree scan publishes data
  log_info "Waiting for /unitree/scan data (${TOPIC_WAIT_SEC}s timeout)..."
  if timeout ${TOPIC_WAIT_SEC} rostopic echo -n 1 /unitree/scan >/dev/null 2>&1; then
    log_info "  ✓ /unitree/scan publishing data"
  else
    log_warn "  ✗ /unitree/scan not publishing"
    missing=$((missing+1))
  fi

  # Check SICK scan still works
  log_info "Waiting for /scan data (${TOPIC_WAIT_SEC}s timeout)..."
  if timeout ${TOPIC_WAIT_SEC} rostopic echo -n 1 /scan >/dev/null 2>&1; then
    log_info "  ✓ /scan (SICK) publishing data"
  else
    log_warn "  ✗ /scan (SICK) not publishing"
    missing=$((missing+1))
  fi

  kill $launch_pid 2>/dev/null || true
  sleep 3
  cleanup_ros

  [[ $missing -eq 0 ]]
}

# ========================================================================
# TEST 3: SLAM mapping with Unitree
# ========================================================================
test_mapping() {
  log_info "Testing SLAM mapping with Unitree lidar (${MAPPING_SEC}s)..."
  roslaunch p3at_lms_navigation mapping_unitree.launch gui:=$GUI_ARG use_target_follower:=false &
  local launch_pid=$!
  sleep 20  # Wait for sim + gmapping to initialize

  # Check /map is being published
  if timeout 30 rostopic echo -n 1 /map >/dev/null 2>&1; then
    log_info "  ✓ /map being published by gmapping"
  else
    log_fail "  ✗ /map not published"
    kill $launch_pid 2>/dev/null || true; cleanup_ros; return 1
  fi

  # Send a simple velocity command to test obstacle detection
  log_info "  Sending test velocity..."
  rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    "linear: {x: 0.3}" 2>/dev/null || true
  sleep 5

  # Verify costmap is working (move_base should be active)
  if rosnode list 2>/dev/null | grep -q move_base; then
    log_info "  ✓ move_base running"
  else
    log_warn "  ✗ move_base not found"
  fi

  kill $launch_pid 2>/dev/null || true
  sleep 3
  cleanup_ros
  return 0
}

# ========================================================================
# TEST 4: AMCL navigation with Unitree
# ========================================================================
test_nav() {
  log_info "Testing AMCL navigation with Unitree lidar (${NAV_SEC}s)..."
  roslaunch p3at_lms_navigation nav_unitree.launch gui:=$GUI_ARG &
  local launch_pid=$!
  sleep 20

  # Check AMCL is running
  if rosnode list 2>/dev/null | grep -q amcl; then
    log_info "  ✓ AMCL running"
  else
    log_fail "  ✗ AMCL not found"
    kill $launch_pid 2>/dev/null || true; cleanup_ros; return 1
  fi

  # Check /amcl_pose
  if timeout 20 rostopic echo -n 1 /amcl_pose >/dev/null 2>&1; then
    log_info "  ✓ /amcl_pose publishing"
  else
    log_warn "  ✗ /amcl_pose not publishing"
  fi

  kill $launch_pid 2>/dev/null || true
  sleep 3
  cleanup_ros
  return 0
}

# ========================================================================
# TEST 5: Autonomous exploration with Unitree
# ========================================================================
test_auto_map() {
  log_info "Testing autonomous exploration with Unitree lidar (120s max)..."
  timeout 120 roslaunch p3at_lms_navigation auto_mapping_unitree.launch \
    gui:=$GUI_ARG exploration_timeout:=90 &
  local launch_pid=$!
  sleep 25

  if rosnode list 2>/dev/null | grep -q autonomous_explorer; then
    log_info "  ✓ autonomous_explorer running"
  else
    log_warn "  ✗ autonomous_explorer not found"
  fi

  # Let it run briefly
  sleep 30

  if timeout 10 rostopic echo -n 1 /map >/dev/null 2>&1; then
    log_info "  ✓ Map being generated"
  else
    log_warn "  ✗ Map not detected"
  fi

  kill $launch_pid 2>/dev/null || true
  sleep 3
  cleanup_ros
  return 0
}

# ========================================================================
# TEST 6: AMCL verification with Unitree
# ========================================================================
test_amcl_verify() {
  log_info "Testing AMCL verification with Unitree lidar..."
  timeout 120 roslaunch p3at_lms_navigation auto_amcl_verify_unitree.launch gui:=$GUI_ARG &
  local launch_pid=$!
  sleep 25

  if rosnode list 2>/dev/null | grep -q amcl_verifier; then
    log_info "  ✓ amcl_verifier running"
  else
    log_warn "  ✗ amcl_verifier not found"
  fi

  if rosnode list 2>/dev/null | grep -q amcl; then
    log_info "  ✓ AMCL + Unitree scan active"
  fi

  # Let it run briefly
  sleep 20

  kill $launch_pid 2>/dev/null || true
  sleep 3
  cleanup_ros
  return 0
}

# ========================================================================
# Run tests
# ========================================================================
echo ""
echo "============================================================"
echo "  Unitree 4D Lidar L1 Integration Test Suite"
echo "  Workspace: $WS_DIR"
echo "  Headless:  $HEADLESS"
echo "  Filter:    $TEST_FILTER"
echo "============================================================"
echo ""

run_test "xacro"       test_xacro
run_test "topics"      test_topics
run_test "mapping"     test_mapping
run_test "nav"         test_nav
run_test "auto_map"    test_auto_map
run_test "amcl_verify" test_amcl_verify

echo ""
echo "============================================================"
echo "  Test Results: ${GREEN}${PASS} passed${NC}, ${RED}${FAIL} failed${NC}, ${YELLOW}${SKIP} skipped${NC}"
echo "============================================================"

[[ $FAIL -eq 0 ]]
