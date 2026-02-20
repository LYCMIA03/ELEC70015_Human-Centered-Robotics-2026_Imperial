#!/bin/bash
# ==============================================================================
#   run_full_pipeline.sh — One-click autonomous mapping + AMCL verification
#
#   This script orchestrates the complete pipeline:
#     Phase 1: Build the catkin workspace
#     Phase 2: Launch Gazebo + gmapping + autonomous frontier exploration
#              Wait for exploration to complete, map is auto-saved.
#     Phase 3: Launch Gazebo + map_server + AMCL + waypoint verification
#              Generate AMCL accuracy report.
#
#   Usage:
#     ./run_full_pipeline.sh                  # Full pipeline with GUI
#     ./run_full_pipeline.sh --headless       # No Gazebo/RViz GUI
#     ./run_full_pipeline.sh --skip-build     # Skip catkin_make
#     ./run_full_pipeline.sh --map-only       # Only run mapping phase
#     ./run_full_pipeline.sh --verify-only    # Only run AMCL verification
#     ./run_full_pipeline.sh --timeout 300    # Set exploration timeout (seconds)
#
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/catkin_ws"
NAV_PKG_DIR="$WS_DIR/src/p3at_lms_navigation"
MAP_DIR="$NAV_PKG_DIR/maps"

# Defaults
GUI=true
SKIP_BUILD=false
MAP_ONLY=false
VERIFY_ONLY=false
EXPLORE_TIMEOUT=600
MAP_NAME="explored_map"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'  # No Color

log_info()  { echo -e "${CYAN}[INFO]${NC}  $1"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_err()   { echo -e "${RED}[ERROR]${NC} $1"; }

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --headless      Run without Gazebo/RViz GUI"
    echo "  --skip-build    Skip catkin_make build step"
    echo "  --map-only      Only run the autonomous mapping phase"
    echo "  --verify-only   Only run AMCL verification (requires existing map)"
    echo "  --timeout SEC   Exploration timeout in seconds (default: 600)"
    echo "  --map-name NAME Map file name without extension (default: explored_map)"
    echo "  -h, --help      Show this help message"
    exit 0
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --headless)     GUI=false; shift ;;
        --skip-build)   SKIP_BUILD=true; shift ;;
        --map-only)     MAP_ONLY=true; shift ;;
        --verify-only)  VERIFY_ONLY=true; shift ;;
        --timeout)      EXPLORE_TIMEOUT="$2"; shift 2 ;;
        --map-name)     MAP_NAME="$2"; shift 2 ;;
        -h|--help)      usage ;;
        *) log_err "Unknown option: $1"; usage ;;
    esac
done

# ==============================================================================
# Helper: cleanup function to kill ROS processes
# ==============================================================================
cleanup() {
    log_info "Cleaning up ROS processes..."
    rosnode kill -a 2>/dev/null || true
    sleep 2
    killall -q gzserver gzclient rosmaster roscore roslaunch 2>/dev/null || true
    sleep 1
    log_ok "Cleanup complete."
}
trap cleanup EXIT

# ==============================================================================
# Phase 0: Source ROS and build workspace
# ==============================================================================
echo ""
echo "=============================================================="
echo "  P3-AT Autonomous Mapping & AMCL Verification Pipeline"
echo "=============================================================="
echo ""

# Source ROS
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
else
    log_err "No ROS installation found in /opt/ros/"
    exit 1
fi

if [ "$SKIP_BUILD" = false ]; then
    log_info "Phase 0: Building catkin workspace..."
    cd "$WS_DIR"
    catkin_make
    log_ok "Build successful."
else
    log_info "Phase 0: Skipping build (--skip-build)."
fi

# Source workspace
source "$WS_DIR/devel/setup.bash"
log_ok "Workspace sourced."

# Make scripts executable
chmod +x "$NAV_PKG_DIR/scripts/autonomous_explorer.py" 2>/dev/null || true
chmod +x "$NAV_PKG_DIR/scripts/amcl_verifier.py" 2>/dev/null || true
chmod +x "$NAV_PKG_DIR/scripts/waypoint_test.py" 2>/dev/null || true

# Create maps directory if needed
mkdir -p "$MAP_DIR"

# ==============================================================================
# Phase 1: Autonomous Mapping
# ==============================================================================
if [ "$VERIFY_ONLY" = false ]; then
    echo ""
    echo "=============================================================="
    echo "  Phase 1: Autonomous Exploration & Mapping"
    echo "  World: complex_maze.world"
    echo "  Timeout: ${EXPLORE_TIMEOUT}s"
    echo "  GUI: $GUI"
    echo "=============================================================="
    echo ""

    # Start roscore if not running
    if ! pgrep -x roscore > /dev/null; then
        log_info "Starting roscore..."
        roscore &
        sleep 3
    fi

    log_info "Launching auto_mapping.launch..."
    roslaunch p3at_lms_navigation auto_mapping.launch \
        gui:=$GUI \
        exploration_timeout:=$EXPLORE_TIMEOUT \
        map_save_name:=$MAP_NAME \
        --wait &

    MAPPING_PID=$!

    # Wait for the explorer node to finish (it exits when done)
    log_info "Waiting for autonomous exploration to complete..."
    log_info "  (This may take up to ${EXPLORE_TIMEOUT}s)"
    log_info "  Press Ctrl+C to abort and proceed to cleanup."

    # Monitor the explorer node
    EXPLORER_DONE=false
    WAIT_START=$(date +%s)
    MAX_WAIT=$((EXPLORE_TIMEOUT + 120))  # Extra buffer for startup

    while true; do
        sleep 10
        ELAPSED=$(( $(date +%s) - WAIT_START ))

        # Check if explorer node is still running
        if ! rosnode list 2>/dev/null | grep -q "autonomous_explorer"; then
            # Could be: hasn't started yet, or has finished
            if [ $ELAPSED -gt 30 ]; then
                log_ok "Explorer node finished. (elapsed: ${ELAPSED}s)"
                EXPLORER_DONE=true
                break
            fi
        fi

        # Check if map file was created
        if [ -f "${MAP_DIR}/${MAP_NAME}.pgm" ] && [ $ELAPSED -gt 60 ]; then
            # Map file exists and we've been running a while — check if explorer is gone
            if ! rosnode list 2>/dev/null | grep -q "autonomous_explorer"; then
                log_ok "Map file detected and explorer finished."
                EXPLORER_DONE=true
                break
            fi
        fi

        if [ $ELAPSED -gt $MAX_WAIT ]; then
            log_warn "Maximum wait time exceeded. Proceeding..."
            break
        fi

        # Progress update
        COVERAGE=$(rostopic echo -n 1 /map 2>/dev/null | grep -c "data" || echo "?")
        log_info "  Exploring... (${ELAPSED}s elapsed)"
    done

    # Kill the mapping launch
    log_info "Shutting down mapping phase..."
    kill $MAPPING_PID 2>/dev/null || true
    wait $MAPPING_PID 2>/dev/null || true
    sleep 3

    # Verify map was saved
    if [ -f "${MAP_DIR}/${MAP_NAME}.pgm" ]; then
        log_ok "Map saved: ${MAP_DIR}/${MAP_NAME}.pgm"
        log_ok "Map YAML: ${MAP_DIR}/${MAP_NAME}.yaml"
    else
        log_err "Map file not found! Exploration may have failed."
        if [ "$MAP_ONLY" = true ]; then
            exit 1
        fi
    fi

    # Brief pause between phases
    log_info "Pausing 5s between phases..."
    cleanup
    sleep 5
fi

# ==============================================================================
# Phase 2: AMCL Verification
# ==============================================================================
if [ "$MAP_ONLY" = false ]; then
    echo ""
    echo "=============================================================="
    echo "  Phase 2: AMCL Localization Verification"
    echo "  Map: ${MAP_DIR}/${MAP_NAME}.yaml"
    echo "  GUI: $GUI"
    echo "=============================================================="
    echo ""

    MAP_FILE="${MAP_DIR}/${MAP_NAME}.yaml"
    if [ ! -f "$MAP_FILE" ]; then
        log_err "Map file not found: $MAP_FILE"
        log_err "Run the mapping phase first, or specify --map-name."
        exit 1
    fi

    # Start roscore if not running
    if ! pgrep -x roscore > /dev/null; then
        log_info "Starting roscore..."
        roscore &
        sleep 3
    fi

    log_info "Launching auto_amcl_verify.launch..."
    roslaunch p3at_lms_navigation auto_amcl_verify.launch \
        gui:=$GUI \
        map_file:=$MAP_FILE \
        --wait &

    VERIFY_PID=$!

    # Wait for verification to complete
    log_info "Waiting for AMCL verification to complete..."
    log_info "  (This typically takes 3-8 minutes)"

    WAIT_START=$(date +%s)
    MAX_VERIFY_WAIT=900  # 15 minutes max

    while true; do
        sleep 10
        ELAPSED=$(( $(date +%s) - WAIT_START ))

        if ! rosnode list 2>/dev/null | grep -q "amcl_verifier"; then
            if [ $ELAPSED -gt 30 ]; then
                log_ok "AMCL verifier finished. (elapsed: ${ELAPSED}s)"
                break
            fi
        fi

        if [ $ELAPSED -gt $MAX_VERIFY_WAIT ]; then
            log_warn "Maximum verification wait time exceeded."
            break
        fi

        log_info "  Verifying... (${ELAPSED}s elapsed)"
    done

    # Kill the verification launch
    log_info "Shutting down verification phase..."
    kill $VERIFY_PID 2>/dev/null || true
    wait $VERIFY_PID 2>/dev/null || true
    sleep 2

    # Display results
    REPORT_FILE="${MAP_DIR}/amcl_report.txt"
    if [ -f "$REPORT_FILE" ]; then
        echo ""
        echo "=============================================================="
        echo "  AMCL VERIFICATION RESULTS"
        echo "=============================================================="
        cat "$REPORT_FILE"
    else
        log_warn "Report file not found: $REPORT_FILE"
    fi
fi

# ==============================================================================
# Summary
# ==============================================================================
echo ""
echo "=============================================================="
echo "  PIPELINE COMPLETE"
echo "=============================================================="
echo ""
echo "  Generated files:"
[ "$VERIFY_ONLY" = false ] && echo "    Map:     ${MAP_DIR}/${MAP_NAME}.pgm"
[ "$VERIFY_ONLY" = false ] && echo "    Map cfg: ${MAP_DIR}/${MAP_NAME}.yaml"
[ "$MAP_ONLY" = false ]    && echo "    Report:  ${MAP_DIR}/amcl_report.txt"
[ "$MAP_ONLY" = false ]    && echo "    JSON:    ${MAP_DIR}/amcl_report.json"
echo ""
echo "  To re-run navigation on the saved map:"
echo "    roslaunch p3at_lms_navigation nav.launch map_file:=${MAP_DIR}/${MAP_NAME}.yaml"
echo ""
log_ok "Done."
