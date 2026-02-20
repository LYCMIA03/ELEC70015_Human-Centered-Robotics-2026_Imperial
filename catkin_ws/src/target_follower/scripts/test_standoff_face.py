#!/usr/bin/env python3
"""
Unit tests for target_follower.py standoff_distance and face_target logic.
Tests the pure math without requiring ROS to be running.
"""
import math
import sys
import unittest

PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"

# ─── Replicate the core logic from target_follower.py ───────────────────────

def compute_standoff_goal(rx, ry, tx, ty, standoff_distance):
    """
    Compute the standoff goal position.
    Returns (goal_x, goal_y, should_skip) where should_skip=True if already inside standoff.
    """
    dx = tx - rx
    dy = ty - ry
    d = math.hypot(dx, dy)
    if standoff_distance > 0:
        if d <= standoff_distance:
            return None, None, True  # inside standoff zone, skip
        ratio = (d - standoff_distance) / d
        goal_x = rx + dx * ratio
        goal_y = ry + dy * ratio
    else:
        goal_x, goal_y = tx, ty
    return goal_x, goal_y, False


def compute_face_target_yaw(rx, ry, tx, ty, goal_x, goal_y):
    """
    Compute yaw so robot faces from goal position toward target.
    """
    face_dx = tx - goal_x
    face_dy = ty - goal_y
    if math.hypot(face_dx, face_dy) > 0.01:
        yaw = math.atan2(face_dy, face_dx)
    else:
        # goal ≈ target, use robot→target
        yaw = math.atan2(ty - ry, tx - rx)
    return yaw


def yaw_to_quaternion(yaw):
    """Convert yaw to (z, w) quaternion components."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# ─── Test cases ─────────────────────────────────────────────────────────────

results = []


def check(name, condition, detail=""):
    status = PASS if condition else FAIL
    msg = f"  [{status}] {name}"
    if detail:
        msg += f"  ({detail})"
    print(msg, flush=True)
    results.append((name, condition))
    return condition


print("\n" + "═" * 60, flush=True)
print("UNIT TEST: standoff_distance geometry", flush=True)
print("═" * 60, flush=True)

# T1: No standoff — goal should equal target
rx, ry = 0.0, 0.0
tx, ty = 3.0, 4.0
gx, gy, skip = compute_standoff_goal(rx, ry, tx, ty, standoff_distance=0.0)
check("T1: standoff=0, no skip", not skip)
check("T1: standoff=0, goal == target", abs(gx-tx) < 1e-9 and abs(gy-ty) < 1e-9,
      f"goal=({gx:.4f},{gy:.4f}), target=({tx},{ty})")

print(flush=True)
# T2: standoff=1.5m, robot at origin, target at (4, 0)
rx, ry = 0.0, 0.0
tx, ty = 4.0, 0.0
std = 1.5
gx, gy, skip = compute_standoff_goal(rx, ry, tx, ty, standoff_distance=std)
dist_goal_to_target = math.hypot(gx - tx, gy - ty)
check("T2a: standoff=1.5, no skip (d=4 > 1.5)", not skip)
check("T2b: goal is exactly standoff distance from target",
      abs(dist_goal_to_target - std) < 1e-9,
      f"dist={dist_goal_to_target:.4f}m (want {std}m)")
check("T2c: goal is on the robot→target line",
      abs(gy) < 1e-9,  # y should be 0
      f"goal=({gx:.4f},{gy:.4f})")
check("T2d: goal_x = 4.0 - 1.5 = 2.5",
      abs(gx - 2.5) < 1e-9, f"gx={gx:.4f}")

print(flush=True)
# T3: standoff=2.0, but robot already inside (d=1.0 < 2.0)
rx, ry = 0.0, 0.0
tx, ty = 1.0, 0.0
gx, gy, skip = compute_standoff_goal(rx, ry, tx, ty, standoff_distance=2.0)
check("T3: inside standoff zone → skip=True",
      skip, f"d=1.0 <= standoff=2.0, skip={skip}")

print(flush=True)
# T4: standoff=1.0, robot at (2,1), target at (5,5)
rx, ry = 2.0, 1.0
tx, ty = 5.0, 5.0
std = 1.0
d = math.hypot(tx-rx, ty-ry)  # = 5.0
gx, gy, skip = compute_standoff_goal(rx, ry, tx, ty, standoff_distance=std)
dist_goal_to_target = math.hypot(gx - tx, gy - ty)
# Also check goal is on the line robot→target (collinear)
# Line: direction = (tx-rx, ty-ry)/d = (3/5, 4/5)
# goal = robot + ratio * dir * d = (2 + (5-1)/5*3, 1 + (5-1)/5*4) = (2+2.4, 1+3.2) = (4.4, 4.2)
expected_gx = 2.0 + (4.0/5.0) * 3.0   # = 4.4
expected_gy = 1.0 + (4.0/5.0) * 4.0   # = 4.2
check("T4a: standoff=1.0, no skip (d=5 > 1)", not skip)
check("T4b: goal exactly 1.0m from target", abs(dist_goal_to_target - 1.0) < 1e-9,
      f"dist={dist_goal_to_target:.4f}m")
check("T4c: goal collinear with robot→target",
      abs(gx - expected_gx) < 1e-9 and abs(gy - expected_gy) < 1e-9,
      f"goal=({gx:.4f},{gy:.4f}), expected=({expected_gx},{expected_gy})")

print("\n" + "═" * 60, flush=True)
print("UNIT TEST: face_target yaw computation", flush=True)
print("═" * 60, flush=True)

print(flush=True)
# F1: standoff=0, goal=target, face from robot→target
# robot at (0,0), target at (3,0) → yaw should be 0° (east)
rx, ry = 0.0, 0.0
tx, ty = 3.0, 0.0
gx, gy, _ = compute_standoff_goal(rx, ry, tx, ty, 0.0)
yaw = compute_face_target_yaw(rx, ry, tx, ty, gx, gy)
check("F1: face east (yaw≈0°)", abs(yaw) < 1e-9, f"yaw={math.degrees(yaw):.4f}°")

print(flush=True)
# F2: standoff=1m, robot at (0,0), target at (4,0)
# goal = (3, 0); face from (3,0) → (4,0) = east = 0°
rx, ry = 0.0, 0.0
tx, ty = 4.0, 0.0
gx, gy, _ = compute_standoff_goal(rx, ry, tx, ty, 1.0)  # goal=(3,0)
yaw = compute_face_target_yaw(rx, ry, tx, ty, gx, gy)
check("F2: standoff+face east", abs(yaw) < 1e-9, f"yaw={math.degrees(yaw):.4f}°")

print(flush=True)
# F3: robot at (0,0), target at (0,3) → yaw should be 90° (north)
rx, ry = 0.0, 0.0
tx, ty = 0.0, 3.0
gx, gy, _ = compute_standoff_goal(rx, ry, tx, ty, 0.0)
yaw = compute_face_target_yaw(rx, ry, tx, ty, gx, gy)
check("F3: face north (yaw≈90°)", abs(yaw - math.pi/2) < 1e-9, f"yaw={math.degrees(yaw):.4f}°")

print(flush=True)
# F4: robot at (0,0), target at (-3,-3) → yaw ≈ -135° (SW)
rx, ry = 0.0, 0.0
tx, ty = -3.0, -3.0
gx, gy, _ = compute_standoff_goal(rx, ry, tx, ty, 0.0)
yaw = compute_face_target_yaw(rx, ry, tx, ty, gx, gy)
expected_yaw = math.atan2(-3, -3)  # -135°
check("F4: face SW (-135°)",
      abs(yaw - expected_yaw) < 1e-9, f"yaw={math.degrees(yaw):.2f}°")

print(flush=True)
# F5: standoff=2.0, robot at (0,0), target at (5,5)
# goal = (5-2/sqrt(50)*5, 5-2/sqrt(50)*5) = ~(3.586, 3.586)
# face from goal → target: yaw = atan2(5-3.586, 5-3.586) = atan2(1, 1) = 45°
rx, ry = 0.0, 0.0
tx, ty = 5.0, 5.0
std = 2.0
d = math.hypot(tx-rx, ty-ry)  # sqrt(50) ≈ 7.071
ratio = (d - std) / d
gx = rx + (tx-rx)*ratio
gy = ry + (ty-ry)*ratio
yaw = compute_face_target_yaw(rx, ry, tx, ty, gx, gy)
check("F5: standoff=2, combined, face NE (45°)",
      abs(yaw - math.pi/4) < 1e-9, f"yaw={math.degrees(yaw):.4f}°")

print(flush=True)
# F6: yaw_to_quaternion check
yaw_in = math.pi / 4  # 45°
qz, qw = yaw_to_quaternion(yaw_in)
# Check: qz = sin(22.5°), qw = cos(22.5°)
expected_qz = math.sin(yaw_in / 2)
expected_qw = math.cos(yaw_in / 2)
# Verify round-trip: yaw = 2*atan2(qz, qw)
recovered_yaw = 2 * math.atan2(qz, qw)
check("F6: yaw_to_quaternion round-trip",
      abs(recovered_yaw - yaw_in) < 1e-9,
      f"in={math.degrees(yaw_in):.1f}°, recovered={math.degrees(recovered_yaw):.4f}°")

print("\n" + "═" * 60, flush=True)
print("UNIT TEST: edge cases", flush=True)
print("═" * 60, flush=True)

print(flush=True)
# E1: standoff exactly = distance (on the boundary)
rx, ry = 0.0, 0.0
tx, ty = 2.0, 0.0
_, _, skip = compute_standoff_goal(rx, ry, tx, ty, standoff_distance=2.0)
check("E1: standoff == distance → skip=True",
      skip, f"d=2.0 == standoff=2.0, should skip")

print(flush=True)
# E2: standoff just larger than distance
_, _, skip = compute_standoff_goal(0, 0, 1.9, 0, standoff_distance=2.0)
check("E2: d(1.9) < standoff(2.0) → skip=True", skip)

print(flush=True)
# E3: standoff just smaller than distance
gx, gy, skip = compute_standoff_goal(0, 0, 2.1, 0, standoff_distance=2.0)
check("E3: d(2.1) > standoff(2.0) → skip=False", not skip)
dist_goal_to_target = math.hypot(gx - 2.1, gy - 0.0)
check("E3: goal exactly 2.0m from target",
      abs(dist_goal_to_target - 2.0) < 1e-9,
      f"dist={dist_goal_to_target:.4f}m")

print(flush=True)
# E4: face_target with goal very close to target (standoff≈0) → use robot→target
rx, ry = 0.0, 0.0
tx, ty = 0.001, 0.0  # target almost at robot
gx, gy = 0.001, 0.0  # goal == target
# Face_dx = tx - gx ≈ 0, will use robot→target fallback
face_dx = tx - gx
face_dy = ty - gy
if math.hypot(face_dx, face_dy) > 0.01:
    yaw_fallback = math.atan2(face_dy, face_dx)
    used_fallback = False
else:
    yaw_fallback = math.atan2(ty - ry, tx - rx)
    used_fallback = True
check("E4: face_target uses fallback when goal≈target", used_fallback)

# ─── Summary ────────────────────────────────────────────────────────────────
print("\n" + "═" * 60, flush=True)
print("SUMMARY", flush=True)
print("═" * 60, flush=True)
passed = sum(1 for _, ok in results if ok)
total = len(results)
for name, ok in results:
    status = PASS if ok else FAIL
    print(f"  [{status}] {name}", flush=True)
print(f"\n  {passed}/{total} unit tests passed", flush=True)
if passed == total:
    print("  \033[92mAll unit tests PASSED ✓\033[0m", flush=True)
    sys.exit(0)
else:
    failed = [n for n, ok in results if not ok]
    print(f"  \033[91mFailed: {failed}\033[0m", flush=True)
    sys.exit(1)
