#!/usr/bin/env python3
"""Frontier planner utilities for map-driven exploration.

This module is intentionally ROS-light: callers push map/costmap updates and
request candidate exploration goals.  It does not own move_base goals.
"""

import math
from collections import deque

import numpy as np


class FrontierPlanner(object):
    """Detect frontier clusters and return short-horizon approach goals."""

    FREE = 0
    UNKNOWN = -1

    def __init__(
        self,
        min_frontier_size=8,
        occupied_threshold=65,
        map_clearance_m=0.35,
        costmap_lethal_threshold=70,
        approach_pull_m=1.0,
        min_approach_cells=2,
        blacklist_radius_m=0.5,
        max_blacklist_size=200,
    ):
        self.min_frontier_size = max(1, int(min_frontier_size))
        self.occupied_threshold = int(occupied_threshold)
        self.map_clearance_m = float(map_clearance_m)
        self.costmap_lethal_threshold = int(costmap_lethal_threshold)
        self.approach_pull_m = float(approach_pull_m)
        self.min_approach_cells = max(1, int(min_approach_cells))
        self.blacklist_radius_m = float(blacklist_radius_m)
        self.blacklisted_goals = deque(maxlen=max(16, int(max_blacklist_size)))

        self.map_info = None
        self.map_data = None
        self.map_stamp = None

        self.costmap_info = None
        self.costmap_data = None
        self.costmap_stamp = None

    def has_map(self):
        return self.map_info is not None and self.map_data is not None

    def update_map(self, msg):
        h = int(msg.info.height)
        w = int(msg.info.width)
        self.map_info = msg.info
        self.map_stamp = msg.header.stamp
        self.map_data = np.array(msg.data, dtype=np.int16).reshape((h, w))

    def update_costmap(self, msg):
        h = int(msg.info.height)
        w = int(msg.info.width)
        self.costmap_info = msg.info
        self.costmap_stamp = msg.header.stamp
        self.costmap_data = np.array(msg.data, dtype=np.int16).reshape((h, w))

    def blacklist_goal(self, wx, wy):
        self.blacklisted_goals.append((float(wx), float(wy)))

    def is_blacklisted(self, wx, wy):
        for bx, by in self.blacklisted_goals:
            if math.hypot(wx - bx, wy - by) < self.blacklist_radius_m:
                return True
        return False

    def world_to_grid(self, wx, wy):
        if self.map_info is None:
            return None
        gx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = self.map_info.origin.position.x + (float(gx) + 0.5) * self.map_info.resolution
        wy = self.map_info.origin.position.y + (float(gy) + 0.5) * self.map_info.resolution
        return wx, wy

    def _find_frontier_mask(self):
        if self.map_data is None:
            return None
        data = self.map_data
        free_mask = data == self.FREE
        frontier_mask = np.zeros_like(free_mask, dtype=bool)
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(np.roll(data, -dy, axis=0), -dx, axis=1)
            frontier_mask |= (free_mask & (shifted == self.UNKNOWN))
        frontier_mask[0, :] = False
        frontier_mask[-1, :] = False
        frontier_mask[:, 0] = False
        frontier_mask[:, -1] = False
        return frontier_mask

    def _cluster_frontiers(self, frontier_mask):
        h, w = frontier_mask.shape
        visited = np.zeros((h, w), dtype=bool)
        ys, xs = np.where(frontier_mask)
        clusters = []
        for i in range(len(xs)):
            x0 = int(xs[i])
            y0 = int(ys[i])
            if visited[y0, x0]:
                continue
            queue = deque([(x0, y0)])
            visited[y0, x0] = True
            cells = []
            unknown_vx = 0.0
            unknown_vy = 0.0
            while queue:
                cx, cy = queue.popleft()
                cells.append((cx, cy))
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        if self.map_data[ny, nx] == self.UNKNOWN:
                            unknown_vx += dx
                            unknown_vy += dy
                        if not visited[ny, nx] and frontier_mask[ny, nx]:
                            visited[ny, nx] = True
                            queue.append((nx, ny))
                for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h and not visited[ny, nx] and frontier_mask[ny, nx]:
                        visited[ny, nx] = True
                        queue.append((nx, ny))
            if len(cells) < self.min_frontier_size:
                continue
            mean_x = sum(c[0] for c in cells) / float(len(cells))
            mean_y = sum(c[1] for c in cells) / float(len(cells))
            clusters.append(
                {
                    "mean_x": mean_x,
                    "mean_y": mean_y,
                    "size": len(cells),
                    "unknown_vx": unknown_vx,
                    "unknown_vy": unknown_vy,
                }
            )
        return clusters

    def _is_map_cell_free(self, gx, gy):
        h, w = self.map_data.shape
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return False
        return self.map_data[gy, gx] == self.FREE

    def _is_map_world_safe(self, wx, wy):
        gg = self.world_to_grid(wx, wy)
        if gg is None:
            return False
        gx, gy = gg
        if not self._is_map_cell_free(gx, gy):
            return False
        clearance_cells = max(1, int(self.map_clearance_m / self.map_info.resolution))
        h, w = self.map_data.shape
        for ddy in range(-clearance_cells, clearance_cells + 1):
            for ddx in range(-clearance_cells, clearance_cells + 1):
                nx = gx + ddx
                ny = gy + ddy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                if self.map_data[ny, nx] >= self.occupied_threshold:
                    return False
        return True

    def _is_costmap_world_safe(self, wx, wy):
        if self.costmap_info is None or self.costmap_data is None:
            return True
        res = self.costmap_info.resolution
        gx = int((wx - self.costmap_info.origin.position.x) / res)
        gy = int((wy - self.costmap_info.origin.position.y) / res)
        h = int(self.costmap_info.height)
        w = int(self.costmap_info.width)
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return False
        clearance_cells = max(1, int(self.map_clearance_m / res))
        for ddy in range(-clearance_cells, clearance_cells + 1):
            for ddx in range(-clearance_cells, clearance_cells + 1):
                nx = gx + ddx
                ny = gy + ddy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                cell = int(self.costmap_data[ny, nx])
                if cell < 0:
                    continue
                if cell >= self.costmap_lethal_threshold:
                    return False
        return True

    def is_world_safe(self, wx, wy):
        return self._is_map_world_safe(wx, wy) and self._is_costmap_world_safe(wx, wy)

    def _find_approach_cell(self, frontier_gx, frontier_gy, robot_gx, robot_gy):
        dx = robot_gx - frontier_gx
        dy = robot_gy - frontier_gy
        length = math.hypot(dx, dy)
        if length < 1e-6:
            length = 1.0
        ux = dx / length
        uy = dy / length
        max_steps = max(self.min_approach_cells, int(self.approach_pull_m / self.map_info.resolution))
        for step in range(self.min_approach_cells, max_steps + 1):
            cx = int(round(frontier_gx + ux * step))
            cy = int(round(frontier_gy + uy * step))
            if not self._is_map_cell_free(cx, cy):
                continue
            wx, wy = self.grid_to_world(cx, cy)
            if self.is_world_safe(wx, wy):
                return cx, cy

        # Local fallback around frontier centroid.
        rmax = max(3, int(0.5 / self.map_info.resolution))
        for r in range(1, rmax + 1):
            for ddy in range(-r, r + 1):
                for ddx in range(-r, r + 1):
                    cx = int(round(frontier_gx + ddx))
                    cy = int(round(frontier_gy + ddy))
                    if not self._is_map_cell_free(cx, cy):
                        continue
                    wx, wy = self.grid_to_world(cx, cy)
                    if self.is_world_safe(wx, wy):
                        return cx, cy
        return None

    def _project_short_horizon_goal(self, robot_x, robot_y, goal_x, goal_y, max_step_m):
        if max_step_m <= 0.0:
            return goal_x, goal_y
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist = math.hypot(dx, dy)
        if dist <= max_step_m:
            return goal_x, goal_y
        ratios = [max_step_m / dist, 0.85 * max_step_m / dist, 0.7 * max_step_m / dist, 0.55 * max_step_m / dist]
        for ratio in ratios:
            px = robot_x + dx * ratio
            py = robot_y + dy * ratio
            if self.is_world_safe(px, py):
                return px, py
        return goal_x, goal_y

    def select_goal(
        self,
        robot_x,
        robot_y,
        blocked_fn=None,
        min_goal_dist=0.8,
        short_horizon_m=2.0,
        max_goal_dist=6.0,
    ):
        """Return a dict with approach goal and frontier hint, or None."""
        if not self.has_map():
            return None

        frontier_mask = self._find_frontier_mask()
        if frontier_mask is None:
            return None
        clusters = self._cluster_frontiers(frontier_mask)
        if not clusters:
            return None

        robot_grid = self.world_to_grid(robot_x, robot_y)
        if robot_grid is None:
            return None
        robot_gx, robot_gy = robot_grid

        candidates = []
        for cluster in clusters:
            fx = cluster["mean_x"]
            fy = cluster["mean_y"]
            frontier_wx, frontier_wy = self.grid_to_world(fx, fy)
            if self.is_blacklisted(frontier_wx, frontier_wy):
                continue

            approach = self._find_approach_cell(fx, fy, robot_gx, robot_gy)
            if approach is None:
                continue
            ax, ay = approach
            approach_wx, approach_wy = self.grid_to_world(ax, ay)

            if self.is_blacklisted(approach_wx, approach_wy):
                continue
            if blocked_fn is not None and blocked_fn(approach_wx, approach_wy):
                continue

            goal_dist = math.hypot(approach_wx - robot_x, approach_wy - robot_y)
            if goal_dist < min_goal_dist:
                continue
            if max_goal_dist > 0.0 and goal_dist > max_goal_dist:
                continue

            short_goal_x, short_goal_y = self._project_short_horizon_goal(
                robot_x,
                robot_y,
                approach_wx,
                approach_wy,
                short_horizon_m,
            )
            short_goal_dist = math.hypot(short_goal_x - robot_x, short_goal_y - robot_y)
            if short_goal_dist < min_goal_dist:
                continue

            to_frontier_x = frontier_wx - short_goal_x
            to_frontier_y = frontier_wy - short_goal_y
            if math.hypot(to_frontier_x, to_frontier_y) > 1e-3:
                frontier_yaw = math.atan2(to_frontier_y, to_frontier_x)
            else:
                frontier_yaw = math.atan2(cluster["unknown_vy"], cluster["unknown_vx"])

            distance_term = 1.0 / ((goal_dist + 0.4) * (goal_dist + 0.4))
            size_term = min(cluster["size"], 120)
            short_bonus = 1.25 if goal_dist <= max(short_horizon_m, 0.1) else 1.0
            score = size_term * distance_term * short_bonus
            candidates.append(
                {
                    "score": score,
                    "goal_x": short_goal_x,
                    "goal_y": short_goal_y,
                    "goal_distance": short_goal_dist,
                    "frontier_x": frontier_wx,
                    "frontier_y": frontier_wy,
                    "frontier_yaw": frontier_yaw,
                    "frontier_size": cluster["size"],
                }
            )

        if not candidates:
            return None
        candidates.sort(key=lambda c: c["score"], reverse=True)
        return candidates[0]
