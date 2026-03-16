#!/usr/bin/env python3
"""
Continuous map manager for real robot task workflow.

Use cases:
1) No base map provided:
   Save live SLAM map periodically as a by-product of task execution.
2) Base map provided:
   Merge live SLAM observations into the imported map and keep writing an
   incrementally improved map file during runtime.
"""

import math
import os
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaml_scalar(raw: str):
    txt = raw.strip().strip("'").strip('"')
    if txt.lower() in ("true", "false"):
        return txt.lower() == "true"
    try:
        if "." in txt or "e" in txt.lower():
            return float(txt)
        return int(txt)
    except ValueError:
        return txt


def _parse_simple_map_yaml(path: str) -> dict:
    cfg = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            clean = line.split("#", 1)[0].strip()
            if not clean or ":" not in clean:
                continue
            key, val = clean.split(":", 1)
            key = key.strip()
            val = val.strip()
            if key == "origin":
                payload = val.strip()
                if payload.startswith("[") and payload.endswith("]"):
                    items = [x.strip() for x in payload[1:-1].split(",")]
                    if len(items) == 3:
                        cfg[key] = [float(items[0]), float(items[1]), float(items[2])]
                continue
            cfg[key] = _yaml_scalar(val)
    return cfg


def _next_token(stream) -> Optional[bytes]:
    token = bytearray()
    while True:
        ch = stream.read(1)
        if not ch:
            break
        if ch == b"#":
            stream.readline()
            continue
        if ch.isspace():
            if token:
                break
            continue
        token.extend(ch)
    if not token:
        return None
    return bytes(token)


def _read_pgm(path: str) -> Tuple[int, int, int, List[int]]:
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"Unsupported PGM format in {path}: {magic!r}")

        width_token = _next_token(f)
        height_token = _next_token(f)
        maxval_token = _next_token(f)
        if width_token is None or height_token is None or maxval_token is None:
            raise ValueError(f"Incomplete PGM header: {path}")

        width = int(width_token)
        height = int(height_token)
        maxval = int(maxval_token)
        if maxval <= 0:
            raise ValueError(f"Invalid PGM maxval in {path}: {maxval}")

        count = width * height
        if magic == b"P5":
            raw = f.read(count)
            if len(raw) < count:
                raise ValueError(f"PGM data truncated: {path}")
            pixels = list(raw[:count])
        else:
            pixels = []
            while len(pixels) < count:
                token = _next_token(f)
                if token is None:
                    break
                pixels.append(int(token))
            if len(pixels) < count:
                raise ValueError(f"PGM ASCII data truncated: {path}")

    return width, height, maxval, pixels


def _pgm_to_occ(pixels: List[int], negate: int, occ_thresh: float, free_thresh: float) -> List[int]:
    out = [0] * len(pixels)
    for i, px in enumerate(pixels):
        if negate:
            occ_prob = float(px) / 255.0
        else:
            occ_prob = (255.0 - float(px)) / 255.0
        if occ_prob > occ_thresh:
            out[i] = 100
        elif occ_prob < free_thresh:
            out[i] = 0
        else:
            out[i] = -1
    return out


def _occ_to_pgm_byte(val: int) -> int:
    if val < 0:
        return 205
    clipped = min(max(int(val), 0), 100)
    gray = int(round((100.0 - clipped) * 255.0 / 100.0))
    return min(max(gray, 0), 254)


def _write_pgm(path: str, width: int, height: int, data: List[int]) -> None:
    payload = bytearray(len(data))
    for i, v in enumerate(data):
        payload[i] = _occ_to_pgm_byte(v)
    with open(path, "wb") as f:
        f.write(b"P5\n")
        f.write(f"{width} {height}\n255\n".encode("ascii"))
        f.write(payload)


def _safe_abs(path: str, base_dir: str) -> str:
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(base_dir, path))


@dataclass
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    frame_id: str
    data: List[int]

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        dx = x - self.origin_x
        dy = y - self.origin_y
        c = math.cos(-self.origin_yaw)
        s = math.sin(-self.origin_yaw)
        lx = c * dx - s * dy
        ly = s * dx + c * dy
        gx = int(lx / self.resolution)
        gy = int(ly / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        lx = (gx + 0.5) * self.resolution
        ly = (gy + 0.5) * self.resolution
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        x = self.origin_x + c * lx - s * ly
        y = self.origin_y + s * lx + c * ly
        return x, y

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def idx(self, gx: int, gy: int) -> int:
        return gy * self.width + gx


def _map_from_msg(msg: OccupancyGrid) -> GridMap:
    q = msg.info.origin.orientation
    yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
    return GridMap(
        width=msg.info.width,
        height=msg.info.height,
        resolution=msg.info.resolution,
        origin_x=msg.info.origin.position.x,
        origin_y=msg.info.origin.position.y,
        origin_yaw=yaw,
        frame_id=msg.header.frame_id or "map",
        data=list(msg.data),
    )


def _load_grid_from_yaml(yaml_path: str, frame_id: str) -> GridMap:
    cfg = _parse_simple_map_yaml(yaml_path)
    image = cfg.get("image")
    if not image:
        raise ValueError(f"image field missing in map yaml: {yaml_path}")
    image_path = _safe_abs(str(image), os.path.dirname(os.path.abspath(yaml_path)))

    resolution = float(cfg.get("resolution", 0.05))
    origin = cfg.get("origin", [0.0, 0.0, 0.0])
    negate = int(cfg.get("negate", 0))
    occ_thresh = float(cfg.get("occupied_thresh", 0.65))
    free_thresh = float(cfg.get("free_thresh", 0.196))

    width, height, _, pixels = _read_pgm(image_path)
    data = _pgm_to_occ(pixels, negate=negate, occ_thresh=occ_thresh, free_thresh=free_thresh)

    return GridMap(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]),
        frame_id=frame_id,
        data=data,
    )


class ContinuousMapManager:
    def __init__(self):
        rospy.init_node("continuous_map_manager", anonymous=False)

        self.map_topic = rospy.get_param("~map_topic", "/work_map")
        self.save_interval_s = float(rospy.get_param("~save_interval_s", 60.0))
        self.output_map_prefix = rospy.get_param("~output_map_prefix", "/tmp/task_live_map")
        self.base_map_yaml = str(rospy.get_param("~base_map_yaml", "")).strip()
        self.base_frame = rospy.get_param("~base_frame", "map")
        self.allow_clear_occupied = bool(rospy.get_param("~allow_clear_occupied", False))
        self.free_mark = int(rospy.get_param("~free_mark", 30))
        self.occupied_mark = int(rospy.get_param("~occupied_mark", 65))
        self.save_on_shutdown = bool(rospy.get_param("~save_on_shutdown", True))

        self._lock = threading.Lock()
        self._latest_live_map = None  # type: Optional[GridMap]
        self._last_save_ts = 0.0

        self._base_map = None  # type: Optional[GridMap]
        if self.base_map_yaml:
            self._base_map = _load_grid_from_yaml(self.base_map_yaml, frame_id=self.base_frame)
            rospy.loginfo("[MapManager] Base map loaded: %s", self.base_map_yaml)

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self._on_map, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(self.save_interval_s), self._on_timer)
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo(
            "[MapManager] Running. map_topic=%s, save_interval=%.1fs, output=%s, base_map=%s",
            self.map_topic,
            self.save_interval_s,
            self.output_map_prefix,
            self.base_map_yaml if self.base_map_yaml else "<none>",
        )

    def _on_map(self, msg: OccupancyGrid):
        grid = _map_from_msg(msg)
        with self._lock:
            self._latest_live_map = grid

    def _get_latest_map(self) -> Optional[GridMap]:
        with self._lock:
            return self._latest_live_map

    def _lookup_live_to_base(self, live_frame: str) -> Optional[Tuple[float, float, float]]:
        if self._base_map is None:
            return None
        if live_frame == self._base_map.frame_id:
            return 0.0, 0.0, 0.0
        try:
            tfm = self._tf_buffer.lookup_transform(
                self._base_map.frame_id, live_frame, rospy.Time(0), rospy.Duration(0.5)
            )
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logwarn_throttle(5.0, "[MapManager] TF lookup failed (%s -> %s): %s",
                                   live_frame, self._base_map.frame_id, str(exc))
            return None

        q = tfm.transform.rotation
        yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        t = tfm.transform.translation
        return t.x, t.y, yaw

    @staticmethod
    def _transform_xy(x: float, y: float, tf_2d: Tuple[float, float, float]) -> Tuple[float, float]:
        tx, ty, yaw = tf_2d
        c = math.cos(yaw)
        s = math.sin(yaw)
        return tx + c * x - s * y, ty + s * x + c * y

    def _merge_into_base(self, live_map: GridMap) -> Optional[GridMap]:
        if self._base_map is None:
            return live_map

        tf_2d = self._lookup_live_to_base(live_map.frame_id)
        if tf_2d is None:
            return None

        merged_data = list(self._base_map.data)

        for gy in range(live_map.height):
            row_off = gy * live_map.width
            for gx in range(live_map.width):
                live_val = live_map.data[row_off + gx]
                if live_val < 0:
                    continue

                wx, wy = live_map.grid_to_world(gx, gy)
                bxw, byw = self._transform_xy(wx, wy, tf_2d)
                bgx, bgy = self._base_map.world_to_grid(bxw, byw)
                if not self._base_map.in_bounds(bgx, bgy):
                    continue

                bi = self._base_map.idx(bgx, bgy)
                base_val = merged_data[bi]

                if live_val >= self.occupied_mark:
                    if base_val < self.occupied_mark:
                        merged_data[bi] = 100
                elif live_val <= self.free_mark:
                    if base_val < 0:
                        merged_data[bi] = 0
                    elif self.allow_clear_occupied and base_val >= self.occupied_mark:
                        merged_data[bi] = 0

        return GridMap(
            width=self._base_map.width,
            height=self._base_map.height,
            resolution=self._base_map.resolution,
            origin_x=self._base_map.origin_x,
            origin_y=self._base_map.origin_y,
            origin_yaw=self._base_map.origin_yaw,
            frame_id=self._base_map.frame_id,
            data=merged_data,
        )

    @staticmethod
    def _write_map(prefix: str, grid: GridMap):
        out_dir = os.path.dirname(os.path.abspath(prefix))
        os.makedirs(out_dir, exist_ok=True)

        pgm_path = f"{prefix}.pgm"
        yaml_path = f"{prefix}.yaml"
        pgm_tmp = f"{pgm_path}.tmp"
        yaml_tmp = f"{yaml_path}.tmp"

        _write_pgm(pgm_tmp, grid.width, grid.height, grid.data)
        with open(yaml_tmp, "w", encoding="utf-8") as f:
            f.write(f"image: {pgm_path}\n")
            f.write(f"resolution: {grid.resolution:.6f}\n")
            f.write(f"origin: [{grid.origin_x:.6f}, {grid.origin_y:.6f}, {grid.origin_yaw:.6f}]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.196\n")

        os.replace(pgm_tmp, pgm_path)
        os.replace(yaml_tmp, yaml_path)

    def _save_once(self):
        live_map = self._get_latest_map()
        if live_map is None:
            rospy.logwarn_throttle(5.0, "[MapManager] No live map on topic: %s", self.map_topic)
            return False

        merged = self._merge_into_base(live_map)
        if merged is None:
            return False

        self._write_map(self.output_map_prefix, merged)
        self._last_save_ts = rospy.Time.now().to_sec()
        rospy.loginfo_throttle(3.0, "[MapManager] Map saved: %s.{yaml,pgm}", self.output_map_prefix)
        return True

    def _on_timer(self, _evt):
        self._save_once()

    def _on_shutdown(self):
        if not self.save_on_shutdown:
            return
        try:
            self._save_once()
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logwarn("[MapManager] Final save failed during shutdown: %s", str(exc))

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        ContinuousMapManager().spin()
    except Exception as e:  # pylint: disable=broad-except
        rospy.logerr("[MapManager] Fatal: %s", str(e))
        raise
