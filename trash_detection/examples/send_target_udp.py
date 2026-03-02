#!/usr/bin/env python3
import argparse
import json
import socket
import time


def main():
    p = argparse.ArgumentParser(description="Send test target XYZ via UDP JSON")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=16031)
    p.add_argument("--frame-id", default="camera_link")
    p.add_argument("--x", type=float, default=0.8)
    p.add_argument("--y", type=float, default=0.0)
    p.add_argument("--z", type=float, default=1.2)
    p.add_argument("--rate", type=float, default=5.0)
    p.add_argument("--count", type=int, default=0, help="0 = forever")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dt = 1.0 / max(args.rate, 0.1)
    i = 0
    while True:
        now = time.time()
        payload = {
            "stamp": now,
            "frame_id": args.frame_id,
            "x": args.x,
            "y": args.y,
            "z": args.z,
            "source": "non_ros_trash_detection",
        }
        sock.sendto(json.dumps(payload, separators=(",", ":")).encode("utf-8"), (args.host, args.port))
        print(payload)
        i += 1
        if args.count > 0 and i >= args.count:
            break
        time.sleep(dt)


if __name__ == "__main__":
    main()
