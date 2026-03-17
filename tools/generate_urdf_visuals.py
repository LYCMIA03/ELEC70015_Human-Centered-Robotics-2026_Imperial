#!/usr/bin/env python3
"""
Generate verification visuals for a URDF:
1) Sensor/layout top+side view PNG
2) URDF tree DOT + PNG
3) Key-link poses JSON
"""

import argparse
import json
import math
import os
import subprocess
import xml.etree.ElementTree as ET

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def rot_from_rpy(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def t_from_origin(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = rot_from_rpy(*rpy)
    t[:3, 3] = xyz
    return t


def parse_urdf(urdf_path):
    root = ET.parse(urdf_path).getroot()
    joints = []
    for j in root.findall("joint"):
        parent_el = j.find("parent")
        child_el = j.find("child")
        if parent_el is None or child_el is None:
            continue
        origin = j.find("origin")
        if origin is None:
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
        else:
            xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()]
        joints.append(
            {
                "name": j.attrib.get("name", ""),
                "parent": parent_el.attrib["link"],
                "child": child_el.attrib["link"],
                "xyz": xyz,
                "rpy": rpy,
            }
        )
    return joints


def solve_link_poses(joints):
    poses = {"base_link": np.eye(4), "base_footprint": np.eye(4)}
    for _ in range(400):
        changed = False
        for j in joints:
            p = j["parent"]
            c = j["child"]
            if p in poses and c not in poses:
                poses[c] = poses[p] @ t_from_origin(j["xyz"], j["rpy"])
                changed = True
        if not changed:
            break
    return poses


def save_layout_png(poses, out_png):
    keys = [
        "base_link",
        "base_footprint",
        "laser",
        "unitree_lidar",
        "rgbd_camera_mount_bar",
        "rgbd_camera",
    ]
    points = {k: poses[k][:3, 3] for k in keys if k in poses}

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    ax_top, ax_side = axes

    # Approx base footprint box for visual reference
    body_x = [-0.25, 0.25, 0.25, -0.25, -0.25]
    body_y = [-0.20, -0.20, 0.20, 0.20, -0.20]
    ax_top.plot(body_x, body_y, "k-", lw=2, label="chassis footprint")

    for k, p in points.items():
        ax_top.scatter(p[0], p[1], s=70)
        ax_top.text(p[0] + 0.01, p[1] + 0.01, k, fontsize=8)
    ax_top.set_title("Top View (X-Y)")
    ax_top.set_xlabel("x [m]")
    ax_top.set_ylabel("y [m]")
    ax_top.axis("equal")
    ax_top.grid(True, alpha=0.3)

    body_z = [0.177 - 0.08, 0.177 - 0.08, 0.177 + 0.08, 0.177 + 0.08, 0.177 - 0.08]
    ax_side.plot(body_x, body_z, "k-", lw=2, label="chassis box")
    for k, p in points.items():
        ax_side.scatter(p[0], p[2], s=70)
        ax_side.text(p[0] + 0.01, p[2] + 0.01, k, fontsize=8)
    ax_side.set_title("Side View (X-Z)")
    ax_side.set_xlabel("x [m]")
    ax_side.set_ylabel("z [m]")
    ax_side.axis("equal")
    ax_side.grid(True, alpha=0.3)

    fig.suptitle("URDF Layout Verification (Unitree + RPLIDAR-role + RGBD)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=220)

    return {k: {"x": float(v[0]), "y": float(v[1]), "z": float(v[2])} for k, v in points.items()}


def save_dot_and_png(joints, out_dot, out_png):
    nodes = set()
    for j in joints:
        nodes.add(j["parent"])
        nodes.add(j["child"])

    sensor_nodes = {"laser", "unitree_lidar", "rgbd_camera", "rgbd_camera_mount_bar"}
    with open(out_dot, "w", encoding="utf-8") as f:
        f.write("digraph URDF {\n")
        f.write("  rankdir=LR;\n")
        for n in sorted(nodes):
            fill = "lightblue" if n in sensor_nodes else "white"
            f.write(f'  "{n}" [shape=box, style=filled, fillcolor={fill}];\n')
        for j in joints:
            x, y, z = j["xyz"]
            label = f'{j["name"]}\\nxyz=({x:.3f},{y:.3f},{z:.3f})'
            f.write(f'  "{j["parent"]}" -> "{j["child"]}" [label="{label}"];\n')
        f.write("}\n")

    try:
        subprocess.check_call(["dot", "-Tpng", out_dot, "-o", out_png])
        return True
    except Exception:
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    joints = parse_urdf(args.urdf)
    poses = solve_link_poses(joints)

    layout_png = os.path.join(args.out_dir, "p3at_full_sensor_layout.png")
    dot_file = os.path.join(args.out_dir, "p3at_full_urdf_tree.dot")
    tree_png = os.path.join(args.out_dir, "p3at_full_urdf_tree.png")
    pose_json = os.path.join(args.out_dir, "p3at_full_sensor_pose.json")

    pose_data = save_layout_png(poses, layout_png)
    dot_ok = save_dot_and_png(joints, dot_file, tree_png)

    with open(pose_json, "w", encoding="utf-8") as f:
        json.dump(pose_data, f, indent=2, ensure_ascii=False)

    print(layout_png)
    print(dot_file)
    if dot_ok:
        print(tree_png)
    print(pose_json)


if __name__ == "__main__":
    main()
