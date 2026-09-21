"""Offline renders of the actual URDF assembly, with bounded display detail."""

import json
import math
import xml.etree.ElementTree as ET

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from assembly_geometry import read_collada_batches
from generate import ASSETS, HERE, ry
from matplotlib import font_manager
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def origin_transform(element):
    transform = np.eye(4)
    if element is None:
        return transform
    roll, pitch, yaw = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    transform[:3, :3] = rz @ ry(math.degrees(pitch)) @ rx
    transform[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
    return transform


def link_transforms(robot, standing=False):
    transforms = {"base": np.eye(4)}
    joints = list(robot.findall("joint"))
    while joints:
        pending_count = len(joints)
        for joint in joints[:]:
            parent = joint.find("parent").get("link")
            if parent not in transforms:
                continue
            transform = origin_transform(joint.find("origin"))
            if standing and joint.get("type") == "revolute":
                name = joint.get("name")
                angle = .8 if "_thigh_" in name else -1.6 if "_calf_" in name else 0
                transform[:3, :3] = transform[:3, :3] @ ry(math.degrees(angle))
            transforms[joint.find("child").get("link")] = transforms[parent] @ transform
            joints.remove(joint)
        if len(joints) == pending_count:
            raise ValueError("URDF contains unresolved parents")
    return transforms


def simplify_for_preview(triangles, cell=.004):
    """Cluster original vertices only for display; fit checks use full resolution."""
    raw = triangles.reshape(-1, 3)
    _, ids = np.unique(np.round(raw / cell).astype(int), axis=0, return_inverse=True)
    count = np.bincount(ids)
    vertices = np.column_stack([np.bincount(ids, weights=raw[:, axis]) / count for axis in range(3)])
    faces = ids.reshape(-1, 3)
    valid = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    faces = faces[valid]
    _, unique = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    return vertices[faces[np.sort(unique)]]


def scene_geometry(mount_only=False, exploded=False):
    robot = ET.parse(ASSETS / "urdf/go2_mid360_ground_mount.urdf").getroot()
    transforms = link_transforms(robot, standing=True)
    triangles, colors, cache = [], [], {}
    for link in robot.findall("link"):
        name = link.get("name")
        if mount_only and not (name.startswith("mid360_mount") or name in ("livox_frame", "camera_link")):
            continue
        for visual in link.findall("visual"):
            filename = visual.find("geometry/mesh").get("filename").removeprefix("package://go2_description/")
            if filename not in cache:
                cache[filename] = read_collada_batches(ASSETS / filename)
            transform = transforms[name] @ origin_transform(visual.find("origin"))
            for local, color in cache[filename]:
                custom = name.startswith("mid360_mount") or name in ("livox_frame", "camera_link")
                local = simplify_for_preview(local, .002) if name == "mid360_mount_base" else simplify_for_preview(local, .001) if name in ("livox_frame", "camera_link") else local if custom else simplify_for_preview(local)
                world = local @ transform[:3, :3].T + transform[:3, 3]
                if exploded:
                    if name == "livox_frame":
                        world += ry(35) @ np.array([0, 0, .065])
                    elif name == "mid360_mount_sensor_deck":
                        world += ry(35) @ np.array([0, 0, .028])
                    elif name == "camera_link":
                        world[:, :, 0] += .025
                    elif "_pad" in name:
                        world[:, :, 2] -= .015
                base_color = np.broadcast_to(color, (len(local), 3)).copy()
                if name.startswith("mid360_mount"):
                    base_color[:] = [.18, .23, .27]
                    if "sensor_deck" in name:
                        base_color[:] = [.24, .29, .33]
                elif name == "camera_link":
                    base_color[:] = [.53, .55, .57]
                    front = local[:, :, 0].mean(axis=1) > .0245
                    base_color[front] = [.035, .045, .055]
                elif name == "livox_frame":
                    base_color[:] = [.47, .50, .52]
                    base_color[local[:, :, 2].mean(axis=1) > -.0074] = [.035, .065, .070]
                normal = np.cross(world[:, 1] - world[:, 0], world[:, 2] - world[:, 0])
                length = np.linalg.norm(normal, axis=1)
                valid = length > 1e-12
                normal = normal[valid] / length[valid, None]
                illumination = .52 + .34 * np.clip(normal @ np.array([.3, -.4, .866]), 0, 1)
                triangles.append(world[valid])
                colors.append(np.clip(base_color[valid] * illumination[:, None] + .065, 0, 1))
    return np.concatenate(triangles), np.concatenate(colors)


def panel(fig, rect, triangles, colors, view, detail=False):
    ax = fig.add_axes(rect, projection="3d", facecolor="#f3f5f6")
    if detail:
        limits = [[.075, .34], [-.07, .07], [.045, .275]]
    else:
        limits = [[-.31, .40], [-.25, .25], [-.34, .24]]
    ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors, edgecolor="none", linewidths=0,
                                        antialiased=False, zsort="average"))
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_zlim(*limits[2])
    ax.set_box_aspect([b - a for a, b in limits])
    ax.set_proj_type("ortho")
    ax.view_init(*view)
    ax.set_axis_off()
    return ax


def main():
    if HERE.drive:
        font_path = "C:/Windows/Fonts/msyh.ttc"
        font_manager.fontManager.addfont(font_path)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=font_path).get_name()
    plt.rcParams["axes.unicode_minus"] = False
    triangles, colors = scene_geometry()
    detail_triangles, detail_colors = scene_geometry(mount_only=True, exploded=False)
    values = json.loads((HERE / "design_values.json").read_text())
    mass = round(values["bracket_mass_kg_excluding_fasteners"] * 1000)
    print(f"Offline display triangles: {len(triangles)}; fit checks retain full-resolution meshes", flush=True)
    fig = plt.figure(figsize=(16, 10), facecolor="#f3f5f6")
    fig.text(.05, .945, "Go2 EDU · MID-360 + D435i 共用鞍座", fontsize=24, color="#1c303b", weight="bold")
    fig.text(.05, .903, "V6B  /  按概念重建 · 长鞍座 · 弧形双侧肋 · 一体承托", fontsize=12, color="#526c79")
    panel(fig, [.015, .12, .62, .72], triangles, colors, (23, -62))
    panel(fig, [.625, .46, .36, .38], triangles, colors, (0, -90))
    panel(fig, [.625, .095, .36, .37], detail_triangles, detail_colors, (22, -65), detail=True)
    fig.text(.055, .825, "01 / 整机效果", fontsize=12, color="#526c79")
    fig.text(.66, .825, "02 / 侧面位置", fontsize=12, color="#526c79")
    fig.text(.66, .435, "03 / 一体主架", fontsize=12, color="#526c79")
    fig.text(.055, .10, f"约 {mass} g 装配 · PLA 按实心估算\n35° 前倾 · 58.5 mm 两孔接口\n180 mm 长鞍座 / 开放中心 / 弧形加强肋", fontsize=12, linespacing=1.7, color="#334855")
    fig.text(.055, .035, "安装孔横向间距有社区 CAD 依据；孔位 X/Z 与头壳承载能力仍待实物确认；曲面依据原始网格拟合。", fontsize=11, color="#946021")
    fig.savefig(HERE / "installed_preview.png", dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)
    fig = plt.figure(figsize=(12, 8), facecolor="#f3f5f6")
    panel(fig, [.01, .12, .49, .73], detail_triangles, detail_colors, (23, -65), detail=True)
    panel(fig, [.50, .12, .49, .73], detail_triangles, detail_colors, (-28, -65), detail=True)
    fig.text(.12, .16, "双传感器布局", fontsize=12, color="#526c79")
    fig.text(.60, .16, "底部：开放框架与曲面鞍座", fontsize=12, color="#526c79")
    fig.text(.055, .935, "V6B · 长鞍座与弧形侧肋", fontsize=22, color="#1c303b")
    fig.text(.055, .06, "所有自制结构件均为 PLA；标准螺钉连接。强度、温升与机身配合待实测。", fontsize=11, color="#526c79")
    fig.savefig(HERE / "installed_detail.png", dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == "__main__":
    main()
