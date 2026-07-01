#!/usr/bin/env python3
"""Live visualization via Rerun 鈥?full robot dashboard.

Channels:
  world/point_cloud   鈥?LiDAR voxel blocks (height-colored Boxes3D)
  world/robot         鈥?robot body (wireframe box)
  world/heading       鈥?orientation arrow (yellow)
  world/trajectory    鈥?path traveled (blue line)
  world/costmap       鈥?2D occupancy grid (colored mesh)
  world/detections    鈥?detected objects (labeled 3D points)
  world/nav_path      鈥?planned navigation path (green line)
  world/tf/{frame}    鈥?TF coordinate frames (Transform3D)
  camera/color        鈥?RGB camera (rotated for vertical mount)
  camera/depth        鈥?depth image
  metrics/slam_hz     鈥?SLAM update rate
  metrics/det_count   鈥?detection count

Usage:
  On S100P (web viewer for remote access):
    python3 scripts/visualization/rerun_live.py

  On S100P (local native viewer):
    python3 scripts/visualization/rerun_live.py --native

  View remotely (method 1 鈥?web viewer via SSH tunnel):
    ssh -L 9090:127.0.0.1:9090 -L 9877:127.0.0.1:9877 sunrise@192.168.66.190
    Open http://localhost:9090

  View remotely (method 2 鈥?native viewer connects to S100P gRPC):
    python3 scripts/visualization/rerun_live.py                          # on S100P
    rerun --connect rerun+http://192.168.66.190:9877/proxy # on local PC
"""
import sys, os, time, math, argparse
from pathlib import Path

def find_repo_root(start: Path) -> Path:
    for path in (start, *start.parents):
        if (path / "pyproject.toml").is_file() and (path / "AGENTS.md").is_file():
            return path
    raise RuntimeError(f"Could not find repository root from {start}")

REPO_ROOT = find_repo_root(Path(__file__).resolve().parent)
sys.path.insert(0, str(REPO_ROOT / "src"))
import logging
logging.basicConfig(level=logging.WARNING)
import numpy as np

parser = argparse.ArgumentParser(description="LingTu Rerun Live Viewer")
parser.add_argument("--native", action="store_true", help="Launch native desktop viewer")
parser.add_argument("--web-port", type=int, default=9090)
parser.add_argument("--grpc-port", type=int, default=9877)
_args = parser.parse_args()

from runtime.adapters.ros2.context import ensure_rclpy, get_shared_executor, shutdown_shared_executor
ensure_rclpy()

import rerun as rr
import rerun.blueprint as rrb

rr.init("lingtu_live")

# Layout: 3D world (left) + camera 2D (right top/bottom)
try:
    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="world", name="3D World"),
            rrb.Vertical(
                rrb.Spatial2DView(origin="camera/color", name="Camera"),
                rrb.Spatial2DView(origin="camera/depth", name="Depth"),
            ),
        ),
    )
except Exception:
    blueprint = None

if _args.native:
    rr.spawn(connect=True)
    print("Rerun: native viewer")
else:
    server_uri = rr.serve_grpc(grpc_port=_args.grpc_port)
    rr.serve_web_viewer(open_browser=False, web_port=_args.web_port, connect_to=server_uri)
    print(f"Rerun web:  http://localhost:{_args.web_port}")
    print(f"Rerun gRPC: rerun --connect rerun+http://<robot_ip>:{_args.grpc_port}/proxy")

if blueprint:
    try:
        rr.send_blueprint(blueprint)
    except Exception:
        pass

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage

node = Node("rerun_viz")
qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)
qos_best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)

trajectory = []
counts = {"odom": 0, "cloud": 0, "color": 0, "depth": 0,
          "costmap": 0, "det": 0, "path": 0, "tf": 0}
_last_odom_t = 0.0

# Robot body dimensions (half-sizes in meters) 鈥?Thunder quadruped
ROBOT_HALF = [0.35, 0.155, 0.15]

# Accumulated global map (voxel hash for dedup, keeps map growing)
_MAP_VOXEL_SIZE = 0.1  # 10cm grid for accumulation
_map_voxels = {}  # (ix,iy,iz) 鈫?[r,g,b]
_MAP_MAX = 100000  # max voxels to display


# 鈹€鈹€ Point Cloud (accumulate into global map) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
def on_cloud(msg):
    counts["cloud"] += 1
    if counts["cloud"] % 5 != 0:  # throttle to ~2Hz
        return
    try:
        n = msg.width * msg.height
        if n == 0:
            return
        step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
        xyz = np.zeros((n, 3), dtype=np.float32)
        xyz[:, 0] = np.frombuffer(raw[:, 0:4].tobytes(), dtype=np.float32)
        xyz[:, 1] = np.frombuffer(raw[:, 4:8].tobytes(), dtype=np.float32)
        xyz[:, 2] = np.frombuffer(raw[:, 8:12].tobytes(), dtype=np.float32)
        valid = np.isfinite(xyz).all(axis=1)
        xyz = xyz[valid]
        if len(xyz) == 0:
            return

        # Accumulate into voxel map (global map grows over time)
        vs = _MAP_VOXEL_SIZE
        ix = np.floor(xyz[:, 0] / vs).astype(np.int32)
        iy = np.floor(xyz[:, 1] / vs).astype(np.int32)
        iz = np.floor(xyz[:, 2] / vs).astype(np.int32)
        for i in range(len(ix)):
            key = (int(ix[i]), int(iy[i]), int(iz[i]))
            if key not in _map_voxels:
                # Color by height
                z_val = xyz[i, 2]
                _map_voxels[key] = z_val

        # Display accumulated map (subsample if too large)
        keys = list(_map_voxels.keys())
        if len(keys) > _MAP_MAX:
            import random
            keys = random.sample(keys, _MAP_MAX)

        centers = np.array(keys, dtype=np.float32) * vs + vs * 0.5
        z_vals = np.array([_map_voxels[tuple(k)] for k in keys], dtype=np.float32)
        z_norm = np.clip((z_vals - z_vals.min()) / max(z_vals.max() - z_vals.min(), 0.01), 0, 1)
        colors = np.zeros((len(centers), 3), dtype=np.uint8)
        colors[:, 0] = (z_norm * 255).astype(np.uint8)
        colors[:, 2] = ((1 - z_norm) * 255).astype(np.uint8)
        colors[:, 1] = 80
        rr.log("world/map", rr.Points3D(centers, colors=colors, radii=0.04))
    except Exception:
        pass


# 鈹€鈹€ Robot Pose (wireframe box + heading arrow) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
def on_odom(msg):
    global _last_odom_t
    counts["odom"] += 1
    if counts["odom"] % 2 != 0:  # throttle to ~5Hz
        return
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    x, y, z = p.x, p.y, p.z

    # Robot body 鈥?wireframe box
    rr.log("world/robot", rr.Boxes3D(
        centers=[[x, y, z + ROBOT_HALF[2]]],
        half_sizes=[ROBOT_HALF],
        colors=[[0, 255, 127]],
        fill_mode="MajorWireframe",
    ))

    # Orientation arrow
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    dx, dy = math.cos(yaw) * 0.8, math.sin(yaw) * 0.8
    rr.log("world/heading", rr.Arrows3D(
        origins=[[x, y, z + 0.3]],
        vectors=[[dx, dy, 0]],
        colors=[[255, 255, 0]],
        radii=0.05,
    ))

    # Trajectory
    trajectory.append([x, y, z])
    if len(trajectory) > 2:
        rr.log("world/trajectory", rr.LineStrips3D(
            [trajectory[-1000:]],
            colors=[[0, 100, 255]],
        ))

    # SLAM Hz metric
    now = time.time()
    if _last_odom_t > 0:
        dt = now - _last_odom_t
        if dt > 0:
            rr.log("metrics/slam_hz", rr.Scalars(1.0 / dt))
    _last_odom_t = now


# 鈹€鈹€ TF Coordinate Frames 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
def on_tf(msg):
    counts["tf"] += 1
    try:
        for tf in msg.transforms:
            child = tf.child_frame_id.lstrip("/")
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log(f"world/tf/{child}", rr.Transform3D(
                translation=[t.x, t.y, t.z],
                rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
            ))
    except Exception:
        pass


def on_tf_static(msg):
    """Static TF 鈥?logged once with static=True."""
    try:
        for tf in msg.transforms:
            child = tf.child_frame_id.lstrip("/")
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log(f"world/tf/{child}", rr.Transform3D(
                translation=[t.x, t.y, t.z],
                rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
            ), static=True)
    except Exception:
        pass


# 鈹€鈹€ Costmap (2D occupancy grid 鈫?colored ground plane) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
def on_costmap(msg):
    counts["costmap"] += 1
    if counts["costmap"] % 5 != 0:  # throttle
        return
    try:
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        grid = np.array(msg.data, dtype=np.int8).reshape(h, w)

        # Build colored image
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[grid == 0] = [40, 80, 40]           # dark green = free
        img[grid > 50] = [200, 50, 50]           # red = occupied
        img[grid < 0] = [60, 60, 60]             # gray = unknown
        inflate_mask = (grid > 0) & (grid <= 50)
        img[inflate_mask] = [180, 160, 40]        # yellow = inflation

        # Flat ground mesh
        x0, y0 = ox, oy
        x1, y1 = ox + w * res, oy + h * res
        vertices = np.array([
            [x0, y0, 0.01], [x1, y0, 0.01],
            [x1, y1, 0.01], [x0, y1, 0.01],
        ], dtype=np.float32)
        triangles = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)
        texcoords = np.array([
            [0.0, 1.0], [1.0, 1.0],
            [1.0, 0.0], [0.0, 0.0],
        ], dtype=np.float32)

        rr.log("world/costmap", rr.Mesh3D(
            vertex_positions=vertices,
            triangle_indices=triangles,
            vertex_texcoords=texcoords,
            albedo_texture=img,
        ))
    except Exception:
        pass


# 鈹€鈹€ Detections (3D labeled points from MarkerArray) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
def on_detections(msg):
    counts["det"] += 1
    try:
        positions = []
        labels = []
        colors = []
        for marker in msg.markers:
            if marker.action == 2:  # DELETE
                continue
            px = marker.pose.position.x
            py = marker.pose.position.y
            pz = marker.pose.position.z
            if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                continue
            positions.append([px, py, pz])
            label = marker.text if marker.text else f"obj_{marker.id}"
            labels.append(label)
            r = int(marker.color.r * 255) if marker.color.r <= 1.0 else int(marker.color.r)
            g = int(marker.color.g * 255) if marker.color.g <= 1.0 else int(marker.color.g)
            b = int(marker.color.b * 255) if marker.color.b <= 1.0 else int(marker.color.b)
            colors.append([max(r, 50), max(g, 50), max(b, 50)])

        if positions:
            rr.log("world/detections", rr.Points3D(
                positions, labels=labels, colors=colors, radii=0.12,
            ))
            rr.log("metrics/det_count", rr.Scalars(len(positions)))
    except Exception:
        pass


# 鈹€鈹€ Navigation Path (green line) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
def on_nav_path(msg):
    counts["path"] += 1
    try:
        pts = [[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
               for ps in msg.poses]
        if len(pts) > 1:
            rr.log("world/nav_path", rr.LineStrips3D(
                [pts], colors=[[0, 255, 100]], radii=0.04,
            ))
    except Exception:
        pass


# 鈹€鈹€ Terrain Map (local planner input, PointCloud2 with traversability) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
def on_terrain(msg):
    counts["terrain"] = counts.get("terrain", 0) + 1
    if counts["terrain"] % 3 != 0:
        return
    try:
        n = msg.width * msg.height
        if n == 0:
            return
        step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
        xyz = np.zeros((n, 3), dtype=np.float32)
        xyz[:, 0] = np.frombuffer(raw[:, 0:4].tobytes(), dtype=np.float32)
        xyz[:, 1] = np.frombuffer(raw[:, 4:8].tobytes(), dtype=np.float32)
        xyz[:, 2] = np.frombuffer(raw[:, 8:12].tobytes(), dtype=np.float32)
        valid = np.isfinite(xyz).all(axis=1)
        xyz = xyz[valid]
        if len(xyz) > 5000:
            idx = np.random.choice(len(xyz), 5000, replace=False)
            xyz = xyz[idx]
        if len(xyz) == 0:
            return
        # Green = traversable (low), Red = obstacle (high)
        z = xyz[:, 2]
        z_norm = np.clip((z - z.min()) / max(z.max() - z.min(), 0.01), 0, 1)
        colors = np.zeros((len(xyz), 3), dtype=np.uint8)
        colors[:, 0] = (z_norm * 200).astype(np.uint8)
        colors[:, 1] = ((1 - z_norm) * 200).astype(np.uint8)
        colors[:, 2] = 50
        rr.log("world/terrain_map", rr.Points3D(xyz, colors=colors, radii=0.05))
    except Exception:
        pass


# 鈹€鈹€ Camera Color 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
def _crop_square(img):
    """Crop center square from portrait image after rotation."""
    h, w = img.shape[:2]
    if h > w:
        margin = (h - w) // 2
        return img[margin:margin + w]
    return img

def _downsample(img, max_dim=320):
    """Downsample image so longest side <= max_dim."""
    h, w = img.shape[:2]
    if max(h, w) <= max_dim:
        return img
    scale = max_dim / max(h, w)
    new_h, new_w = int(h * scale), int(w * scale)
    # Nearest-neighbor resize (fast, no OpenCV needed)
    row_idx = (np.arange(new_h) * h // new_h).astype(int)
    col_idx = (np.arange(new_w) * w // new_w).astype(int)
    return img[np.ix_(row_idx, col_idx)]

def on_color(msg):
    counts["color"] += 1
    if counts["color"] % 15 != 0:  # throttle to ~2fps
        return
    try:
        h, w = msg.height, msg.width
        encoding = msg.encoding.lower()
        if encoding in ("bgr8", "rgb8"):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            if encoding == "bgr8":
                img = img[:, :, ::-1]
            img = _downsample(_crop_square(np.rot90(img, k=1)))
            rr.log("camera/color", rr.Image(img))
        elif encoding in ("mono8", "8uc1"):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            img = _downsample(_crop_square(np.rot90(img, k=1)))
            rr.log("camera/color", rr.Image(img))
    except Exception:
        pass


# 鈹€鈹€ Camera Depth 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
def on_depth(msg):
    counts["depth"] += 1
    if counts["depth"] % 15 != 0:  # throttle to ~2fps
        return
    try:
        h, w = msg.height, msg.width
        encoding = msg.encoding.lower()
        if encoding in ("16uc1",):
            img = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            img = _crop_square(np.rot90(img, k=1))
            # Downsample depth (nearest neighbor to preserve values)
            if max(img.shape) > 320:
                scale = 320 / max(img.shape)
                nh, nw = int(img.shape[0]*scale), int(img.shape[1]*scale)
                row_idx = (np.arange(nh) * img.shape[0] // nh).astype(int)
                col_idx = (np.arange(nw) * img.shape[1] // nw).astype(int)
                img = img[np.ix_(row_idx, col_idx)]
            rr.log("camera/depth", rr.DepthImage(img, meter=1000.0))
        elif encoding in ("32fc1",):
            img = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
            img = _crop_square(np.rot90(img, k=1))
            if max(img.shape) > 320:
                scale = 320 / max(img.shape)
                nh, nw = int(img.shape[0]*scale), int(img.shape[1]*scale)
                row_idx = (np.arange(nh) * img.shape[0] // nh).astype(int)
                col_idx = (np.arange(nw) * img.shape[1] // nw).astype(int)
                img = img[np.ix_(row_idx, col_idx)]
            rr.log("camera/depth", rr.DepthImage(img, meter=1.0))
    except Exception:
        pass


# 鈹€鈹€ Subscribe 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
node.create_subscription(Odometry, "/slam/odometry", on_odom, qos)
node.create_subscription(PointCloud2, "/slam/map_cloud", on_cloud, qos)
node.create_subscription(Image, "/camera/color/image_raw", on_color, qos)
node.create_subscription(Image, "/camera/depth/image_raw", on_depth, qos)
node.create_subscription(OccupancyGrid, "/nav/costmap", on_costmap, qos_best)
node.create_subscription(MarkerArray, "/nav/detections", on_detections, qos_best)
node.create_subscription(Path, "/nav/path", on_nav_path, qos_best)
node.create_subscription(TFMessage, "/tf", on_tf, qos_best)
node.create_subscription(TFMessage, "/tf_static", on_tf_static, qos_best)
node.create_subscription(PointCloud2, "/nav/terrain_map_ext", on_terrain, qos_best)
get_shared_executor().add_node(node)

print("Streaming: map + robot + trajectory + TF + costmap + terrain + detections + nav_path + camera")
print("Ctrl+C to stop.")
try:
    while True:
        time.sleep(2)
        print("odom=%d cloud=%d color=%d depth=%d costmap=%d det=%d path=%d tf=%d" % (
            counts["odom"], counts["cloud"], counts["color"], counts["depth"],
            counts["costmap"], counts["det"], counts["path"], counts["tf"]))
except KeyboardInterrupt:
    pass

node.destroy_node()
shutdown_shared_executor()
print("Done.")
