#!/usr/bin/env python3
"""
MapPilot Paper-Quality Trajectory Visualization (Fast Version)
使用 Fast Path 快速生成论文级别的轨迹俯视图
"""
import sys
import os
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

sys.path.insert(0, "/home/bsrl/hongsenpang/habitat/MapPilot/src/semantic_planner")

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings, make_cfg

SCENE_PATH = "/home/bsrl/hongsenpang/habitat/data/hm3d_hf/minival/00800-TEEsavR23oF/TEEsavR23oF.basis.glb"
OUTPUT_DIR = "/home/bsrl/hongsenpang/habitat/paper_figures"

def create_simulator():
    settings = default_sim_settings.copy()
    settings["width"] = 640
    settings["height"] = 480
    settings["scene"] = SCENE_PATH
    settings["sensor_height"] = 0.88
    settings["color_sensor"] = True
    settings["depth_sensor"] = True
    settings["seed"] = 42
    settings["enable_physics"] = True
    cfg = make_cfg(settings)
    sim = habitat_sim.Simulator(cfg)

    navmesh_settings = habitat_sim.NavMeshSettings()
    navmesh_settings.set_defaults()
    navmesh_settings.agent_radius = 0.25
    navmesh_settings.agent_height = 0.88
    sim.recompute_navmesh(sim.pathfinder, navmesh_settings)

    return sim

def sample_navigable_area(sim, n_points=3000):
    """Sample points from navigable area."""
    points = []
    for _ in range(n_points):
        pt = sim.pathfinder.get_random_navigable_point()
        points.append([float(pt[0]), float(pt[2])])
    return np.array(points)

def detect_objects_yolo(sim):
    """Detect objects using YOLO-World."""
    objects = []

    try:
        from ultralytics import YOLO
        yolo = YOLO("yolov8l-world.pt")
        classes = ["chair", "table", "sofa", "couch", "bed", "desk", "tv",
                  "lamp", "door", "refrigerator", "sink", "toilet", "cabinet", "shelf"]
        yolo.set_classes(classes)

        agent = sim.get_agent(0)

        for _ in range(12):
            pos = sim.pathfinder.get_random_navigable_point()
            agent_state = agent.get_state()
            agent_state.position = pos
            agent.set_state(agent_state)

            obs = sim.get_sensor_observations()
            rgb = obs["color_sensor"][:, :, :3]
            depth = obs["depth_sensor"]

            results = yolo.predict(rgb, conf=0.35, verbose=False)
            for box in results[0].boxes:
                cls_id = int(box.cls.cpu().numpy()[0])
                conf = float(box.conf.cpu().numpy()[0])
                xyxy = box.xyxy.cpu().numpy()[0]

                cx = int((xyxy[0] + xyxy[2]) / 2)
                cy = int((xyxy[1] + xyxy[3]) / 2)
                cy = min(cy, depth.shape[0] - 1)
                cx = min(cx, depth.shape[1] - 1)
                d = depth[cy, cx]

                if 0 < d < 8:
                    world_x = float(pos[0] + d * 0.8)
                    world_z = float(pos[2] + (cx - 320) * d / 400)

                    # Dedup
                    exists = False
                    for obj in objects:
                        if obj["label"] == classes[cls_id]:
                            dist = np.sqrt((obj["x"] - world_x)**2 + (obj["z"] - world_z)**2)
                            if dist < 1.5:
                                exists = True
                                break

                    if not exists:
                        objects.append({
                            "id": len(objects) + 1,
                            "label": classes[cls_id],
                            "x": world_x,
                            "z": world_z,
                            "confidence": conf
                        })

        print(f"    Detected {len(objects)} objects")

    except Exception as e:
        print(f"    YOLO error: {e}, using mock objects")
        # Mock objects
        objects = [
            {"id": 1, "label": "sofa", "x": -4.0, "z": -2.0, "confidence": 0.9},
            {"id": 2, "label": "bed", "x": 2.0, "z": -5.0, "confidence": 0.95},
            {"id": 3, "label": "desk", "x": 0.0, "z": -6.0, "confidence": 0.85},
            {"id": 4, "label": "lamp", "x": 3.0, "z": -6.5, "confidence": 0.88},
            {"id": 5, "label": "cabinet", "x": -1.0, "z": -2.0, "confidence": 0.82},
        ]

    return objects

def plan_path(sim, start, goal):
    """Plan path using NavMesh."""
    path = habitat_sim.ShortestPath()
    path.requested_start = start
    path.requested_end = goal

    if sim.pathfinder.find_path(path):
        waypoints = [[float(p[0]), float(p[2])] for p in path.points]
        return waypoints, path.geodesic_distance
    return None, 0

def generate_trajectories(sim, objects):
    """Generate multiple navigation trajectories."""
    trajectories = []

    # Define tasks based on detected objects
    tasks = []
    for obj in objects[:5]:
        tasks.append({
            "instruction": f"Go to the {obj['label']}",
            "target": obj
        })

    agent = sim.get_agent(0)

    for task in tasks:
        # Random start
        start = sim.pathfinder.get_random_navigable_point()
        goal = np.array([task["target"]["x"], 0.16, task["target"]["z"]])

        # Snap goal to navmesh
        goal_snapped = sim.pathfinder.snap_point(goal)
        if np.isnan(goal_snapped).any():
            goal_snapped = sim.pathfinder.get_random_navigable_point()

        waypoints, path_length = plan_path(sim, start, goal_snapped)

        if waypoints and len(waypoints) > 1:
            trajectories.append({
                "instruction": task["instruction"],
                "target_label": task["target"]["label"],
                "start": [float(start[0]), float(start[2])],
                "goal": [float(goal_snapped[0]), float(goal_snapped[2])],
                "waypoints": waypoints,
                "path_length": path_length
            })
            print(f"    {task['instruction']}: {path_length:.1f}m, {len(waypoints)} waypoints")

    return trajectories

def plot_paper_figure(nav_points, objects, trajectories):
    """Generate paper-quality figure."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    fig = plt.figure(figsize=(16, 7))

    # ===== Left: Overview with all trajectories =====
    ax1 = fig.add_subplot(121)

    # Navigable area
    ax1.scatter(nav_points[:, 0], nav_points[:, 1],
               c='#E8E8E8', s=2, alpha=0.6, rasterized=True)

    # Objects
    obj_colors = {
        'sofa': '#8B4513', 'couch': '#8B4513', 'chair': '#A0522D',
        'bed': '#4169E1', 'desk': '#20B2AA', 'table': '#FF8C00',
        'tv': '#32CD32', 'lamp': '#FFD700', 'door': '#DC143C',
        'refrigerator': '#008B8B', 'sink': '#000080', 'toilet': '#FF69B4',
        'cabinet': '#808000', 'shelf': '#696969'
    }

    for obj in objects:
        color = obj_colors.get(obj["label"], '#333333')
        ax1.scatter(obj["x"], obj["z"], c=color, s=120, marker='s',
                   edgecolors='black', linewidths=1.5, zorder=5)
        ax1.annotate(obj["label"], (obj["x"], obj["z"]),
                    fontsize=8, ha='center', va='bottom',
                    fontweight='bold', color='#333333')

    # Trajectories
    traj_colors = ['#E74C3C', '#3498DB', '#2ECC71', '#9B59B6', '#F39C12']

    for i, traj in enumerate(trajectories):
        wp = np.array(traj["waypoints"])
        color = traj_colors[i % len(traj_colors)]

        # Path line
        ax1.plot(wp[:, 0], wp[:, 1], color=color, linewidth=2.5,
                alpha=0.85, label=f'{traj["instruction"][:22]}...')

        # Start marker
        ax1.scatter(traj["start"][0], traj["start"][1],
                   c='#27AE60', s=180, marker='o', edgecolors='white',
                   linewidths=2, zorder=10)

        # Goal marker
        ax1.scatter(traj["goal"][0], traj["goal"][1],
                   c='#E74C3C', s=220, marker='*', edgecolors='white',
                   linewidths=2, zorder=10)

    ax1.set_xlabel('X (meters)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Z (meters)', fontsize=12, fontweight='bold')
    ax1.set_title('(a) Multi-Task Navigation Overview', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper left', fontsize=9, framealpha=0.9)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3, linestyle='--')

    # ===== Right: Single trajectory detail =====
    ax2 = fig.add_subplot(122)

    if trajectories:
        traj = trajectories[0]
        wp = np.array(traj["waypoints"])

        # Zoom to trajectory area
        margin = 3
        x_min, x_max = wp[:, 0].min() - margin, wp[:, 0].max() + margin
        z_min, z_max = wp[:, 1].min() - margin, wp[:, 1].max() + margin

        # Navigable area (zoomed)
        mask = ((nav_points[:, 0] >= x_min) & (nav_points[:, 0] <= x_max) &
               (nav_points[:, 1] >= z_min) & (nav_points[:, 1] <= z_max))
        ax2.scatter(nav_points[mask, 0], nav_points[mask, 1],
                   c='#E8E8E8', s=4, alpha=0.7, rasterized=True)

        # Trajectory with time gradient
        points = wp.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        norm = plt.Normalize(0, len(wp) - 1)
        lc = LineCollection(segments, cmap='plasma', norm=norm, linewidth=4)
        lc.set_array(np.arange(len(wp) - 1))
        ax2.add_collection(lc)

        # Waypoint markers
        scatter = ax2.scatter(wp[:, 0], wp[:, 1],
                             c=np.arange(len(wp)), cmap='plasma',
                             s=50, edgecolors='white', linewidths=1, zorder=5)

        # Start and goal
        ax2.scatter(traj["start"][0], traj["start"][1],
                   c='#27AE60', s=250, marker='o', edgecolors='white',
                   linewidths=3, zorder=10, label='Start')
        ax2.scatter(traj["goal"][0], traj["goal"][1],
                   c='#E74C3C', s=300, marker='*', edgecolors='white',
                   linewidths=3, zorder=10, label=f'Goal: {traj["target_label"]}')

        # Colorbar
        cbar = plt.colorbar(lc, ax=ax2, shrink=0.8)
        cbar.set_label('Time Step', fontsize=10)

        # Info box
        info = (f'Instruction: "{traj["instruction"]}"\n'
               f'Target: {traj["target_label"]}\n'
               f'Path Length: {traj["path_length"]:.2f} m\n'
               f'Waypoints: {len(wp)}')

        props = dict(boxstyle='round,pad=0.5', facecolor='#FFFACD', alpha=0.9,
                    edgecolor='#333333', linewidth=1.5)
        ax2.text(0.03, 0.97, info, transform=ax2.transAxes, fontsize=10,
                verticalalignment='top', bbox=props, fontfamily='monospace')

        ax2.set_xlim(x_min, x_max)
        ax2.set_ylim(z_min, z_max)
        ax2.legend(loc='lower right', fontsize=10)

    ax2.set_xlabel('X (meters)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Z (meters)', fontsize=12, fontweight='bold')
    ax2.set_title('(b) Trajectory Detail with Time Progression', fontsize=14, fontweight='bold')
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3, linestyle='--')

    plt.tight_layout()

    # Save
    for ext in ['pdf', 'png', 'svg']:
        filepath = f"{OUTPUT_DIR}/trajectory_visualization.{ext}"
        plt.savefig(filepath, dpi=300, bbox_inches='tight',
                   facecolor='white', edgecolor='none')
        print(f"    Saved: {filepath}")

    plt.close()

def plot_grid_figure(nav_points, objects, trajectories):
    """Generate grid comparison figure."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    n = min(len(trajectories), 4)
    if n == 0:
        return

    fig, axes = plt.subplots(1, n, figsize=(4.5 * n, 4.5))
    if n == 1:
        axes = [axes]

    for i, (ax, traj) in enumerate(zip(axes, trajectories[:4])):
        wp = np.array(traj["waypoints"])

        margin = 2.5
        x_min, x_max = wp[:, 0].min() - margin, wp[:, 0].max() + margin
        z_min, z_max = wp[:, 1].min() - margin, wp[:, 1].max() + margin

        # Navigable area
        mask = ((nav_points[:, 0] >= x_min) & (nav_points[:, 0] <= x_max) &
               (nav_points[:, 1] >= z_min) & (nav_points[:, 1] <= z_max))
        ax.scatter(nav_points[mask, 0], nav_points[mask, 1],
                  c='#E0E0E0', s=3, alpha=0.6, rasterized=True)

        # Trajectory
        ax.plot(wp[:, 0], wp[:, 1], '#3498DB', linewidth=2.5, alpha=0.9)

        # Markers
        ax.scatter(traj["start"][0], traj["start"][1],
                  c='#27AE60', s=120, marker='o', edgecolors='white', linewidths=2, zorder=10)
        ax.scatter(traj["goal"][0], traj["goal"][1],
                  c='#E74C3C', s=150, marker='*', edgecolors='white', linewidths=2, zorder=10)

        ax.set_title(f'"{traj["instruction"][:18]}..."', fontsize=11, fontweight='bold')
        ax.text(0.5, -0.12, f'{traj["path_length"]:.1f}m | {len(wp)} steps',
               transform=ax.transAxes, ha='center', fontsize=10, style='italic')

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(z_min, z_max)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_xlabel('X (m)', fontsize=10)
        ax.set_ylabel('Z (m)', fontsize=10)

    plt.tight_layout()

    for ext in ['pdf', 'png']:
        filepath = f"{OUTPUT_DIR}/trajectory_grid.{ext}"
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"    Saved: {filepath}")

    plt.close()

def main():
    print("="*70)
    print("  MapPilot Paper-Quality Trajectory Visualization")
    print("="*70)

    print("\n[1] Creating simulator...")
    sim = create_simulator()
    print(f"    Navigable area: {sim.pathfinder.navigable_area:.2f} m²")

    print("\n[2] Sampling navigable area...")
    nav_points = sample_navigable_area(sim, 3000)
    print(f"    Sampled {len(nav_points)} points")

    print("\n[3] Detecting objects...")
    objects = detect_objects_yolo(sim)

    print("\n[4] Generating trajectories...")
    trajectories = generate_trajectories(sim, objects)

    print("\n[5] Generating paper figures...")
    plot_paper_figure(nav_points, objects, trajectories)
    plot_grid_figure(nav_points, objects, trajectories)

    sim.close()

    print("\n" + "="*70)
    print(f"  Done! Figures saved to: {OUTPUT_DIR}")
    print("="*70)

if __name__ == "__main__":
    main()
