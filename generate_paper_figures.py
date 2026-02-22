#!/usr/bin/env python3
"""
MapPilot Paper-Quality Trajectory Visualization
生成论文级别的轨迹俯视图
"""
import sys
import os
import json
import time
import asyncio
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection
from datetime import datetime

sys.path.insert(0, "/home/bsrl/hongsenpang/habitat/MapPilot/src/semantic_planner")

os.environ["KIMI_API_KEY"] = "sk-tpUJJs4F9dlNsPadiWXraVnN1x1PQhHnkbbqo9uMwvLkUtb8"

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings, make_cfg

from semantic_planner.goal_resolver import GoalResolver
from semantic_planner.llm_client import LLMConfig

SCENE_PATH = "/home/bsrl/hongsenpang/habitat/data/hm3d_hf/minival/00800-TEEsavR23oF/TEEsavR23oF.basis.glb"
OUTPUT_DIR = "/home/bsrl/hongsenpang/habitat/paper_figures"

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class TrajectoryVisualizer:
    def __init__(self):
        self.sim = None
        self.resolver = None
        self.trajectories = []
        self.scene_graph = {"objects": [], "relations": [], "regions": []}
        self.navigable_points = []

    def setup(self):
        """Initialize simulator and goal resolver."""
        print("[1] Setting up Habitat Simulator...")
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
        self.sim = habitat_sim.Simulator(cfg)

        # Setup navmesh
        navmesh_settings = habitat_sim.NavMeshSettings()
        navmesh_settings.set_defaults()
        navmesh_settings.agent_radius = 0.25
        navmesh_settings.agent_height = 0.88
        self.sim.recompute_navmesh(self.sim.pathfinder, navmesh_settings)

        print(f"    Navigable area: {self.sim.pathfinder.navigable_area:.2f} m²")

        # Sample navigable points for visualization
        print("[2] Sampling navigable area...")
        for _ in range(2000):
            pt = self.sim.pathfinder.get_random_navigable_point()
            self.navigable_points.append([float(pt[0]), float(pt[2])])  # x, z (top-down)
        self.navigable_points = np.array(self.navigable_points)

        # Setup goal resolver
        print("[3] Setting up Goal Resolver...")
        config = LLMConfig(
            backend="openai",
            model="kimi-k2.5",
            api_key_env="KIMI_API_KEY",
            base_url="https://api.xiaocaseai.com/v1",
            timeout_sec=60.0,
        )
        self.resolver = GoalResolver(primary_config=config)

    def build_scene_graph(self):
        """Build scene graph by exploring the environment."""
        print("[4] Building scene graph...")

        try:
            from ultralytics import YOLO
            yolo = YOLO("yolov8l-world.pt")
            classes = ["chair", "table", "sofa", "couch", "bed", "desk", "tv",
                      "lamp", "door", "refrigerator", "sink", "toilet", "cabinet", "shelf"]
            yolo.set_classes(classes)
            use_yolo = True
        except:
            use_yolo = False
            print("    YOLO not available, using mock detections")

        agent = self.sim.get_agent(0)

        for i in range(10):
            pos = self.sim.pathfinder.get_random_navigable_point()
            agent_state = agent.get_state()
            agent_state.position = pos
            agent.set_state(agent_state)

            obs = self.sim.get_sensor_observations()
            rgb = obs["color_sensor"][:, :, :3]
            depth = obs["depth_sensor"]

            if use_yolo:
                results = yolo.predict(rgb, conf=0.3, verbose=False)
                for box in results[0].boxes:
                    cls_id = int(box.cls.cpu().numpy()[0])
                    conf = float(box.conf.cpu().numpy()[0])
                    xyxy = box.xyxy.cpu().numpy()[0]

                    cx = int((xyxy[0] + xyxy[2]) / 2)
                    cy = int((xyxy[1] + xyxy[3]) / 2)
                    cy = min(cy, depth.shape[0] - 1)
                    cx = min(cx, depth.shape[1] - 1)
                    d = depth[cy, cx]

                    if 0 < d < 10:
                        world_pos = [
                            float(pos[0] + d),
                            float(pos[1]),
                            float(pos[2] + (cx - 320) * d / 320)
                        ]

                        # Check if already exists
                        exists = False
                        for obj in self.scene_graph["objects"]:
                            if obj["label"] == classes[cls_id]:
                                dist = np.sqrt((obj["position"][0] - world_pos[0])**2 +
                                             (obj["position"][2] - world_pos[2])**2)
                                if dist < 1.5:
                                    exists = True
                                    break

                        if not exists:
                            self.scene_graph["objects"].append({
                                "id": len(self.scene_graph["objects"]) + 1,
                                "label": classes[cls_id],
                                "position": world_pos,
                                "confidence": conf
                            })

        print(f"    Detected {len(self.scene_graph['objects'])} objects")

    async def run_navigation_task(self, instruction, start_pos=None):
        """Run a navigation task and record trajectory."""
        agent = self.sim.get_agent(0)

        if start_pos is None:
            start_pos = self.sim.pathfinder.get_random_navigable_point()

        agent_state = agent.get_state()
        agent_state.position = start_pos
        agent.set_state(agent_state)

        # Resolve goal
        scene_graph_json = json.dumps(self.scene_graph)

        t0 = time.time()
        fast_result = self.resolver.fast_resolve(instruction, scene_graph_json)

        if fast_result and fast_result.is_valid:
            goal = np.array([fast_result.target_x, fast_result.target_y, fast_result.target_z])
            target_label = fast_result.target_label
            path_type = "fast"
            resolve_time = (time.time() - t0) * 1000
        else:
            result = await self.resolver.resolve(instruction, scene_graph_json)
            resolve_time = (time.time() - t0) * 1000

            if result and result.is_valid:
                goal = np.array([result.target_x, result.target_y, result.target_z])
                target_label = result.target_label
                path_type = "slow"
            else:
                return None

        # Plan path
        path = habitat_sim.ShortestPath()
        path.requested_start = start_pos
        path.requested_end = goal

        if not self.sim.pathfinder.find_path(path):
            return None

        # Record trajectory
        trajectory = {
            "instruction": instruction,
            "start": [float(start_pos[0]), float(start_pos[2])],
            "goal": [float(goal[0]), float(goal[2])],
            "target_label": target_label,
            "path_type": path_type,
            "resolve_time": resolve_time,
            "path_length": path.geodesic_distance,
            "waypoints": [[float(p[0]), float(p[2])] for p in path.points]
        }

        self.trajectories.append(trajectory)
        return trajectory

    def plot_paper_figure(self, filename="trajectory_visualization.pdf"):
        """Generate paper-quality trajectory visualization."""
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        fig, axes = plt.subplots(1, 2, figsize=(14, 6))

        # ===== Left: Full scene with all trajectories =====
        ax1 = axes[0]

        # Plot navigable area
        ax1.scatter(self.navigable_points[:, 0], self.navigable_points[:, 1],
                   c='lightgray', s=1, alpha=0.3, label='Navigable Area')

        # Plot objects
        colors_obj = {'chair': 'brown', 'table': 'orange', 'sofa': 'purple',
                     'couch': 'purple', 'bed': 'blue', 'desk': 'cyan',
                     'tv': 'green', 'lamp': 'yellow', 'door': 'red',
                     'refrigerator': 'teal', 'sink': 'navy', 'toilet': 'pink',
                     'cabinet': 'olive', 'shelf': 'gray'}

        for obj in self.scene_graph["objects"]:
            color = colors_obj.get(obj["label"], 'black')
            ax1.scatter(obj["position"][0], obj["position"][2],
                       c=color, s=100, marker='s', edgecolors='black', linewidths=1)
            ax1.annotate(obj["label"], (obj["position"][0], obj["position"][2]),
                        fontsize=7, ha='center', va='bottom')

        # Plot trajectories
        colors_traj = plt.cm.Set1(np.linspace(0, 1, len(self.trajectories)))

        for i, traj in enumerate(self.trajectories):
            waypoints = np.array(traj["waypoints"])

            # Plot path
            ax1.plot(waypoints[:, 0], waypoints[:, 1],
                    color=colors_traj[i], linewidth=2, alpha=0.8,
                    label=f'{traj["instruction"][:25]}...')

            # Plot start and goal
            ax1.scatter(traj["start"][0], traj["start"][1],
                       c='green', s=150, marker='o', edgecolors='black',
                       linewidths=2, zorder=10)
            ax1.scatter(traj["goal"][0], traj["goal"][1],
                       c='red', s=150, marker='*', edgecolors='black',
                       linewidths=2, zorder=10)

        ax1.set_xlabel('X (meters)', fontsize=12)
        ax1.set_ylabel('Z (meters)', fontsize=12)
        ax1.set_title('(a) Navigation Trajectories Overview', fontsize=14, fontweight='bold')
        ax1.legend(loc='upper left', fontsize=8)
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)

        # ===== Right: Detailed single trajectory =====
        ax2 = axes[1]

        if self.trajectories:
            traj = self.trajectories[0]  # Use first trajectory
            waypoints = np.array(traj["waypoints"])

            # Plot navigable area (zoomed)
            margin = 3
            x_min, x_max = waypoints[:, 0].min() - margin, waypoints[:, 0].max() + margin
            z_min, z_max = waypoints[:, 1].min() - margin, waypoints[:, 1].max() + margin

            mask = ((self.navigable_points[:, 0] >= x_min) &
                   (self.navigable_points[:, 0] <= x_max) &
                   (self.navigable_points[:, 1] >= z_min) &
                   (self.navigable_points[:, 1] <= z_max))

            ax2.scatter(self.navigable_points[mask, 0], self.navigable_points[mask, 1],
                       c='lightgray', s=3, alpha=0.5)

            # Plot trajectory with gradient color (time progression)
            points = waypoints.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)

            norm = plt.Normalize(0, len(waypoints))
            lc = LineCollection(segments, cmap='viridis', norm=norm, linewidth=3)
            lc.set_array(np.arange(len(waypoints)))
            ax2.add_collection(lc)

            # Plot waypoints
            ax2.scatter(waypoints[:, 0], waypoints[:, 1],
                       c=np.arange(len(waypoints)), cmap='viridis',
                       s=30, edgecolors='black', linewidths=0.5, zorder=5)

            # Plot start and goal
            ax2.scatter(traj["start"][0], traj["start"][1],
                       c='green', s=200, marker='o', edgecolors='black',
                       linewidths=2, zorder=10, label='Start')
            ax2.scatter(traj["goal"][0], traj["goal"][1],
                       c='red', s=200, marker='*', edgecolors='black',
                       linewidths=2, zorder=10, label=f'Goal: {traj["target_label"]}')

            # Add colorbar
            cbar = plt.colorbar(lc, ax=ax2, label='Time Step')

            # Add info box
            info_text = (f'Instruction: "{traj["instruction"]}"\n'
                        f'Path Type: {traj["path_type"].upper()}\n'
                        f'Resolve Time: {traj["resolve_time"]:.0f} ms\n'
                        f'Path Length: {traj["path_length"]:.2f} m\n'
                        f'Waypoints: {len(waypoints)}')

            props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
            ax2.text(0.02, 0.98, info_text, transform=ax2.transAxes, fontsize=9,
                    verticalalignment='top', bbox=props)

            ax2.set_xlim(x_min, x_max)
            ax2.set_ylim(z_min, z_max)
            ax2.legend(loc='lower right', fontsize=10)

        ax2.set_xlabel('X (meters)', fontsize=12)
        ax2.set_ylabel('Z (meters)', fontsize=12)
        ax2.set_title('(b) Detailed Trajectory Analysis', fontsize=14, fontweight='bold')
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()

        # Save figures
        filepath_pdf = f"{OUTPUT_DIR}/{filename}"
        filepath_png = filepath_pdf.replace('.pdf', '.png')

        plt.savefig(filepath_pdf, dpi=300, bbox_inches='tight')
        plt.savefig(filepath_png, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"    Saved: {filepath_pdf}")
        print(f"    Saved: {filepath_png}")

        return filepath_png

    def plot_multi_trajectory_figure(self, filename="multi_trajectory.pdf"):
        """Generate figure with multiple trajectory comparisons."""
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        n_traj = len(self.trajectories)
        if n_traj == 0:
            return None

        fig, axes = plt.subplots(1, min(n_traj, 4), figsize=(4 * min(n_traj, 4), 4))
        if n_traj == 1:
            axes = [axes]

        for i, (ax, traj) in enumerate(zip(axes, self.trajectories[:4])):
            waypoints = np.array(traj["waypoints"])

            # Plot navigable area (zoomed)
            margin = 2
            x_min, x_max = waypoints[:, 0].min() - margin, waypoints[:, 0].max() + margin
            z_min, z_max = waypoints[:, 1].min() - margin, waypoints[:, 1].max() + margin

            mask = ((self.navigable_points[:, 0] >= x_min) &
                   (self.navigable_points[:, 0] <= x_max) &
                   (self.navigable_points[:, 1] >= z_min) &
                   (self.navigable_points[:, 1] <= z_max))

            ax.scatter(self.navigable_points[mask, 0], self.navigable_points[mask, 1],
                      c='lightgray', s=2, alpha=0.5)

            # Plot trajectory
            ax.plot(waypoints[:, 0], waypoints[:, 1],
                   'b-', linewidth=2, alpha=0.8)

            # Plot start and goal
            ax.scatter(traj["start"][0], traj["start"][1],
                      c='green', s=100, marker='o', edgecolors='black', linewidths=2)
            ax.scatter(traj["goal"][0], traj["goal"][1],
                      c='red', s=100, marker='*', edgecolors='black', linewidths=2)

            # Title with instruction
            title = f'"{traj["instruction"][:20]}..."'
            ax.set_title(title, fontsize=10)

            # Add metrics
            metrics = f'{traj["path_type"].upper()} | {traj["path_length"]:.1f}m'
            ax.text(0.5, -0.1, metrics, transform=ax.transAxes,
                   ha='center', fontsize=9, style='italic')

            ax.set_xlim(x_min, x_max)
            ax.set_ylim(z_min, z_max)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_xlabel('X (m)', fontsize=9)
            ax.set_ylabel('Z (m)', fontsize=9)

        plt.tight_layout()

        filepath_pdf = f"{OUTPUT_DIR}/{filename}"
        filepath_png = filepath_pdf.replace('.pdf', '.png')

        plt.savefig(filepath_pdf, dpi=300, bbox_inches='tight')
        plt.savefig(filepath_png, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"    Saved: {filepath_pdf}")
        print(f"    Saved: {filepath_png}")

        return filepath_png

    async def run(self):
        """Run full visualization pipeline."""
        print("="*70)
        print("  MapPilot Paper-Quality Trajectory Visualization")
        print("="*70)

        self.setup()
        self.build_scene_graph()

        # Run multiple navigation tasks
        instructions = [
            "Go to the sofa",
            "Find a place to sleep",
            "Navigate to the desk",
            "Go to where I can sit and relax",
        ]

        print("\n[5] Running navigation tasks...")
        for instr in instructions:
            print(f'    Task: "{instr}"')
            traj = await self.run_navigation_task(instr)
            if traj:
                print(f'        -> {traj["target_label"]} ({traj["path_type"]}, {traj["path_length"]:.1f}m)')
            else:
                print(f'        -> Failed')

        # Generate figures
        print("\n[6] Generating paper figures...")
        fig1 = self.plot_paper_figure("trajectory_overview.pdf")
        fig2 = self.plot_multi_trajectory_figure("trajectory_comparison.pdf")

        # Cleanup
        self.sim.close()

        print("\n" + "="*70)
        print("  Visualization Complete!")
        print(f"  Output: {OUTPUT_DIR}")
        print("="*70)

        return [fig1, fig2]

async def main():
    viz = TrajectoryVisualizer()
    await viz.run()

if __name__ == "__main__":
    asyncio.run(main())
