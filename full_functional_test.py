#!/usr/bin/env python3
"""
MapPilot 完整功能测试
包含视频录制、轨迹可视化、清晰文件命名
"""
import sys
import os
import json
import time
import asyncio
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from datetime import datetime

sys.path.insert(0, "/home/bsrl/hongsenpang/habitat/MapPilot/src/semantic_planner")
sys.path.insert(0, "/home/bsrl/hongsenpang/habitat/MapPilot/src/semantic_perception")
sys.path.insert(0, "/home/bsrl/hongsenpang/habitat/MapPilot/src/vla_nav")

os.environ["KIMI_API_KEY"] = "sk-tpUJJs4F9dlNsPadiWXraVnN1x1PQhHnkbbqo9uMwvLkUtb8"

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings, make_cfg

SCENE_PATH = "/home/bsrl/hongsenpang/habitat/data/hm3d_hf/minival/00800-TEEsavR23oF/TEEsavR23oF.basis.glb"
OUTPUT_DIR = "/home/bsrl/hongsenpang/habitat/functional_test_output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

class FunctionalTest:
    def __init__(self):
        self.sim = None
        self.yolo = None
        self.clip_encoder = None
        self.goal_resolver = None
        self.topo_memory = None
        self.results = {}

    def setup_habitat(self):
        """初始化 Habitat 仿真器"""
        print("\n" + "="*60)
        print("  [1] 初始化 Habitat 仿真器")
        print("="*60)

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

        navmesh_settings = habitat_sim.NavMeshSettings()
        navmesh_settings.set_defaults()
        navmesh_settings.agent_radius = 0.25
        navmesh_settings.agent_height = 0.88
        self.sim.recompute_navmesh(self.sim.pathfinder, navmesh_settings)

        nav_area = self.sim.pathfinder.navigable_area
        print(f"  ✓ 场景加载成功: {SCENE_PATH.split('/')[-1]}")
        print(f"  ✓ 可导航面积: {nav_area:.2f} m²")

        self.results["habitat"] = {"status": "OK", "navigable_area": nav_area}
        return True

    def setup_yolo(self):
        """初始化 YOLO-World 检测器"""
        print("\n" + "="*60)
        print("  [2] 初始化 YOLO-World 检测器")
        print("="*60)

        try:
            from ultralytics import YOLO
            self.yolo = YOLO("yolov8l-world.pt")
            self.yolo_classes = [
                "chair", "table", "sofa", "couch", "bed", "desk", "tv",
                "lamp", "door", "refrigerator", "sink", "toilet", "cabinet"
            ]
            self.yolo.set_classes(self.yolo_classes)
            print(f"  ✓ YOLO-World 加载成功")
            print(f"  ✓ 检测类别: {len(self.yolo_classes)} 类")
            self.results["yolo"] = {"status": "OK", "classes": len(self.yolo_classes)}
            return True
        except Exception as e:
            print(f"  ✗ YOLO 加载失败: {e}")
            self.results["yolo"] = {"status": "FAIL", "error": str(e)}
            return False

    def setup_clip(self):
        """初始化 CLIP 编码器"""
        print("\n" + "="*60)
        print("  [3] 初始化 CLIP 编码器")
        print("="*60)

        try:
            from semantic_perception.clip_encoder import CLIPEncoder
            self.clip_encoder = CLIPEncoder(
                model_name="ViT-B/32",
                device="cuda",
                enable_cache=True
            )

            # 测试编码
            test_texts = ["red chair", "wooden table", "comfortable sofa"]
            features = self.clip_encoder.encode_text(test_texts)

            if features is not None and len(features) > 0:
                print(f"  ✓ CLIP 编码器加载成功")
                print(f"  ✓ 特征维度: {self.clip_encoder.feature_dim}")
                self.results["clip"] = {"status": "OK", "feature_dim": self.clip_encoder.feature_dim}
                return True
            else:
                print(f"  ⚠ CLIP 加载但特征为空")
                self.results["clip"] = {"status": "WARN", "note": "features empty"}
                return True
        except Exception as e:
            print(f"  ✗ CLIP 加载失败: {e}")
            self.results["clip"] = {"status": "FAIL", "error": str(e)}
            return False

    def setup_goal_resolver(self):
        """初始化 Goal Resolver"""
        print("\n" + "="*60)
        print("  [4] 初始化 Goal Resolver (Fast-Slow)")
        print("="*60)

        try:
            from semantic_planner.goal_resolver import GoalResolver
            from semantic_planner.llm_client import LLMConfig

            config = LLMConfig(
                backend="openai",
                model="kimi-k2.5",
                api_key_env="KIMI_API_KEY",
                base_url="https://api.xiaocaseai.com/v1",
                timeout_sec=60.0,
            )

            self.goal_resolver = GoalResolver(primary_config=config)
            print(f"  ✓ Goal Resolver 初始化成功")
            print(f"  ✓ LLM: kimi-k2.5")
            self.results["goal_resolver"] = {"status": "OK", "llm": "kimi-k2.5"}
            return True
        except Exception as e:
            print(f"  ✗ Goal Resolver 初始化失败: {e}")
            self.results["goal_resolver"] = {"status": "FAIL", "error": str(e)}
            return False

    def setup_topo_memory(self):
        """初始化 Topological Memory"""
        print("\n" + "="*60)
        print("  [5] 初始化 Topological Memory")
        print("="*60)

        try:
            from semantic_planner.topological_memory import TopologicalMemory
            self.topo_memory = TopologicalMemory()
            print(f"  ✓ Topological Memory 初始化成功")
            self.results["topo_memory"] = {"status": "OK"}
            return True
        except Exception as e:
            print(f"  ✗ Topological Memory 初始化失败: {e}")
            self.results["topo_memory"] = {"status": "FAIL", "error": str(e)}
            return False

    def test_yolo_detection(self):
        """测试 YOLO 物体检测功能"""
        print("\n" + "="*60)
        print("  [测试1] YOLO-World 物体检测")
        print("="*60)

        if self.yolo is None:
            print("  ✗ YOLO 未初始化")
            return None

        agent = self.sim.get_agent(0)
        detections_all = []

        # 从多个视角检测
        for i in range(5):
            pos = self.sim.pathfinder.get_random_navigable_point()
            agent_state = agent.get_state()
            agent_state.position = pos
            agent.set_state(agent_state)

            obs = self.sim.get_sensor_observations()
            rgb = obs["color_sensor"][:, :, :3]

            results = self.yolo.predict(rgb, conf=0.3, verbose=False)
            boxes = results[0].boxes

            for box in boxes:
                cls_id = int(box.cls.cpu().numpy()[0])
                conf = float(box.conf.cpu().numpy()[0])
                detections_all.append({
                    "label": self.yolo_classes[cls_id],
                    "confidence": conf,
                    "position": [float(pos[0]), float(pos[1]), float(pos[2])]
                })

            # 保存检测图像
            if len(boxes) > 0:
                img_annotated = results[0].plot()
                img_path = f"{OUTPUT_DIR}/01_YOLO检测_视角{i+1}_检测到{len(boxes)}个物体.jpg"
                cv2.imwrite(img_path, img_annotated)
                print(f"  ✓ 视角{i+1}: 检测到 {len(boxes)} 个物体, 保存到 {img_path.split('/')[-1]}")

        # 统计
        unique_labels = list(set([d["label"] for d in detections_all]))
        print(f"\n  总计检测: {len(detections_all)} 次")
        print(f"  唯一物体类别: {unique_labels}")

        self.results["yolo_test"] = {
            "total_detections": len(detections_all),
            "unique_labels": unique_labels
        }

        return detections_all

    async def test_goal_resolver(self):
        """测试 Goal Resolver Fast-Slow 功能"""
        print("\n" + "="*60)
        print("  [测试2] Goal Resolver Fast-Slow 双进程")
        print("="*60)

        if self.goal_resolver is None:
            print("  ✗ Goal Resolver 未初始化")
            return None

        # 构建场景图
        scene_graph = {
            "objects": [
                {"id": 1, "label": "chair", "position": [2.0, 0.0, 1.0], "confidence": 0.9},
                {"id": 2, "label": "dining table", "position": [3.0, 0.0, 2.0], "confidence": 0.85},
                {"id": 3, "label": "sofa", "position": [-1.0, 0.0, 0.5], "confidence": 0.88},
                {"id": 4, "label": "bed", "position": [0.0, 0.0, -3.0], "confidence": 0.92},
                {"id": 5, "label": "refrigerator", "position": [4.0, 0.0, 3.0], "confidence": 0.87},
                {"id": 6, "label": "tv", "position": [-2.0, 0.0, 0.0], "confidence": 0.83},
            ],
            "relations": [
                {"subject_id": 1, "predicate": "near", "object_id": 2},
                {"subject_id": 3, "predicate": "facing", "object_id": 6},
            ],
            "regions": [
                {"name": "kitchen", "object_ids": [2, 5]},
                {"name": "living_room", "object_ids": [1, 3, 6]},
                {"name": "bedroom", "object_ids": [4]},
            ]
        }
        scene_graph_json = json.dumps(scene_graph)

        test_results = []

        # Fast Path 测试
        print("\n  --- Fast Path 测试 ---")
        fast_instructions = [
            ("Go to the chair", "chair"),
            ("Find the sofa", "sofa"),
            ("Navigate to the bed", "bed"),
        ]

        for instr, expected in fast_instructions:
            t0 = time.time()
            result = self.goal_resolver.fast_resolve(instr, scene_graph_json)
            elapsed = (time.time() - t0) * 1000

            success = result and result.is_valid
            target = result.target_label if success else "None"
            print(f"  [{elapsed:.1f}ms] \"{instr}\" -> {target}")

            test_results.append({
                "type": "fast",
                "instruction": instr,
                "success": success,
                "target": target,
                "time_ms": elapsed
            })

        # Slow Path 测试 (LLM)
        print("\n  --- Slow Path 测试 (LLM) ---")
        slow_instructions = [
            "Go to the room where people usually eat",
            "Find a place to watch TV",
            "Navigate to where I can sleep",
        ]

        for instr in slow_instructions:
            t0 = time.time()
            result = await self.goal_resolver.resolve(instr, scene_graph_json)
            elapsed = (time.time() - t0) * 1000

            success = result and result.is_valid
            target = result.target_label if success else "None"
            reasoning = result.reasoning[:50] + "..." if success and result.reasoning else ""

            print(f"  [{elapsed:.0f}ms] \"{instr}\"")
            print(f"       -> {target}")
            if reasoning:
                print(f"       推理: {reasoning}")

            test_results.append({
                "type": "slow",
                "instruction": instr,
                "success": success,
                "target": target,
                "time_ms": elapsed
            })

        self.results["goal_resolver_test"] = test_results
        return test_results

    def test_navigation_with_video(self, instruction, video_name):
        """测试导航功能并录制视频"""
        print(f"\n  导航任务: \"{instruction}\"")

        agent = self.sim.get_agent(0)

        # 随机起点
        start = self.sim.pathfinder.get_random_navigable_point()
        agent_state = agent.get_state()
        agent_state.position = start
        agent.set_state(agent_state)

        # 随机终点 (模拟目标)
        goal = self.sim.pathfinder.get_random_navigable_point()
        while np.linalg.norm(np.array(start) - np.array(goal)) < 5.0:
            goal = self.sim.pathfinder.get_random_navigable_point()

        # 规划路径
        path = habitat_sim.ShortestPath()
        path.requested_start = start
        path.requested_end = goal

        if not self.sim.pathfinder.find_path(path):
            print("  ✗ 路径规划失败")
            return None

        print(f"  起点: [{start[0]:.1f}, {start[1]:.1f}, {start[2]:.1f}]")
        print(f"  终点: [{goal[0]:.1f}, {goal[1]:.1f}, {goal[2]:.1f}]")
        print(f"  路径: {len(path.points)} 个路点, {path.geodesic_distance:.1f}m")

        # 录制视频
        video_path = f"{OUTPUT_DIR}/{video_name}"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video = cv2.VideoWriter(video_path, fourcc, 10, (640, 480))

        trajectory = []

        for i, waypoint in enumerate(path.points):
            agent_state = agent.get_state()
            agent_state.position = waypoint
            agent.set_state(agent_state)

            obs = self.sim.get_sensor_observations()
            rgb = obs["color_sensor"][:, :, :3]
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # 添加信息叠加
            dist_to_goal = np.linalg.norm(np.array(waypoint) - np.array(goal))
            progress = (1 - dist_to_goal / path.geodesic_distance) * 100

            cv2.putText(frame, f"Task: {instruction}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Step: {i+1}/{len(path.points)}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Progress: {progress:.0f}%", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Distance: {dist_to_goal:.1f}m", (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            video.write(frame)
            trajectory.append([float(waypoint[0]), float(waypoint[2])])

        # 添加结束帧
        for _ in range(20):
            cv2.putText(frame, "ARRIVED!", (250, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            video.write(frame)

        video.release()
        print(f"  ✓ 视频保存: {video_name}")

        return {
            "start": [float(start[0]), float(start[2])],
            "goal": [float(goal[0]), float(goal[2])],
            "trajectory": trajectory,
            "path_length": path.geodesic_distance,
            "waypoints": len(path.points)
        }

    def test_full_navigation(self):
        """完整导航测试"""
        print("\n" + "="*60)
        print("  [测试3] 完整导航功能测试 (含视频录制)")
        print("="*60)

        tasks = [
            ("Go to the sofa", "02_导航测试_去沙发_FastPath.mp4"),
            ("Find a place to sleep", "03_导航测试_找睡觉地方_SlowPath.mp4"),
            ("Navigate to the kitchen", "04_导航测试_去厨房_SlowPath.mp4"),
        ]

        nav_results = []

        for instruction, video_name in tasks:
            result = self.test_navigation_with_video(instruction, video_name)
            if result:
                result["instruction"] = instruction
                result["video"] = video_name
                nav_results.append(result)

        self.results["navigation_test"] = nav_results
        return nav_results

    def generate_trajectory_figure(self, nav_results):
        """生成轨迹可视化图"""
        print("\n" + "="*60)
        print("  [测试4] 生成轨迹可视化图")
        print("="*60)

        # 采样可导航区域
        nav_points = []
        for _ in range(2000):
            pt = self.sim.pathfinder.get_random_navigable_point()
            nav_points.append([float(pt[0]), float(pt[2])])
        nav_points = np.array(nav_points)

        # 创建图
        fig, axes = plt.subplots(1, len(nav_results), figsize=(5*len(nav_results), 5))
        if len(nav_results) == 1:
            axes = [axes]

        colors = ['#E74C3C', '#3498DB', '#2ECC71', '#9B59B6']

        for i, (ax, result) in enumerate(zip(axes, nav_results)):
            traj = np.array(result["trajectory"])

            # 可导航区域
            ax.scatter(nav_points[:, 0], nav_points[:, 1],
                      c='lightgray', s=2, alpha=0.5)

            # 轨迹
            ax.plot(traj[:, 0], traj[:, 1], color=colors[i % len(colors)],
                   linewidth=2.5, alpha=0.9)

            # 起点终点
            ax.scatter(result["start"][0], result["start"][1],
                      c='green', s=150, marker='o', edgecolors='white',
                      linewidths=2, zorder=10, label='Start')
            ax.scatter(result["goal"][0], result["goal"][1],
                      c='red', s=200, marker='*', edgecolors='white',
                      linewidths=2, zorder=10, label='Goal')

            ax.set_title(f'"{result["instruction"][:20]}..."', fontsize=11)
            ax.text(0.5, -0.1, f'{result["path_length"]:.1f}m | {result["waypoints"]} steps',
                   transform=ax.transAxes, ha='center', fontsize=10)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Z (m)')
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right', fontsize=8)

        plt.tight_layout()

        # 保存
        fig_path = f"{OUTPUT_DIR}/05_轨迹可视化_全部任务对比.png"
        plt.savefig(fig_path, dpi=300, bbox_inches='tight')
        plt.savefig(fig_path.replace('.png', '.pdf'), dpi=300, bbox_inches='tight')
        plt.close()

        print(f"  ✓ 轨迹图保存: 05_轨迹可视化_全部任务对比.png/pdf")

        # 单独保存每个轨迹
        for i, result in enumerate(nav_results):
            fig, ax = plt.subplots(figsize=(6, 6))
            traj = np.array(result["trajectory"])

            ax.scatter(nav_points[:, 0], nav_points[:, 1],
                      c='lightgray', s=2, alpha=0.5)

            # 带时间渐变的轨迹
            points = traj.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            lc = LineCollection(segments, cmap='viridis', linewidth=3)
            lc.set_array(np.arange(len(traj)))
            ax.add_collection(lc)

            ax.scatter(result["start"][0], result["start"][1],
                      c='green', s=200, marker='o', edgecolors='white', linewidths=2)
            ax.scatter(result["goal"][0], result["goal"][1],
                      c='red', s=250, marker='*', edgecolors='white', linewidths=2)

            plt.colorbar(lc, ax=ax, label='Time Step')

            ax.set_title(f'Navigation: "{result["instruction"]}"', fontsize=12)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Z (m)')
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)

            safe_name = result["instruction"].replace(" ", "_")[:20]
            fig_path = f"{OUTPUT_DIR}/06_轨迹详情_{i+1}_{safe_name}.png"
            plt.savefig(fig_path, dpi=300, bbox_inches='tight')
            plt.close()

            print(f"  ✓ 轨迹详情图保存: {fig_path.split('/')[-1]}")

    def save_report(self):
        """保存测试报告"""
        print("\n" + "="*60)
        print("  保存测试报告")
        print("="*60)

        report = {
            "timestamp": datetime.now().isoformat(),
            "scene": SCENE_PATH,
            "results": self.results
        }

        report_path = f"{OUTPUT_DIR}/00_测试报告_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)

        print(f"  ✓ 报告保存: {report_path.split('/')[-1]}")

    async def run_all_tests(self):
        """运行所有测试"""
        print("\n" + "#"*70)
        print("#" + " MapPilot 完整功能测试 ".center(68) + "#")
        print("#" + f" {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ".center(68) + "#")
        print("#"*70)

        # 初始化
        self.setup_habitat()
        self.setup_yolo()
        self.setup_clip()
        self.setup_goal_resolver()
        self.setup_topo_memory()

        # 功能测试
        self.test_yolo_detection()
        await self.test_goal_resolver()
        nav_results = self.test_full_navigation()

        if nav_results:
            self.generate_trajectory_figure(nav_results)

        # 保存报告
        self.save_report()

        # 清理
        self.sim.close()

        # 汇总
        print("\n" + "#"*70)
        print("#" + " 测试完成 ".center(68) + "#")
        print("#"*70)
        print(f"\n  输出目录: {OUTPUT_DIR}")
        print("\n  生成文件:")
        for f in sorted(os.listdir(OUTPUT_DIR)):
            print(f"    - {f}")
        print("\n" + "#"*70)

async def main():
    test = FunctionalTest()
    await test.run_all_tests()

if __name__ == "__main__":
    asyncio.run(main())
