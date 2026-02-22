#!/usr/bin/env python3
"""
MapPilot 关键模块完整测试
测试所有论文相关的核心模块
"""
import sys
import os
import json
import time
import asyncio
import numpy as np
from datetime import datetime

sys.path.insert(0, "/home/bsrl/hongsenpang/habitat/MapPilot/src/semantic_planner")
sys.path.insert(0, "/home/bsrl/hongsenpang/habitat/MapPilot/src/semantic_perception")
sys.path.insert(0, "/home/bsrl/hongsenpang/habitat/MapPilot/src/vla_nav")

os.environ["KIMI_API_KEY"] = "sk-tpUJJs4F9dlNsPadiWXraVnN1x1PQhHnkbbqo9uMwvLkUtb8"

OUTPUT_DIR = "/home/bsrl/hongsenpang/habitat/test_results"
os.makedirs(OUTPUT_DIR, exist_ok=True)

class TestResult:
    def __init__(self, name):
        self.name = name
        self.passed = False
        self.details = ""
        self.metrics = {}
        self.time_ms = 0

    def __str__(self):
        status = "PASS" if self.passed else "FAIL"
        return f"[{status}] {self.name}: {self.details}"

def print_header(title):
    print("\n" + "="*70)
    print(f"  {title}")
    print("="*70)

def print_result(result):
    status = "\033[92mPASS\033[0m" if result.passed else "\033[91mFAIL\033[0m"
    print(f"  [{status}] {result.name}")
    if result.details:
        print(f"         {result.details}")
    if result.metrics:
        for k, v in result.metrics.items():
            print(f"         {k}: {v}")

# ============================================================
# 1. Frontier Scorer (MTU3D) 测试
# ============================================================
def test_frontier_scorer():
    result = TestResult("Frontier Scorer (MTU3D)")
    t0 = time.time()

    try:
        from semantic_planner.frontier_scorer import FrontierScorer, Frontier

        # 创建测试场景
        scorer = FrontierScorer(
            grounding_weight=0.4,
            distance_weight=0.3,
            information_weight=0.3
        )

        # 模拟场景图
        scene_graph = {
            "objects": [
                {"id": 1, "label": "chair", "position": [2.0, 0.0, 1.0], "confidence": 0.9},
                {"id": 2, "label": "table", "position": [3.0, 0.0, 2.0], "confidence": 0.85},
            ]
        }

        # 创建测试边界点
        frontiers = [
            Frontier(position=np.array([5.0, 0.0, 0.0]), normal=np.array([1.0, 0.0, 0.0]), size=2.0),
            Frontier(position=np.array([0.0, 0.0, 5.0]), normal=np.array([0.0, 0.0, 1.0]), size=1.5),
            Frontier(position=np.array([-3.0, 0.0, 0.0]), normal=np.array([-1.0, 0.0, 0.0]), size=1.0),
        ]

        robot_pos = np.array([0.0, 0.0, 0.0])
        target_instruction = "find the kitchen"

        # 评分
        scores = scorer.score_frontiers(
            frontiers=frontiers,
            robot_position=robot_pos,
            target_instruction=target_instruction,
            scene_graph=scene_graph
        )

        result.passed = len(scores) == len(frontiers) and all(0 <= s <= 1 for s in scores)
        result.details = f"Scored {len(frontiers)} frontiers"
        result.metrics = {
            "scores": [f"{s:.3f}" for s in scores],
            "best_frontier": int(np.argmax(scores))
        }

    except Exception as e:
        result.details = str(e)

    result.time_ms = (time.time() - t0) * 1000
    return result

# ============================================================
# 2. Topological Memory 测试
# ============================================================
def test_topological_memory():
    result = TestResult("Topological Memory")
    t0 = time.time()

    try:
        from semantic_planner.topological_memory import TopologicalMemory, MemoryNode

        memory = TopologicalMemory(
            distance_threshold=2.0,
            similarity_threshold=0.8
        )

        # 添加节点
        nodes_added = 0
        positions = [
            [0.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [3.0, 0.0, 3.0],
            [0.0, 0.0, 3.0],
            [1.5, 0.0, 1.5],  # 中心点，应该连接多个节点
        ]

        for i, pos in enumerate(positions):
            node = MemoryNode(
                position=np.array(pos),
                observation=f"observation_{i}",
                timestamp=time.time()
            )
            memory.add_node(node)
            nodes_added += 1

        # 查询最近节点
        query_pos = np.array([1.0, 0.0, 1.0])
        nearest = memory.get_nearest_node(query_pos)

        # 获取路径
        path = memory.get_path(0, 2)  # 从节点0到节点2

        result.passed = nodes_added == len(positions) and nearest is not None
        result.details = f"Added {nodes_added} nodes, path length: {len(path) if path else 0}"
        result.metrics = {
            "total_nodes": memory.num_nodes,
            "total_edges": memory.num_edges,
            "nearest_to_query": nearest.position.tolist() if nearest else None
        }

    except Exception as e:
        result.details = str(e)

    result.time_ms = (time.time() - t0) * 1000
    return result

# ============================================================
# 3. Instance Tracker (ConceptGraphs) 测试
# ============================================================
def test_instance_tracker():
    result = TestResult("Instance Tracker (ConceptGraphs)")
    t0 = time.time()

    try:
        from semantic_perception.instance_tracker import InstanceTracker

        tracker = InstanceTracker(
            distance_threshold=1.0,
            iou_threshold=0.5,
            max_instances=100
        )

        # 模拟检测序列
        detections_seq = [
            # Frame 1
            [
                {"label": "chair", "bbox": [100, 100, 200, 200], "position": [1.0, 0.0, 2.0], "confidence": 0.9},
                {"label": "table", "bbox": [300, 100, 400, 200], "position": [3.0, 0.0, 2.0], "confidence": 0.85},
            ],
            # Frame 2 (chair moved slightly)
            [
                {"label": "chair", "bbox": [105, 102, 205, 202], "position": [1.1, 0.0, 2.1], "confidence": 0.88},
                {"label": "table", "bbox": [300, 100, 400, 200], "position": [3.0, 0.0, 2.0], "confidence": 0.87},
                {"label": "lamp", "bbox": [500, 50, 550, 150], "position": [5.0, 0.0, 1.0], "confidence": 0.75},
            ],
            # Frame 3 (new object)
            [
                {"label": "chair", "bbox": [110, 105, 210, 205], "position": [1.2, 0.0, 2.2], "confidence": 0.86},
                {"label": "sofa", "bbox": [200, 300, 350, 400], "position": [2.0, 0.0, 4.0], "confidence": 0.92},
            ],
        ]

        for frame_id, detections in enumerate(detections_seq):
            tracker.update(detections, frame_id)

        # 获取场景图
        scene_graph = tracker.get_scene_graph()

        result.passed = len(scene_graph["objects"]) >= 3
        result.details = f"Tracked {len(scene_graph['objects'])} unique objects from {len(detections_seq)} frames"
        result.metrics = {
            "unique_objects": len(scene_graph["objects"]),
            "labels": [obj["label"] for obj in scene_graph["objects"]],
            "relations": len(scene_graph.get("relations", []))
        }

    except Exception as e:
        result.details = str(e)

    result.time_ms = (time.time() - t0) * 1000
    return result

# ============================================================
# 4. CLIP Encoder 测试
# ============================================================
def test_clip_encoder():
    result = TestResult("CLIP Encoder")
    t0 = time.time()

    try:
        from semantic_perception.clip_encoder import CLIPEncoder

        encoder = CLIPEncoder(
            model_name="ViT-B/32",
            device="cuda"
        )

        # 测试文本编码
        texts = ["a red chair", "a wooden table", "a comfortable sofa"]
        text_features = encoder.encode_text(texts)

        # 测试相似度计算
        query = "furniture for sitting"
        query_feature = encoder.encode_text([query])

        similarities = []
        for i, feat in enumerate(text_features):
            sim = float(np.dot(query_feature[0], feat) / (np.linalg.norm(query_feature[0]) * np.linalg.norm(feat)))
            similarities.append((texts[i], sim))

        similarities.sort(key=lambda x: x[1], reverse=True)

        result.passed = text_features.shape[0] == len(texts) and text_features.shape[1] == 512
        result.details = f"Encoded {len(texts)} texts, feature dim: {text_features.shape[1]}"
        result.metrics = {
            "feature_shape": text_features.shape,
            "query": query,
            "top_match": similarities[0] if similarities else None,
            "cache_hit_rate": f"{encoder.cache_hit_rate*100:.1f}%" if hasattr(encoder, 'cache_hit_rate') else "N/A"
        }

    except Exception as e:
        result.details = str(e)

    result.time_ms = (time.time() - t0) * 1000
    return result

# ============================================================
# 5. Task Decomposer (SayCan) 测试
# ============================================================
async def test_task_decomposer():
    result = TestResult("Task Decomposer (SayCan)")
    t0 = time.time()

    try:
        from semantic_planner.task_decomposer import TaskDecomposer
        from semantic_planner.llm_client import LLMConfig

        config = LLMConfig(
            backend="openai",
            model="kimi-k2.5",
            api_key_env="KIMI_API_KEY",
            base_url="https://api.xiaocaseai.com/v1",
            timeout_sec=60.0,
        )

        decomposer = TaskDecomposer(llm_config=config)

        # 测试复杂任务分解
        complex_task = "Go to the kitchen, find a cup, and bring it to the living room"

        subtasks = await decomposer.decompose(complex_task)

        result.passed = len(subtasks) >= 2
        result.details = f"Decomposed into {len(subtasks)} subtasks"
        result.metrics = {
            "original_task": complex_task,
            "subtasks": subtasks[:5] if subtasks else []
        }

    except Exception as e:
        result.details = str(e)

    result.time_ms = (time.time() - t0) * 1000
    return result

# ============================================================
# 6. Goal Resolver (Fast-Slow) 测试
# ============================================================
async def test_goal_resolver():
    result = TestResult("Goal Resolver (Fast-Slow)")
    t0 = time.time()

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

        resolver = GoalResolver(primary_config=config)

        scene_graph = {
            "objects": [
                {"id": 1, "label": "chair", "position": [2.0, 0.0, 1.0], "confidence": 0.9},
                {"id": 2, "label": "dining table", "position": [3.0, 0.0, 2.0], "confidence": 0.85},
                {"id": 3, "label": "sofa", "position": [-1.0, 0.0, 0.5], "confidence": 0.88},
                {"id": 4, "label": "bed", "position": [0.0, 0.0, -3.0], "confidence": 0.92},
                {"id": 5, "label": "refrigerator", "position": [4.0, 0.0, 3.0], "confidence": 0.87},
            ],
            "relations": [
                {"subject_id": 1, "predicate": "near", "object_id": 2},
                {"subject_id": 5, "predicate": "in_same_room", "object_id": 2},
            ],
            "regions": [
                {"name": "kitchen", "object_ids": [2, 5]},
                {"name": "living_room", "object_ids": [1, 3]},
                {"name": "bedroom", "object_ids": [4]},
            ]
        }
        scene_graph_json = json.dumps(scene_graph)

        # 测试 Fast Path
        fast_tests = [
            ("Go to the chair", "chair"),
            ("Find the sofa", "sofa"),
            ("Navigate to the bed", "bed"),
        ]

        fast_results = []
        for instr, expected in fast_tests:
            t1 = time.time()
            res = resolver.fast_resolve(instr, scene_graph_json)
            elapsed = (time.time() - t1) * 1000
            success = res and res.is_valid and expected in res.target_label.lower()
            fast_results.append({
                "instruction": instr,
                "success": success,
                "target": res.target_label if res and res.is_valid else None,
                "time_ms": elapsed
            })

        # 测试 Slow Path
        slow_instr = "Go to the room where people usually eat"
        t1 = time.time()
        slow_res = await resolver.resolve(slow_instr, scene_graph_json)
        slow_time = (time.time() - t1) * 1000

        fast_success = sum(1 for r in fast_results if r["success"])
        slow_success = slow_res and slow_res.is_valid

        result.passed = fast_success >= 2 and slow_success
        result.details = f"Fast: {fast_success}/{len(fast_tests)}, Slow: {'OK' if slow_success else 'FAIL'}"
        result.metrics = {
            "fast_path_results": fast_results,
            "slow_path": {
                "instruction": slow_instr,
                "target": slow_res.target_label if slow_res and slow_res.is_valid else None,
                "time_ms": slow_time
            }
        }

    except Exception as e:
        result.details = str(e)
        import traceback
        traceback.print_exc()

    result.time_ms = (time.time() - t0) * 1000
    return result

# ============================================================
# 7. VLA Model Components 测试
# ============================================================
def test_vla_model():
    result = TestResult("VLA Model Components")
    t0 = time.time()

    try:
        import torch

        components = {}

        # AdaCoT Module
        try:
            from vla_nav.model.adacot import AdaCoTModule
            adacot = AdaCoTModule(hidden_dim=1024)
            x = torch.randn(2, 1024)
            out = adacot(x)
            components["AdaCoT"] = {"status": "OK", "output_shape": list(out.shape)}
        except Exception as e:
            components["AdaCoT"] = {"status": "FAIL", "error": str(e)}

        # Action Head
        try:
            from vla_nav.model.action_head import ActionHead
            head = ActionHead(hidden_dim=1024, action_dim=3, horizon=5)
            x = torch.randn(2, 1024)
            out = head(x)
            components["ActionHead"] = {"status": "OK", "output_shape": list(out.shape)}
        except Exception as e:
            components["ActionHead"] = {"status": "FAIL", "error": str(e)}

        # VLingMem
        try:
            from vla_nav.model.vlingmem import VLingMemModule
            mem = VLingMemModule(hidden_dim=1024, max_entries=100)
            components["VLingMem"] = {"status": "OK", "max_entries": 100}
        except Exception as e:
            components["VLingMem"] = {"status": "FAIL", "error": str(e)}

        passed_count = sum(1 for c in components.values() if c["status"] == "OK")
        result.passed = passed_count >= 2
        result.details = f"{passed_count}/{len(components)} components OK"
        result.metrics = components

    except Exception as e:
        result.details = str(e)

    result.time_ms = (time.time() - t0) * 1000
    return result

# ============================================================
# 8. Training Pipeline 测试
# ============================================================
def test_training_pipeline():
    result = TestResult("Training Pipeline")
    t0 = time.time()

    try:
        components = {}

        # Dataset
        try:
            from vla_nav.training.dataset import NavTrajectoryDataset
            components["NavTrajectoryDataset"] = "OK"
        except Exception as e:
            components["NavTrajectoryDataset"] = f"FAIL: {e}"

        # SFT Trainer
        try:
            from vla_nav.training.sft_trainer import VLANavSFTTrainer
            components["SFT Trainer"] = "OK"
        except Exception as e:
            components["SFT Trainer"] = f"FAIL: {e}"

        # RL Trainer
        try:
            from vla_nav.training.rl_trainer import VLANavRLTrainer
            components["RL Trainer"] = "OK"
        except Exception as e:
            components["RL Trainer"] = f"FAIL: {e}"

        # AdaCoT Annotator
        try:
            from vla_nav.training.adacot_annotator import AdaCoTAnnotator
            components["AdaCoT Annotator"] = "OK"
        except Exception as e:
            components["AdaCoT Annotator"] = f"FAIL: {e}"

        passed_count = sum(1 for v in components.values() if v == "OK")
        result.passed = passed_count >= 3
        result.details = f"{passed_count}/{len(components)} components OK"
        result.metrics = components

    except Exception as e:
        result.details = str(e)

    result.time_ms = (time.time() - t0) * 1000
    return result

# ============================================================
# 9. Exploration Strategy 测试
# ============================================================
def test_exploration_strategy():
    result = TestResult("Exploration Strategy")
    t0 = time.time()

    try:
        from semantic_planner.exploration_strategy import ExplorationStrategy

        strategy = ExplorationStrategy(
            exploration_radius=5.0,
            min_frontier_size=0.5
        )

        # 模拟已探索区域
        explored_positions = [
            [0.0, 0.0],
            [2.0, 0.0],
            [0.0, 2.0],
        ]

        # 获取下一个探索目标
        robot_pos = np.array([1.0, 0.0, 1.0])
        next_target = strategy.get_next_exploration_target(
            robot_position=robot_pos,
            explored_positions=explored_positions,
            target_hint="kitchen"
        )

        result.passed = next_target is not None
        result.details = f"Next target: {next_target}"
        result.metrics = {
            "explored_count": len(explored_positions),
            "next_target": next_target.tolist() if next_target is not None else None
        }

    except Exception as e:
        result.details = str(e)

    result.time_ms = (time.time() - t0) * 1000
    return result

# ============================================================
# 10. YOLO-World Detection 测试
# ============================================================
def test_yolo_detection():
    result = TestResult("YOLO-World Detection")
    t0 = time.time()

    try:
        from ultralytics import YOLO
        import cv2

        model = YOLO("yolov8l-world.pt")
        classes = ["chair", "table", "sofa", "bed", "lamp", "door", "tv", "refrigerator"]
        model.set_classes(classes)

        # 创建测试图像 (如果有真实图像更好)
        test_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        results = model.predict(test_img, conf=0.1, verbose=False)
        boxes = results[0].boxes

        result.passed = True  # YOLO 加载成功即可
        result.details = f"YOLO-World loaded, {len(classes)} classes, detected {len(boxes)} objects"
        result.metrics = {
            "model": "yolov8l-world.pt",
            "classes": classes,
            "detections": len(boxes)
        }

    except Exception as e:
        result.details = str(e)

    result.time_ms = (time.time() - t0) * 1000
    return result

# ============================================================
# Main
# ============================================================
async def main():
    print("\n" + "#"*70)
    print("#" + " MapPilot 关键模块完整测试 ".center(68) + "#")
    print("#" + f" {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ".center(68) + "#")
    print("#"*70)

    results = []

    # 同步测试
    sync_tests = [
        ("Frontier Scorer", test_frontier_scorer),
        ("Topological Memory", test_topological_memory),
        ("Instance Tracker", test_instance_tracker),
        ("CLIP Encoder", test_clip_encoder),
        ("VLA Model", test_vla_model),
        ("Training Pipeline", test_training_pipeline),
        ("Exploration Strategy", test_exploration_strategy),
        ("YOLO Detection", test_yolo_detection),
    ]

    for name, test_func in sync_tests:
        print_header(name)
        try:
            r = test_func()
            results.append(r)
            print_result(r)
        except Exception as e:
            r = TestResult(name)
            r.details = f"Exception: {e}"
            results.append(r)
            print_result(r)

    # 异步测试
    async_tests = [
        ("Task Decomposer", test_task_decomposer),
        ("Goal Resolver", test_goal_resolver),
    ]

    for name, test_func in async_tests:
        print_header(name)
        try:
            r = await test_func()
            results.append(r)
            print_result(r)
        except Exception as e:
            r = TestResult(name)
            r.details = f"Exception: {e}"
            results.append(r)
            print_result(r)

    # 汇总
    print("\n" + "="*70)
    print("  测试汇总")
    print("="*70)

    passed = sum(1 for r in results if r.passed)
    total = len(results)

    for r in results:
        status = "\033[92m✓\033[0m" if r.passed else "\033[91m✗\033[0m"
        print(f"  {status} {r.name}: {r.details[:50]}{'...' if len(r.details) > 50 else ''}")

    print("\n" + "-"*70)
    print(f"  结果: {passed}/{total} 测试通过 ({100*passed/total:.0f}%)")

    if passed == total:
        print("\n  \033[92m★ 所有测试通过！系统已准备好进行论文实验 ★\033[0m")
    else:
        print(f"\n  \033[93m⚠ {total-passed} 个测试失败，请检查上述错误\033[0m")

    print("\n" + "#"*70)

    # 保存结果
    report = {
        "timestamp": datetime.now().isoformat(),
        "total": total,
        "passed": passed,
        "results": [
            {
                "name": r.name,
                "passed": r.passed,
                "details": r.details,
                "metrics": r.metrics,
                "time_ms": r.time_ms
            }
            for r in results
        ]
    }

    report_path = f"{OUTPUT_DIR}/test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"\n  报告已保存: {report_path}")

    return results

if __name__ == "__main__":
    asyncio.run(main())
