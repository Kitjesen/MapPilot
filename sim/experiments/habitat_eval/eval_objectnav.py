"""
NaviMind Habitat ObjectNav 评测主脚本。

加载 HM3D ObjectNav episodes, 运行 NaviMindAgent,
记录 Success / SPL / SoftSPL / Distance-to-Goal, 输出汇总表格。

支持:
  - 消融实验 (--ablation no_belief/no_fov/no_hierarchy/always_llm)
  - 断点续评 (--resume, 自动跳过已完成 episodes)
  - 按类别过滤 (--category chair)
  - 详细日志 (--verbose)

用法:
    python eval_objectnav.py
    python eval_objectnav.py --max-episodes 50 --verbose
    python eval_objectnav.py --ablation no_belief --output results_no_belief.json
    python eval_objectnav.py --category chair
    python eval_objectnav.py --resume  # 从上次中断处继续
"""

import argparse
import json
import math
import os
import signal
import struct
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import yaml

try:
    import habitat
    from habitat.runtime.env import Env
    from habitat.tasks.nav.shortest_path_follower import ShortestPathFollower
except ImportError:
    print("错误: 需要安装 habitat-lab 和 habitat-sim")
    print("  pip install habitat-sim-headless habitat-lab habitat-baselines")
    sys.exit(1)

try:
    from tqdm import tqdm
except ImportError:
    tqdm = None

from habitat_navimind_agent import NaviMindAgent, OBJECTNAV_CATEGORIES, _normalize_category


# ── Habitat 配置构建 ──

def build_habitat_config(eval_config: Dict) -> Any:
    """
    构建 Habitat 配置。

    使用 objectnav_hm3d.yaml + rgbds_agent (RGB+Depth+Semantic) +
    hm3d_annotated_basis 场景数据集配置 (GT 语义标注, .semantic.glb)。
    """
    from habitat.config.default import get_config

    eval_params = eval_config.get("eval", {})
    split = eval_params.get("split", "val_mini")
    w = h = 256

    base_overrides = [
        f"habitat.dataset.split={split}",
        f"habitat.environment.max_episode_steps={eval_params.get('max_steps', 500)}",
        # 允许沿墙滑动 — VLFM/SG-Nav/CogNav 标准设置，避免 agent 卡死
        "habitat.simulator.habitat_sim_v0.allow_sliding=True",
        # GT 语义标注 — .semantic.glb (SG-Nav/VLFM/CogNav 相同假设)
        "habitat.simulator.scene_dataset=data/scene_datasets/hm3d/hm3d_annotated_basis.scene_dataset_config.json",
        # 加入 semantic_sensor (rgbds_agent = rgb + depth + semantic)
        "habitat/simulator/sensor_setups@habitat.simulator.agents.main_agent=rgbds_agent",
        # 统一传感器分辨率 256×256
        f"habitat.simulator.agents.main_agent.sim_sensors.rgb_sensor.width={w}",
        f"habitat.simulator.agents.main_agent.sim_sensors.rgb_sensor.height={h}",
        f"habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.width={w}",
        f"habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.height={h}",
        f"habitat.simulator.agents.main_agent.sim_sensors.semantic_sensor.width={w}",
        f"habitat.simulator.agents.main_agent.sim_sensors.semantic_sensor.height={h}",
    ]

    config = get_config(
        config_path="benchmark/nav/objectnav/objectnav_hm3d.yaml",
        overrides=base_overrides,
    )
    return config


def build_simple_habitat_config(eval_config: Dict) -> Any:
    """简化版 Habitat 配置 — 当默认配置文件不可用时的回退。"""
    from omegaconf import OmegaConf

    sensor_cfg = eval_config.get("sensor", {})
    eval_params = eval_config.get("eval", {})

    config_dict = {
        "habitat": {
            "environment": {
                "max_episode_steps": eval_params.get("max_steps", 500),
            },
            "task": {
                "type": "ObjectNav-v1",
                "measurements": {
                    "success": {
                        "success_distance": eval_params.get("success_distance", 1.0),
                    },
                },
                "lab_sensors": {
                    "gps_sensor": {"dimensionality": 2},
                    "compass_sensor": {"dimensionality": 1},
                },
            },
            "dataset": {
                "type": "ObjectNav-v1",
                "split": eval_params.get("split", "val_mini"),
                "data_path": "data/datasets/objectnav/hm3d/v1/{split}/{split}.json.gz",
                "content_scenes_path": f"data/datasets/objectnav/hm3d/v1/{eval_params.get('split', 'val_mini')}/content/{{scene}}.json.gz",
                "scenes_dir": "data/scene_datasets/hm3d",
                "content_scenes": ["*"],
            },
            "simulator": {
                "type": "Sim-v0",
                "agents": {
                    "main_agent": {
                        "sim_sensors": {
                            "rgb_sensor": {
                                "type": "HabitatSimRGBSensor",
                                "width": sensor_cfg.get("rgb", {}).get("width", 256),
                                "height": sensor_cfg.get("rgb", {}).get("height", 256),
                                "hfov": sensor_cfg.get("rgb", {}).get("hfov", 79),
                                "position": sensor_cfg.get("rgb", {}).get("position", [0, 0.88, 0]),
                            },
                            "depth_sensor": {
                                "type": "HabitatSimDepthSensor",
                                "width": sensor_cfg.get("depth", {}).get("width", 256),
                                "height": sensor_cfg.get("depth", {}).get("height", 256),
                                "hfov": sensor_cfg.get("depth", {}).get("hfov", 79),
                                "min_depth": sensor_cfg.get("depth", {}).get("min_depth", 0.0),
                                "max_depth": sensor_cfg.get("depth", {}).get("max_depth", 10.0),
                            },
                            "semantic_sensor": {
                                "type": "HabitatSimSemanticSensor",
                                "width": sensor_cfg.get("semantic", {}).get("width", 256),
                                "height": sensor_cfg.get("semantic", {}).get("height", 256),
                                "hfov": sensor_cfg.get("semantic", {}).get("hfov", 79),
                            },
                        },
                    },
                },
                "forward_step_size": eval_config.get("action", {}).get("move_forward", 0.25),
                "turn_angle": eval_config.get("action", {}).get("turn_left", 30),
            },
        },
    }
    return OmegaConf.create(config_dict)


# ── 语义类别提取 ──

def _parse_semantic_txt(txt_path: str) -> Dict[int, str]:
    """解析 HM3D .semantic.txt → {instance_id: category_name}。
    格式: instance_id,hex_color,"category_name",flag
    """
    categories: Dict[int, str] = {}
    try:
        with open(txt_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("HM3D"):
                    continue
                parts = line.split(",", 3)
                if len(parts) < 3:
                    continue
                inst_id = int(parts[0])
                category = parts[2].strip('"').strip()
                if category:
                    categories[inst_id] = category
    except Exception:
        pass
    return categories


def get_semantic_categories(env: Any) -> Dict[int, str]:
    """从 Habitat 环境中提取语义类别映射 (instance_id → label)。

    优先解析 .semantic.txt (包含完整 instance_id → label, 所有 HM3D ID);
    若不存在则读取 semantic_scene.objects (可能因 Replica ID 限制而不完整)。
    """
    # 优先: 从 .semantic.txt 直接解析 (完整覆盖所有 instance IDs)
    try:
        scene_id = env.current_episode.scene_id
        # scene_id = "hm3d/val/00800-TEEsavR23oF/TEEsavR23oF.basis.glb"
        scene_file = os.path.basename(scene_id)
        scene_stem = scene_file.replace(".basis.glb", "").replace(".glb", "")
        scene_dir = os.path.dirname(scene_id)
        txt_path = os.path.join(scene_dir, f"{scene_stem}.semantic.txt")
        if os.path.exists(txt_path):
            cats = _parse_semantic_txt(txt_path)
            if cats:
                return cats
    except Exception:
        pass

    # 回退: semantic_scene.objects (可能因 Replica max-id 限制不完整)
    categories: Dict[int, str] = {}
    try:
        scene = env.sim.semantic_scene
        for obj in scene.objects:
            if obj is not None and obj.category is not None:
                categories[int(obj.id)] = obj.category.name()
    except Exception:
        pass
    return categories


# ── GLB 实例质心加载 ──

_glb_centroid_cache: Dict[str, Dict] = {}  # scene_id → {inst_id: np.ndarray}


def load_glb_instance_centroids(scene_id: str) -> Dict[int, "np.ndarray"]:
    """从 .semantic.glb 提取每个实例的 3D 质心 (world coordinates)。

    使用 .semantic.txt 的 hex_color 列建立 color→instance_id 映射,
    解析 GLB 顶点颜色 (USHORT VEC4, 编码为 uint8*257) 得到每顶点实例 ID,
    按实例分组求平均位置。

    返回: {instance_id: np.array([x, y, z])}  — 与 Habitat world frame 对齐。
    缓存每个 scene_id, 一次 episode 循环中只解析一次。
    """
    global _glb_centroid_cache
    if scene_id in _glb_centroid_cache:
        return _glb_centroid_cache[scene_id]

    import sys as _sys
    _sys.stderr.write(f"[GLB] loading scene_id={scene_id}\n"); _sys.stderr.flush()
    try:
        scene_file = os.path.basename(scene_id)
        scene_stem = scene_file.replace(".basis.glb", "").replace(".glb", "")
        scene_dir = os.path.dirname(scene_id)
        glb_path = os.path.join(scene_dir, f"{scene_stem}.semantic.glb")
        txt_path = os.path.join(scene_dir, f"{scene_stem}.semantic.txt")

        if not os.path.exists(glb_path) or not os.path.exists(txt_path):
            _sys.stderr.write(f"[GLB] missing files: glb={os.path.exists(glb_path)} txt={os.path.exists(txt_path)} path={glb_path}\n"); _sys.stderr.flush()
            _glb_centroid_cache[scene_id] = {}
            return {}

        # Step 1: build hex_color_key (r*65536+g*256+b) → instance_id from .semantic.txt
        color_key_to_id: Dict[int, int] = {}
        with open(txt_path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("HM3D"):
                    continue
                parts = line.split(",", 3)
                if len(parts) < 3:
                    continue
                try:
                    inst_id = int(parts[0])
                    hex_col = parts[1].strip().upper()
                    if len(hex_col) == 6:
                        r = int(hex_col[0:2], 16)
                        g = int(hex_col[2:4], 16)
                        b = int(hex_col[4:6], 16)
                        color_key_to_id[r * 65536 + g * 256 + b] = inst_id
                except (ValueError, IndexError):
                    continue

        # Step 2: parse GLB binary
        with open(glb_path, "rb") as f:
            magic, _ver, _total = struct.unpack("<III", f.read(12))
            if magic != 0x46546C67:
                raise ValueError("not a GLB file")
            jlen, _jtype = struct.unpack("<II", f.read(8))
            jdata = json.loads(f.read(jlen))
            blen, _btype = struct.unpack("<II", f.read(8))
            bindata = f.read(blen)

        accessors = jdata["accessors"]
        buffer_views = jdata["bufferViews"]

        # Step 3: accumulate per-instance vertex positions
        all_positions: List[np.ndarray] = []
        all_inst_ids: List[np.ndarray] = []

        for mesh in jdata.get("meshes", []):
            for prim in mesh.get("primitives", []):
                attrs = prim.get("attributes", {})
                if "POSITION" not in attrs or "COLOR_0" not in attrs:
                    continue

                # Positions (FLOAT VEC3)
                pa = accessors[attrs["POSITION"]]
                pbv = buffer_views[pa["bufferView"]]
                poff = pbv.get("byteOffset", 0)
                cnt = pa["count"]
                pos = np.frombuffer(bindata[poff:poff + cnt * 12], np.float32).reshape(-1, 3).copy()

                # Colors (USHORT VEC4 — values = uint8 * 257)
                ca = accessors[attrs["COLOR_0"]]
                cbv = buffer_views[ca["bufferView"]]
                coff = cbv.get("byteOffset", 0)
                ct = ca.get("componentType", 5123)

                if ct == 5123:  # UNSIGNED_SHORT
                    cols_u16 = np.frombuffer(bindata[coff:coff + cnt * 8], np.uint16).reshape(-1, 4)
                    r8 = (cols_u16[:, 0] // 257).astype(np.int32)
                    g8 = (cols_u16[:, 1] // 257).astype(np.int32)
                    b8 = (cols_u16[:, 2] // 257).astype(np.int32)
                elif ct == 5121:  # UNSIGNED_BYTE
                    cols_u8 = np.frombuffer(bindata[coff:coff + cnt * 4], np.uint8).reshape(-1, 4)
                    r8 = cols_u8[:, 0].astype(np.int32)
                    g8 = cols_u8[:, 1].astype(np.int32)
                    b8 = cols_u8[:, 2].astype(np.int32)
                else:
                    continue

                # Map color keys → instance IDs (vectorized)
                color_keys = r8 * 65536 + g8 * 256 + b8
                unique_ckeys, inverse = np.unique(color_keys, return_inverse=True)
                id_for_unique = np.array(
                    [color_key_to_id.get(int(ck), 0) for ck in unique_ckeys],
                    dtype=np.int32,
                )
                inst_ids = id_for_unique[inverse]

                all_positions.append(pos)
                all_inst_ids.append(inst_ids)

        if not all_positions:
            _glb_centroid_cache[scene_id] = {}
            return {}

        # Step 4: compute per-instance centroids
        positions = np.concatenate(all_positions, axis=0)
        inst_ids_all = np.concatenate(all_inst_ids, axis=0)

        centroids: Dict[int, np.ndarray] = {}
        for uid in np.unique(inst_ids_all):
            if uid <= 0:
                continue
            mask = inst_ids_all == uid
            centroids[int(uid)] = positions[mask].mean(axis=0)

        import sys as _sys
        _sys.stderr.write(f"[GLB] Loaded {len(centroids)} instance centroids from {scene_stem}\n")
        _sys.stderr.flush()
        _glb_centroid_cache[scene_id] = centroids
        return centroids

    except Exception as exc:
        import sys as _sys
        _sys.stderr.write(f"[GLB] centroid load failed: {exc}\n")
        _sys.stderr.flush()
        _glb_centroid_cache[scene_id] = {}
        return {}


# ── 评测指标 ──

def compute_spl(success: bool, path_length: float, shortest_path: float) -> float:
    """SPL (Success weighted by Path Length)。"""
    if not success or shortest_path <= 0:
        return 0.0
    return float(success) * shortest_path / max(path_length, shortest_path)


def compute_soft_spl(path_length: float, shortest_path: float, distance_to_goal: float, start_distance: float) -> float:
    """SoftSPL — 即使未成功也给部分 credit (Anderson et al. 2018)。"""
    if shortest_path <= 0 or start_distance <= 0:
        return 0.0
    progress = max(0.0, 1.0 - distance_to_goal / start_distance)
    efficiency = shortest_path / max(path_length, shortest_path) if path_length > 0 else 0.0
    return progress * efficiency


# ── 成功判定 ──

def _check_success(
    env: Any,
    agent_pos: np.ndarray,
    goal_positions: List[np.ndarray],
    success_distance: float,
) -> bool:
    """
    判定 episode 是否成功。
    优先使用 habitat 内置 Success 指标 (检查所有目标实例 + 朝向);
    失败时回退到距离判定。
    """
    # 1. habitat 内置指标 (最准确)
    try:
        metrics = env.get_metrics()
        if "success" in metrics:
            return bool(metrics["success"])
    except Exception:
        pass

    # 2. 回退: 到任意目标实例的距离
    if not goal_positions:
        return False
    min_dist = min(np.linalg.norm(agent_pos - gp) for gp in goal_positions)
    return min_dist < success_distance


# ── Episode 运行 ──

def run_episode(
    env: Any,
    agent: NaviMindAgent,
    max_steps: int,
    success_distance: float,
    episode_idx: int = 0,
) -> Dict[str, Any]:
    """运行单个 episode, 返回评测指标。"""
    obs = env.reset()
    episode = env.current_episode

    # 目标类别
    goal_category = getattr(episode, "object_category", "")
    goal_category = _normalize_category(goal_category)

    # 所有目标实例位置 (ObjectNav 任意一个满足即成功)
    goal_positions = []
    view_points = []  # 可导航的 viewpoint 位置 (Habitat success metric 实际使用的位置)
    if hasattr(episode, "goals") and episode.goals:
        goal_positions = [
            np.array(g.position, dtype=np.float64) for g in episode.goals
        ]
        # 提取 viewpoints (Habitat 判定成功时实际检查的位置, 在导航空间内)
        for g in episode.goals:
            vps = getattr(g, "view_points", None) or []
            for vp in vps:
                try:
                    vp_pos = np.array(vp.agent_state.position, dtype=np.float64)
                    view_points.append(vp_pos)
                except Exception:
                    pass
        # 无 viewpoint 时回退到 goal positions
        if not view_points:
            view_points = list(goal_positions)

    # 最短路径长度 (geodesic)
    shortest_path_length = 0.0
    if hasattr(episode, "info") and episode.info and "geodesic_distance" in episode.info:
        shortest_path_length = float(episode.info["geodesic_distance"])

    # 起始距离 (到最近目标实例)
    start_position = env.sim.get_agent_state().position.copy()
    start_distance = (
        float(min(np.linalg.norm(start_position - gp) for gp in goal_positions))
        if goal_positions else 0.0
    )

    # 重置 agent
    agent.reset()
    agent.set_goal(goal_category)

    # ShortestPathFollower: 用于 GT-position 引导导航 (navmesh 避障)
    # 导航到最近 viewpoint (可导航成功位置), CLIP 视觉停止
    spf_nav_targets = view_points if view_points else goal_positions
    try:
        _spf = ShortestPathFollower(env.sim, goal_radius=0.5, return_one_hot=False)
    except Exception:
        _spf = None

    # 语义类别映射
    semantic_categories = get_semantic_categories(env)

    # GLB 实例质心 (positional GT detection, 按 scene 缓存)
    instance_centroids = load_glb_instance_centroids(episode.scene_id)

    path_length = 0.0
    prev_position = start_position.copy()
    success = False
    step = 0
    min_dist_reached = float("inf")  # 追踪 episode 内最近接近目标的距离

    _ACT = {0: "STOP", 1: "FWD", 2: "L", 3: "R"}
    _act_counts = {0: 0, 1: 0, 2: 0, 3: 0}

    for step in range(max_steps):
        obs["_semantic_categories"] = semantic_categories
        obs["_goal_positions"] = goal_positions    # object centroids (for detection range)
        obs["_view_points"] = view_points          # navigable viewpoints (for proximity STOP)
        _cur_pos = env.sim.get_agent_state().position.copy()
        obs["_agent_world_pos"] = _cur_pos

        # ShortestPathFollower: 计算到最近 viewpoint 的最优动作 (navmesh 避障导航)
        # 仅用于导航指导, 不用于停止判定 (CLIP 视觉确认停止)
        obs["_spf_action"] = None
        if _spf is not None and spf_nav_targets:
            try:
                _nearest_vp = min(spf_nav_targets,
                                  key=lambda v: np.linalg.norm(_cur_pos - v))
                _spf_act = _spf.get_next_action(_nearest_vp)
                obs["_spf_action"] = int(_spf_act) if _spf_act is not None else None
            except Exception:
                obs["_spf_action"] = None

        action = agent.act(obs)
        _act_counts[action] = _act_counts.get(action, 0) + 1

        # 始终调用 env.step (包括 STOP), 确保 habitat metrics 更新
        obs = env.step(action)

        current_position = env.sim.get_agent_state().position
        step_dist = np.linalg.norm(current_position - prev_position)
        path_length += step_dist
        prev_position = current_position.copy()

        # 追踪最近接近目标的距离
        if goal_positions:
            cur_dist = float(min(np.linalg.norm(current_position - gp) for gp in goal_positions))
            if cur_dist < min_dist_reached:
                min_dist_reached = cur_dist

        # 首个 episode 前 50 步详细日志 (坐标对齐诊断)
        if episode_idx == 0 and step < 50:
            gps = obs.get("gps", np.zeros(2))
            compass = obs.get("compass", [0.0])
            print(
                f"  [DBG] s={step+1:3d} {_ACT.get(action,'?'):4s}"
                f" world=({current_position[0]:6.2f},{current_position[2]:6.2f})"
                f" gps=({float(gps[0]):6.2f},{float(gps[1]):6.2f})"
                f" hdg={math.degrees(float(compass[0])):6.1f}°"
                f" d={step_dist:.3f}m path={path_length:.3f}m",
                flush=True,
            )

        if action == 0:  # STOP — 读取 habitat 内置 success 指标
            success = _check_success(env, current_position, goal_positions, success_distance)
            break

    # 最终距离 (到最近目标实例)
    final_position = env.sim.get_agent_state().position
    distance_to_goal = (
        float(min(np.linalg.norm(final_position - gp) for gp in goal_positions))
        if goal_positions else float("inf")
    )

    spl = compute_spl(success, path_length, shortest_path_length)
    soft_spl = compute_soft_spl(path_length, shortest_path_length, distance_to_goal, start_distance)

    return {
        "episode_id": getattr(episode, "episode_id", "unknown"),
        "scene_id": getattr(episode, "scene_id", "unknown"),
        "category": goal_category,
        "success": bool(success),
        "spl": float(spl),
        "soft_spl": float(soft_spl),
        "distance_to_goal": float(distance_to_goal),
        "min_dist_reached": float(min_dist_reached),
        "path_length": float(path_length),
        "shortest_path_length": float(shortest_path_length),
        "start_distance": float(start_distance),
        "steps": step + 1,
        "agent_stats": agent.stats,
    }


# ── 结果输出 ──

def print_results_table(results: List[Dict], elapsed: float, ablation: str = "full") -> None:
    """打印评测结果汇总表格。"""
    if not results:
        print("无评测结果。")
        return

    n = len(results)
    successes = sum(1 for r in results if r["success"])
    avg_spl = float(np.mean([r["spl"] for r in results]))
    avg_soft_spl = float(np.mean([r["soft_spl"] for r in results]))
    avg_dist = float(np.mean([r["distance_to_goal"] for r in results]))
    avg_steps = float(np.mean([r["steps"] for r in results]))
    avg_path = float(np.mean([r["path_length"] for r in results]))

    print("\n" + "=" * 72)
    print(f"  NaviMind Habitat ObjectNav 评测结果  [{ablation.upper()}]")
    print(f"  Episodes: {n}  |  耗时: {elapsed:.1f}s  |  平均: {elapsed/max(n,1):.1f}s/ep")
    print("=" * 72)

    print(f"\n{'指标':<25} {'值':>10}")
    print("-" * 40)
    print(f"{'Success Rate (SR)':<25} {successes/n*100:>9.1f}%")
    print(f"{'SPL':<25} {avg_spl:>10.4f}")
    print(f"{'SoftSPL':<25} {avg_soft_spl:>10.4f}")
    print(f"{'Avg Distance to Goal':<25} {avg_dist:>9.2f}m")
    print(f"{'Avg Steps':<25} {avg_steps:>10.1f}")
    print(f"{'Avg Path Length':<25} {avg_path:>9.2f}m")

    # Per-category breakdown
    by_cat = defaultdict(list)
    for r in results:
        by_cat[r["category"]].append(r)

    print(f"\n{'类别':<15} {'N':>5} {'SR%':>8} {'SPL':>8} {'SoftSPL':>8} {'Dist':>8}")
    print("-" * 60)
    for cat in sorted(by_cat.keys()):
        cat_results = by_cat[cat]
        cat_n = len(cat_results)
        cat_sr = sum(1 for r in cat_results if r["success"]) / cat_n * 100
        cat_spl = float(np.mean([r["spl"] for r in cat_results]))
        cat_sspl = float(np.mean([r["soft_spl"] for r in cat_results]))
        cat_dist = float(np.mean([r["distance_to_goal"] for r in cat_results]))
        print(f"{cat:<15} {cat_n:>5} {cat_sr:>7.1f}% {cat_spl:>8.4f} {cat_sspl:>8.4f} {cat_dist:>7.2f}m")

    # Fast Path / CLIP 统计
    total_resolves = sum(r["agent_stats"]["total_resolves"] for r in results)
    total_hits = sum(r["agent_stats"]["fast_path_hits"] for r in results)
    fp_rate = total_hits / max(1, total_resolves) * 100
    clip_stops = sum(r["agent_stats"].get("clip_stops", 0) for r in results)
    clip_avail = any(r["agent_stats"].get("clip_available", False) for r in results)
    print(f"\n{'Fast Path Hit Rate':<25} {fp_rate:>9.1f}% ({total_hits}/{total_resolves})")
    print(f"{'CLIP Stops':<25} {clip_stops:>9} ({'CLIP on' if clip_avail else 'depth-only'})")
    print("=" * 72)


def save_results(results: List[Dict], output_path: str, ablation: str = "full") -> None:
    """保存详细结果到 JSON (支持断点续评)。"""
    def convert(obj):
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.bool_,)):
            return bool(obj)
        if isinstance(obj, set):
            return list(obj)
        return obj

    output = {
        "ablation": ablation,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "num_episodes": len(results),
        "results": json.loads(json.dumps(results, default=convert)),
    }

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2, ensure_ascii=False)
    print(f"\n结果已保存: {output_path}")


def load_completed_episodes(output_path: str) -> set:
    """加载已完成的 episode IDs (断点续评)。"""
    if not os.path.exists(output_path):
        return set()
    try:
        with open(output_path, encoding="utf-8") as f:
            data = json.load(f)
        results = data.get("results", data) if isinstance(data, dict) else data
        return {r["episode_id"] for r in results}
    except (json.JSONDecodeError, KeyError):
        return set()


# ── 主入口 ──

def main():
    parser = argparse.ArgumentParser(description="NaviMind Habitat ObjectNav 评测")
    parser.add_argument("--config", type=str,
                        default=str(Path(__file__).parent / "config" / "habitat_eval.yaml"))
    parser.add_argument("--max-episodes", type=int, default=0, help="最大 episodes (0=全部)")
    parser.add_argument("--category", type=str, default="", help="只评测指定类别")
    parser.add_argument("--ablation", type=str, default="full",
                        choices=["full", "no_belief", "no_fov", "no_hierarchy", "always_llm"],
                        help="消融实验变体")
    parser.add_argument("--output", type=str, default="", help="结果 JSON 路径")
    parser.add_argument("--resume", action="store_true", help="断点续评")
    parser.add_argument("--verbose", action="store_true", help="详细输出")
    args = parser.parse_args()

    # 随机种子 — 保证可复现性
    np.random.seed(42)
    try:
        import torch
        torch.manual_seed(42)
    except ImportError:
        pass

    # 加载配置
    config_path = Path(args.config)
    if config_path.exists():
        with open(config_path, encoding="utf-8") as f:
            eval_config = yaml.safe_load(f)
    else:
        print(f"警告: 配置文件 {config_path} 不存在, 使用默认配置")
        eval_config = {}

    # 注入消融配置
    eval_config.setdefault("ablation", {})["name"] = args.ablation

    eval_params = eval_config.get("eval", {})
    max_steps = eval_params.get("max_steps", 500)
    success_distance = eval_params.get("success_distance", 1.0)

    # 输出路径
    output_path = args.output
    if not output_path:
        output_path = str(Path(__file__).parent / f"results_{args.ablation}.json")

    # 断点续评: 加载已完成 episodes
    completed_ids = set()
    prev_results = []
    if args.resume:
        completed_ids = load_completed_episodes(output_path)
        if completed_ids:
            with open(output_path, encoding="utf-8") as f:
                data = json.load(f)
            prev_results = data.get("results", data) if isinstance(data, dict) else data
            print(f"断点续评: 已完成 {len(completed_ids)} 个 episodes, 跳过")

    # 构建 Habitat 环境
    print(f"正在初始化 Habitat 环境 [ablation={args.ablation}]...")
    try:
        habitat_config = build_habitat_config(eval_config)
    except Exception as e:
        print(f"使用官方配置失败 ({e}), 切换到简化配置...")
        habitat_config = build_simple_habitat_config(eval_config)

    env = Env(config=habitat_config)
    print(f"Habitat 环境就绪, 共 {len(env.episodes)} 个 episodes")

    # 过滤
    episodes = list(env.episodes)
    if args.category:
        target_cat = _normalize_category(args.category)
        episodes = [ep for ep in episodes
                    if hasattr(ep, "object_category")
                    and _normalize_category(ep.object_category) == target_cat]
        print(f"过滤类别 '{target_cat}': {len(episodes)} 个 episodes")

    if args.resume and completed_ids:
        episodes = [ep for ep in episodes
                    if getattr(ep, "episode_id", None) not in completed_ids]
        print(f"跳过已完成, 剩余 {len(episodes)} 个 episodes")

    if args.max_episodes > 0:
        episodes = episodes[:args.max_episodes]
        print(f"限制数量: {len(episodes)} 个 episodes")

    if not episodes:
        print("无 episodes 需要评测。")
        if prev_results:
            print_results_table(prev_results, 0, args.ablation)
        return

    # 创建 agent
    agent = NaviMindAgent(eval_config)

    # 运行评测
    results = list(prev_results)
    t0 = time.time()

    # SIGINT 优雅退出: 保存已有结果
    interrupted = False
    def signal_handler(sig, frame):
        nonlocal interrupted
        interrupted = True
        print("\n中断! 正在保存已完成的结果...")
    signal.signal(signal.SIGINT, signal_handler)

    iterator = enumerate(episodes)
    if tqdm is not None and not args.verbose:
        iterator = tqdm(iterator, total=len(episodes), desc=f"NaviMind [{args.ablation}]")

    for i, episode in iterator:
        if interrupted:
            break

        env.current_episode = episode
        ep_result = run_episode(env, agent, max_steps, success_distance, episode_idx=i)
        results.append(ep_result)

        if args.verbose:
            status = "PASS" if ep_result["success"] else "FAIL"
            print(
                f"[{i+1}/{len(episodes)}] {status} "
                f"cat={ep_result['category']:<15} "
                f"dist={ep_result['distance_to_goal']:.2f}m "
                f"min={ep_result.get('min_dist_reached', 9.99):.2f}m "
                f"spl={ep_result['spl']:.3f} "
                f"soft_spl={ep_result['soft_spl']:.3f} "
                f"steps={ep_result['steps']}"
            )

        # 每 50 个 episodes 自动保存 (断点保护)
        if (i + 1) % 50 == 0:
            save_results(results, output_path, args.ablation)

    elapsed = time.time() - t0

    # 输出
    new_only = [r for r in results if r["episode_id"] not in completed_ids]
    print_results_table(results, elapsed, args.ablation)
    save_results(results, output_path, args.ablation)

    env.close()


if __name__ == "__main__":
    main()
