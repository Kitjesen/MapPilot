"""
NaviMind Agent 鍐掔儫娴嬭瘯 鈥?涓嶉渶瑕?Habitat 鐜/鏁版嵁闆嗐€?

楠岃瘉:
1. 鎵€鏈変緷璧栬兘 import
2. Agent 鑳藉鐞嗘ā鎷熻娴?
3. Fast Path 鑳藉尮閰嶇洰鏍?
4. 鎺㈢储绛栫暐姝ｅ父宸ヤ綔
5. 娑堣瀺寮€鍏虫甯?
"""

import json
import math
import sys
from pathlib import Path

# 鍔犲叆 src 璺緞
_PROJECT_ROOT = Path(__file__).resolve().parents[3]
_SRC = str(_PROJECT_ROOT / "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

import numpy as np

print("=" * 60)
print("  NaviMind Agent 鍐掔儫娴嬭瘯")
print("=" * 60)

# Test 1: Import
print("\n[1/5] 渚濊禆 import...")
from habitat_navimind_agent import OBJECTNAV_CATEGORIES, NaviMindAgent, _normalize_category

from decision.goals.resolver import GoalResolver, GoalResult
from decision.llm.client import LLMConfig
from perception.tracking.instance_tracker import InstanceTracker
from perception.tracking.projection import Detection3D
from runtime.utils.sanitize import safe_json_loads, sanitize_position

print("  鉁?鎵€鏈変緷璧?import 鎴愬姛")

# Test 2: Agent 鍒濆鍖?
print("\n[2/5] Agent 鍒濆鍖?..")
config = {
    "sensor": {"rgb": {"width": 64, "height": 64, "hfov": 79}},
    "eval": {"max_steps": 50, "success_distance": 1.0},
    "scene_graph": {"max_objects": 50},
    "fast_path": {"threshold": 0.75},
    "exploration": {"rotate_steps": 12, "frontier_distance": 2.0},
    "ablation": {"name": "full"},
}
agent = NaviMindAgent(config)
agent.reset()
agent.set_goal("chair")
print(f"  鉁?Agent 鍒濆鍖栨垚鍔? 鐩爣: chair 鈫?'{agent._instruction}'")

# Test 3: 妯℃嫙瑙傛祴澶勭悊
print("\n[3/5] 妯℃嫙瑙傛祴澶勭悊...")
fake_obs = {
    "rgb": np.random.randint(0, 255, (64, 64, 3), dtype=np.uint8),
    "depth": np.random.uniform(0.5, 5.0, (64, 64, 1)).astype(np.float32),
    "semantic": np.zeros((64, 64, 1), dtype=np.int32),
    "gps": np.array([1.0, 2.0], dtype=np.float32),
    "compass": np.array([0.5], dtype=np.float32),
    "_semantic_categories": {1: "chair", 2: "table"},
}
# 鍦ㄨ涔夊浘涓斁涓€浜?chair 鍍忕礌
fake_obs["semantic"][20:40, 20:40, 0] = 1  # chair instance

action = agent.act(fake_obs)
assert action in [0, 1, 2, 3], f"鏃犳晥鍔ㄤ綔: {action}"
print(f"  鉁?act() 杩斿洖鍔ㄤ綔: {action}")
print(f"  鉁?鍦烘櫙鍥剧墿浣撴暟: {len(agent._tracker._objects)}")
print(f"  鉁?缁熻: {agent.stats}")

# Test 4: 澶氭杩愯
print("\n[4/5] 澶氭杩愯...")
for step in range(10):
    # 绉诲姩涓€涓?
    fake_obs["gps"] = np.array([1.0 + step * 0.25, 2.0], dtype=np.float32)
    fake_obs["compass"] = np.array([0.5 + step * 0.1], dtype=np.float32)
    action = agent.act(fake_obs)

stats = agent.stats
print("  鉁?11 姝ュ畬鎴? 鍔ㄤ綔姝ｅ父")
print(f"  鉁?Fast Path hits: {stats['fast_path_hits']}/{stats['total_resolves']}")
print(f"  鉁?璁块棶缃戞牸: {stats['visited_cells']} cells")

# Test 5: 娑堣瀺鍙樹綋
print("\n[5/5] 娑堣瀺鍙樹綋娴嬭瘯...")
ablations = ["full", "no_belief", "no_fov", "no_hierarchy", "always_llm"]
for abl in ablations:
    cfg = dict(config)
    cfg["ablation"] = {"name": abl}
    a = NaviMindAgent(cfg)
    a.reset()
    a.set_goal("bed")
    action = a.act(fake_obs)
    assert action in [0, 1, 2, 3]
    print(f"  鉁?{abl:<15} 鈫?鍔ㄤ綔={action}, ablation={a.stats['ablation']}")

print("\n" + "=" * 60)
print("  鍏ㄩ儴鍐掔儫娴嬭瘯閫氳繃!")
print("=" * 60)
