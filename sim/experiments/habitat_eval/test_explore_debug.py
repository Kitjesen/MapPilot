"""
鎺㈢储琛屼负璋冭瘯娴嬭瘯 鈥?楠岃瘉淇鍚?rotation scan 涓嶈 stuck_counter 鎵撴柇, path_length > 0銆?
杩愯: python test_explore_debug.py
"""
import sys
import logging
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

sys.path.insert(0, str(Path(__file__).parents[2] / "src/perception"))
sys.path.insert(0, str(Path(__file__).parents[2] / "src/decision"))
sys.path.insert(0, str(Path(__file__).parents[2] / "src"))

import numpy as np
import habitat
from habitat.runtime.env import Env
from habitat.config.default import get_config

from habitat_navimind_agent import NaviMindAgent, HABITAT_MOVE_FORWARD, HABITAT_TURN_LEFT, HABITAT_TURN_RIGHT, HABITAT_STOP

ACTION_NAMES = {0: "STOP", 1: "FWD", 2: "L", 3: "R"}

def main():
    print("=" * 60)
    print("鎺㈢储琛屼负璋冭瘯娴嬭瘯")
    print("=" * 60)

    config = get_config(
        config_path="benchmark/nav/objectnav/objectnav_hm3d.yaml",
        overrides=[
            "habitat.dataset.split=val_mini",
            "habitat.environment.max_episode_steps=100",
        ],
    )
    env = Env(config=config)
    print(f"鐜灏辩华, 鍏?{len(env.episodes)} 涓?episodes")

    agent = NaviMindAgent()

    ep = env.episodes[0]
    env.current_episode = ep
    obs = env.reset()

    print(f"\nRGB shape: {obs['rgb'].shape}")
    print(f"Depth shape: {obs['depth'].shape}")
    print(f"鐩爣绫诲埆: {getattr(env.current_episode, 'object_category', '?')}")

    goal_cat = getattr(env.current_episode, "object_category", "chair")
    agent.reset()
    agent.set_goal(goal_cat)

    start_pos = env.sim.get_agent_state().position.copy()
    prev_pos = start_pos.copy()
    path_length = 0.0

    print(f"\n璧峰浣嶇疆: {start_pos}")
    print(f"\n{'姝?:>4} {'鍔ㄤ綔':>6} {'浣嶇疆 (x,z)':>22} {'姝ヨ窛':>7} {'绱Н璺緞':>9} "
          f"{'stuck':>6} {'rotating':>9} {'fwd_steps':>9}")
    print("-" * 80)

    for step in range(80):
        obs["_semantic_categories"] = {}
        action = agent.act(obs)
        obs = env.step(action)

        cur_pos = env.sim.get_agent_state().position
        step_dist = float(np.linalg.norm(cur_pos - prev_pos))
        path_length += step_dist
        prev_pos = cur_pos.copy()

        st = agent._state
        print(
            f"{step+1:>4} {ACTION_NAMES.get(action,'?'):>6} "
            f"({cur_pos[0]:>7.3f}, {cur_pos[2]:>7.3f}) "
            f"{step_dist:>7.3f}m {path_length:>8.3f}m "
            f"{st.stuck_counter:>6} {str(st.is_rotating):>9} {st.forward_steps:>9}"
        )

        if action == HABITAT_STOP:
            print("鈫?STOP 鍔ㄤ綔")
            break

    final_pos = env.sim.get_agent_state().position
    dist_moved = float(np.linalg.norm(final_pos - start_pos))
    print(f"\n鎬昏矾寰勯暱搴? {path_length:.3f}m")
    print(f"鐩寸嚎浣嶇Щ:   {dist_moved:.3f}m")
    print(f"浼犳劅鍣ㄦ牎姝? {agent._img_h}脳{agent._img_w}, fx={agent._fx:.1f}")
    print(f"\n{'鉁?PASS' if path_length > 0.3 else '鉂?FAIL'}: path_length={path_length:.3f}m")

    env.close()


if __name__ == "__main__":
    main()
