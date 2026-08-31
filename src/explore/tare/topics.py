"""TARE topic remapping contract used by external runtime adapters."""

from __future__ import annotations

from runtime.runtime_interface import TOPICS, adapter_remappings

EXPLORATION_START = "/exploration/start"
EXPLORATION_GLOBAL_PATH = "/exploration/global_path"
EXPLORATION_GLOBAL_PATH_FULL = "/exploration/global_path_full"
EXPLORATION_LOCAL_PATH = "/exploration/local_path"
EXPLORATION_OLD_GLOBAL_PATH = "/exploration/old_global_path"
EXPLORATION_NEAREST_GLOBAL_SUBSPACE_PATH = "/exploration/to_nearest_global_subspace_path"
EXPLORATION_PATH = "/exploration/path"
EXPLORATION_RUNTIME_BREAKDOWN = "/exploration/runtime_breakdown"
EXPLORATION_RUNTIME = "/exploration/runtime"
EXPLORATION_FINISH = "/exploration/finish"

TARE_REMAPS = {
    **adapter_remappings("tare"),
    # Subscriptions from canonical LingTu runtime topics.
    "/overall_map": TOPICS.map_cloud,
    # Publications exposed to LingTu exploration/navigation modules.
    "/global_path_full": EXPLORATION_GLOBAL_PATH_FULL,
    "/global_path": EXPLORATION_GLOBAL_PATH,
    "/local_path": EXPLORATION_LOCAL_PATH,
    "/old_global_path": EXPLORATION_OLD_GLOBAL_PATH,
    "/to_nearest_global_subspace_path": EXPLORATION_NEAREST_GLOBAL_SUBSPACE_PATH,
    "/exploration_path": EXPLORATION_PATH,
    "/runtime_breakdown": EXPLORATION_RUNTIME_BREAKDOWN,
    "/runtime": EXPLORATION_RUNTIME,
    "/exploration_finish": EXPLORATION_FINISH,
    "/start_exploration": EXPLORATION_START,
}
