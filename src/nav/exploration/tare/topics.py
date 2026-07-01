"""TARE topic remapping contract used by external runtime adapters."""

from __future__ import annotations

from runtime.runtime_interface import TOPICS, adapter_remappings


TARE_REMAPS = {
    **adapter_remappings("tare"),
    # Subscriptions from canonical LingTu runtime topics.
    "/overall_map": TOPICS.map_cloud,
    # Publications exposed to LingTu exploration/navigation modules.
    "/global_path_full": TOPICS.exploration_global_path_full,
    "/global_path": TOPICS.exploration_global_path,
    "/local_path": TOPICS.exploration_local_path,
    "/old_global_path": TOPICS.exploration_old_global_path,
    "/to_nearest_global_subspace_path": (
        TOPICS.exploration_nearest_global_subspace_path
    ),
    "/exploration_path": TOPICS.exploration_path,
    "/runtime_breakdown": TOPICS.exploration_runtime_breakdown,
    "/runtime": TOPICS.exploration_runtime,
    "/exploration_finish": TOPICS.exploration_finish,
    "/start_exploration": TOPICS.exploration_start,
}
