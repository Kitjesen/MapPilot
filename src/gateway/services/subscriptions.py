"""Gateway stream subscription wiring."""

from __future__ import annotations

from typing import Any


def setup_subscriptions(gw: Any) -> None:
    gw.odometry.subscribe(gw._on_odometry)
    gw.lidar_scan.subscribe(gw._on_lidar_scan)
    gw.lidar_scan.set_policy("latest")
    gw.map_scene.subscribe(gw._on_map_scene)
    gw.map_scene.set_policy("latest")
    gw.localization_quality.subscribe(gw._on_icp_quality)
    gw.localization_quality.set_policy("latest")
    gw.map_odom_tf.subscribe(gw._on_map_odom_tf)
    gw.map_odom_tf.set_policy("latest")
    gw.scene_graph.subscribe(gw._on_scene_graph)
    visual_servo_status = getattr(gw, "visual_servo_status", None)
    if visual_servo_status is not None:
        visual_servo_status.subscribe(gw._on_visual_servo_status)
        visual_servo_status.set_policy("latest")
    gw.navigation_state.subscribe(gw._on_navigation_state)
    gw.navigation_state.set_policy("latest")
    gw.navigation_goal_status.subscribe(gw._on_navigation_goal_status)
    gw.navigation_goal_status.set_policy("buffer", size=64)
    gw.exploration_run_event.subscribe(gw._on_exploration_run_event)
    gw.exploration_run_event.set_policy("buffer", size=512)
    gw.inspection_task_event.subscribe(gw._on_inspection_task_event)
    gw.inspection_task_event.set_policy("buffer", size=512)
    gw.global_path.subscribe(gw._on_global_path)
    gw.local_path.subscribe(gw._on_local_path)
    gw.native_traversability.subscribe(gw._on_native_traversability)
    gw.native_traversability.set_policy("latest")
    gw.agent_message.subscribe(gw._on_agent_message)
    gw.gnss_fusion_health.subscribe(gw._on_gnss_fusion_health)
    gw.localization_status.subscribe(gw._on_localization_status)
    gw.localization_status.set_policy("latest")
    gw.tare_stats.subscribe(gw._on_tare_stats)
    gw.supervisor_state.subscribe(gw._on_exploration_supervisor)
    gw._app = gw._build_app()
