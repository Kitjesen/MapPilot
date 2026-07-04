from __future__ import annotations

from runtime.blueprints.products.thunder import thunder_blueprint


def _has_connection(system, out_mod: str, out_port: str, in_mod: str, in_port: str) -> bool:
    return any(
        c[0] == out_mod and c[1] == out_port and c[2] == in_mod and c[3] == in_port
        for c in system.connections
    )


def test_non_native_navigation_uses_python_autonomy_chain():
    system = thunder_blueprint(
        robot="stub",
        slam_profile="none",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
    ).build()

    try:
        nav = system.get_module("nav.mission")
        local_planner = system.get_module("nav.local_planner")
        path_follower = system.get_module("nav.path_follower")

        assert not hasattr(nav, "_enable_ros2_bridge")
        assert local_planner._backend in ("nanobind", "cmu_py", "simple")
        assert path_follower._backend in ("nav_kernel", "pid")

        assert _has_connection(system, "StubDogModule", "odometry", "nav.mission", "odometry")
        assert _has_connection(system, "nav.mission", "waypoint", "nav.local_planner", "waypoint")
        assert _has_connection(system, "nav.terrain", "terrain_map_ext", "nav.local_planner", "terrain_map_ext")
        assert _has_connection(system, "nav.local_planner", "local_path", "nav.path_follower", "local_path")
        # cmd_vel goes through VelocityMux for priority arbitration
        assert _has_connection(system, "nav.path_follower", "cmd_vel", "nav.velocity_mux", "path_follower_cmd_vel")
        assert _has_connection(system, "nav.velocity_mux", "driver_cmd_vel", "StubDogModule", "cmd_vel")
    finally:
        system.stop()
