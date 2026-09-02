from __future__ import annotations

from lingtu.assembly.products.host import host_blueprint


def _has_connection(system, out_mod: str, out_port: str, in_mod: str, in_port: str) -> bool:
    return any(
        c[0] == out_mod and c[1] == out_port and c[2] == in_mod and c[3] == in_port
        for c in system.connections
    )


def test_development_host_uses_native_navigation_shape():
    system = host_blueprint(
        robot="stub",
        slam_profile="none",
        enable_semantic=False,
        enable_gateway=False,
    ).build()

    try:
        assert system.get_module("host.bus") is not None
        assert system.get_module("nav.commands") is not None
        assert system.get_module("nav.goals") is not None
        assert system.get_module("nav.skills") is not None
        assert _has_connection(system, "nav.skills", "goal_command", "nav.goals", "goal_command")
        assert _has_connection(system, "nav.goals", "goal_status", "nav.skills", "goal_status")
    finally:
        system.stop()
