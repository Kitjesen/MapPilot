"""Product Host blueprint assembly."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from lingtu.assembly.products.runtime_paths import DEFAULT_GATEWAY_PORT
from runtime.blueprint import Blueprint
from runtime.runtime_policy import normalize_slam_profile


def _semantic_save_dir(
    config: Mapping[str, Any],
    *,
    enabled: bool,
) -> str:
    configured = str(config.get("semantic_save_dir") or "").strip()
    if configured:
        return configured
    if not enabled:
        return ""
    try:
        return str(Path.home() / ".nova" / "semantic")
    except RuntimeError as exc:
        raise ValueError(
            "semantic_save_dir is required when semantic modules run without "
            "a user home directory"
        ) from exc


def host_blueprint(
    config: Mapping[str, Any] | None = None,
    **overrides: Any,
) -> Blueprint:
    """Build a Product Host blueprint from an already resolved config."""

    cfg = dict(config or {})
    cfg.update({key: value for key, value in overrides.items() if value is not None})
    robot = str(cfg.pop("robot", "stub"))
    slam_profile = normalize_slam_profile(cfg.pop("slam_profile", "native_dds"))
    detector = str(cfg.pop("detector", "bpu"))
    encoder = str(cfg.pop("encoder", "none"))
    llm = str(cfg.pop("llm", "qwen"))
    gateway_port = int(cfg.pop("gateway_port", DEFAULT_GATEWAY_PORT))
    enable_semantic = bool(cfg.pop("enable_semantic", True))
    enable_gateway = bool(cfg.pop("enable_gateway", True))
    enable_teleop = bool(cfg.pop("enable_teleop", True))
    enable_navigation = bool(cfg.pop("enable_navigation", True))
    if enable_navigation:
        cfg.setdefault("native_navigation_endpoint", "lingtu-nav-dds")
    run_startup_checks = bool(cfg.pop("run_startup_checks", True))
    namespace = cfg.pop("namespace", None)
    semantic_save_dir = _semantic_save_dir(cfg, enabled=enable_semantic)
    from lingtu.assembly.stacks.composition import compose_full_stack_modules
    from lingtu.assembly.stacks.driver import driver_name
    from lingtu.assembly.stacks.system import run_startup_preflight
    from lingtu.assembly.wires.full_stack import apply_full_stack_wires

    driver_module = driver_name(robot) if bool(cfg.get("enable_robot_driver", True)) else ""

    bp = compose_full_stack_modules(
        robot=robot,
        driver_module=driver_module,
        slam_profile=slam_profile,
        detector=detector,
        encoder=encoder,
        llm=llm,
        gateway_port=gateway_port,
        enable_semantic=enable_semantic,
        enable_gateway=enable_gateway,
        enable_teleop=enable_teleop,
        enable_navigation=enable_navigation,
        semantic_save_dir=semantic_save_dir,
        config=cfg,
    )
    bp = apply_full_stack_wires(
        bp,
        driver_module=driver_module,
        slam_profile=slam_profile,
        enable_semantic=enable_semantic,
    )

    if run_startup_checks:
        bp.before_build(
            lambda: run_startup_preflight(
                enable_semantic=enable_semantic,
                slam_profile=slam_profile,
            )
        )

    if namespace:
        bp.namespace(str(namespace))

    return bp
