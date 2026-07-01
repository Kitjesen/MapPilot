"""Contract tests: verify planner subpackages import without error."""


def test_import_octoplanner3d_backend():
    """Product OctoPlanner3D backend is importable without ROS2."""
    from nav.services.plan.global_planner.algorithm import octoplanner3d as backend

    assert backend is not None


def test_import_pct_planner_config():
    """pct_planner.planner.config.Config is importable."""
    from nav.services.plan.global_planner.algorithm.pct.vendor.pct_planner.planner.config import Config

    assert Config is not None


def test_import_pct_runtime():
    """Product PCT runtime location remains importable for benchmarks."""
    import nav.services.plan.global_planner.algorithm.pct.runtime  # noqa: F401


def test_import_all_subpackages():
    """Product planner subpackages import cleanly together."""
    from nav.services.plan.global_planner.algorithm import octoplanner3d as octoplanner3d_backend
    import nav.services.plan.global_planner.algorithm.pct.runtime as pct_runtime

    assert octoplanner3d_backend is not None
    assert pct_runtime is not None


def test_pct_planner_config_and_instantiation():
    """PCT planner Config and its nested sub-configs instantiate cleanly."""
    from nav.services.plan.global_planner.algorithm.pct.vendor.pct_planner.planner.config.param import (
        Config, ConfigNode, ConfigPlanner, ConfigWrapper,
    )

    cfg = Config()
    assert isinstance(cfg.node, ConfigNode)
    assert isinstance(cfg.planner, ConfigPlanner)
    assert isinstance(cfg.wrapper, ConfigWrapper)

    # Verify default values on the node config
    assert cfg.node.map_frame == "map"
    assert cfg.node.robot_frame == "body"
    assert cfg.node.min_plan_interval > 0.0


def test_global_planner_module_instantiation():
    """GlobalPlanner can be instantiated without ROS2."""
    from nav.services.plan.global_planner.service import GlobalPlanner

    svc = GlobalPlanner()
    assert svc is not None
    assert hasattr(svc, "plan")
    assert hasattr(svc, "_find_safe_goal")
