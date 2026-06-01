"""Contract tests: verify global_planning subpackages import without error."""


def test_import_pct_adapters():
    """global_planning.pct_adapters package is importable."""
    import global_planning.pct_adapters  # noqa: F401


def test_import_pct_adapters_module():
    """pct_adapters.global_planner_module is importable."""
    from global_planning.pct_adapters import global_planner_module

    assert global_planner_module is not None


def test_import_pct_planner_config():
    """pct_planner.planner.config.Config is importable."""
    from global_planning.pct_planner.planner.config import Config

    assert Config is not None


def test_import_pct_planner_runnable():
    """global_planning.pct_planner_runnable is importable."""
    import global_planning.pct_planner_runnable  # noqa: F401


def test_import_all_subpackages():
    """All global_planning subpackages import cleanly together."""
    from global_planning import pct_adapters, pct_planner_runnable

    assert pct_adapters is not None
    assert pct_planner_runnable is not None


def test_pct_planner_config_and_instantiation():
    """PCT planner Config and its nested sub-configs instantiate cleanly."""
    from global_planning.pct_planner.planner.config.param import (
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
    """GlobalPlannerModule can be instantiated (stub/AStar backend, no ROS2)."""
    from nav.global_planner_service import GlobalPlannerService

    svc = GlobalPlannerService()
    assert svc is not None
    assert hasattr(svc, "plan")
    assert hasattr(svc, "_find_safe_goal")
