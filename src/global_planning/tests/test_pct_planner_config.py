"""Tests for PCT planner Config (no C++ .so required)."""

import pytest


# ---------------------------------------------------------------------------
# Config Import & Defaults
# ---------------------------------------------------------------------------

class TestPCTConfig:
    """PCT planner config defaults and construction."""

    def test_config_import(self):
        """Config and all sub-configs import cleanly."""
        from global_planning.pct_planner.planner.config.param import (
            Config, ConfigNode, ConfigPlanner, ConfigWrapper,
        )
        assert Config is not None
        assert ConfigNode is not None
        assert ConfigPlanner is not None
        assert ConfigWrapper is not None

    def test_config_package_import(self):
        """The config package __init__ re-exports Config."""
        from global_planning.pct_planner.planner.config import Config
        assert Config is not None

    def test_config_default_node_values(self):
        """ConfigNode fields have expected defaults."""
        from global_planning.pct_planner.planner.config.param import ConfigNode
        node = ConfigNode()
        assert node.map_file == 'spiral0.3_2'
        assert node.map_frame == 'map'
        assert node.robot_frame == 'body'
        assert node.min_plan_interval > 0.0
        assert node.default_goal_height == 0.0
        assert node.publish_map_pointcloud is True
        assert node.publish_tomogram is True

    def test_config_default_planner_values(self):
        """ConfigPlanner fields have expected defaults."""
        from global_planning.pct_planner.planner.config.param import ConfigPlanner
        planner = ConfigPlanner()
        assert planner.use_quintic is True
        assert planner.max_heading_rate == 10
        assert planner.obstacle_thr == 50

    def test_config_default_wrapper_values(self):
        """ConfigWrapper fields have expected defaults."""
        from global_planning.pct_planner.planner.config.param import ConfigWrapper
        wrapper = ConfigWrapper()
        assert wrapper.tomo_dir == '/rsc/tomogram/'
        assert wrapper.pcd_dir is None
        assert wrapper.tomogram_resolution == 0.2
        assert wrapper.tomogram_slice_dh == 0.2
        assert wrapper.tomogram_ground_h == 0.0

    def test_config_compose_defaults(self):
        """Config aggregates all sub-configs with correct defaults."""
        from global_planning.pct_planner.planner.config.param import Config
        cfg = Config()
        assert cfg.node.map_frame == 'map'
        assert cfg.planner.obstacle_thr == 50
        assert cfg.wrapper.tomo_dir == '/rsc/tomogram/'

    def test_config_mutation(self):
        """Config fields can be overridden after construction."""
        from global_planning.pct_planner.planner.config.param import Config
        cfg = Config()
        cfg.node.map_frame = 'odom'
        cfg.planner.obstacle_thr = 75
        cfg.wrapper.tomogram_resolution = 0.5
        assert cfg.node.map_frame == 'odom'
        assert cfg.planner.obstacle_thr == 75
        assert cfg.wrapper.tomogram_resolution == 0.5

    def test_config_attr_names(self):
        """Config has expected sub-config attributes."""
        from global_planning.pct_planner.planner.config.param import Config
        cfg = Config()
        assert hasattr(cfg, "node")
        assert hasattr(cfg, "planner")
        assert hasattr(cfg, "wrapper")
        # Verify attribute types
        from global_planning.pct_planner.planner.config.param import (
            ConfigNode, ConfigPlanner, ConfigWrapper,
        )
        assert isinstance(cfg.node, ConfigNode)
        assert isinstance(cfg.planner, ConfigPlanner)
        assert isinstance(cfg.wrapper, ConfigWrapper)
