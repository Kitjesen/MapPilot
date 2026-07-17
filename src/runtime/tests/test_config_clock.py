"""Tests for runtime.config and runtime.clock modules."""

import os
import textwrap
import time

import pytest

# ---------------------------------------------------------------------------
# Config tests
# ---------------------------------------------------------------------------
from runtime.config import (
    RobotConfig,
    get_config,
    load_config,
    reset_config,
)


class TestLoadConfigDefaults:
    """load_config() with no file returns sensible dataclass defaults."""

    def test_returns_robot_config(self):
        cfg = load_config(path="/nonexistent/path.yaml")
        assert isinstance(cfg, RobotConfig)

    def test_speed_defaults(self):
        cfg = load_config(path="/nonexistent/path.yaml")
        assert cfg.speed.max_speed == 0.875
        assert cfg.speed.max_linear == 1.0

    def test_geometry_defaults(self):
        cfg = load_config(path="/nonexistent/path.yaml")
        assert cfg.geometry.vehicle_height == 0.5
        assert cfg.geometry.sensor_offset_x == 0.3

    def test_driver_defaults(self):
        cfg = load_config(path="/nonexistent/path.yaml")
        assert cfg.driver.dog_port == 13145
        assert cfg.driver.auto_enable is False
        assert cfg.driver.auto_standup is False

    def test_safety_defaults(self):
        cfg = load_config(path="/nonexistent/path.yaml")
        assert cfg.safety.tilt_limit_deg == 30.0

    def test_raw_empty_on_missing_file(self):
        cfg = load_config(path="/nonexistent/path.yaml")
        assert cfg.raw == {}


class TestLoadConfigFromYAML:
    """load_config() reads the actual repo config/robot_config.yaml."""

    def test_reads_real_config(self):
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
        real_path = os.path.join(repo_root, "config", "robot_config.yaml")
        if not os.path.isfile(real_path):
            pytest.skip("config/robot_config.yaml not found")

        cfg = load_config(path=real_path)
        # These values come from the actual YAML, not defaults
        assert cfg.speed.max_speed == 0.875
        assert cfg.geometry.vehicle_width == 0.6
        assert cfg.driver.dog_host == "127.0.0.1"
        assert cfg.safety.obstacle_height_thre == 0.2
        assert cfg.lidar.lidar_ip == "192.168.1.178"
        assert cfg.lidar.host_ip == "192.168.1.5"
        assert "robot" in cfg.raw  # raw dict contains full YAML

    def test_custom_yaml(self, tmp_path):
        yaml_content = textwrap.dedent("""\
            speed:
              max_speed: 2.5
              max_linear: 3.0
            geometry:
              vehicle_height: 0.8
            driver:
              dog_host: "10.0.0.1"
              dog_port: 9999
            safety:
              tilt_limit_deg: 45.0
        """)
        cfg_file = tmp_path / "test_config.yaml"
        cfg_file.write_text(yaml_content, encoding="utf-8")

        cfg = load_config(path=str(cfg_file))
        assert cfg.speed.max_speed == 2.5
        assert cfg.speed.max_linear == 3.0
        # Fields not in YAML keep their defaults
        assert cfg.speed.autonomy_speed == 0.875
        assert cfg.geometry.vehicle_height == 0.8
        assert cfg.driver.dog_host == "10.0.0.1"
        assert cfg.driver.dog_port == 9999
        assert cfg.safety.tilt_limit_deg == 45.0

    def test_ignores_unknown_keys(self, tmp_path):
        yaml_content = textwrap.dedent("""\
            speed:
              max_speed: 1.5
              unknown_key: 42
        """)
        cfg_file = tmp_path / "test_config.yaml"
        cfg_file.write_text(yaml_content, encoding="utf-8")

        cfg = load_config(path=str(cfg_file))
        assert cfg.speed.max_speed == 1.5
        assert not hasattr(cfg.speed, "unknown_key")


class TestGetConfigSingleton:
    """get_config() returns a singleton and reset_config() clears it."""

    def setup_method(self):
        reset_config()

    def teardown_method(self):
        reset_config()

    def test_singleton_returns_same_object(self):
        a = get_config()
        b = get_config()
        assert a is b

    def test_reset_creates_new_instance(self):
        a = get_config()
        reset_config()
        b = get_config()
        assert a is not b


# ---------------------------------------------------------------------------
# Clock tests
# ---------------------------------------------------------------------------

from runtime.clock import Clock, clock


class TestClockRealTime:
    """Clock in real-time (default) mode."""

    def test_default_is_real(self):
        c = Clock()
        assert c.is_sim is False

    def test_now_returns_wall_time(self):
        c = Clock()
        before = time.time()
        now = c.now()
        after = time.time()
        assert before <= now <= after

    def test_sleep_real(self):
        c = Clock()
        t0 = time.time()
        c.sleep(0.05)
        elapsed = time.time() - t0
        assert elapsed >= 0.04  # allow small tolerance

    def test_sleep_zero_or_negative(self):
        c = Clock()
        t0 = time.time()
        c.sleep(0)
        c.sleep(-1.0)
        elapsed = time.time() - t0
        assert elapsed < 0.1  # should return immediately


class TestClockSimTime:
    """Clock in simulation mode."""

    def test_set_sim_time(self):
        c = Clock()
        c.set_sim_time(100.0)
        assert c.is_sim is True
        assert c.now() == 100.0

    def test_sim_time_updates(self):
        c = Clock()
        c.set_sim_time(10.0)
        assert c.now() == 10.0
        c.set_sim_time(20.0)
        assert c.now() == 20.0

    def test_set_real_time_exits_sim(self):
        c = Clock()
        c.set_sim_time(50.0)
        assert c.is_sim is True
        c.set_real_time()
        assert c.is_sim is False
        # now() should return wall time, not 50.0
        assert abs(c.now() - time.time()) < 1.0

    def test_sleep_respects_sim_speed(self):
        c = Clock()
        c.set_sim_time(0.0)
        c.set_sim_speed(10.0)  # 10x speed -> 0.05s sleep becomes 0.005s wall
        t0 = time.time()
        c.sleep(0.05)
        elapsed = time.time() - t0
        assert elapsed < 0.03  # should be ~0.005s, well under 0.03s

    def test_sim_speed_clamped(self):
        c = Clock()
        c.set_sim_speed(0.0)
        assert c.sim_speed == 0.01
        c.set_sim_speed(-5.0)
        assert c.sim_speed == 0.01


class TestGlobalClockSingleton:
    """The module-level ``clock`` singleton works correctly."""

    def test_is_clock_instance(self):
        assert isinstance(clock, Clock)

    def test_default_real_mode(self):
        # Ensure we are in real mode (clean up any prior sim state)
        clock.set_real_time()
        assert clock.is_sim is False
