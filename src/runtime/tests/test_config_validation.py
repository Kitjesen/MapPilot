import unittest

from runtime.config import GeometryConfig, RobotConfig, SafetyConfig, SpeedConfig, validate_config


class TestConfigValidation(unittest.TestCase):
    def test_default_config_valid(self):
        cfg = RobotConfig()
        errors = validate_config(cfg)
        self.assertEqual(errors, [])

    def test_negative_speed_fails(self):
        cfg = RobotConfig(speed=SpeedConfig(max_linear=-1.0))
        errors = validate_config(cfg)
        self.assertTrue(any("max_linear" in e for e in errors))

    def test_zero_vehicle_width_fails(self):
        cfg = RobotConfig(geometry=GeometryConfig(vehicle_width=0))
        errors = validate_config(cfg)
        self.assertTrue(any("vehicle_width" in e for e in errors))

    def test_stop_distance_exceeds_slow(self):
        cfg = RobotConfig(safety=SafetyConfig(stop_distance=3.0, slow_distance=2.0))
        errors = validate_config(cfg)
        self.assertTrue(any("slow_distance" in e for e in errors))
