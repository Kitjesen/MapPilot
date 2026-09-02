import unittest

from runtime.config import (
    DriverConfig,
    GeometryConfig,
    RobotConfig,
    SafetyConfig,
    SpeedConfig,
    validate_config,
)


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

    def test_invalid_local_collision_envelope_fails(self):
        cfg = RobotConfig(
            geometry=GeometryConfig(
                collision_cylinder_radius=0.0,
                collision_cylinder_offset=-0.1,
                collision_clearance_below=-0.1,
            )
        )
        errors = validate_config(cfg)
        self.assertTrue(any("collision_cylinder_radius" in error for error in errors))
        self.assertTrue(any("collision_cylinder_offset" in error for error in errors))
        self.assertTrue(any("collision clearances" in error for error in errors))

    def test_stop_distance_exceeds_slow(self):
        cfg = RobotConfig(safety=SafetyConfig(stop_distance=3.0, slow_distance=2.0))
        errors = validate_config(cfg)
        self.assertTrue(any("slow_distance" in e for e in errors))

    def test_driver_control_rate_must_honor_lease_refresh(self):
        cfg = RobotConfig(driver=DriverConfig(control_rate=9.0))
        errors = validate_config(cfg)
        self.assertTrue(any("control_rate" in error for error in errors))

    def test_go2_requires_one_static_robot_subnet_binding(self):
        cfg = RobotConfig(
            driver=DriverConfig(
                backend="go2",
                network_interface="eth0",
            )
        )

        errors = validate_config(cfg)

        self.assertTrue(any("network_address" in error for error in errors))
        self.assertTrue(any("probe_ip" in error for error in errors))

    def test_go2_probe_must_share_the_configured_subnet(self):
        cfg = RobotConfig(
            driver=DriverConfig(
                backend="go2",
                network_interface="eth0",
                network_address="192.168.123.18/24",
                probe_ip="192.168.66.204",
            )
        )

        errors = validate_config(cfg)

        self.assertTrue(any("same subnet" in error for error in errors))
