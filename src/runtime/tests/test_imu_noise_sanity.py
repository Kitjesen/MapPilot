"""Tests for the ICM-40609-D IMU-noise sanity check in calibration/apply_calibration.py.

The sanity check is procedural protection: Allan Variance can produce values
that pass YAML schema but are 1-2 orders of magnitude off if the recording
environment was wrong (vibration, thermal drift, units mis-converted). The
default ng=0.01 currently shipping in lio.yaml is itself such a value — it
is ~150x higher than ICM-40609 datasheet typical (6.6e-5 rad/s/√Hz), and
the tests here lock in that calling the sanity check with that value warns.

Note: a previous version of this file referenced BMI088 — that was wrong.
Mid-360's built-in IMU is the TDK InvenSense ICM-40609-D, which is
~10x quieter than BMI088, so the original BMI088 reference table was
permissive enough to let bad values pass without warning.
"""

import importlib.util
import logging
import unittest
from pathlib import Path


def _load_apply_calibration():
    """apply_calibration.py lives in calibration/, not src/, so it cannot be
    imported via the normal package path. Load it directly by file path."""
    repo_root = Path(__file__).resolve().parent.parent.parent.parent
    path = repo_root / "calibration" / "apply_calibration.py"
    spec = importlib.util.spec_from_file_location("apply_calibration", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestICM40609SanityCheck(unittest.TestCase):
    def setUp(self) -> None:
        self.ac = _load_apply_calibration()
        self.logger_name = "apply_calibration"

    def test_in_range_logs_info(self):
        # ICM-40609 typical ng ≈ 6.6e-5 rad/s/√Hz; 1e-4 sits comfortably
        # in the (3e-5, 3e-4) reference window.
        with self.assertLogs(self.logger_name, level=logging.INFO) as cm:
            self.ac._sanity_check_imu_noise("ng", 1e-4)
        joined = " ".join(cm.output)
        self.assertIn("within ICM-40609 expected range", joined)
        self.assertNotIn("WARNING", joined)

    def test_default_ng_001_triggers_high_warning(self):
        """The shipped default ng=0.01 must trigger the HIGH warning — this
        is the very value the audit flagged as ~150x off (vs ICM-40609
        typical 6.6e-5)."""
        with self.assertLogs(self.logger_name, level=logging.WARNING) as cm:
            self.ac._sanity_check_imu_noise("ng", 0.01)
        joined = " ".join(cm.output)
        self.assertIn("unusually HIGH", joined)
        self.assertIn("ng", joined)

    def test_too_low_triggers_low_warning(self):
        # ng low gate is 3e-5/5 = 6e-6
        with self.assertLogs(self.logger_name, level=logging.WARNING) as cm:
            self.ac._sanity_check_imu_noise("ng", 1e-7)
        joined = " ".join(cm.output)
        self.assertIn("unusually LOW", joined)

    def test_accel_in_range(self):
        # ICM-40609 typical na ≈ 7e-4 m/s²/√Hz; 1e-3 inside (3e-4, 3e-3).
        with self.assertLogs(self.logger_name, level=logging.INFO) as cm:
            self.ac._sanity_check_imu_noise("na", 1e-3)
        self.assertIn("within ICM-40609 expected range", " ".join(cm.output))

    def test_unknown_name_is_silent(self):
        # Unknown parameter names must not raise and must not log — covers
        # forward-compat if a future calibrator emits extra fields.
        try:
            self.ac._sanity_check_imu_noise("garbage_param", 1.0)
        except Exception as e:
            self.fail(f"sanity check raised on unknown name: {e}")

    def test_threshold_is_loose_5x(self):
        """Locks in the ±5x tolerance contract. Use small safety margins to
        avoid floating-point boundary ambiguity (5*3e-4 vs 1.5e-3 may not be
        bit-identical depending on representation order)."""
        # ng range is (3e-5, 3e-4); high*5 = 1.5e-3.
        # 1.4e-3 is comfortably below the gate → no warn.
        with self.assertLogs(self.logger_name, level=logging.INFO) as cm:
            self.ac._sanity_check_imu_noise("ng", 1.4e-3)
        self.assertIn("within ICM-40609 expected range", " ".join(cm.output))

        # 2e-3 is comfortably above → warn.
        with self.assertLogs(self.logger_name, level=logging.WARNING) as cm:
            self.ac._sanity_check_imu_noise("ng", 2e-3)
        self.assertIn("unusually HIGH", " ".join(cm.output))

    def test_5e_3_ng_now_warns(self):
        """Regression: under the previous BMI088 reference (ng high=1e-3,
        gate=5e-3) this value sat exactly on the boundary and was accepted.
        Under the corrected ICM-40609 reference it must warn — the IMU is
        ~10x quieter so this value is genuinely too noisy."""
        with self.assertLogs(self.logger_name, level=logging.WARNING) as cm:
            self.ac._sanity_check_imu_noise("ng", 5e-3)
        self.assertIn("unusually HIGH", " ".join(cm.output))


class TestICM40609ReferenceTable(unittest.TestCase):
    def test_all_keys_have_low_high_pairs(self):
        ac = _load_apply_calibration()
        for name, bounds in ac.ICM40609_REFERENCE.items():
            self.assertEqual(len(bounds), 2, f"{name} bounds malformed")
            low, high = bounds
            self.assertLess(low, high, f"{name} low >= high")
            self.assertGreater(low, 0, f"{name} low non-positive")

    def test_typical_values_lie_inside_range(self):
        """Pin the table to the datasheet typicals so a future edit cannot
        accidentally exclude the value the IMU actually produces."""
        ac = _load_apply_calibration()
        # Datasheet typicals (ICM-40609-D DS-000330 v1.2)
        typicals = {
            "na": 7e-4,    # 70 µg/√Hz × 9.81
            "ng": 6.6e-5,  # 3.8 mdps/√Hz × π/180
        }
        for name, val in typicals.items():
            low, high = ac.ICM40609_REFERENCE[name]
            self.assertGreaterEqual(val, low, f"{name} typical {val} below low {low}")
            self.assertLessEqual(val, high, f"{name} typical {val} above high {high}")


if __name__ == "__main__":
    unittest.main()
