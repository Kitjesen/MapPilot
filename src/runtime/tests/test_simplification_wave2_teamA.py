"""Wave 2 Team A 茅聢?behavioural tests for items W2-2, W2-6, W2-7, W2-9.

All tests are mock-based and require no hardware, ROS2, or heavy dependencies.
"""

from __future__ import annotations

import types
import unittest
from unittest.mock import MagicMock, patch

# ---------------------------------------------------------------------------
# W2-2: InstanceTracker 茅聢?fx=0 should warn once and skip 3D extent
# ---------------------------------------------------------------------------

class TestInstanceTrackerFxZero(unittest.TestCase):
    """W2-2: when intrinsics_fx=0, warn once per tracker and skip extent."""

    def _make_tracker(self):
        """Import and instantiate InstanceTracker with minimal deps mocked."""
        # Stub heavy deps that InstanceTracker transitively imports
        stub_modules = [
            "torch", "torchvision", "clip", "PIL", "PIL.Image",
            "open3d", "chromadb",
        ]
        patches = {}
        for mod in stub_modules:
            patches[mod] = types.ModuleType(mod)
        with patch.dict("sys.modules", patches):
            from perception.tracking.instance_tracker import InstanceTracker
        return InstanceTracker()

    def test_warned_no_fx_flag_initialises_false(self):
        """_warned_no_fx must exist and start False."""
        tracker = self._make_tracker()
        self.assertFalse(tracker._warned_no_fx)

    def test_warning_fires_once_not_per_frame(self):
        """A single WARNING is logged for fx=0, not repeated on subsequent calls."""
        tracker = self._make_tracker()

        with patch("logging.Logger.warning") as mock_warn:
            # Call update 3 times with fx=0 and empty detections
            for _ in range(3):
                tracker.update([], intrinsics_fx=0.0)

        # Warning about missing fx must fire exactly once
        fx_warnings = [
            c for c in mock_warn.call_args_list
            if "intrinsics_fx" in str(c)
        ]
        self.assertEqual(len(fx_warnings), 1, "Expected exactly one fx=0 warning across 3 calls")

    def test_no_warning_when_fx_valid(self):
        """No fx warning when intrinsics_fx > 0."""
        tracker = self._make_tracker()

        with patch("logging.Logger.warning") as mock_warn:
            tracker.update([], intrinsics_fx=615.0)

        fx_warnings = [c for c in mock_warn.call_args_list if "intrinsics_fx" in str(c)]
        self.assertEqual(len(fx_warnings), 0)


# ---------------------------------------------------------------------------
# W2-6: LocalPlanner 茅聢?grid params loaded from config
# ---------------------------------------------------------------------------

class TestLocalPlannerGridParams(unittest.TestCase):
    """W2-6: _score_paths_numpy accepts grid params; _setup_cmu_py loads them."""

    def _import_score_fn(self):
        from nav.services.plan.local_planner.service import _score_paths_numpy
        return _score_paths_numpy

    def test_score_paths_numpy_accepts_grid_kwargs(self):
        """_score_paths_numpy must accept the 4 grid keyword params without error."""
        import numpy as np
        fn = self._import_score_fn()
        result = fn(
            obstacle_pts=np.zeros((0, 3), dtype=np.float32),
            rel_goal_x=1.0,
            rel_goal_y=0.0,
            rel_goal_dis=1.0,
            joy_dir_deg=0.0,
            correspondences={},
            group_of_path=np.zeros(343, dtype=np.int32),
            grid_voxel_num_x=161,
            grid_voxel_num_y=531,
            grid_voxel_size=0.02,
            grid_voxel_offset_x=3.2,
            grid_voxel_offset_y=5.25,
            search_radius=0.45,
        )
        self.assertEqual(result.shape, (36 * 7,))

    def test_grid_attrs_exist_on_module(self):
        """LocalPlanner must expose _grid_* attributes after __init__."""
        # Stub nav_kernel loader so it doesn't fail import
        stub = types.ModuleType("lingtu_nav_kernel")
        with patch.dict("sys.modules", {"lingtu_nav_kernel": stub}):
            from nav.services.plan.local_planner.service import LocalPlanner
        m = LocalPlanner(backend="simple")
        self.assertEqual(m._grid_voxel_size, 0.02)
        self.assertEqual(m._grid_voxel_offset_x, 3.2)
        self.assertEqual(m._grid_voxel_offset_y, 5.25)
        self.assertEqual(m._grid_search_radius, 0.45)


_scipy_available = True
try:
    import scipy.ndimage  # noqa: F401 -- availability check for skipUnless
except ImportError:
    _scipy_available = False


# ---------------------------------------------------------------------------
# W2-7: TraversabilityCost 茅聢?bilinear resampling + scipy guard
# ---------------------------------------------------------------------------

@unittest.skipUnless(_scipy_available, "scipy not installed in this environment")
class TestTraversabilityCostResample(unittest.TestCase):
    """W2-7: _resample_to_grid uses bilinear; setup raises if scipy absent."""

    def test_resample_bilinear_centre_cell(self):
        """Bilinear resample of a uniform grid returns the same constant value."""
        import numpy as np

        from nav.services.map_layers.traversability_cost_module import _resample_to_grid

        src = np.full((4, 4), 50.0, dtype=np.float32)
        src_origin = np.array([0.0, 0.0])
        src_res = 1.0
        dst_shape = (8, 8)
        dst_origin = np.array([0.0, 0.0])
        dst_res = 0.5

        out = _resample_to_grid(src, src_origin, src_res, dst_shape, dst_origin, dst_res, fill=0.0)
        # Interior cells of a uniform source must come back as 50
        self.assertAlmostEqual(float(out[2, 2]), 50.0, places=3)

    def test_resample_bilinear_interpolates_gradient(self):
        """Bilinear resample produces strict intermediate values within src extent.

        Source: 2x2 grid with a single corner spike:
            src = [[0, 0],
                   [0, 100]]
        src_origin = (0, 0), src_res = 1.0 茅聢?so src cell centres sit at
        world coords (0.5, 0.5), (0.5, 1.5), (1.5, 0.5), (1.5, 1.5).

        Sample ONE destination cell whose centre sits exactly at world (1.0, 1.0)
        茅聢?the geometric centroid of the 4 src cells. Bilinear must return the
        4-cell average (0+0+0+100)/4 = 25. A nearest-neighbour resampler would
        snap to one of the four corners (0 or 100, depending on tie-break), so
        a value in the 20-30 band unambiguously proves bilinear is active.
        """
        import numpy as np

        from nav.services.map_layers.traversability_cost_module import _resample_to_grid

        src = np.array([[0.0, 0.0], [0.0, 100.0]], dtype=np.float32)
        src_origin = np.array([0.0, 0.0])
        src_res = 1.0
        dst_shape = (1, 1)
        dst_origin = np.array([0.5, 0.5])  # dst cell centre at world (1.0, 1.0)
        dst_res = 1.0

        out = _resample_to_grid(src, src_origin, src_res, dst_shape, dst_origin, dst_res, fill=0.0)
        val = float(out[0, 0])
        self.assertAlmostEqual(
            val, 25.0, delta=1.0,
            msg=f"Bilinear at (1.0, 1.0) must average 4 cells = 25 (got {val}). "
                "A value near 0 or 100 would mean nearest-neighbour is still active.",
        )

    def test_setup_raises_when_scipy_absent(self):
        """TraversabilityCostModule.setup() raises RuntimeError if scipy is missing."""
        import nav.services.map_layers.traversability_cost_module as tcm_mod
        original = tcm_mod._SCIPY_AVAILABLE
        try:
            tcm_mod._SCIPY_AVAILABLE = False
            from nav.services.map_layers.traversability_cost_module import TraversabilityCostModule
            m = TraversabilityCostModule()
            with self.assertRaises(RuntimeError, msg="Expected RuntimeError when scipy absent"):
                m.setup()
        finally:
            tcm_mod._SCIPY_AVAILABLE = original


# ---------------------------------------------------------------------------
# W2-9: SLAM bridge 茅聢?strict DOF mask preference over heuristic
# ---------------------------------------------------------------------------

class TestSlamBridgeDofMaskPreference(unittest.TestCase):
    """W2-9: _fuse_odometry uses eigenvalue path when _dof_mask is available."""

    def _make_bridge(self):
        """Build a minimal SlamBridgeModule instance with mocked ROS2 deps."""
        ros_stubs = {
            "rclpy": types.ModuleType("rclpy"),
            "rclpy.node": types.ModuleType("rclpy.node"),
            "rclpy.qos": types.ModuleType("rclpy.qos"),
        }
        ros_stubs["rclpy.node"].Node = object
        ros_stubs["rclpy.qos"].QoSProfile = MagicMock()
        ros_stubs["rclpy.qos"].ReliabilityPolicy = MagicMock()
        ros_stubs["rclpy.qos"].HistoryPolicy = MagicMock()

        with patch.dict("sys.modules", ros_stubs):
            from localization.bridge import SlamBridgeModule

        # Construct without calling Module.__init__ wiring
        obj = object.__new__(SlamBridgeModule)
        # Populate just what _fuse_odometry touches
        obj._visual_odom_fusion = True
        obj._degen_level = "SEVERE"        # triggers fusion path
        obj._last_visual_odom = None        # will be set per-test
        obj._visual_alpha = 0.6
        obj._feature_ratio_critical = 0.3
        obj._fitness_critical = 0.3
        obj._effective_ratio = 0.1         # would trigger heuristic full-XYZ
        obj._icp_fitness = 0.5             # would trigger heuristic corridor
        obj._visual_fused_count = 0
        obj._last_slam_odom = None
        obj._dof_mask = None
        # GNSS fusion was introduced in a later merge; disable it here so the
        # test only exercises the visual-odometry / DOF-mask path.
        obj._gnss_fusion = False
        obj._last_gnss_odom = None
        return obj

    def _make_odom(self, x=0.0, y=0.0, z=0.0):
        """Build a minimal Odometry-like object."""
        from runtime.msgs.geometry import Pose, Quaternion, Vector3
        from runtime.msgs.nav import Odometry
        return Odometry(
            pose=Pose(
                position=Vector3(x, y, z),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            twist=None,
            ts=0.0,
        )

    def test_eigenvalue_path_used_when_dof_mask_available(self):
        """When _dof_mask has 3+ elements, eigenvalue path runs (debug log)."""
        import numpy as np
        bridge = self._make_bridge()
        bridge._last_visual_odom = self._make_odom(1.0, 2.0, 0.0)
        # All 3 translational DOFs fully degenerate (0.0 = degenerate)
        bridge._dof_mask = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0])

        slam_odom = self._make_odom(0.0, 0.0, 0.0)
        with patch("logging.Logger.debug") as mock_debug:
            bridge._fuse_odometry(slam_odom)

        debug_msgs = [str(c) for c in mock_debug.call_args_list]
        eigenvalue_logged = any("eigenvalue" in m for m in debug_msgs)
        self.assertTrue(eigenvalue_logged, "Expected 'eigenvalue' in debug log when dof_mask present")

    def test_heuristic_path_used_when_dof_mask_is_none(self):
        """When _dof_mask is None, heuristic path runs (debug log)."""
        bridge = self._make_bridge()
        bridge._last_visual_odom = self._make_odom(1.0, 2.0, 0.0)
        bridge._dof_mask = None  # explicitly no mask

        slam_odom = self._make_odom(0.0, 0.0, 0.0)
        with patch("logging.Logger.debug") as mock_debug:
            bridge._fuse_odometry(slam_odom)

        debug_msgs = [str(c) for c in mock_debug.call_args_list]
        heuristic_logged = any("heuristic" in m for m in debug_msgs)
        self.assertTrue(heuristic_logged, "Expected 'heuristic' in debug log when dof_mask is None")

    def test_short_dof_mask_falls_to_heuristic(self):
        """When _dof_mask has fewer than 3 elements, heuristic path is used."""
        import numpy as np
        bridge = self._make_bridge()
        bridge._last_visual_odom = self._make_odom(1.0, 2.0, 0.0)
        bridge._dof_mask = np.array([0.0, 1.0])  # only 2 elements

        slam_odom = self._make_odom(0.0, 0.0, 0.0)
        with patch("logging.Logger.debug") as mock_debug:
            bridge._fuse_odometry(slam_odom)

        debug_msgs = [str(c) for c in mock_debug.call_args_list]
        heuristic_logged = any("heuristic" in m for m in debug_msgs)
        self.assertTrue(heuristic_logged, "Expected heuristic path for short dof_mask (len < 3)")


if __name__ == "__main__":
    unittest.main()
