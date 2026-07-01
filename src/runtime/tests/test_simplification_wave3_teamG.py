"""Wave 3 Team G — remediation tests.

W3-3: TSDFColorVolume interface + error paths (Open3D mocked).
W3-2: Semantic scoring weights config-loaded + audit log emitted.
"""
from __future__ import annotations

import logging
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_REPO = Path(__file__).resolve().parent.parent.parent.parent


def _make_o3d_mock() -> types.ModuleType:
    """Build a minimal open3d mock that satisfies TSDFColorVolume.__init__."""
    o3d = types.ModuleType("open3d")

    # geometry sub-module
    geom = types.ModuleType("open3d.geometry")
    geom.Image = MagicMock(return_value=MagicMock())
    geom.RGBDImage = MagicMock()
    geom.RGBDImage.create_from_color_and_depth = MagicMock(return_value=MagicMock())
    o3d.geometry = geom

    # camera sub-module
    cam = types.ModuleType("open3d.camera")
    cam.PinholeCameraIntrinsic = MagicMock(return_value=MagicMock())
    o3d.camera = cam

    # pipelines.integration sub-module
    integration = types.ModuleType("open3d.pipelines.integration")
    mock_volume = MagicMock()

    # extract_point_cloud returns an object with .points and .colors
    fake_pcd = MagicMock()
    fake_pcd.points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float64)
    fake_pcd.colors = np.array([[0.5, 0.5, 0.5], [1.0, 0.0, 0.0]], dtype=np.float64)
    mock_volume.extract_point_cloud.return_value = fake_pcd

    fake_mesh = MagicMock()
    mock_volume.extract_triangle_mesh.return_value = fake_mesh
    mock_volume.integrate = MagicMock()

    integration.ScalableTSDFVolume = MagicMock(return_value=mock_volume)

    class _ColorType:
        RGB8 = "RGB8"

    integration.TSDFVolumeColorType = _ColorType
    pipelines = types.ModuleType("open3d.pipelines")
    pipelines.integration = integration
    o3d.pipelines = pipelines

    return o3d


# ---------------------------------------------------------------------------
# W3-3 tests
# ---------------------------------------------------------------------------

class TestTSDFColorVolumeInterface:
    """TSDFColorVolume wrapper interface tests using mocked Open3D."""

    def setup_method(self) -> None:
        # Inject mock open3d before importing reconstruction_module
        self._o3d_mock = _make_o3d_mock()
        sys.modules["open3d"] = self._o3d_mock
        sys.modules["open3d.geometry"] = self._o3d_mock.geometry
        sys.modules["open3d.camera"] = self._o3d_mock.camera
        sys.modules["open3d.pipelines"] = self._o3d_mock.pipelines
        sys.modules["open3d.pipelines.integration"] = self._o3d_mock.pipelines.integration

        # Fresh import of the module under test
        if "perception.reconstruction.reconstruction_module" in sys.modules:
            del sys.modules["perception.reconstruction.reconstruction_module"]
        from perception.reconstruction.reconstruction_module import TSDFColorVolume
        self.TSDFColorVolume = TSDFColorVolume

    def teardown_method(self) -> None:
        for key in list(sys.modules):
            if "open3d" in key:
                del sys.modules[key]

    def test_init_succeeds_with_mock_open3d(self) -> None:
        vol = self.TSDFColorVolume(voxel_length=0.04, sdf_trunc=0.15)
        assert vol.voxel_length == 0.04

    def test_init_raises_without_open3d(self) -> None:
        # Temporarily hide open3d
        saved = {}
        for key in list(sys.modules):
            if "open3d" in key:
                saved[key] = sys.modules.pop(key)
        real_import = __import__

        def block_open3d_import(name, *args, **kwargs):
            if name == "open3d" or name.startswith("open3d."):
                raise ImportError("open3d hidden for test")
            return real_import(name, *args, **kwargs)

        try:
            if "perception.reconstruction.reconstruction_module" in sys.modules:
                del sys.modules["perception.reconstruction.reconstruction_module"]
            from perception.reconstruction.reconstruction_module import TSDFColorVolume
            with patch("builtins.__import__", side_effect=block_open3d_import):
                with pytest.raises(RuntimeError, match="open3d"):
                    TSDFColorVolume()
        finally:
            sys.modules.update(saved)

    def test_extract_point_cloud_returns_pts_and_colors(self) -> None:
        vol = self.TSDFColorVolume()
        pts, colors = vol.extract_point_cloud()
        assert pts.shape == (2, 3)
        assert colors.shape == (2, 3)
        assert pts.dtype == np.float32
        assert colors.dtype == np.float32

    def test_extract_mesh_delegates_to_volume(self) -> None:
        vol = self.TSDFColorVolume()
        mesh = vol.extract_mesh()
        assert mesh is not None
        vol._volume.extract_triangle_mesh.assert_called_once()

    def test_integrate_calls_volume_integrate(self) -> None:
        vol = self.TSDFColorVolume()
        depth = np.ones((4, 4), dtype=np.uint16) * 1000  # 1 metre in mm
        color = np.zeros((4, 4, 3), dtype=np.uint8)
        K = np.array([[200, 0, 2], [0, 200, 2], [0, 0, 1]], dtype=np.float64)
        extrinsic = np.eye(4, dtype=np.float64)
        vol.integrate(depth, color, K, extrinsic)
        vol._volume.integrate.assert_called_once()


# ---------------------------------------------------------------------------
# W3-2 tests
# ---------------------------------------------------------------------------

class TestSemanticScoringWeightsConfig:
    """Verify each module loads weights from semantic_scoring.yaml and falls back correctly."""

    def _reset_module_globals(self) -> None:
        """Reset loaded flags so tests are independent."""

        # fast_path
        if "decision.goal_resolution.fast_path" in sys.modules:
            mod = sys.modules["decision.goal_resolution.fast_path"]
            mod._fast_path_weights_loaded = False

        # sgnav_reasoner
        if "decision.goal_resolution.sgnav_reasoner" in sys.modules:
            mod = sys.modules["decision.goal_resolution.sgnav_reasoner"]
            mod._sgnav_weights_loaded = False
            mod._SGNAV_WEIGHTS = dict(mod._DEFAULTS_SGNAV)

        # frontier_scorer
        if "decision.exploration.frontier_scorer" in sys.modules:
            mod = sys.modules["decision.exploration.frontier_scorer"]
            mod._frontier_weights_loaded = False
            mod._frontier_config_weight = mod._DEFAULT_SEMANTIC_PRIOR_WEIGHT

    def test_fast_path_loads_weights_from_yaml(self) -> None:
        """fast_path module reads label/clip/detector/spatial from the YAML file."""
        self._reset_module_globals()
        import decision.goal_resolution.fast_path as fp

        fp._fast_path_weights_loaded = False
        fake_yaml = {
            "fast_path_fusion": {
                "label": 0.40,
                "clip": 0.30,
                "detector": 0.20,
                "spatial": 0.10,
            }
        }
        with patch.object(fp, "_load_semantic_scoring_yaml", return_value=fake_yaml):
            fp._load_fast_path_weights()

        assert abs(fp.WEIGHT_LABEL_MATCH - 0.40) < 1e-6
        assert abs(fp.WEIGHT_CLIP_SIM - 0.30) < 1e-6
        assert abs(fp.WEIGHT_DETECTOR_SCORE - 0.20) < 1e-6
        assert abs(fp.WEIGHT_SPATIAL_HINT - 0.10) < 1e-6

    def test_fast_path_uses_defaults_when_section_absent(self, caplog) -> None:
        """fast_path logs INFO and retains defaults when section is missing."""
        import decision.goal_resolution.fast_path as fp

        # Fully reset module globals to defaults before testing the absent-section path
        fp._fast_path_weights_loaded = False
        fp.WEIGHT_LABEL_MATCH = fp._DEFAULTS_FAST_PATH["label"]
        fp.WEIGHT_CLIP_SIM = fp._DEFAULTS_FAST_PATH["clip"]
        fp.WEIGHT_DETECTOR_SCORE = fp._DEFAULTS_FAST_PATH["detector"]
        fp.WEIGHT_SPATIAL_HINT = fp._DEFAULTS_FAST_PATH["spatial"]

        with patch.object(fp, "_load_semantic_scoring_yaml", return_value={}):
            with caplog.at_level(logging.INFO):
                fp._load_fast_path_weights()

        assert abs(fp.WEIGHT_LABEL_MATCH - 0.35) < 1e-6
        assert "default weights" in caplog.text

    def test_sgnav_loads_weights_from_yaml(self) -> None:
        """sgnav_reasoner reads keyword/relation/distance/richness from YAML."""
        self._reset_module_globals()
        import decision.goal_resolution.sgnav_reasoner as sr

        sr._sgnav_weights_loaded = False
        sr._SGNAV_WEIGHTS = dict(sr._DEFAULTS_SGNAV)
        fake_yaml = {
            "sgnav_heuristic": {
                "keyword": 0.60,
                "relation": 0.15,
                "distance": 0.15,
                "richness": 0.10,
            }
        }
        with patch.object(sr, "_load_semantic_scoring_yaml", return_value=fake_yaml):
            sr._load_sgnav_weights()

        assert abs(sr._SGNAV_WEIGHTS["keyword"] - 0.60) < 1e-6
        assert abs(sr._SGNAV_WEIGHTS["relation"] - 0.15) < 1e-6

    def test_frontier_scorer_loads_weight_from_yaml(self) -> None:
        """FrontierScorer picks up semantic_prior_weight from YAML."""
        self._reset_module_globals()
        import decision.exploration.frontier_scorer as fs

        fs._frontier_weights_loaded = False
        fs._frontier_config_weight = fs._DEFAULT_SEMANTIC_PRIOR_WEIGHT
        fake_yaml = {"frontier_scorer": {"semantic_prior_weight": 0.25}}
        with patch.object(fs, "_load_semantic_scoring_yaml", return_value=fake_yaml):
            fs._load_frontier_weights()

        assert abs(fs._frontier_config_weight - 0.25) < 1e-6

    def test_sgnav_audit_log_emitted_at_debug(self, caplog) -> None:
        """_score_subgraphs_heuristic emits a DEBUG audit entry per candidate."""
        self._reset_module_globals()
        from decision.goal_resolution.sgnav_reasoner import (
            SGNavReasoner,
            SubgraphCandidate,
        )

        reasoner = SGNavReasoner()
        cand = SubgraphCandidate(
            subgraph_id="sg_0",
            level="room",
            center=np.array([1.0, 1.0]),
            object_ids=[1, 2],
            object_labels=["chair", "table"],
            relation_count=2,
        )
        with caplog.at_level(logging.DEBUG):
            scores, _reasons = reasoner._score_subgraphs_heuristic(
                instruction="find the chair",
                candidates=[cand],
                robot_position={"x": 0.0, "y": 0.0},
            )

        assert "sgnav_heuristic" in caplog.text
        assert "sg_0" in caplog.text
        assert "sg_0" in scores
