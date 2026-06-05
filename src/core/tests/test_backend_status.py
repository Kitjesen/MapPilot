from __future__ import annotations

from unittest import mock
from argparse import Namespace

import pytest

from core.backend_status import BackendStatus


def test_backend_status_reports_configured_effective_and_degraded_reason():
    status = BackendStatus.configured_as("nanobind")

    assert status.as_health_fields() == {
        "configured_backend": "nanobind",
        "backend": "nanobind",
        "degraded": False,
        "degraded_reason": "",
    }

    status.use("cmu_py", reason="compatible _nav_core missing")

    assert status.as_health_fields() == {
        "configured_backend": "nanobind",
        "backend": "cmu_py",
        "degraded": True,
        "degraded_reason": "compatible _nav_core missing",
    }


def test_backend_status_can_mark_same_backend_degraded():
    status = BackendStatus.configured_as("native")

    status.mark_degraded("native process failed to start")

    assert status.as_health_fields() == {
        "configured_backend": "native",
        "backend": "native",
        "degraded": True,
        "degraded_reason": "native process failed to start",
    }


def test_path_follower_nav_core_fallback_reports_backend_status():
    from base_autonomy.modules import path_follower_module as mod

    follower = mod.PathFollowerModule(backend="nav_core")
    with mock.patch.object(mod, "try_import_nav_core", return_value=None):
        follower._setup_nav_core()

    info = follower.health()["path_follower"]
    assert info["configured_backend"] == "nav_core"
    assert info["backend"] == "pid"
    assert info["degraded"] is True
    assert "_nav_core" in info["degraded_reason"]


def test_terrain_simple_backend_reports_not_degraded():
    from base_autonomy.modules.terrain_module import TerrainModule

    terrain = TerrainModule(backend="simple")

    info = terrain.health()["terrain"]
    assert info["configured_backend"] == "simple"
    assert info["backend"] == "simple"
    assert info["degraded"] is False
    assert info["degraded_reason"] == ""


def test_perception_health_reports_detector_and_encoder_backend_status():
    from semantic.perception.perception_module import PerceptionModule

    module = PerceptionModule(detector_type="bpu", encoder_type="mobileclip")

    health = module.health()

    assert health["detector"]["configured_backend"] == "bpu"
    assert health["detector"]["backend"] in {"bpu", "mock", "unavailable"}
    assert "degraded" in health["detector"]
    assert "degraded_reason" in health["detector"]
    assert health["encoder"]["configured_backend"] == "mobileclip"
    assert health["encoder"]["backend"] in {"mobileclip", "mock", "unavailable"}
    assert "degraded" in health["encoder"]
    assert "degraded_reason" in health["encoder"]


def test_autonomy_backend_registry_names_are_visible():
    from base_autonomy.modules.local_planner_module import LocalPlannerModule  # noqa: F401
    from base_autonomy.modules.path_follower_module import PathFollowerModule  # noqa: F401
    from base_autonomy.modules.terrain_module import TerrainModule  # noqa: F401
    from core.registry import list_plugins

    assert {"nanobind", "native", "cmu", "simple"} <= set(list_plugins("terrain"))
    assert {"nav_core", "pure_pursuit", "pid"} <= set(list_plugins("path_follower"))
    assert {"nanobind", "cmu", "cmu_py", "simple"} <= set(list_plugins("local_planner"))


def test_autonomy_backend_allowlists_match_registry_catalog():
    from base_autonomy.modules import local_planner_module
    from base_autonomy.modules import path_follower_module
    from base_autonomy.modules import terrain_module
    from core.registry import list_plugins

    assert set(terrain_module._AVAILABLE_TERRAIN_BACKENDS) <= set(list_plugins("terrain"))
    assert set(local_planner_module._AVAILABLE_LOCAL_PLANNER_BACKENDS) <= set(
        list_plugins("local_planner")
    )
    assert set(path_follower_module._AVAILABLE_PATH_FOLLOWER_BACKENDS) <= set(
        list_plugins("path_follower")
    )


def test_nav_core_loader_prefers_fresh_build_dirs_before_src_artifacts():
    from base_autonomy.modules import _nav_core_loader

    candidates = _nav_core_loader._candidate_dirs()

    build_nb_win = next(index for index, path in enumerate(candidates) if path.endswith("build_nb_win"))
    build_nb = next(index for index, path in enumerate(candidates) if path.endswith("build_nb"))
    src_dir = next(index for index, path in enumerate(candidates) if path.endswith("src"))

    assert build_nb_win < src_dir
    assert build_nb < src_dir


def test_terrain_cmu_backend_uses_native_setup(monkeypatch):
    from base_autonomy.modules.terrain_module import TerrainModule

    called = []
    terrain = TerrainModule(backend="cmu")
    monkeypatch.setattr(terrain, "_setup_native", lambda: called.append("native"))

    terrain.setup()

    assert called == ["native"]


@pytest.mark.parametrize(
    ("module_path", "class_name", "category"),
    [
        ("base_autonomy.modules.local_planner_module", "LocalPlannerModule", "local_planner"),
        ("base_autonomy.modules.path_follower_module", "PathFollowerModule", "path_follower"),
        ("base_autonomy.modules.terrain_module", "TerrainModule", "terrain"),
    ],
)
def test_unknown_autonomy_backends_fail_fast(module_path, class_name, category):
    module = __import__(module_path, fromlist=[class_name])
    cls = getattr(module, class_name)

    with pytest.raises(ValueError, match=f"Unknown {category} backend 'bogus'"):
        cls(backend="bogus")


def test_cli_backend_overrides_enter_resolved_blueprint_config():
    from cli.main import _resolve_config

    args = Namespace(
        target="show-config",
        endpoint=None,
        robot=None,
        dog_host=None,
        dog_port=None,
        detector=None,
        encoder=None,
        llm=None,
        planner=None,
        tomogram=None,
        plan_safety_policy=None,
        fallback_planner_name=None,
        gateway_port=None,
        no_semantic=False,
        no_gateway=False,
        native=False,
        no_native=False,
        rerun=False,
        slam_profile="bridge",
        exploration_backend="none",
        local_planner_backend="simple",
        path_follower_backend="pid",
        terrain_backend="simple",
    )

    cfg = _resolve_config("nav", args, allow_wizard=False)

    assert cfg["slam_profile"] == "bridge"
    assert cfg["exploration_backend"] == "none"
    assert cfg["local_planner_backend"] == "simple"
    assert cfg["path_follower_backend"] == "pid"
    assert cfg["terrain_backend"] == "simple"
    assert cfg["python_autonomy_backend"] == "simple"
    assert cfg["python_path_follower_backend"] == "pid"


def test_cli_backend_overrides_fail_fast_for_unknown_backend():
    from cli.main import _resolve_config

    args = Namespace(
        target="show-config",
        endpoint=None,
        robot=None,
        dog_host=None,
        dog_port=None,
        detector=None,
        encoder=None,
        llm=None,
        planner=None,
        tomogram=None,
        plan_safety_policy=None,
        fallback_planner_name=None,
        gateway_port=None,
        no_semantic=False,
        no_gateway=False,
        native=False,
        no_native=False,
        rerun=False,
        slam_profile=None,
        exploration_backend=None,
        local_planner_backend="missing",
        path_follower_backend=None,
        terrain_backend=None,
    )

    with pytest.raises(ValueError, match="Unknown local_planner backend 'missing'"):
        _resolve_config("nav", args, allow_wizard=False)
