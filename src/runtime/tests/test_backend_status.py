from __future__ import annotations

from unittest import mock
from argparse import Namespace

import pytest

from runtime.backend_status import BackendStatus


def test_backend_status_reports_configured_effective_and_degraded_reason():
    status = BackendStatus.configured_as("nanobind")

    assert status.as_health_fields() == {
        "configured_backend": "nanobind",
        "backend": "nanobind",
        "degraded": False,
        "degraded_reason": "",
    }

    status.use("cmu_py", reason="compatible LingTu native navigation kernel missing")

    assert status.as_health_fields() == {
        "configured_backend": "nanobind",
        "backend": "cmu_py",
        "degraded": True,
        "degraded_reason": "compatible LingTu native navigation kernel missing",
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


def test_path_follower_nav_kernel_missing_fails_fast():
    from nav.local import path_follower as mod
    from nav.local import path_follower_runtime as runtime
    from nav.local.path_follower_backend import (
        NavKernelPathFollowerAdapter,
    )

    follower = mod.PathFollower(backend="nav_kernel")
    with mock.patch.object(
        runtime,
        "create_nav_kernel_path_follower_adapter_from_tuning",
        return_value=NavKernelPathFollowerAdapter(
            runtime=None,
            degraded_reason="compatible LingTu native navigation kernel missing",
            build_hint="build nav core",
        ),
    ):
        with pytest.raises(RuntimeError, match="compatible LingTu native navigation kernel missing"):
            follower.setup()

    info = follower.health()["path_follower"]
    assert info["configured_backend"] == "nav_kernel"
    assert info["backend"] == "nav_kernel"
    assert info["degraded"] is False
    assert info["degraded_reason"] == ""


def test_terrain_simple_backend_reports_not_degraded():
    from nav.local.terrain import Terrain

    terrain = Terrain(backend="simple")

    info = terrain.health()["terrain"]
    assert info["configured_backend"] == "simple"
    assert info["backend"] == "simple"
    assert info["degraded"] is False
    assert info["degraded_reason"] == ""


def test_perception_health_reports_detector_and_encoder_backend_status():
    from perception.perception_module import PerceptionModule

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
    from nav.local.local_planner import LocalPlanner  # noqa: F401
    from nav.local.path_follower import PathFollower  # noqa: F401
    from nav.local.terrain import Terrain  # noqa: F401
    from runtime.registry import list_plugins

    assert {"nanobind", "simple"} <= set(list_plugins("terrain"))
    assert {"nav_kernel", "pid"} <= set(list_plugins("path_follower"))
    assert {"nanobind", "cmu_py", "simple"} <= set(list_plugins("local_planner"))


def test_autonomy_backend_allowlists_match_registry_catalog():
    from nav.local import local_planner as local_planner
    from nav.local import path_follower
    from nav.local import terrain
    from runtime.registry import list_plugins

    assert set(terrain._AVAILABLE_TERRAIN_BACKENDS) <= set(list_plugins("terrain"))
    assert set(local_planner._AVAILABLE_LOCAL_PLANNER_BACKENDS) <= set(
        list_plugins("local_planner")
    )
    assert set(path_follower._AVAILABLE_PATH_FOLLOWER_BACKENDS) <= set(
        list_plugins("path_follower")
    )


def test_nav_kernel_runtime_prefers_fresh_build_dirs_before_src_artifacts():
    from nav.kernel import nav_kernel_candidate_dirs

    candidates = nav_kernel_candidate_dirs()

    build_nb_win = next(index for index, path in enumerate(candidates) if path.endswith("build_nb_win"))
    build_nb = next(index for index, path in enumerate(candidates) if path.endswith("build_nb"))
    src_dir = next(index for index, path in enumerate(candidates) if path.endswith("src"))

    assert build_nb_win < src_dir
    assert build_nb < src_dir


@pytest.mark.parametrize("backend", ["native", "cmu"])
def test_legacy_terrain_native_backends_fail_fast(backend):
    from nav.local.terrain import Terrain

    with pytest.raises(ValueError, match=f"Unknown terrain backend '{backend}'"):
        Terrain(backend=backend)


@pytest.mark.parametrize(
    ("module_path", "class_name", "category"),
    [
        ("nav.local.local_planner", "LocalPlanner", "local_planner"),
        ("nav.local.path_follower", "PathFollower", "path_follower"),
        ("nav.local.terrain", "Terrain", "terrain"),
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

    cfg = _resolve_config("sim_nav", args, allow_wizard=False)

    assert cfg["slam_profile"] == "bridge"
    assert cfg["exploration_backend"] == "none"
    assert cfg["local_planner_backend"] == "simple"
    assert cfg["path_follower_backend"] == "pid"
    assert cfg["terrain_backend"] == "simple"
    assert cfg["python_autonomy_backend"] == "simple"
    assert cfg["python_path_follower_backend"] == "pid"


@pytest.mark.parametrize(
    ("field", "backend", "match"),
    [
        (
            "local_planner_backend",
            "missing",
            "Unknown local_planner backend 'missing'",
        ),
        (
            "local_planner_backend",
            "cmu",
            "Unknown local_planner backend 'cmu'",
        ),
        (
            "path_follower_backend",
            "pure_pursuit",
            "Unknown path_follower backend 'pure_pursuit'",
        ),
        (
            "terrain_backend",
            "native",
            "Unknown terrain backend 'native'",
        ),
        (
            "terrain_backend",
            "cmu",
            "Unknown terrain backend 'cmu'",
        ),
    ],
)
def test_cli_backend_overrides_fail_fast_for_unknown_backend(
    capsys,
    field,
    backend,
    match,
):
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
        local_planner_backend=None,
        path_follower_backend=None,
        terrain_backend=None,
    )
    setattr(args, field, backend)

    with pytest.raises(SystemExit) as exc:
        _resolve_config("sim_nav", args, allow_wizard=False)
    assert exc.value.code == 2
    assert match in capsys.readouterr().out
