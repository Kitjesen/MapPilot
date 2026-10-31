import os
import subprocess
import sys
from pathlib import Path

import pytest

from lingtu.assembly.build_scripts import product_native_build_scripts

ROOT = Path(__file__).resolve().parents[3]


def test_exploration_build_plan_includes_default_variant_processes() -> None:
    scripts = product_native_build_scripts("explore")

    assert "scripts/build/build_livox_sdk2_stream.sh" in scripts
    assert "scripts/build/build_slam_core.sh" in scripts
    assert "scripts/build/build_nav_endpoint.sh" in scripts


def test_product_build_plan_does_not_resolve_an_environment(monkeypatch) -> None:
    def fail_host_compile(*_args, **_kwargs):
        raise AssertionError("build planning must not resolve Product + env")

    monkeypatch.setattr(
        "lingtu.assembly.config.product.resolve_product_host_runtime",
        fail_host_compile,
    )

    scripts = product_native_build_scripts("teleop_avoid")

    assert "scripts/build/build_livox_sdk2_stream.sh" in scripts
    assert "scripts/build/build_camera_dds.sh" not in scripts
    assert scripts[-1] == "scripts/build/build_native_recording.sh"


@pytest.mark.parametrize("camera_driver", ["orbbec_native", "realsense_native"])
def test_camera_variant_builds_selected_capture_before_dds(camera_driver: str) -> None:
    scripts = product_native_build_scripts("map", product_variant="camera", camera_driver=camera_driver)

    capture = f"scripts/build/build_{camera_driver}.sh"
    other = "orbbec_native" if camera_driver == "realsense_native" else "realsense_native"
    assert scripts[scripts.index(capture) + 1] == "scripts/build/build_camera_dds.sh"
    assert f"scripts/build/build_{other}.sh" not in scripts


def test_camera_default_preserves_existing_orbbec_build() -> None:
    scripts = product_native_build_scripts("map", product_variant="camera")

    assert "scripts/build/build_orbbec_native.sh" in scripts
    assert "scripts/build/build_realsense_native.sh" not in scripts


def test_standard_variant_does_not_build_an_unused_camera() -> None:
    scripts = product_native_build_scripts("map", product_variant="standard", camera_driver="realsense_native")

    assert not any("camera" in script or "realsense" in script or "orbbec" in script for script in scripts)


@pytest.mark.skipif(os.name == "nt", reason="Deployment plan requires native Bash")
def test_deploy_camera_plan_uses_robot_config_and_existing_variant() -> None:
    env = os.environ.copy()
    env.pop("LINGTU_DRIVER_BACKEND", None)
    env.update({
        "LINGTU_REPO": str(ROOT),
        "LINGTU_CONFIG_PATH": str(ROOT / "config/robots/unitree/go2/robot.yaml"),
        "LINGTU_DEPLOY_ROBOT": "unitree/go2",
        "LINGTU_DEPLOY_PLAN_ONLY": "1",
        "LINGTU_PYTHON": sys.executable,
        "LINGTU_CAMERA_DRIVER": "orbbec_native",
    })

    result = subprocess.run(
        ["bash", str(ROOT / "scripts/deploy/deploy_robot.sh"), "map", "--variant", "camera"],
        env=env, capture_output=True, text=True, check=False, timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "scripts/build/build_realsense_native.sh" in result.stdout
    assert "scripts/build/build_orbbec_native.sh" not in result.stdout
    assert "scripts/build/build_camera_dds.sh" in result.stdout
    assert "no files, services, or network settings were changed" in result.stdout


@pytest.mark.skipif(os.name == "nt", reason="Native Makefile requires a POSIX shell")
@pytest.mark.parametrize("camera_driver", ["orbbec_native", "realsense_native"])
def test_native_make_build_uses_selected_capture_without_running_builders(tmp_path: Path, camera_driver: str) -> None:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_bash = fake_bin / "bash"
    fake_bash.write_text('#!/bin/sh\nprintf "%s\\n" "$1" >> "$LINGTU_TEST_BUILD_LOG"\n', encoding="utf-8")
    fake_bash.chmod(0o755)
    log = tmp_path / "builders.log"
    env = {**os.environ, "PATH": f"{fake_bin}{os.pathsep}{os.environ['PATH']}", "LINGTU_TEST_BUILD_LOG": str(log)}

    result = subprocess.run(
        ["make", "--no-print-directory", "build", f"LINGTU_CAMERA_DRIVER={camera_driver}"],
        cwd=ROOT, env=env, capture_output=True, text=True, check=False, timeout=15,
    )

    assert result.returncode == 0, result.stderr
    builders = log.read_text(encoding="utf-8").splitlines()
    assert f"scripts/build/build_{camera_driver}.sh" in builders
    other = "orbbec_native" if camera_driver == "realsense_native" else "realsense_native"
    assert f"scripts/build/build_{other}.sh" not in builders
    assert "scripts/build/build_camera_dds.sh" in builders
    if camera_driver == "realsense_native":
        assert "scripts/build/fetch_orbbec_sdk.sh" not in builders
