from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from lingtu.assembly.config.camera import camera_environment
from lingtu.real.systemd import _ServiceInspector
from runtime.config import CameraConfig
from runtime.service_catalogs.real import service_spec

ROOT = Path(__file__).resolve().parents[2]


def _read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def _bash_path(path: Path) -> str:
    if os.name != "nt":
        return str(path)
    result = subprocess.run(
        ["bash", "-lc", f"wslpath -a {shlex.quote(str(path))}"],
        check=True,
        cwd=ROOT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return result.stdout.strip()


class _Systemd:
    def available(self, _target: str) -> bool:
        return True

    def active(self, _target: str) -> bool:
        return True


def test_video_service_catalog_and_unit_follow_product_lifecycle() -> None:
    spec = service_spec("video")
    assert spec is not None
    assert spec.start_units == ("lt-video.service",)
    assert spec.optional is True
    assert spec.checks == ("systemd", "native_binary", "status_file", "whep")
    assert spec.files == ("/dev/shm/lingtu/video_status.json",)
    assert spec.status_max_age_s == 3.0
    assert spec.binaries == (
        (
            "camera_webrtc",
            "LINGTU_VIDEO_BIN",
            "/opt/lingtu/current/bin/lingtu_camera_webrtc",
        ),
    )

    unit = _read("scripts/deploy/thunder/lt-video.service")
    assert "EnvironmentFile=/run/lingtu/session.env" in unit
    assert "EnvironmentFile=-/etc/lingtu/video.env" in unit
    assert "require_product_session.sh video" in unit
    assert "ExecStart=/bin/bash /opt/lingtu/current/scripts/deploy/thunder/run_video.sh" in unit
    assert "lt-camera.service" not in unit
    assert "WantedBy=" not in unit
    assert "Restart=" not in unit
    assert "Watchdog" not in unit


def test_video_launcher_uses_exact_encoder_and_preinstalled_runtime() -> None:
    launcher = _read("scripts/deploy/thunder/run_video.sh")

    assert "prepare_camera_video.sh" not in launcher
    assert 'source "${LINGTU_VIDEO_RUNTIME_ENV}"' in launcher
    assert ': "${LINGTU_VIDEO_ENCODER:?Set an exact target encoder in /etc/lingtu/video.env}"' in launcher
    assert '--encoder "${LINGTU_VIDEO_ENCODER}"' in launcher
    assert '--color-shm "${LINGTU_VIDEO_COLOR_SHM}"' in launcher
    assert '--status-file "${LINGTU_VIDEO_STATUS_FILE}"' in launcher
    assert '--idle-timeout-ms "${LINGTU_VIDEO_IDLE_TIMEOUT_MS}"' in launcher
    assert '--metrics "${LINGTU_VIDEO_METRICS_FILE}"' in launcher
    assert 'exec "${LINGTU_VIDEO_BIN}"' in launcher

    example = _read("config/video.env.example")
    assert "LINGTU_VIDEO_ENCODER=" in example
    assert "LINGTU_VIDEO_RUNTIME_ENV=/opt/lingtu/current/lib/lingtu/video/env.sh" in example
    assert "Do not copy build/deps/video/prefix/env.sh" in example

    packager = _read("scripts/deploy/package_native_release.sh")
    assert "build/camera_video/lingtu_camera_webrtc" in packager
    assert "install_cmake_tree \"${ROOT}/build/${component}\"" in packager
    assert "    camera_video " in packager
    assert 'VIDEO_RUNTIME_ARCH="$(tr -d' in packager
    assert '"${VIDEO_RUNTIME_ARCH}" != "${ARCH}"' in packager
    assert "lib/lingtu/video/bin/gst-inspect-1.0" in packager

    prepare = _read("scripts/build/prepare_camera_video.sh")
    assert "gcc -print-multiarch" in prepare
    assert '$(dirname "${BASH_SOURCE[0]}")' in prepare
    assert "GST_PLUGIN_SCANNER" in prepare
    assert 'printf \'%s\\n\' "${RUNTIME_ARCH}" > "${PREFIX}/runtime-arch"' in prepare
    assert 'lib/x86_64-linux-gnu' not in prepare

    build = _read("scripts/build/build_camera_video.sh")
    assert "Run scripts/build/prepare_camera_video.sh explicitly before make build." in build
    assert "runtime architecture ${RUNTIME_ARCH} does not match target ${TARGET_ARCH}" in build
    assert '-DLINGTU_VIDEO_RUNTIME_PREFIX="${RUNTIME_PREFIX}"' in build

    makefile = _read("Makefile")
    assert makefile.index("build_camera_dds.sh") < makefile.index("build_camera_video.sh")


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is required")
def test_video_launcher_fails_clearly_without_target_runtime(tmp_path: Path) -> None:
    launcher = _bash_path(ROOT / "scripts/deploy/thunder/run_video.sh")
    missing = _bash_path(tmp_path / "missing-env.sh")
    command = (
        f"LINGTU_VIDEO_BIN=/bin/true LINGTU_VIDEO_ENCODER=x264enc "
        f"LINGTU_VIDEO_RUNTIME_ENV={shlex.quote(missing)} bash {shlex.quote(launcher)}"
    )

    result = subprocess.run(
        ["bash", "-lc", command],
        check=False,
        cwd=ROOT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode == 2
    assert "pinned camera-video runtime environment is missing" in result.stderr
    assert "service start never builds dependencies" in result.stderr


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is required")
def test_video_launcher_requires_explicit_target_encoder(tmp_path: Path) -> None:
    launcher = _bash_path(ROOT / "scripts/deploy/thunder/run_video.sh")
    runtime_env = tmp_path / "env.sh"
    runtime_env.write_text("# test runtime\n", encoding="utf-8")
    runtime = _bash_path(runtime_env)
    command = (
        f"LINGTU_VIDEO_BIN=/bin/true LINGTU_VIDEO_ENCODER= "
        f"LINGTU_VIDEO_RUNTIME_ENV={shlex.quote(runtime)} bash {shlex.quote(launcher)}"
    )

    result = subprocess.run(
        ["bash", "-lc", command],
        check=False,
        cwd=ROOT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode != 0
    assert "Set an exact target encoder" in result.stderr


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is required")
@pytest.mark.parametrize("video", [False, True])
def test_yaml_settings_reach_native_launcher_arguments(tmp_path: Path, video: bool) -> None:
    settings = yaml.safe_load(_read("config/system/settings/camera.yaml"))["camera"]
    settings["capture"]["color_fps"] = 24
    settings["capture"]["timeout_ms"] = 1234
    settings["orbbec"]["frame_sync"] = True
    settings["orbbec"]["product_id"] = "0"
    settings["shm"]["color"] = "/fixture_color"
    settings["video"]["fps"] = 17
    settings["video"]["start_bitrate"] = 900000
    settings["video"]["idle_timeout_ms"] = 0
    environment = camera_environment(settings, CameraConfig(video_encoder="x264enc"), include_video=video)
    recorder = tmp_path / "record-args.sh"
    recorder.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n', encoding="utf-8", newline="\n")
    executable = _bash_path(recorder)
    runtime = tmp_path / "runtime-env.sh"
    runtime.write_text("# test runtime\n", encoding="utf-8")
    environment.update({
        "LINGTU_CAMERA_DDS_BIN": executable,
        "LINGTU_CAMERA_STATUS_FILE": _bash_path(tmp_path / "camera.json"),
        "LINGTU_CAMERA_COLOR_TOPIC": "rt/test/camera/color",
        "LINGTU_ORBBEC_CAPTURE_BIN": "/bin/true",
        "LINGTU_VIDEO_BIN": executable,
        "LINGTU_VIDEO_STATUS_FILE": _bash_path(tmp_path / "video.json"),
        "LINGTU_VIDEO_RUNTIME_ENV": _bash_path(runtime),
    })
    launcher = _bash_path(ROOT / "scripts/deploy/thunder" / ("run_video.sh" if video else "run_camera_dds.sh"))
    command = (
        f"chmod +x {shlex.quote(executable)} && env "
        + " ".join(f"{key}={shlex.quote(value)}" for key, value in environment.items())
        + f" bash {shlex.quote(launcher)}"
    )
    result = subprocess.run(
        ["bash", "-lc", command], cwd=ROOT, capture_output=True, text=True,
        encoding="utf-8", errors="replace", check=False,
    )

    assert result.returncode == 0, result.stderr
    args = result.stdout.splitlines()
    if video:
        assert args[args.index("--fps") + 1] == "17"
        assert args[args.index("--start-bitrate") + 1] == "900000"
        assert args[args.index("--idle-timeout-ms") + 1] == "0"
        assert args[args.index("--color-shm") + 1] == "/fixture_color"
        assert args[args.index("--stun-server") + 1] == ""
    else:
        assert args[args.index("--color-fps") + 1] == "24"
        assert args[args.index("--timeout-ms") + 1] == "1234"
        assert args[args.index("--color-topic") + 1] == "rt/test/camera/color"
        assert "--depth-topic" not in args
        assert "--enable-frame-sync" in args
        assert "--no-publish-image-dds" in args
        assert args[args.index("--product-id") + 1] == "0"
        assert args[args.index("--sdk-config") + 1] == ""


def test_video_readiness_requires_fresh_source_status_and_whep(
    monkeypatch,
    tmp_path: Path,
) -> None:
    status_path = tmp_path / "video.json"
    status_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.camera_video.status.v1",
                "status": "running",
                "ready": True,
                "source_fresh": True,
                "heartbeat_timestamp_s": 99.5,
                "source_generation": 7,
                "source_sequence": 42,
                "frame_age_ms": 12.0,
                "endpoint": "http://127.0.0.1:8443/whep/endpoint",
                "encoder": "nvv4l2h264enc",
                "last_error": "",
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_VIDEO_STATUS_FILE", str(status_path))
    monkeypatch.setenv("LINGTU_VIDEO_BIN", str(tmp_path / "lingtu_camera_webrtc"))
    monkeypatch.setenv("LINGTU_WEBRTC_URL", "http://127.0.0.1:19999/stale-controller-endpoint")
    (tmp_path / "lingtu_camera_webrtc").write_text("", encoding="utf-8")
    monkeypatch.setattr(Path, "exists", lambda self: True)
    monkeypatch.setattr("os.access", lambda _path, _mode: True)

    observed_methods: list[str] = []

    class _Response:
        status = 204

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return None

    def open_request(request, *, timeout):
        assert timeout == 1.0
        assert request.full_url == "http://127.0.0.1:8443/whep/endpoint"
        observed_methods.append(request.get_method())
        return _Response()

    monkeypatch.setattr("urllib.request.urlopen", open_request)
    spec = service_spec("video")
    assert spec is not None
    local_spec = replace(spec, files=(str(status_path),))
    monkeypatch.setattr("lingtu.real.systemd.service_spec", lambda _name: local_spec)

    result = _ServiceInspector(systemd=_Systemd(), now=lambda: status_path.stat().st_mtime).status_details(
        "video",
        dds_check=False,
        http_check=True,
    )["video"]

    assert result["ready"] is True
    assert result["blockers"] == []
    assert observed_methods == ["OPTIONS"]
    assert result["observed"]["status_file"]["files"][0]["source_fresh"] is True
    assert result["observed"]["whep"]["status"] == 204


def test_video_readiness_distinguishes_alive_from_stale_source(monkeypatch, tmp_path: Path) -> None:
    status_path = tmp_path / "video.json"
    status_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.camera_video.status.v1",
                "status": "waiting_for_source",
                "ready": False,
                "source_fresh": False,
                "heartbeat_timestamp_s": 100.0,
                "source_generation": 7,
                "source_sequence": 42,
                "frame_age_ms": 4000.0,
                "endpoint": "http://127.0.0.1:8443/whep/endpoint",
                "encoder": "nvv4l2h264enc",
                "last_error": "",
            }
        ),
        encoding="utf-8",
    )
    spec = service_spec("video")
    assert spec is not None
    local_spec = replace(spec, files=(str(status_path),))

    blockers: list[str] = []
    observed = _ServiceInspector(systemd=_Systemd(), now=lambda: status_path.stat().st_mtime)._status_files(
        local_spec,
        blockers,
    )

    assert observed["files"][0]["status"] == "waiting_for_source"
    assert observed["files"][0]["source_fresh"] is False
    assert f"video_source_not_fresh:{status_path}" in blockers
    assert f"status_file_not_ready:{status_path}" in blockers

