# ruff: noqa: S101

from __future__ import annotations

import inspect
import json
import os
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from sim.scripts.mujoco import teleop_native_acceptance as acceptance


def _valid_case() -> dict:
    status = {
        "snapshot_written_at_s": 10.0,
        "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
    }
    return {
        "startup_ok": True,
        "physical_arm": {"ok": True, "reason": "physical_arm_acknowledged"},
        "forbidden_goal": {"returncode": 1, "stderr": "goal rejected: control_mode_teleop"},
        "operator_motion": {
            "returncode": 0,
            "events": [{"action": action, "accepted": True} for action in ("claim", "sample", "hold", "release")],
        },
        "stop": {"returncode": 0, "stdout": "accepted stop: test"},
        "post_stop_samples": [dict(status, snapshot_written_at_s=10.0 + index) for index in range(4)],
        "sensor_report": {
            "ok": True,
            "command_source": "dds",
            "policy_loaded": True,
            "external_arm": {"acknowledged": True, "scenario": "teleop"},
            "motion": {"sim_path_length_xy_m": 0.3},
            "cmd_vel": {
                "driver_ready": False,
                "driver_ready_observed": True,
                "accepted_sequence": 0,
                "accepted_producer_boot_id": "",
                "accepted_output_sequence": 0,
                "nonzero_samples": 8,
                "observed_output_ack": {
                    "accepted_sequence": 7,
                    "producer_boot_id": "host-boot",
                    "output_sequence": 42,
                },
                "stopped_evidence": {
                    "bridge_boot_id": "a" * 32,
                    "controller_boot_id": "b" * 32,
                    "bridge_command_seq": 9,
                    "applied_step_seq": 10,
                    "kind": "deactivate_zero",
                },
                "process_cleanup": {"clean": True},
            },
        },
        "process_cleanup": [
            {"name": "navigation", "clean": True},
            {"name": "sensor", "clean": True},
        ],
    }


def test_manifest_is_pure_teleop_and_component_scoped() -> None:
    manifest = json.loads(acceptance.DEFAULT_MANIFEST.read_text(encoding="utf-8"))

    assert acceptance._contract_evidence(manifest)["ok"] is True
    assert manifest["acceptance_scope"]["coverage"] == "component"
    assert "exact ProductControl RunPlan process realization" in manifest["acceptance_scope"]["excluded_claims"]
    assert set(manifest["binaries"]) == set(acceptance.REQUIRED_BINARIES)
    assert "slam" not in manifest
    assert "traversability" not in manifest


@pytest.mark.parametrize("value", ["-1", "233", "not-an-integer"])
def test_cli_rejects_unusable_dds_domain(value: str) -> None:
    with pytest.raises(SystemExit):
        acceptance.build_parser().parse_args(["--domain-id", value])


@pytest.mark.parametrize("value", ["nan", "inf", "-inf", "0", "-0.1"])
def test_parser_rejects_non_finite_or_non_positive_motion_threshold(value: str) -> None:
    with pytest.raises(SystemExit):
        acceptance.build_parser().parse_args(["--min-motion-m", value])


def test_operator_motion_events_are_parsed_before_bounded_output_truncation(
    monkeypatch,
) -> None:
    event = (
        "LT_OPERATOR_MOTION_EVENT_V1 action=claim accepted=true source_id=test "
        "source_epoch=1 source_sequence=1 sample_count=0\n"
    )
    tail = "x" * 9000
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *_args, **_kwargs: SimpleNamespace(
            returncode=0,
            stdout=event + tail,
            stderr="",
        ),
    )

    result = acceptance._run_native_command(
        ["trusted-native-command"],
        {},
        timeout_s=1.0,
    )

    assert [item["action"] for item in result["events"]] == ["claim"]
    assert result["output_truncated"] is True
    assert len(result["stdout"]) < 8100


def test_windows_native_library_path_uses_only_pinned_binary_directories(
    tmp_path: Path,
    monkeypatch,
) -> None:
    nav_dir = tmp_path / "nav"
    bridge_dir = tmp_path / "bridge"
    nav_dir.mkdir()
    bridge_dir.mkdir()
    (nav_dir / "ddsc.dll").write_bytes(b"nav-dds")
    (bridge_dir / "ddsc.dll").write_bytes(b"dds")
    binaries = {
        "navigation": nav_dir / "navd.exe",
        "driver_bridge": bridge_dir / "bridge.exe",
    }
    monkeypatch.setattr(acceptance.os, "name", "nt")

    evidence = acceptance._native_library_evidence(binaries)
    environment = acceptance._process_environment({"IDENTITY": "x"}, binaries["navigation"])

    assert evidence["ok"] is True
    assert evidence["runtime_dlls"] == {
        "navigation": str(nav_dir / "ddsc.dll"),
        "driver_bridge": str(bridge_dir / "ddsc.dll"),
    }
    assert environment["PATH"].split(acceptance.os.pathsep)[0] == str(nav_dir)


def test_windows_native_library_evidence_rejects_cross_process_dll_fallback(
    tmp_path: Path,
    monkeypatch,
) -> None:
    nav_dir = tmp_path / "nav"
    bridge_dir = tmp_path / "bridge"
    nav_dir.mkdir()
    bridge_dir.mkdir()
    (bridge_dir / "ddsc.dll").write_bytes(b"bridge-dds")
    monkeypatch.setattr(acceptance.os, "name", "nt")

    evidence = acceptance._native_library_evidence(
        {
            "navigation": nav_dir / "navd.exe",
            "driver_bridge": bridge_dir / "bridge.exe",
        }
    )

    assert evidence["ok"] is False
    assert evidence["blockers"] == ["native_runtime_library_missing:navigation:ddsc.dll"]


def test_nav_runtime_bundles_its_linked_cyclonedds_dll() -> None:
    cmake = (acceptance.ROOT / "src/nav/cpp/endpoint/CMakeLists.txt").read_text(encoding="utf-8")

    assert "function(lingtu_endpoint_bundle_cyclonedds target)" in cmake
    assert "$<TARGET_FILE:CycloneDDS::ddsc>" in cmake
    assert "lingtu_endpoint_bundle_cyclonedds(${_target})" in cmake


def test_mujoco_dds_targets_use_an_app_local_runtime_closure() -> None:
    cmake = (acceptance.ROOT / "sim/adapters/dds/CMakeLists.txt").read_text(encoding="utf-8")

    assert "function(lingtu_mujoco_bundle_cyclonedds target)" in cmake
    assert 'set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")' in cmake
    assert "_LINGTU_MUJOCO_TARGETS_BEFORE_DDS_RUNTIME" in cmake
    assert "_LINGTU_MUJOCO_TARGETS_AFTER_DDS_RUNTIME" in cmake
    assert "lingtu_mujoco_bundle_cyclonedds(${_LINGTU_MUJOCO_DDS_TARGET})" in cmake
    assert "_LINGTU_CYCLONEDDS_TEST_PATH" not in cmake

    build_script = (acceptance.ROOT / "scripts/build/build_mujoco_native_dds_windows.ps1").read_text(encoding="utf-8")
    assert "Get-Command ctest -ErrorAction Stop" in build_script
    assert 'Join-Path $env:SystemRoot "System32"' in build_script
    assert "function Assert-WindowsVcRuntime" in build_script
    assert "Microsoft Visual C++ Redistributable x64 is incomplete" in build_script
    assert "VC\\Runtimes\\x64" in build_script
    assert "[Environment+SpecialFolder]::System" in build_script
    assert "[Environment]::Is64BitProcess" in build_script
    assert "function Get-ConfiguredMsvcToolsetVersion" in build_script
    assert "$Reader.ReadUInt16() -ne 0x8664" in build_script
    assert "$Reader.ReadUInt16() -ne 0x020B" in build_script
    for runtime_dll in (
        "msvcp140.dll",
        "msvcp140_2.dll",
        "vcruntime140.dll",
        "vcruntime140_1.dll",
        "vcomp140.dll",
    ):
        assert runtime_dll in build_script
    assert "\"$(Join-Path $CycloneDDSPrefix 'bin');$OriginalPath\"" not in build_script
    assert "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=" not in build_script
    assert "$OutputDirectory = if ($UsesMultiConfigGenerator)" in build_script


@pytest.mark.skipif(os.name != "nt", reason="Windows PowerShell contract")
@pytest.mark.parametrize(
    ("linker_path", "accepted"),
    (
        ("C:/VS/VC/Tools/MSVC/14.44.35207/bin/Hostx64/x64/link.exe", True),
        ("C:/VS/VC/Tools/MSVC/14.44.35207/bin/Hostx86/x86/link.exe", False),
    ),
)
def test_windows_adapter_build_reuses_cached_single_config_generator(
    tmp_path: Path,
    linker_path: str,
    accepted: bool,
) -> None:
    pwsh = shutil.which("pwsh")
    if pwsh is None:
        pytest.skip("pwsh is required for the Windows build-script contract")

    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    command_log = tmp_path / "cmake-commands.txt"
    (fake_bin / "cmake.cmd").write_text(
        '@echo off\r\necho %*>>"%LINGTU_FAKE_CMAKE_LOG%"\r\nexit /b 0\r\n',
        encoding="ascii",
    )
    cyclone_prefix = tmp_path / "cyclonedds"
    for relative in (
        "lib/cmake/CycloneDDS/CycloneDDSConfig.cmake",
        "bin/idlc.exe",
        "bin/cycloneddsidlc.dll",
        "bin/ddsc.dll",
    ):
        path = cyclone_prefix / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"fixture")
    build_directory = tmp_path / "build"
    build_directory.mkdir()
    (build_directory / "CMakeCache.txt").write_text(
        f"CMAKE_BUILD_TYPE:STRING=Release\nCMAKE_GENERATOR:INTERNAL=Ninja\nCMAKE_LINKER:FILEPATH={linker_path}\n",
        encoding="utf-8",
    )
    env = os.environ.copy()
    env["PATH"] = f"{fake_bin}{os.pathsep}{env['PATH']}"
    env["LINGTU_FAKE_CMAKE_LOG"] = str(command_log)
    env.pop("CMAKE_GENERATOR", None)

    result = subprocess.run(  # noqa: S603 - pwsh is resolved to an absolute executable.
        [
            pwsh,
            "-NoProfile",
            "-File",
            str(acceptance.ROOT / "scripts/build/build_mujoco_native_dds_windows.ps1"),
            "-CycloneDDSPrefix",
            str(cyclone_prefix),
            "-BuildDirectory",
            str(build_directory),
            "-Configuration",
            "Release",
            "-SkipTests",
        ],
        cwd=acceptance.ROOT,
        env=env,
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )

    if not accepted:
        assert result.returncode != 0
        assert "Configured linker is not from an MSVC x64 toolset" in result.stderr
        return

    assert result.returncode == 0, result.stderr
    configure_command = command_log.read_text(encoding="utf-8").splitlines()[0]
    assert "-A x64" not in configure_command
    assert "-G Ninja" in configure_command
    assert "-DCMAKE_BUILD_TYPE=Release" in configure_command
    assert f"Built Windows native MuJoCo DDS adapter in {build_directory}" in result.stdout


def test_attach_only_case_uses_existing_product_processes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[str, ...]] = []
    native_calls: list[tuple[tuple[str, ...], float]] = []
    post_stop_timeouts: list[float] = []
    post_stop_boundaries: list[float] = []

    def control(_binary, command, **_kwargs):
        calls.append(tuple(command))
        if command[0] == "goal":
            return {"returncode": 1, "stderr": "goal rejected: control_mode_teleop"}
        return {"returncode": 0, "stdout": "accepted stop: attach-only"}

    monkeypatch.setattr(acceptance.shared, "_run_control", control)

    def run_native(command, _environment, *, timeout_s):
        native_calls.append((tuple(command), float(timeout_s)))
        return {
            "returncode": 0,
            "events": [{"action": action, "accepted": True} for action in ("claim", "sample", "hold", "release")],
        }

    monkeypatch.setattr(acceptance, "_run_native_command", run_native)

    def post_stop_samples(_path, *, after_stamp_s, timeout_s):
        post_stop_boundaries.append(float(after_stamp_s))
        post_stop_timeouts.append(float(timeout_s))
        return [
            {
                "stamp_s": 101.0 + index,
                "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            }
            for index in range(acceptance.POST_STOP_SAMPLES)
        ]

    monkeypatch.setattr(acceptance, "_post_stop_samples", post_stop_samples)
    monkeypatch.setattr(
        acceptance,
        "_read_json",
        lambda _path: {
            "stamp_s": 90.0,
            "final_cmd_vel": {"vx": 0.1, "vy": 0.0, "wz": 0.0},
        },
    )
    monkeypatch.setattr(acceptance.time, "time", lambda: 100.0)
    args = SimpleNamespace(
        domain_id=231,
        command_vx=0.18,
        duration_s=16.0,
        min_motion_m=0.15,
    )
    plan = SimpleNamespace(
        product="teleop",
        native_process_environment={},
        processes=(
            SimpleNamespace(
                name="nav_runtime",
                command=SimpleNamespace(argv=("navd", "--status-s", "1")),
            ),
        ),
    )

    result = acceptance.run_attached(
        plan=plan,
        run_plan_path=tmp_path / "plan.json",
        product_session_id="a" * 32,
        prepared={
            "binaries": {"navigation_control": tmp_path / "navctl.exe"},
        },
        args=args,
    )

    assert result["ok"] is True
    assert [command[0] for command in calls] == ["goal", "stop"]
    assert native_calls[0][0][native_calls[0][0].index("--duration-s") + 1] == "16.0"
    assert native_calls[0][1] >= 22.0
    assert post_stop_boundaries == [100.0]
    assert post_stop_timeouts == [5.5]
    assert result["pre_stop_status_stamp_s"] == 90.0
    assert result["stop_ack_wall_s"] == 100.0
    source = inspect.getsource(acceptance.run_attached)
    assert "ManagedProcess" not in source
    assert "Popen" not in source


def test_post_stop_samples_require_fresh_strictly_increasing_stamps(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    statuses = iter(
        [
            {"stamp_s": 99.0, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}},
            {"final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}},
            {"stamp_s": 101.0, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}},
            {"stamp_s": 101.0, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}},
            {"stamp_s": 100.5, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}},
            {"stamp_s": 102.0, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}},
            {"stamp_s": 103.0, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}},
            {"stamp_s": 104.0, "final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}},
        ]
    )
    monotonic = iter([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
    monkeypatch.setattr(acceptance, "_read_json", lambda _path: next(statuses))
    monkeypatch.setattr(acceptance.time, "monotonic", lambda: next(monotonic))
    monkeypatch.setattr(acceptance.time, "sleep", lambda _seconds: None)

    samples = acceptance._post_stop_samples(
        tmp_path / "nav.status.json",
        after_stamp_s=100.0,
        timeout_s=1.0,
    )

    assert [sample["stamp_s"] for sample in samples] == [101.0, 102.0, 103.0, 104.0]


def test_attach_only_evaluation_does_not_count_stale_or_unstamped_zero_samples() -> None:
    zero = {"final_cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}}
    case = {
        "forbidden_goal": {
            "returncode": 1,
            "stderr": "goal rejected: control_mode_teleop",
        },
        "operator_motion": {
            "returncode": 0,
            "events": [{"action": action, "accepted": True} for action in ("claim", "sample", "hold", "release")],
        },
        "stop": {"returncode": 0, "stdout": "accepted stop: attach-only"},
        "pre_stop_status_stamp_s": 90.0,
        "stop_ack_wall_s": 100.0,
        "post_stop_not_before_stamp_s": 100.0,
        "post_stop_samples": [
            dict(zero),
            dict(zero, stamp_s=99.0),
            dict(zero, stamp_s=100.0),
            dict(zero, stamp_s=101.0),
        ],
    }

    blockers = acceptance._evaluate_attached_case(case)

    assert "post_stop_status_samples_missing" in blockers

    case["post_stop_samples"] = [dict(zero, stamp_s=101.0 + index) for index in range(acceptance.POST_STOP_SAMPLES)]
    assert acceptance._evaluate_attached_case(case) == []

    case["stop_ack_wall_s"] = 200.0
    blockers = acceptance._evaluate_attached_case(case)
    assert "post_stop_status_samples_missing" in blockers


@pytest.mark.parametrize(
    "argv",
    (
        ("navd",),
        ("navd", "--status-s", "1", "--status-s", "2"),
        ("navd", "--status-s", "nan"),
        ("navd", "--status-s", "60"),
    ),
)
def test_attach_only_rejects_invalid_exact_nav_status_period(
    argv: tuple[str, ...],
) -> None:
    plan = SimpleNamespace(
        processes=(
            SimpleNamespace(
                name="nav_runtime",
                command=SimpleNamespace(argv=argv),
            ),
        ),
    )

    with pytest.raises(ValueError, match="status"):
        acceptance._attached_post_stop_timeout_s(plan)
