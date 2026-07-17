from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from maps.services.opt import MapOpt


def _lt_pgo_binary() -> Path:
    root = Path(__file__).resolve().parents[3]
    executable = "lt_pgo.exe" if os.name == "nt" else "lt_pgo"
    configured = os.environ.get("LINGTU_PGO_BIN")
    candidates = [
        Path(configured) if configured else None,
        root / "build" / "native_runtime_components" / "Release" / executable,
        root / "build" / "native_runtime_components" / executable,
    ]
    found = shutil.which("lt_pgo")
    if found:
        candidates.append(Path(found))
    for candidate in candidates:
        if candidate is not None and candidate.is_file():
            return candidate
    pytest.skip("lt_pgo binary is not built; native consistency test is unavailable")


def _lt_loop_verify_binary() -> Path:
    root = Path(__file__).resolve().parents[3]
    executable = "lt_loop_verify.exe" if os.name == "nt" else "lt_loop_verify"
    configured = os.environ.get("LINGTU_LOOP_VERIFY_BIN")
    candidates = [
        Path(configured) if configured else None,
        root / "build" / "native-runtime-debug" / "Debug" / executable,
        root / "build" / "native_runtime_components" / "Release" / executable,
        root / "build" / "native_runtime_components" / "Debug" / executable,
        root / "build" / "native_runtime_components" / executable,
    ]
    found = shutil.which("lt_loop_verify")
    if found:
        candidates.append(Path(found))
    for candidate in candidates:
        if candidate is not None and candidate.is_file():
            return candidate
    pytest.skip("lt_loop_verify binary is not built; shadow-loop CLI test is unavailable")


def _write_ascii_pcd(path: Path, point: tuple[float, float, float]) -> None:
    path.write_text(
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
        "COUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n"
        f"{point[0]} {point[1]} {point[2]} 1\n",
        encoding="utf-8",
    )


def test_map_opt_resolves_explicit_native_source_options_without_execution(tmp_path: Path) -> None:
    optimizer = tmp_path / "lt_pgo"
    optimizer.write_text("this file must not be executed\n", encoding="utf-8")

    options = MapOpt(strategy="pgo", command=[str(optimizer)], required=True).source_options()

    assert options == {
        "strategy": "pgo",
        "required": True,
        "command": [str(optimizer)],
        "timeout_sec": 120.0,
    }


def test_map_opt_defaults_to_safe_noop_without_verified_geometric_constraints(
    monkeypatch,
) -> None:
    monkeypatch.delenv("LINGTU_MAP_OPT", raising=False)

    options = MapOpt().source_options()

    assert options["strategy"] == "off"
    assert options["command"] is None


def test_map_opt_disables_command_resolution_for_off_strategy(tmp_path: Path) -> None:
    optimizer = tmp_path / "lt_pgo"
    optimizer.write_text("unused\n", encoding="utf-8")

    options = MapOpt(command=[str(optimizer)]).source_options("off")

    assert options["strategy"] == "off"
    assert options["command"] is None


def test_native_pgo_preserves_map_when_graph_has_only_odometry_constraints(
    tmp_path: Path,
) -> None:
    optimizer = _lt_pgo_binary()
    map_dir = tmp_path / "map"
    patches = map_dir / "patches"
    patches.mkdir(parents=True)
    _write_ascii_pcd(map_dir / "map.pcd", (99.0, 99.0, 99.0))
    _write_ascii_pcd(patches / "000001.pcd", (1.0, 2.0, 3.0))
    (map_dir / "poses.txt").write_text(
        "000001.pcd 0 0 0 1 0 0 0\n",
        encoding="utf-8",
    )
    map_before = (map_dir / "map.pcd").read_bytes()
    poses_before = (map_dir / "poses.txt").read_bytes()

    completed = subprocess.run(
        [str(optimizer), "--map", str(map_dir), "--out", str(map_dir)],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert completed.returncode == 0, completed.stderr
    assert '"code":"skipped_no_independent_constraints"' in completed.stdout
    assert '"performed":false' in completed.stdout
    assert '"changed":false' in completed.stdout
    assert (map_dir / "map.pcd").read_bytes() == map_before
    assert (map_dir / "poses.txt").read_bytes() == poses_before
    assert not list(map_dir.glob("*.preopt-*"))


def test_native_loop_shadow_cli_is_read_only_and_emits_valid_json(tmp_path: Path) -> None:
    verifier = _lt_loop_verify_binary()
    map_dir = tmp_path / "map"
    patches = map_dir / "patches"
    patches.mkdir(parents=True)
    _write_ascii_pcd(map_dir / "map.pcd", (9.0, 9.0, 9.0))
    _write_ascii_pcd(patches / "000001.pcd", (1.0, 2.0, 3.0))
    (map_dir / "poses.txt").write_text(
        "000001.pcd 0 0 0 1 0 0 0\n",
        encoding="utf-8",
    )
    map_before = (map_dir / "map.pcd").read_bytes()
    poses_before = (map_dir / "poses.txt").read_bytes()
    report_path = tmp_path / "loop_constraints.json"

    completed = subprocess.run(
        [
            str(verifier),
            "--map",
            str(map_dir),
            "--report",
            str(report_path),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert completed.returncode == 0, completed.stderr
    summary = json.loads(completed.stdout)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert summary["mode"] == "shadow"
    assert summary["code"] == "shadow_no_verified_loops"
    assert report["schema_version"] == "lingtu.loop_constraints.shadow.v3"
    assert report["information_convention"] == (
        "shadow_only;information_diagonal=zero;not_graph_compatible"
    )
    assert report["frame_convention"].endswith("constraint=T_from_to")
    assert report["pose_count"] == 1
    assert report["patch_count"] == 1
    assert report["constraints"] == []
    assert report["poses_fingerprint"].startswith("fnv1a64:")
    assert report["patches_fingerprint"].startswith("fnv1a64:")
    assert (map_dir / "map.pcd").read_bytes() == map_before
    assert (map_dir / "poses.txt").read_bytes() == poses_before
    assert not list(map_dir.glob("*.preopt-*"))


@pytest.mark.parametrize(
    ("extra_args", "report_inside_map"),
    [
        (["--xy-radius", "nan"], False),
        (["--xy-radius", "1e300"], False),
        (["--max-candidates", "-1"], False),
        (["--max-candidates", "4097"], False),
        ([], True),
    ],
)
def test_native_loop_shadow_cli_rejects_unsafe_inputs(
    tmp_path: Path,
    extra_args: list[str],
    report_inside_map: bool,
) -> None:
    verifier = _lt_loop_verify_binary()
    map_dir = tmp_path / "map"
    patches = map_dir / "patches"
    patches.mkdir(parents=True)
    _write_ascii_pcd(map_dir / "map.pcd", (9.0, 9.0, 9.0))
    _write_ascii_pcd(patches / "000001.pcd", (1.0, 2.0, 3.0))
    (map_dir / "poses.txt").write_text(
        "000001.pcd 0 0 0 1 0 0 0\n",
        encoding="utf-8",
    )
    map_before = (map_dir / "map.pcd").read_bytes()
    poses_before = (map_dir / "poses.txt").read_bytes()
    report_path = (
        map_dir / "unsafe_report.json"
        if report_inside_map
        else tmp_path / "rejected_report.json"
    )

    completed = subprocess.run(
        [
            str(verifier),
            "--map",
            str(map_dir),
            "--report",
            str(report_path),
            *extra_args,
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert completed.returncode == 2
    assert not report_path.exists()
    assert (map_dir / "map.pcd").read_bytes() == map_before
    assert (map_dir / "poses.txt").read_bytes() == poses_before
