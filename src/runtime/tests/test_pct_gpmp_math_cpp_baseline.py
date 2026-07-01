from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "bench" / "pct_gpmp_math_cpp_baseline.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("pct_gpmp_math_cpp_baseline", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_cpp_baseline_source_uses_legacy_gpmp_headers_without_gtsam() -> None:
    module = _load_module()

    source = module.generate_runner_source()

    assert '#include "trajectory_optimization/gpmp_optimizer/models/wnoj.hpp"' in source
    assert '#include "trajectory_optimization/gpmp_optimizer/models/wnoa.hpp"' in source
    assert "WhiteNoiseOnJerkModel2D::LambdaAndPsi" in source
    assert "WhiteNoiseOnAcceleration2D::LambdaAndPsi" in source
    assert "GPHeadingRateFactor" not in source
    assert "gtsam" not in source.lower()
    assert "lingtu.pct_gpmp_math.result.v1" in source
    assert "legacy_cpp_gpmp_math" in source
    assert "wnoj_positive_heading_rate" in source
    assert "wnoj_negative_heading_rate" in source
    assert "wnoj_heading_rate_inside_limit" in source
    assert "wnoa_nominal" in source


def test_cpp_baseline_cmake_supports_eigen_target_and_include_dir_fallback() -> None:
    module = _load_module()

    cmake = module.generate_cmake()

    assert "find_package(Eigen3 REQUIRED)" in cmake
    assert "Eigen3::Eigen" in cmake
    assert "EIGEN3_INCLUDE_DIR" in cmake
    assert "PCT_LIB_SRC" in cmake
    assert "CMAKE_CXX_STANDARD 17" in cmake


def test_cpp_baseline_cli_exposes_optional_build_controls() -> None:
    module = _load_module()
    parser = module.build_parser()

    options = {action.dest for action in parser._actions}

    assert {
        "repo_root",
        "json_out",
        "build_dir",
        "cmake",
        "config",
        "eigen_include_dir",
        "keep_build",
        "print_source",
        "json",
    } <= options


def test_cpp_baseline_validates_emitted_schema() -> None:
    module = _load_module()

    payload = module._load_json_text(
        '{"schema":"lingtu.pct_gpmp_math.result.v1","cases":[]}'
    )

    assert payload["schema"] == "lingtu.pct_gpmp_math.result.v1"


def test_cpp_baseline_can_detect_repo_local_eigen_include(tmp_path: Path) -> None:
    module = _load_module()
    eigen_core = tmp_path / "third_party" / "eigen" / "Eigen" / "Core"
    eigen_runtime.parent.mkdir(parents=True)
    eigen_runtime.write_text("// fake eigen marker\n", encoding="utf-8")

    assert module._find_eigen_include_dir(tmp_path) == (
        tmp_path / "third_party" / "eigen"
    ).resolve()


def test_cpp_baseline_can_detect_conda_pkg_eigen_include(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = _load_module()
    conda_prefix = tmp_path / "miniconda"
    eigen_core = (
        conda_prefix
        / "pkgs"
        / "eigen-3.4.0-test"
        / "Library"
        / "include"
        / "eigen3"
        / "Eigen"
        / "Core"
    )
    eigen_runtime.parent.mkdir(parents=True)
    eigen_runtime.write_text("// fake eigen marker\n", encoding="utf-8")
    monkeypatch.setenv("CONDA_PREFIX", str(conda_prefix))
    monkeypatch.setattr(module.sys, "prefix", str(tmp_path / "unused_prefix"))
    monkeypatch.setattr(module.sys, "base_prefix", str(tmp_path / "unused_base"))

    assert module._find_eigen_include_dir(tmp_path) == eigen_runtime.parents[1].resolve()
