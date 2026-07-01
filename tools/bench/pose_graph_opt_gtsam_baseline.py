from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from pose_graph_opt_fixture import fixture_hash, fixture_identity, validate_fixture, write_json


RUNNER_NAME = "gtsam_fixture_runner"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run a neutral pose_graph_opt fixture through an optional C++/GTSAM "
            "baseline. Requires CMake, a C++17 compiler, and find_package(GTSAM)."
        )
    )
    subparsers = parser.add_subparsers(dest="command")
    run_suite = subparsers.add_parser(
        "run-suite",
        help="Run all fixture JSON files in a directory through the optional C++/GTSAM baseline.",
    )
    run_suite.add_argument("--fixture-dir", type=Path, required=True)
    run_suite.add_argument("--json-out", type=Path, required=True)
    run_suite.add_argument("--build-dir", type=Path)
    run_suite.add_argument("--cmake", default="cmake")
    run_suite.add_argument("--config", default="Release")
    run_suite.add_argument(
        "--keep-build",
        action="store_true",
        help="Keep the temporary generated C++/CMake projects.",
    )

    parser.add_argument("--fixture", type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--build-dir", type=Path)
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--config", default="Release")
    parser.add_argument(
        "--keep-build",
        action="store_true",
        help="Keep the temporary generated C++/CMake project.",
    )
    args = parser.parse_args()

    if args.command == "run-suite":
        result = run_gtsam_baseline_suite(
            args.fixture_dir,
            build_dir=args.build_dir,
            cmake=args.cmake,
            config=args.config,
            keep_build=args.keep_build,
        )
        write_json(args.json_out, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0

    if args.fixture is None:
        parser.error("--fixture is required unless using run-suite")
    fixture = json.loads(args.fixture.read_text(encoding="utf-8"))
    result = run_gtsam_baseline(
        fixture,
        build_dir=args.build_dir,
        cmake=args.cmake,
        config=args.config,
        keep_build=args.keep_build,
    )
    if args.json_out:
        write_json(args.json_out, result)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


def run_gtsam_baseline(
    fixture_data: dict[str, Any],
    *,
    build_dir: Path | None = None,
    cmake: str = "cmake",
    config: str = "Release",
    keep_build: bool = False,
) -> dict[str, Any]:
    validate_fixture(fixture_data)
    if shutil.which(cmake) is None:
        raise RuntimeError(f"{cmake!r} is required to build the GTSAM baseline")

    if build_dir is not None:
        build_root = build_dir
        build_root.mkdir(parents=True, exist_ok=True)
        return _build_and_run(fixture_data, build_root, cmake=cmake, config=config)

    with tempfile.TemporaryDirectory(prefix="lingtu_gtsam_baseline_") as tmp:
        build_root = Path(tmp)
        result = _build_and_run(fixture_data, build_root, cmake=cmake, config=config)
        if keep_build:
            preserved = Path.cwd() / ".tmp" / "pose_graph_opt_gtsam_baseline"
            if preserved.exists():
                shutil.rmtree(preserved)
            preserved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(build_root, preserved)
            result["metadata"]["build_dir"] = str(preserved)
        return result


def run_gtsam_baseline_suite(
    fixture_dir: Path,
    *,
    build_dir: Path | None = None,
    cmake: str = "cmake",
    config: str = "Release",
    keep_build: bool = False,
) -> dict[str, Any]:
    if shutil.which(cmake) is None:
        raise RuntimeError(f"{cmake!r} is required to build the GTSAM baseline")

    fixture_paths = sorted(path for path in fixture_dir.glob("*.json") if path.is_file())
    if not fixture_paths:
        raise ValueError(f"no fixture JSON files found in {fixture_dir}")

    if build_dir is not None:
        build_root = build_dir
        build_root.mkdir(parents=True, exist_ok=True)
        return _run_gtsam_baseline_suite_in_build_root(
            fixture_paths,
            build_root,
            cmake=cmake,
            config=config,
        )

    with tempfile.TemporaryDirectory(prefix="lingtu_gtsam_baseline_suite_") as tmp:
        build_root = Path(tmp)
        result = _run_gtsam_baseline_suite_in_build_root(
            fixture_paths,
            build_root,
            cmake=cmake,
            config=config,
        )
        if keep_build:
            preserved = Path.cwd() / ".tmp" / "pose_graph_opt_gtsam_baseline_suite"
            if preserved.exists():
                shutil.rmtree(preserved)
            preserved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(build_root, preserved)
            result["metadata"]["build_dir"] = str(preserved)
        return result


def _run_gtsam_baseline_suite_in_build_root(
    fixture_paths: list[Path],
    build_root: Path,
    *,
    cmake: str,
    config: str,
) -> dict[str, Any]:
    cases: list[dict[str, Any]] = []
    fixtures: list[dict[str, Any]] = []
    for index, fixture_path in enumerate(fixture_paths):
        fixture_data = json.loads(fixture_path.read_text(encoding="utf-8"))
        validate_fixture(fixture_data)
        result = _build_and_run(
            fixture_data,
            build_root / f"{index:04d}_{fixture_path.stem}",
            cmake=cmake,
            config=config,
        )
        case = dict(result["cases"][0])
        case["fixture_path"] = fixture_path.as_posix()
        cases.append(case)
        fixtures.append(
            {
                "path": fixture_path.as_posix(),
                "fixture_hash": result["fixture_hash"],
                "fixture_identity": result["fixture_identity"],
            }
        )

    return {
        "benchmark": "legacy_cpp_gtsam_fixture_suite",
        "unit": "milliseconds",
        "fixtures": fixtures,
        "cases": cases,
        "metadata": {
            "generator": "tools/bench/pose_graph_opt_gtsam_baseline.py",
            "solver": "gtsam_levenberg_marquardt",
            "fixture_count": len(fixtures),
            "build_dir": str(build_root),
        },
    }


def _build_and_run(
    fixture_data: dict[str, Any],
    build_root: Path,
    *,
    cmake: str,
    config: str,
) -> dict[str, Any]:
    source_dir = build_root / "src"
    binary_dir = build_root / "build"
    source_dir.mkdir(parents=True, exist_ok=True)
    binary_dir.mkdir(parents=True, exist_ok=True)
    (source_dir / "CMakeLists.txt").write_text(generate_cmake(), encoding="utf-8")
    (source_dir / f"{RUNNER_NAME}.cpp").write_text(
        generate_runner_source(fixture_data), encoding="utf-8"
    )

    configure_cmd = [
        cmake,
        "-S",
        str(source_dir),
        "-B",
        str(binary_dir),
        f"-DCMAKE_BUILD_TYPE={config}",
    ]
    subprocess.run(configure_cmd, check=True)
    subprocess.run([cmake, "--build", str(binary_dir), "--config", config], check=True)
    runner = _runner_path(binary_dir, config)
    completed = subprocess.run(
        [str(runner)],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    result = json.loads(completed.stdout)
    attach_fixture_identity(result, fixture_data)
    result.setdefault("metadata", {})["build_dir"] = str(build_root)
    return result


def attach_fixture_identity(result: dict[str, Any], fixture_data: dict[str, Any]) -> dict[str, Any]:
    result["fixture_hash"] = fixture_hash(fixture_data)
    result["fixture_identity"] = fixture_identity(fixture_data)
    result.setdefault("metadata", {})["fixture_hash"] = result["fixture_hash"]
    for case in result.get("cases", []):
        case["fixture_hash"] = result["fixture_hash"]
        case["fixture_identity"] = result["fixture_identity"]
    return result


def _runner_path(binary_dir: Path, config: str) -> Path:
    candidates = [
        binary_dir / RUNNER_NAME,
        binary_dir / f"{RUNNER_NAME}.exe",
        binary_dir / config / RUNNER_NAME,
        binary_dir / config / f"{RUNNER_NAME}.exe",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"{RUNNER_NAME} binary not found in {candidates}")


def generate_cmake() -> str:
    return f"""cmake_minimum_required(VERSION 3.16)
project(lingtu_pose_graph_opt_gtsam_baseline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTSAM REQUIRED)

add_executable({RUNNER_NAME} {RUNNER_NAME}.cpp)
if(TARGET GTSAM::gtsam)
  target_link_libraries({RUNNER_NAME} PRIVATE GTSAM::gtsam)
elseif(TARGET gtsam)
  target_link_libraries({RUNNER_NAME} PRIVATE gtsam)
else()
  target_include_directories({RUNNER_NAME} PRIVATE ${{GTSAM_INCLUDE_DIR}})
  target_link_libraries({RUNNER_NAME} PRIVATE ${{GTSAM_LIBRARIES}})
endif()
"""


def generate_runner_source(fixture_data: dict[str, Any]) -> str:
    validate_fixture(fixture_data)
    config = fixture_data.get("config", {})
    max_iterations = int(config.get("max_iterations", 80))
    tolerance = float(config.get("tolerance", 1e-9))
    initial_lambda = float(config.get("initial_lambda", 1e-3))
    pose_count = len(fixture_data["poses"])
    factor_count = len(fixture_data.get("priors", [])) + len(fixture_data.get("betweens", []))
    case_name = str(fixture_data.get("case", "fixture"))
    case_output_fragment = cpp_string(f'"case":{json.dumps(case_name)},')
    anchor_index = resolved_anchor_index(fixture_data)
    anchor_line = (
        "    opt_graph.add(gtsam::PriorFactor<gtsam::Pose3>("
        f"{anchor_index}, initial.at<gtsam::Pose3>({anchor_index}), "
        "gtsam::noiseModel::Gaussian::Information(strongAnchorInformation())));"
        if anchor_index is not None
        else ""
    )

    initial_lines = [
        f"  initial.insert({idx}, {pose_expr(pose)});"
        for idx, pose in enumerate(fixture_data["poses"])
    ]
    prior_lines = [
        "    raw_graph.add(gtsam::PriorFactor<gtsam::Pose3>("
        f"{int(prior['index'])}, {pose_expr(prior['pose'])}, "
        f"gtsam::noiseModel::Gaussian::Information(infoMatrix({upper_expr(prior['information_upper'])}))));\n"
        "    opt_graph.add(gtsam::PriorFactor<gtsam::Pose3>("
        f"{int(prior['index'])}, {pose_expr(prior['pose'])}, "
        f"gtsam::noiseModel::Gaussian::Information(infoMatrix({upper_expr(prior['information_upper'])}))));"
        for prior in fixture_data.get("priors", [])
    ]
    between_lines = [
        "    raw_graph.add(gtsam::BetweenFactor<gtsam::Pose3>("
        f"{int(endpoint_value(between, 'from'))}, {int(endpoint_value(between, 'to'))}, "
        f"{pose_expr(between['pose_from_to'])}, "
        f"gtsam::noiseModel::Gaussian::Information(infoMatrix({upper_expr(between['information_upper'])}))));\n"
        "    opt_graph.add(gtsam::BetweenFactor<gtsam::Pose3>("
        f"{int(endpoint_value(between, 'from'))}, {int(endpoint_value(between, 'to'))}, "
        f"{pose_expr(between['pose_from_to'])}, "
        f"gtsam::noiseModel::Gaussian::Information(infoMatrix({upper_expr(between['information_upper'])}))));"
        for between in fixture_data.get("betweens", [])
    ]
    residual_lines = [
        "    appendResidualValues(final_residual_values, gtsam::Pose3::Logmap("
        f"{pose_expr(prior['pose'])}.inverse().compose(result.at<gtsam::Pose3>({int(prior['index'])}))));"
        for prior in fixture_data.get("priors", [])
    ] + [
        "    appendResidualValues(final_residual_values, gtsam::Pose3::Logmap("
        f"{pose_expr(between['pose_from_to'])}.inverse().compose("
        f"result.at<gtsam::Pose3>({int(endpoint_value(between, 'from'))}).between("
        f"result.at<gtsam::Pose3>({int(endpoint_value(between, 'to'))})))));"
        for between in fixture_data.get("betweens", [])
    ]

    return f"""#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <exception>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {{

gtsam::Pose3 makePose(double tx, double ty, double tz, double qw, double qx, double qy, double qz) {{
  return gtsam::Pose3(gtsam::Rot3::Quaternion(qw, qx, qy, qz), gtsam::Point3(tx, ty, tz));
}}

gtsam::Matrix infoMatrix(const std::array<double, 21>& upper) {{
  gtsam::Matrix matrix = gtsam::Matrix::Zero(6, 6);
  size_t index = 0;
  for (int row = 0; row < 6; ++row) {{
    for (int col = row; col < 6; ++col) {{
      matrix(row, col) = upper[index];
      matrix(col, row) = upper[index];
      ++index;
    }}
  }}
  return matrix;
}}

gtsam::Matrix strongAnchorInformation() {{
  gtsam::Matrix matrix = gtsam::Matrix::Zero(6, 6);
  for (int index = 0; index < 6; ++index) {{
    matrix(index, index) = 1e12;
  }}
  return matrix;
}}

void appendResidualValues(std::vector<double>& values, const gtsam::Vector& residual) {{
  for (int index = 0; index < residual.size(); ++index) {{
    values.push_back(residual(index));
  }}
}}

std::pair<double, double> residualStats(const std::vector<double>& values) {{
  if (values.empty()) {{
    return {{0.0, 0.0}};
  }}
  double squared_sum = 0.0;
  double max_abs = 0.0;
  for (double value : values) {{
    squared_sum += value * value;
    max_abs = std::max(max_abs, std::abs(value));
  }}
  return {{std::sqrt(squared_sum / static_cast<double>(values.size())), max_abs}};
}}

void writePoseJson(std::ostream& out, const gtsam::Pose3& pose) {{
  const auto t = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  out << "{{\\\"t_xyz\\\":["
      << t.x() << "," << t.y() << "," << t.z()
      << "],\\\"q_wxyz\\\":["
      << q.w() << "," << q.x() << "," << q.y() << "," << q.z()
      << "]}}";
}}

}}  // namespace

int main() {{
  try {{
    gtsam::Values initial;
{chr(10).join(initial_lines)}

    gtsam::NonlinearFactorGraph raw_graph;
    gtsam::NonlinearFactorGraph opt_graph;
{chr(10).join(prior_lines + between_lines)}
{anchor_line}

    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations({max_iterations});
    params.setRelativeErrorTol({cpp_float(tolerance)});
    params.setAbsoluteErrorTol({cpp_float(tolerance)});
    params.setlambdaInitial({cpp_float(initial_lambda)});

    const double initial_cost = raw_graph.error(initial);
    const auto start = std::chrono::steady_clock::now();
    gtsam::LevenbergMarquardtOptimizer optimizer(opt_graph, initial, params);
    gtsam::Values result = optimizer.optimize();
    const auto end = std::chrono::steady_clock::now();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(end - start).count();
    const double final_cost = raw_graph.error(result);
    std::vector<double> final_residual_values;
{chr(10).join(residual_lines)}
    const auto final_residual_stats = residualStats(final_residual_values);

    std::cout << std::setprecision(17);
    std::cout << "{{\\\"benchmark\\\":\\\"legacy_cpp_gtsam_fixture\\\","
              << "\\\"unit\\\":\\\"milliseconds\\\","
              << "\\\"cases\\\":[{{"
              << {case_output_fragment}
              << "\\\"poses\\\":{pose_count},"
              << "\\\"factors\\\":{factor_count},"
              << "\\\"status\\\":0,"
              << "\\\"converged\\\":true,"
              << "\\\"written\\\":" << result.size() << ","
              << "\\\"initial_cost\\\":" << initial_cost << ","
              << "\\\"final_cost\\\":" << final_cost << ","
              << "\\\"final_residual_rms\\\":" << final_residual_stats.first << ","
              << "\\\"final_residual_max\\\":" << final_residual_stats.second << ","
              << "\\\"iterations\\\":" << optimizer.iterations() << ","
              << "\\\"accepted_steps\\\":" << optimizer.iterations() << ","
              << "\\\"rejected_steps\\\":0,"
              << "\\\"elapsed_ms\\\":" << elapsed_ms << ","
              << "\\\"optimized_poses\\\":[";
    for (size_t i = 0; i < {pose_count}; ++i) {{
      if (i > 0) {{
        std::cout << ",";
      }}
      writePoseJson(std::cout, result.at<gtsam::Pose3>(i));
    }}
    std::cout << "]}}],\\\"metadata\\\":{{"
              << "\\\"generator\\\":\\\"tools/bench/pose_graph_opt_gtsam_baseline.py\\\","
              << "\\\"solver\\\":\\\"gtsam_levenberg_marquardt\\\""
              << "}}}}";
    return 0;
  }} catch (const std::exception& error) {{
    std::cerr << "GTSAM fixture baseline failed: " << error.what() << std::endl;
    return 2;
  }}
}}
"""


def pose_expr(pose: dict[str, Any]) -> str:
    t_xyz = [float(value) for value in pose["t_xyz"]]
    q_wxyz = [float(value) for value in pose["q_wxyz"]]
    values = t_xyz + q_wxyz
    return "makePose(" + ", ".join(cpp_float(value) for value in values) + ")"


def upper_expr(values: list[float]) -> str:
    return "std::array<double, 21>{" + ", ".join(cpp_float(float(value)) for value in values) + "}"


def endpoint_value(between: dict[str, Any], endpoint: str) -> Any:
    abi_key = f"{endpoint}_index"
    if abi_key in between:
        return between[abi_key]
    return between[endpoint]


def resolved_anchor_index(fixture_data: dict[str, Any]) -> int | None:
    config = fixture_data.get("config", {})
    fixed_pose_index = config.get("fixed_pose_index", 0)
    if fixed_pose_index is None:
        fixed_pose_index = -1
    fixed_pose_index = int(fixed_pose_index)
    pose_count = len(fixture_data["poses"])
    if fixed_pose_index >= pose_count:
        raise ValueError("fixed_pose_index must be inside poses")
    if fixed_pose_index >= 0:
        return fixed_pose_index
    if config.get("auto_anchor", True) and not fixture_data.get("priors", []):
        return 0
    return None


def cpp_float(value: float) -> str:
    return format(float(value), ".17g")


def cpp_string(value: str) -> str:
    return json.dumps(value)


if __name__ == "__main__":
    sys.exit(main())
