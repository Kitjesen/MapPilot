#!/usr/bin/env python3
"""Build and run a C++ GPMP pure-math baseline fixture.

This optional baseline intentionally avoids the nonlinear optimizer dependency.
It includes LingTu's legacy WNOJ/WNOA model headers for process-model and
interpolation math, then emits the same JSON schema as the Rust GPMP math
fixture.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[2]
RUNNER_NAME = "pct_gpmp_math_cpp_baseline"
RESULT_SCHEMA = "lingtu.pct_gpmp_math.result.v1"


def generate_cmake() -> str:
    return r"""cmake_minimum_required(VERSION 3.16)
project(lingtu_pct_gpmp_math_cpp_baseline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(pct_gpmp_math_cpp_baseline pct_gpmp_math_cpp_baseline.cpp)
target_include_directories(pct_gpmp_math_cpp_baseline PRIVATE "${PCT_LIB_SRC}")

if(DEFINED EIGEN_INCLUDE_DIR AND NOT "${EIGEN_INCLUDE_DIR}" STREQUAL "")
  target_include_directories(pct_gpmp_math_cpp_baseline PRIVATE "${EIGEN_INCLUDE_DIR}")
else()
  find_package(Eigen3 REQUIRED)
  if(TARGET Eigen3::Eigen)
    target_link_libraries(pct_gpmp_math_cpp_baseline PRIVATE Eigen3::Eigen)
  else()
    target_include_directories(pct_gpmp_math_cpp_baseline PRIVATE "${EIGEN3_INCLUDE_DIR}")
  endif()
endif()
"""


def generate_runner_source() -> str:
    return r'''#include <Eigen/Core>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "trajectory_optimization/gpmp_optimizer/models/wnoj.hpp"
#include "trajectory_optimization/gpmp_optimizer/models/wnoa.hpp"

using Vec4 = Eigen::Matrix<double, 4, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat4 = Eigen::Matrix<double, 4, 4>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Row6 = Eigen::Matrix<double, 1, 6>;

std::string f64_json(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}

template <typename Derived>
std::string vec_json(const Eigen::MatrixBase<Derived>& values) {
  std::ostringstream out;
  out << "[";
  for (int row = 0; row < values.rows(); ++row) {
    if (row) out << ",";
    out << f64_json(values(row));
  }
  out << "]";
  return out.str();
}

std::string row6_json(const Row6& values) {
  std::ostringstream out;
  out << "[";
  for (int col = 0; col < 6; ++col) {
    if (col) out << ",";
    out << f64_json(values(0, col));
  }
  out << "]";
  return out.str();
}

template <typename Derived>
std::string mat_json(const Eigen::MatrixBase<Derived>& values) {
  std::ostringstream out;
  out << "[";
  for (int row = 0; row < values.rows(); ++row) {
    if (row) out << ",";
    out << "[";
    for (int col = 0; col < values.cols(); ++col) {
      if (col) out << ",";
      out << f64_json(values(row, col));
    }
    out << "]";
  }
  out << "]";
  return out.str();
}

Mat4 wnoa_q_inverse(double qc, double tau) {
  Mat4 out = Mat4::Zero();
  const double qc_inv = 1.0 / qc;
  const double q00 = 12.0 * std::pow(tau, -3) * qc_inv;
  const double q01 = -6.0 * std::pow(tau, -2) * qc_inv;
  const double q11 = 4.0 / tau * qc_inv;
  out(0, 0) = q00;
  out(0, 1) = q01;
  out(1, 0) = q01;
  out(1, 1) = q11;
  out(2, 2) = q00;
  out(2, 3) = q01;
  out(3, 2) = q01;
  out(3, 3) = q11;
  return out;
}

Vec6 wnoj_path_point_state(double x, double y, double heading, double reference_velocity) {
  const double velocity = std::max(reference_velocity, 1.0);
  Vec6 out;
  out << x, std::cos(heading) * velocity, 0.0, y, std::sin(heading) * velocity, 0.0;
  return out;
}

Vec4 wnoa_path_point_state(double x, double y, double heading, double reference_velocity) {
  const double velocity = std::max(reference_velocity, 1.0);
  Vec4 out;
  out << x, std::cos(heading) * velocity, y, std::sin(heading) * velocity;
  return out;
}

double heading_rate(const Vec6& state) {
  const double dx = state(1);
  const double ddx = state(2);
  const double dy = state(4);
  const double ddy = state(5);
  return (ddy * dx - dy * ddx) / (dx * dx + dy * dy + 1e-6);
}

double heading_rate_residual(const Vec6& state, double max_heading_rate) {
  const double rate = heading_rate(state);
  if (rate > max_heading_rate) {
    return rate - max_heading_rate;
  }
  if (rate < -max_heading_rate) {
    return -max_heading_rate - rate;
  }
  return 0.0;
}

Row6 heading_rate_jacobian(const Vec6& state, double max_heading_rate) {
  const double dx = state(1);
  const double ddx = state(2);
  const double dy = state(4);
  const double ddy = state(5);
  const double denom = dx * dx + dy * dy + 1e-6;
  const double rate = heading_rate(state);
  Row6 jac = Row6::Zero();
  if (std::abs(rate) <= max_heading_rate) {
    return jac;
  }
  jac(0, 1) = (ddy * dy * dy - ddy * dx * dx + 2.0 * dx * dy * ddx) / (denom * denom);
  jac(0, 2) = -dy / denom;
  jac(0, 4) = (-ddx * dx * dx + ddx * dy * dy - 2.0 * dx * dy * ddy) / (denom * denom);
  jac(0, 5) = dx / denom;
  if (rate < -max_heading_rate) {
    return -jac;
  }
  return jac;
}

Vec6 wnoj_interpolate(const Vec6& x1, const Vec6& x2, double qc, double delta, double tau) {
  Mat6 lambda;
  Mat6 psi;
  WhiteNoiseOnJerkModel2D::LambdaAndPsi(qc, delta, tau, &lambda, &psi);
  return lambda * x1 + psi * x2;
}

Vec4 wnoa_interpolate(const Vec4& x1, const Vec4& x2, double qc, double delta, double tau) {
  Mat4 lambda;
  Mat4 psi;
  WhiteNoiseOnAcceleration2D::LambdaAndPsi(qc, delta, tau, &lambda, &psi);
  return lambda * x1 + psi * x2;
}

std::string wnoj_trajectory_json(const Vec6& x1, const Vec6& x2, double qc, double delta, int sub_sample_num) {
  const double sample_dt = delta / (sub_sample_num + 1.0);
  std::ostringstream out;
  out << "[" << vec_json(x1);
  for (int index = 0; index < sub_sample_num; ++index) {
    out << "," << vec_json(wnoj_interpolate(x1, x2, qc, delta, sample_dt * (index + 1)));
  }
  out << "," << vec_json(x2) << "]";
  return out.str();
}

std::string wnoa_trajectory_json(const Vec4& x1, const Vec4& x2, double qc, double delta, int sub_sample_num) {
  const double sample_dt = delta / (sub_sample_num + 1.0);
  std::ostringstream out;
  out << "[" << vec_json(x1);
  for (int index = 0; index < sub_sample_num; ++index) {
    out << "," << vec_json(wnoa_interpolate(x1, x2, qc, delta, sample_dt * (index + 1)));
  }
  out << "," << vec_json(x2) << "]";
  return out.str();
}

std::string wnoj_case(const std::string& case_name, const Vec6& x1, const Vec6& x2, double max_heading_rate) {
  const double qc = 0.1;
  const double delta = 1.0;
  const double tau = 0.35;
  const int sub_sample_num = 2;
  Mat6 lambda;
  Mat6 psi;
  WhiteNoiseOnJerkModel2D::LambdaAndPsi(qc, delta, tau, &lambda, &psi);
  const Vec6 interpolated_state = lambda * x1 + psi * x2;
  const Row6 interpolated_heading_jacobian = heading_rate_jacobian(interpolated_state, max_heading_rate);
  const Mat6 prior_jacobian_x2 = -Mat6::Identity();

  std::ostringstream out;
  out << "{";
  out << "\"case\":\"" << case_name << "\",";
  out << "\"mode\":\"wnoj\",";
  out << "\"state_order\":[\"x\",\"vx\",\"ax\",\"y\",\"vy\",\"ay\"],";
  out << "\"input\":{\"qc\":" << f64_json(qc)
      << ",\"delta\":" << f64_json(delta)
      << ",\"tau\":" << f64_json(tau)
      << ",\"max_heading_rate\":" << f64_json(max_heading_rate)
      << ",\"sub_sample_num\":" << sub_sample_num
      << ",\"x1\":" << vec_json(x1)
      << ",\"x2\":" << vec_json(x2)
      << ",\"path_point\":{\"x\":2.0,\"y\":3.0,\"heading\":0.78539816339744828,\"reference_velocity\":0.2}},";
  out << "\"output\":{";
  out << "\"q\":" << mat_json(WhiteNoiseOnJerkModel2D::Q(qc, delta)) << ",";
  out << "\"q_inverse\":" << mat_json(WhiteNoiseOnJerkModel2D::QInverse(qc, delta)) << ",";
  out << "\"phi\":" << mat_json(WhiteNoiseOnJerkModel2D::Phi(delta)) << ",";
  out << "\"q_delta\":" << mat_json(WhiteNoiseOnJerkModel2D::Q(qc, delta)) << ",";
  out << "\"q_inverse_delta\":" << mat_json(WhiteNoiseOnJerkModel2D::QInverse(qc, delta)) << ",";
  out << "\"phi_delta\":" << mat_json(WhiteNoiseOnJerkModel2D::Phi(delta)) << ",";
  out << "\"q_tau\":" << mat_json(WhiteNoiseOnJerkModel2D::Q(qc, tau)) << ",";
  out << "\"phi_tau\":" << mat_json(WhiteNoiseOnJerkModel2D::Phi(tau)) << ",";
  out << "\"phi_delta_minus_tau\":" << mat_json(WhiteNoiseOnJerkModel2D::Phi(delta - tau)) << ",";
  out << "\"lambda\":" << mat_json(lambda) << ",";
  out << "\"psi\":" << mat_json(psi) << ",";
  out << "\"prior_residual\":" << vec_json(WhiteNoiseOnJerkModel2D::Phi(delta) * x1 - x2) << ",";
  out << "\"prior_jacobian_x1\":" << mat_json(WhiteNoiseOnJerkModel2D::Phi(delta)) << ",";
  out << "\"prior_jacobian_x2\":" << mat_json(prior_jacobian_x2) << ",";
  out << "\"interpolated_state\":" << vec_json(interpolated_state) << ",";
  out << "\"interpolate_jacobian_x1\":" << mat_json(lambda) << ",";
  out << "\"interpolate_jacobian_x2\":" << mat_json(psi) << ",";
  out << "\"heading_rate\":" << f64_json(heading_rate(x1)) << ",";
  out << "\"heading_rate_residual\":" << f64_json(heading_rate_residual(x1, max_heading_rate)) << ",";
  out << "\"heading_rate_jacobian\":" << row6_json(heading_rate_jacobian(x1, max_heading_rate)) << ",";
  out << "\"interpolated_heading_rate\":" << f64_json(heading_rate(interpolated_state)) << ",";
  out << "\"interpolated_heading_rate_residual\":" << f64_json(heading_rate_residual(interpolated_state, max_heading_rate)) << ",";
  out << "\"interpolated_heading_jacobian_x1\":" << row6_json(interpolated_heading_jacobian * lambda) << ",";
  out << "\"interpolated_heading_jacobian_x2\":" << row6_json(interpolated_heading_jacobian * psi) << ",";
  out << "\"path_point_state\":" << vec_json(wnoj_path_point_state(2.0, 3.0, 0.78539816339744828, 0.2)) << ",";
  out << "\"trajectory\":" << wnoj_trajectory_json(x1, x2, qc, delta, sub_sample_num);
  out << "}}";
  return out.str();
}

std::string wnoa_case() {
  const double qc = 0.1;
  const double delta = 1.0;
  const double tau = 0.35;
  const int sub_sample_num = 2;
  Vec4 x1;
  x1 << 0.2, 1.1, -0.3, 0.7;
  Vec4 x2;
  x2 << 0.9, 1.0, 0.4, 0.8;
  Mat4 lambda;
  Mat4 psi;
  WhiteNoiseOnAcceleration2D::LambdaAndPsi(qc, delta, tau, &lambda, &psi);
  const Mat4 prior_jacobian_x2 = -Mat4::Identity();

  std::ostringstream out;
  out << "{";
  out << "\"case\":\"wnoa_nominal\",";
  out << "\"mode\":\"wnoa\",";
  out << "\"state_order\":[\"x\",\"vx\",\"y\",\"vy\"],";
  out << "\"input\":{\"qc\":" << f64_json(qc)
      << ",\"delta\":" << f64_json(delta)
      << ",\"tau\":" << f64_json(tau)
      << ",\"sub_sample_num\":" << sub_sample_num
      << ",\"x1\":" << vec_json(x1)
      << ",\"x2\":" << vec_json(x2)
      << ",\"path_point\":{\"x\":2.0,\"y\":3.0,\"heading\":0.78539816339744828,\"reference_velocity\":2.0}},";
  out << "\"output\":{";
  out << "\"q\":" << mat_json(WhiteNoiseOnAcceleration2D::Q(qc, delta)) << ",";
  out << "\"q_inverse\":" << mat_json(wnoa_q_inverse(qc, delta)) << ",";
  out << "\"phi\":" << mat_json(WhiteNoiseOnAcceleration2D::Phi(delta)) << ",";
  out << "\"q_delta\":" << mat_json(WhiteNoiseOnAcceleration2D::Q(qc, delta)) << ",";
  out << "\"q_inverse_delta\":" << mat_json(wnoa_q_inverse(qc, delta)) << ",";
  out << "\"phi_delta\":" << mat_json(WhiteNoiseOnAcceleration2D::Phi(delta)) << ",";
  out << "\"q_tau\":" << mat_json(WhiteNoiseOnAcceleration2D::Q(qc, tau)) << ",";
  out << "\"phi_tau\":" << mat_json(WhiteNoiseOnAcceleration2D::Phi(tau)) << ",";
  out << "\"phi_delta_minus_tau\":" << mat_json(WhiteNoiseOnAcceleration2D::Phi(delta - tau)) << ",";
  out << "\"lambda\":" << mat_json(lambda) << ",";
  out << "\"psi\":" << mat_json(psi) << ",";
  out << "\"prior_residual\":" << vec_json(WhiteNoiseOnAcceleration2D::Phi(delta) * x1 - x2) << ",";
  out << "\"prior_jacobian_x1\":" << mat_json(WhiteNoiseOnAcceleration2D::Phi(delta)) << ",";
  out << "\"prior_jacobian_x2\":" << mat_json(prior_jacobian_x2) << ",";
  out << "\"interpolated_state\":" << vec_json(lambda * x1 + psi * x2) << ",";
  out << "\"interpolate_jacobian_x1\":" << mat_json(lambda) << ",";
  out << "\"interpolate_jacobian_x2\":" << mat_json(psi) << ",";
  out << "\"path_point_state\":" << vec_json(wnoa_path_point_state(2.0, 3.0, 0.78539816339744828, 2.0)) << ",";
  out << "\"trajectory\":" << wnoa_trajectory_json(x1, x2, qc, delta, sub_sample_num);
  out << "}}";
  return out.str();
}

int main() {
  Vec6 positive_x1;
  positive_x1 << 0.2, 1.1, -0.4, -0.3, 0.7, 1.8;
  Vec6 positive_x2;
  positive_x2 << 0.9, 1.0, -0.2, 0.4, 0.8, 1.3;
  Vec6 negative_x1;
  negative_x1 << 0.2, 1.1, 0.4, -0.3, 0.7, -1.8;
  Vec6 negative_x2;
  negative_x2 << 0.9, 1.0, 0.2, 0.4, 0.8, -1.3;
  Vec6 inside_x1;
  inside_x1 << 0.2, 1.1, 0.0, -0.3, 0.7, 0.0;
  Vec6 inside_x2;
  inside_x2 << 0.9, 1.0, 0.0, 0.4, 0.8, 0.0;

  std::cout << "{\"schema\":\"lingtu.pct_gpmp_math.result.v1\","
            << "\"producer\":\"legacy_cpp_gpmp_math\","
            << "\"cases\":["
            << wnoj_case("wnoj_positive_heading_rate", positive_x1, positive_x2, 0.2) << ","
            << wnoj_case("wnoj_negative_heading_rate", negative_x1, negative_x2, 0.2) << ","
            << wnoj_case("wnoj_heading_rate_inside_limit", inside_x1, inside_x2, 0.2) << ","
            << wnoa_case()
            << "]}" << std::endl;
  return 0;
}
'''


def _repo_pct_lib_src(repo_root: Path) -> Path:
    return (
        repo_root
        / "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src"
    ).resolve()


def _candidate_eigen_include_dirs(repo_root: Path) -> list[Path]:
    candidates: list[Path] = []
    for env_name in ("EIGEN3_INCLUDE_DIR", "EIGEN_INCLUDE_DIR"):
        value = os.environ.get(env_name)
        if value:
            candidates.append(Path(value))
    candidates.extend(
        [
            repo_root / "third_party/eigen",
            repo_root / "third_party/eigen3",
            repo_root / "external/eigen",
            repo_root / "external/eigen3",
        ]
    )
    conda_prefixes: list[Path] = []
    for value in (os.environ.get("CONDA_PREFIX"), sys.prefix, sys.base_prefix):
        if value:
            path = Path(value)
            if path not in conda_prefixes:
                conda_prefixes.append(path)
    for conda_prefix in conda_prefixes:
        candidates.extend(
            [
                conda_prefix / "Library/include/eigen3",
                conda_prefix / "Library/include",
                conda_prefix / "Lib/site-packages/casadi/include/eigen3",
                conda_prefix / "include/eigen3",
                conda_prefix / "include",
            ]
        )
        packages = conda_prefix / "pkgs"
        if packages.is_dir():
            candidates.extend(
                sorted(
                    packages.glob("eigen-*/Library/include/eigen3"),
                    reverse=True,
                )
            )
    candidates.extend(
        [
            Path("C:/Program Files/Eigen3/include/eigen3"),
            Path("C:/Program Files (x86)/Eigen3/include/eigen3"),
            Path("C:/vcpkg/installed/x64-windows/include"),
            Path("D:/vcpkg/installed/x64-windows/include"),
            Path("/usr/include/eigen3"),
            Path("/usr/local/include/eigen3"),
            Path("/opt/homebrew/include/eigen3"),
        ]
    )
    return candidates


def _find_eigen_include_dir(repo_root: Path) -> Path | None:
    for candidate in _candidate_eigen_include_dirs(repo_root):
        if (candidate / "Eigen/Core").is_file():
            return candidate.resolve()
    return None


def _load_json_text(text: str) -> dict[str, Any]:
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError("C++ baseline did not emit a JSON object")
    if data.get("schema") != RESULT_SCHEMA:
        raise ValueError(f"unexpected C++ baseline schema: {data.get('schema')!r}")
    return data


def _find_executable(binary_dir: Path, config: str) -> Path:
    candidates = [
        binary_dir / f"{RUNNER_NAME}.exe",
        binary_dir / RUNNER_NAME,
        binary_dir / config / f"{RUNNER_NAME}.exe",
        binary_dir / config / RUNNER_NAME,
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    matches = list(binary_dir.rglob(f"{RUNNER_NAME}*"))
    for match in matches:
        if match.is_file() and match.suffix.lower() in {"", ".exe"}:
            return match
    raise RuntimeError(f"built executable {RUNNER_NAME!r} was not found under {binary_dir}")


def _run_command(command: list[str], *, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            command,
            cwd=str(cwd) if cwd else None,
            check=True,
            text=True,
            encoding="utf-8",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except subprocess.CalledProcessError as exc:
        stdout = (exc.stdout or "").strip()
        stderr = (exc.stderr or "").strip()
        details = "\n".join(
            part
            for part in [
                f"command failed ({exc.returncode}): {' '.join(command)}",
                f"stdout:\n{stdout}" if stdout else "",
                f"stderr:\n{stderr}" if stderr else "",
            ]
            if part
        )
        raise RuntimeError(details) from exc


def _build_and_run(
    repo_root: Path,
    build_root: Path,
    *,
    cmake: str,
    config: str,
    eigen_include_dir: Path | None,
) -> dict[str, Any]:
    pct_lib_src = _repo_pct_lib_src(repo_root)
    if not pct_lib_src.is_dir():
        raise FileNotFoundError(f"PCT library source directory not found: {pct_lib_src}")
    eigen_include_dir = eigen_include_dir or _find_eigen_include_dir(repo_root)

    source_dir = build_root / "src"
    binary_dir = build_root / "build"
    source_dir.mkdir(parents=True, exist_ok=True)
    binary_dir.mkdir(parents=True, exist_ok=True)
    (source_dir / "CMakeLists.txt").write_text(generate_cmake(), encoding="utf-8")
    (source_dir / f"{RUNNER_NAME}.cpp").write_text(generate_runner_source(), encoding="utf-8")

    configure = [
        cmake,
        "-S",
        str(source_dir),
        "-B",
        str(binary_dir),
        f"-DPCT_LIB_SRC={pct_lib_src.as_posix()}",
    ]
    if eigen_include_dir is not None:
        configure.append(f"-DEIGEN_INCLUDE_DIR={eigen_include_dir.resolve().as_posix()}")
    _run_command(configure)
    _run_command([cmake, "--build", str(binary_dir), "--config", config])
    executable = _find_executable(binary_dir, config)
    completed = _run_command([str(executable)])
    result = _load_json_text(completed.stdout)
    result["metadata"] = {
        "generator": "tools/bench/pct_gpmp_math_cpp_baseline.py",
        "source": "legacy_cpp_gpmp_model_headers",
        "build_dir": str(build_root),
        "pct_lib_src": pct_lib_src.as_posix(),
        "eigen_include_dir": eigen_include_dir.as_posix()
        if eigen_include_dir is not None
        else None,
    }
    return result


def run_cpp_baseline(
    *,
    repo_root: Path = ROOT_DIR,
    build_dir: Path | None = None,
    cmake: str = "cmake",
    config: str = "Release",
    eigen_include_dir: Path | None = None,
    keep_build: bool = False,
) -> dict[str, Any]:
    if shutil.which(cmake) is None:
        raise RuntimeError(f"{cmake!r} is required to build the GPMP C++ baseline")

    repo_root = repo_root.resolve()
    if build_dir is not None:
        build_dir.mkdir(parents=True, exist_ok=True)
        return _build_and_run(
            repo_root,
            build_dir,
            cmake=cmake,
            config=config,
            eigen_include_dir=eigen_include_dir,
        )

    with tempfile.TemporaryDirectory(prefix="lingtu_gpmp_math_cpp_") as tmp:
        build_root = Path(tmp)
        result = _build_and_run(
            repo_root,
            build_root,
            cmake=cmake,
            config=config,
            eigen_include_dir=eigen_include_dir,
        )
        if keep_build:
            preserved = Path.cwd() / ".tmp" / "pct_gpmp_math_cpp_baseline"
            if preserved.exists():
                shutil.rmtree(preserved)
            preserved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(build_root, preserved)
            result["metadata"]["build_dir"] = str(preserved)
        return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=ROOT_DIR)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--build-dir", type=Path)
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--config", default="Release")
    parser.add_argument(
        "--eigen-include-dir",
        "--eigen-include",
        dest="eigen_include_dir",
        type=Path,
    )
    parser.add_argument("--keep-build", action="store_true")
    parser.add_argument("--print-source", action="store_true")
    parser.add_argument("--json", action="store_true", help="emit JSON only")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.print_source:
        print(generate_runner_source())
        return 0

    try:
        result = run_cpp_baseline(
            repo_root=args.repo_root,
            build_dir=args.build_dir,
            cmake=args.cmake,
            config=args.config,
            eigen_include_dir=args.eigen_include_dir,
            keep_build=args.keep_build,
        )
    except RuntimeError as exc:
        print(f"PCT/GPMP C++ math baseline failed:\n{exc}", file=sys.stderr)
        return 2
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json:
        print(text)
    else:
        print("PCT/GPMP C++ math baseline: generated")
        print(f"  cases: {len(result.get('cases', []))}")
        if args.json_out:
            print(f"  output: {args.json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
