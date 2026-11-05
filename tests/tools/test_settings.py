import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.skipif(shutil.which("cmake") is None, reason="CMake is required")
def test_yaml_change_rebuilds_native_defaults(tmp_path):
    shutil.copytree(ROOT / "config/system/settings", tmp_path / "config/system/settings")
    (tmp_path / "scripts/build/cmake").mkdir(parents=True)
    for source in ("scripts/build/settings.py", "scripts/build/cmake/settings.cmake"):
        shutil.copyfile(ROOT / source, tmp_path / source)
    (tmp_path / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.16)\n"
        "project(settings_probe LANGUAGES CXX)\n"
        "include(scripts/build/cmake/settings.cmake)\n"
        "add_executable(probe main.cpp)\n"
        "target_link_libraries(probe PRIVATE lingtu_settings)\n", encoding="utf-8",
    )
    (tmp_path / "main.cpp").write_text(
        "#include <lingtu/settings.hpp>\n#include <iostream>\n"
        "int main() { std::cout << lingtu::settings::lidar::lidar::scan_hz; }\n",
        encoding="utf-8",
    )
    build = tmp_path / "build"
    configure = ["cmake", "-S", str(tmp_path), "-B", str(build), f"-DPython3_EXECUTABLE={sys.executable}"]
    if os.name == "nt":
        configure += ["-G", "Visual Studio 17 2022", "-A", "x64"]
    subprocess.run(configure, check=True, capture_output=True, text=True)
    compile_command = ["cmake", "--build", str(build), "--config", "Release"]
    subprocess.run(compile_command, check=True, capture_output=True, text=True)
    executable = build / "Release/probe.exe" if os.name == "nt" else build / "probe"
    path = tmp_path / "config/system/settings/lidar.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert float(subprocess.check_output([str(executable)], text=True)) == data["lidar"]["scan_hz"]

    data["lidar"]["scan_hz"] = 12.0
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    # A normal incremental build must regenerate; no second configure invocation.
    subprocess.run(compile_command, check=True, capture_output=True, text=True)

    assert float(subprocess.check_output([str(executable)], text=True)) == 12.0
