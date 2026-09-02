from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOCK = ROOT / "scripts" / "build" / "locks" / "cyclonedds-windows-x64.json"


def test_source_lock_pins_the_official_cyclonedds_release_and_build() -> None:
    lock = json.loads(LOCK.read_text(encoding="utf-8"))

    assert lock["schema_version"] == 1
    assert lock["repository"] == "https://github.com/eclipse-cyclonedds/cyclonedds.git"
    assert lock["tag"] == "11.0.1"
    assert lock["commit"] == "e54e991f75a3e67f8e628da3171122e36ea5b872"
    assert lock["tree"] == "56508d35826c362782fc8a388cad351a3d491f51"
    assert lock["architecture"] == "x64"
    assert lock["configuration"] == "Release"
    assert lock["msvc_runtime"] == "MultiThreadedDLL"
    assert lock["build_shared_libs"] is True
    assert lock["enable_security"] is False
    assert lock["build_idlc"] is True
    assert lock["install_system_runtime_libs_skip"] is True
