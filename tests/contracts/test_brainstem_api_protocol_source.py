"""Contracts for the Brainstem Client SDK seam used by the native driver."""

# ruff: noqa: D103, S101 - pytest contracts use assertions by design.

from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NATIVE = ROOT / "src" / "drivers" / "real" / "motion"
DOSO = NATIVE / "robots" / "doso"
PROTOCOL = DOSO / "protocol" / "brainstem_api"
TESTS = ROOT / "tests" / "drivers" / "real" / "motion"

EXPECTED_SOURCE = {
    "url": "https://github.com/Kitjesen/brainstem_api.git",
    "commit": "e252d4125c4ac5d045143148d999249d796e8f54",
    "version": "2.1.0",
    "files": {
        "brainstem_api/cms.proto": (
            "fbd9a858ab986eb6d72e85281cfa940712d01af886c5f84aab583ddbfa230aca"
        ),
        "brainstem_api/common.proto": (
            "b4b0616642bbe41111995b2d8efa2585889e609d9e11cb5acf9281cecfdd6efc"
        ),
    },
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def test_native_driver_uses_one_pinned_brainstem_api_snapshot() -> None:
    assert not (NATIVE / "brainstem.proto").exists()
    assert (PROTOCOL / "VERSION").read_text(encoding="utf-8").strip() == "2.1.0"

    source = json.loads((PROTOCOL / "SOURCE.json").read_text(encoding="utf-8"))
    assert source == EXPECTED_SOURCE
    for relative_path, expected_hash in source["files"].items():
        assert _sha256(PROTOCOL / relative_path) == expected_hash


def test_doso_uses_the_brainstem_client_sdk() -> None:
    cmake = (NATIVE / "CMakeLists.txt").read_text(encoding="utf-8")
    doso_header = (DOSO / "doso.hpp").read_text(encoding="utf-8")
    doso = (DOSO / "doso.cpp").read_text(encoding="utf-8")
    test_io = (TESTS / "test_doso_io.cpp").read_text(encoding="utf-8")

    assert "find_package(BrainstemClient CONFIG REQUIRED)" in cmake
    assert "brainstem::client" in cmake
    assert "protocol/brainstem_api" in cmake
    assert "brainstem_api/cms.proto" in cmake
    assert "brainstem_api/common.proto" in cmake
    assert "brainstem.proto" not in cmake
    assert "class Doso final" in doso_header
    assert "DosoBackend" not in doso_header + doso
    assert "ThunderBackend" not in doso_header + doso
    assert '#include <brainstem/client.hpp>' in doso
    assert "#include <grpc" not in doso
    assert "grpc::" not in doso
    assert "#include <google/protobuf" not in doso
    assert "lease_token" not in doso
    assert not (DOSO / "brainstem.cpp").exists()
    assert '"brainstem_api/cms.grpc.pb.h"' in test_io
