"""The distributable protocol must preserve the running Gateway's wire contract."""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest

pytest.importorskip("google.protobuf")

from google.protobuf import descriptor_pb2
from tools.sdk.proto import check_descriptor, compile_descriptor, gateway_descriptors


@pytest.mark.skipif(shutil.which("protoc") is None, reason="protoc is required to compile the public protocol")
def test_public_sources_match_gateway_descriptors(tmp_path: Path) -> None:
    compiled = compile_descriptor(tmp_path / "public.protoset")
    check_descriptor(compiled)
    assert {file.name for file in compiled.file if file.package == "google.protobuf"} == {
        "google/protobuf/duration.proto",
        "google/protobuf/struct.proto",
        "google/protobuf/timestamp.proto",
    }


def test_error_trailer_field_changes_are_rejected() -> None:
    compiled = descriptor_pb2.FileDescriptorSet()
    compiled.file.extend(gateway_descriptors().values())
    error = next(file for file in compiled.file if file.name == "messages/error.proto")
    error.message_type[0].field[0].number = 99
    with pytest.raises(ValueError, match="Public protocol differs from gateway"):
        check_descriptor(compiled)
