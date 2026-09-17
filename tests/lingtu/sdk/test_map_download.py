"""Exercise stdlib HTTP streaming, including EOF without a read exception."""

from http.client import HTTPResponse
from io import BytesIO
from unittest.mock import Mock

import pytest

from lingtu.sdk import LingTuClient


@pytest.mark.parametrize("payload", [b"partial", b""])
def test_truncated_download_preserves_existing_map(monkeypatch, tmp_path, payload):
    # HTTPResponse.read(size) can return early EOF without IncompleteRead.
    wire = b"HTTP/1.1 200 OK\r\nContent-Length: 100\r\n\r\n" + payload
    response = HTTPResponse(Mock(makefile=lambda *args: BytesIO(wire)))
    response.begin()
    monkeypatch.setattr("urllib.request.urlopen", lambda *args, **kwargs: response)
    target = tmp_path / "map.pcd"
    target.write_bytes(b"previous-map")

    with pytest.raises(OSError, match="Incomplete map download"):
        LingTuClient().download_map_pcd("yard", target)

    assert target.read_bytes() == b"previous-map"
    assert list(tmp_path.glob("*.part")) == []


@pytest.mark.parametrize("length_header", [b"Content-Length: 9\r\n", b""])
def test_complete_download_replaces_map(monkeypatch, tmp_path, length_header):
    wire = b"HTTP/1.1 200 OK\r\n" + length_header + b"\r\npcd-bytes"
    response = HTTPResponse(Mock(makefile=lambda *args: BytesIO(wire)))
    response.begin()
    monkeypatch.setattr("urllib.request.urlopen", lambda *args, **kwargs: response)
    target = tmp_path / "map.pcd"
    target.write_bytes(b"previous-map")

    assert LingTuClient().download_map_pcd("yard", target) == target
    assert target.read_bytes() == b"pcd-bytes"
    assert list(tmp_path.glob("*.part")) == []
