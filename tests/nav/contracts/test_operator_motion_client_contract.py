from __future__ import annotations

from pathlib import Path


def test_cpp_operator_motion_client_uses_declared_frames_and_clock_sync() -> None:
    source = Path("src/nav/cpp/client/client.cpp").read_text(encoding="utf-8")
    control = source[
        source.index("OperatorMotionCommandReceipt writeOperatorMotionControl") : source.index(
            "void writeOperatorMotionSample"
        )
    ]
    sample = source[source.index("void writeOperatorMotionSample") : source.index("static bool requiresEndpointClock")]

    assert 'fillHeader(message.header, nowSeconds(), "");' in control
    assert 'fillHeader(message.header, send_source_stamp_s, "body");' in sample
    assert "synchronizeEndpointClock(timeout_ms);" in sample
    assert "message.source_stamp_ns = sourceStampNs(send_source_stamp_s);" in sample


def test_cpp_operator_motion_client_keeps_clock_sync_as_stop_handshake() -> None:
    source = Path("src/nav/cpp/client/client.cpp").read_text(encoding="utf-8")
    sync = source[source.index("void synchronizeEndpointClock") : source.index("std::string writeCommand")]

    assert "CommandKind::Stop" in sync
    assert 'sync.reason = const_cast<char*>("client_clock_sync");' in sync
