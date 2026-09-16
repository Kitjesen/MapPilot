"""Product-documentation contracts for native recording and replay."""



from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GUIDE = ROOT / "docs" / "operations.md"
API = ROOT / "docs" / "api.md"
IMPLEMENTATION = ROOT / "src" / "native" / "recording" / "README.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_native_recording_has_one_truthful_operator_guide() -> None:
    guide = _read(GUIDE)

    for expected in (
        "MCAP",
        "lingtu_recorder",
        "lingtu_dds_player",
        "Recording does not authorize motion",
    ):
        assert expected in guide

    implementation = _read(IMPLEMENTATION)
    for expected in (
        "C++",
        "CycloneDDS",
        "--min-free-gib",
        "5 GiB",
        "isolated DDS domain 84",
        "session.json",
        ".mcap.tmp",
        "record-only",
    ):
        assert expected in implementation



def test_recording_docs_expose_only_native_mcap_routes() -> None:
    api = _read(API)

    for endpoint in (
        "/api/v1/recordings/start",
        "/api/v1/recordings/status",
        "/api/v1/recordings/stop",
    ):
        assert endpoint in api

    assert "/api/v1/bag/" not in api
