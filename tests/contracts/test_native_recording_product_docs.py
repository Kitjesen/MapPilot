"""Product-documentation contracts for native recording and replay."""

# ruff: noqa: D103, S101 - pytest contracts use assertions by design.

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GUIDE = ROOT / "docs" / "04-deployment" / "native_recording.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _bash_blocks(markdown: str) -> str:
    return "\n".join(re.findall(r"```bash\n(.*?)```", markdown, flags=re.DOTALL))


def test_native_recording_has_one_truthful_operator_guide() -> None:
    guide = _read(GUIDE)

    for expected in (
        "C++",
        "CycloneDDS",
        "MCAP",
        "scripts/lingtu record",
        "scripts/lingtu record --camera",
        "scripts/lingtu record status",
        "scripts/lingtu record stop",
        "scripts/lingtu record info",
        "scripts/lingtu record topics",
        "scripts/lingtu record verify",
        "scripts/lingtu play",
        "--min-free-gib",
        "5 GiB",
        "DDS domain `84`",
        "session.json",
        ".mcap.tmp",
        "record-only",
    ):
        assert expected in guide

    assert "startup preflight" in guide.lower()
    assert "0" in guide and "disable" in guide.lower()
    assert "running quota" in guide.lower()
    assert "not yet" in guide.lower()


def test_planned_rosbag2_parity_is_not_advertised_as_available_cli() -> None:
    guide = _read(GUIDE)
    commands = _bash_blocks(guide)

    for unsupported in (
        "scripts/lingtu record -a",
        "scripts/lingtu record --all",
        "scripts/lingtu record reindex",
        "scripts/lingtu record split",
        "scripts/lingtu record pause",
        "scripts/lingtu record snapshot",
        "scripts/lingtu record convert",
        "scripts/lingtu record list",
    ):
        assert unsupported not in commands

    for planned in (
        "automatic topic discovery",
        "split/rotation",
        "reindex/recovery",
        "pause/resume",
        "snapshot",
        "convert",
    ):
        assert planned in guide.lower()


def test_ros_bag_surfaces_are_explicit_compatibility_only() -> None:
    guide = _read(GUIDE)
    compat = _read(ROOT / "scripts" / "compat" / "ros2" / "README.md")

    for endpoint in (
        "/api/v1/recordings/start",
        "/api/v1/recordings/status",
        "/api/v1/recordings/stop",
    ):
        assert endpoint in guide

    for document in (guide, compat):
        assert "/api/v1/bag/start" in document
        assert "/api/v1/bag/status" in document
        assert "/api/v1/bag/stop" in document
        assert "compatibility" in document.lower()

    assert "native recording" in compat.lower()
    assert "scripts/lingtu record" in compat
    assert "deprecated" in guide.lower()
