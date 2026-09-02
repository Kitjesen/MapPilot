from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / "sim"
SESSION = (
    ROOT
    / "runtime"
    / "visual"
    / "RobotSimUE"
    / "Plugins"
    / "LingTuSim"
    / "Source"
    / "LingTuSimSession"
)


def _session_source() -> str:
    return "\n".join(
        path.read_text(encoding="utf-8")
        for path in SESSION.rglob("*")
        if path.is_file() and path.suffix.lower() in {".h", ".cpp", ".cs"}
    )


def test_snapshot_mailbox_has_the_public_session_aware_api() -> None:
    header = (SESSION / "Public" / "LingTuSimSnapshotMailbox.h").read_text(
        encoding="utf-8"
    )

    assert "namespace LingTuSim" in header
    assert "class LINGTUSIMSESSION_API FSnapshotMailbox" in header
    assert "BindSession(FString" in header
    assert "Publish(const FSnapshotEnvelope&" in header
    assert "TryTakeLatest(FSnapshotEnvelope&" in header
    assert "Clear()" in header
    assert "PendingCount() const" in header
    for result in ("Accepted", "Replaced", "Stale", "SessionMismatch", "ModelMismatch"):
        assert result in header


def test_snapshot_mailbox_uses_core_locking_and_value_copies_only() -> None:
    source = _session_source()

    assert "FCriticalSection" in source
    assert "FScopeLock" in source
    assert "PendingSnapshot = Snapshot" in source
    assert "OutSnapshot = PendingSnapshot" in source
    assert "UObject" not in source
    assert '"Engine/' not in source


def test_snapshot_mailbox_has_dev_automation_behavior_coverage() -> None:
    test_source = (
        SESSION / "Private" / "Tests" / "LingTuSimSnapshotMailboxTest.cpp"
    ).read_text(encoding="utf-8")

    assert "#if WITH_DEV_AUTOMATION_TESTS" in test_source
    assert "Sequence 3 replaces sequence 1" in test_source
    assert "Sequence 2 is stale after sequence 3" in test_source
    assert "PublishCount = 10'000" in test_source
    assert "newer reset generation accepts sequence 0" in test_source
    assert "A stale model generation is rejected" in test_source
    assert "A future model generation is rejected" in test_source
    assert "A different session is rejected" in test_source


def test_session_service_exposes_one_shared_mailbox_without_engine_types() -> None:
    header = (SESSION / "Public" / "LingTuSimSessionService.h").read_text(
        encoding="utf-8"
    )

    assert "FSessionService" in header
    assert "GetSnapshotMailbox()" in header
    assert "FSnapshotMailbox&" in header
    assert "UObject" not in header
    assert "Engine" not in header
