from decision.vision.follow_tasks import FollowTaskLedger


def _create(ledger: FollowTaskLedger, task_id: str) -> dict:
    return ledger.create(
        follow_task_id=task_id,
        product_session_id="session",
        request_id=f"request-{task_id}",
        client_id="client",
        target="person in red",
        target_id="",
    )


def test_follow_task_ledger_retains_recent_tasks_and_replays_requests():
    ledger = FollowTaskLedger(retention=2)
    first = _create(ledger, "one")
    _create(ledger, "two")
    _create(ledger, "three")

    assert "one" not in ledger
    assert ledger.find_request(first["request_id"], "client") is None
    assert ledger.find_request("request-three", "client")["follow_task_id"] == "three"


def test_follow_task_ledger_snapshots_and_finishes_cancellation():
    ledger = FollowTaskLedger()
    task = _create(ledger, "one")
    task["state"] = "cancelling"
    task["reason"] = "operator_cancel"

    view = ledger.snapshot(
        "one",
        current=True,
        details={"state": "cancelling"},
        phase="cancelling",
        reason="operator_cancel",
    )
    assert "client_id" not in view
    assert view["state"] == "cancelling"
    assert view["available_operations"] == ["cancel"]

    assert ledger.finish_cancel("one", {"state": "cancelled"}) is True
    assert ledger["one"]["terminal"] is True
    assert ledger["one"]["result"] == "cancelled"
