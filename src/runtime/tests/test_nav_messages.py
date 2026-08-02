from __future__ import annotations

import pytest

from runtime.msgs import OperatorMotionAction as ExportedOperatorMotionAction
from runtime.msgs import OperatorMotionReceipt as ExportedOperatorMotionReceipt
from runtime.msgs.nav import OperatorMotionAction, OperatorMotionReceipt


def _receipt_kwargs(**overrides: object) -> dict[str, object]:
    values: dict[str, object] = {
        "accepted": True,
        "action": int(OperatorMotionAction.HOLD),
        "request_id": "req-17",
        "source_id": "operator-panel",
        "source_epoch": 2,
        "source_sequence": 9,
        "accepted_sequence": 9,
        "final_output_sequence": 31,
        "endpoint_timestamp_s": 123.5,
        "reason": "accepted",
    }
    values.update(overrides)
    return values


def test_operator_motion_receipt_accepts_known_actions_and_exports() -> None:
    receipt = OperatorMotionReceipt(**_receipt_kwargs())

    assert ExportedOperatorMotionAction is OperatorMotionAction
    assert ExportedOperatorMotionReceipt is OperatorMotionReceipt
    assert receipt.source_accepted is True
    assert receipt.final_output_published is True
    assert receipt.to_dict() == {
        "accepted": True,
        "action": int(OperatorMotionAction.HOLD),
        "action_name": "HOLD",
        "request_id": "req-17",
        "source_id": "operator-panel",
        "source_epoch": 2,
        "source_sequence": 9,
        "accepted_sequence": 9,
        "final_output_sequence": 31,
        "endpoint_timestamp_s": 123.5,
        "reason": "accepted",
        "source_accepted": True,
        "final_output_published": True,
    }


def test_operator_motion_receipt_reject_path_has_no_accepted_or_final_output() -> None:
    receipt = OperatorMotionReceipt(
        **_receipt_kwargs(
            accepted=False,
            action=OperatorMotionAction.CLAIM,
            accepted_sequence=0,
            final_output_sequence=0,
            reason="stale source",
        )
    )

    assert receipt.source_accepted is False
    assert receipt.final_output_published is False
    assert receipt.to_dict()["action_name"] == "CLAIM"


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("action", 99, "99"),
        ("accepted", 1, "accepted"),
        ("request_id", "", "request_id"),
        ("source_id", "   ", "source_id"),
        ("source_epoch", 0, "positive"),
        ("source_sequence", 0, "positive"),
        ("accepted_sequence", -1, "negative"),
        ("final_output_sequence", -1, "negative"),
        ("endpoint_timestamp_s", 0.0, "positive finite"),
        ("endpoint_timestamp_s", float("nan"), "positive finite"),
    ],
)
def test_operator_motion_receipt_fail_closed_validation(
    field: str,
    value: object,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        OperatorMotionReceipt(**_receipt_kwargs(**{field: value}))


def test_operator_motion_receipt_enforces_accepted_sequence_contract() -> None:
    with pytest.raises(ValueError, match="equal source_sequence"):
        OperatorMotionReceipt(**_receipt_kwargs(accepted_sequence=8))

    with pytest.raises(ValueError, match="accepted_sequence must be 0 when rejected"):
        OperatorMotionReceipt(
            **_receipt_kwargs(accepted=False, accepted_sequence=9, reason="denied")
        )

    with pytest.raises(ValueError, match="final_output_sequence must be 0 when rejected"):
        OperatorMotionReceipt(
            **_receipt_kwargs(
                accepted=False,
                accepted_sequence=0,
                final_output_sequence=31,
                reason="denied",
            )
        )


def test_operator_motion_receipt_final_output_only_for_hold_or_release() -> None:
    claim = OperatorMotionReceipt(
        **_receipt_kwargs(action=OperatorMotionAction.CLAIM, final_output_sequence=31)
    )
    hold_without_output = OperatorMotionReceipt(
        **_receipt_kwargs(action=OperatorMotionAction.HOLD, final_output_sequence=0)
    )
    release_with_output = OperatorMotionReceipt(
        **_receipt_kwargs(action=OperatorMotionAction.RELEASE, final_output_sequence=32)
    )

    assert claim.final_output_published is False
    assert hold_without_output.final_output_published is False
    assert release_with_output.final_output_published is True