"""Project read-only body and source control status without acquiring control."""

from .generated import motion_pb2 as pb


def body_state(event):
    return pb.BodyState(kind=event.kind, transition=event.transition, gesture_name=event.gesture_name)


def control_status(event):
    status = pb.BodyControlStatus(
        motor_output_enabled=event.motor_output_enabled,
        source_owner=event.owner,
        critical_motor_fault=event.critical_motor_fault,
        grpc_lease_active=event.grpc_lease_active,
        lease_remaining_ms=event.lease_remaining_ms,
        last_accepted_sequence=event.last_accepted_sequence,
        source_last_decision=event.last_decision,
        source_ready_for_walk=event.ready_for_walk,
        source_owner_id=event.owner_id,
        accepted_count=event.accepted_count,
        rejected_count=event.rejected_count,
    )
    if event.HasField("fsm"):
        status.body.CopyFrom(body_state(event.fsm))
    return status
