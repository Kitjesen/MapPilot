"""Project the native operator authority snapshot without acquiring control."""

from gateway.navigation.commands import navigation_commands

from .generated import system_pb2 as pb


def state(gateway, identity=None):
    commands = navigation_commands(gateway)
    read = getattr(commands, "operator_motion_status", None)
    if not callable(read):
        return pb.ControlState(reason="operator_status_unavailable")
    try:
        snapshot = read(max_age_ms=250)
    except (OSError, RuntimeError, ValueError) as error:
        return pb.ControlState(reason=str(error))
    if snapshot is None:
        return pb.ControlState(reason="operator_status_unavailable")
    result = pb.ControlState(
        owner=snapshot["active_source_id"],
        owner_epoch=snapshot["active_source_epoch"],
        reason=snapshot["authority_reason"],
        received_age_ms=snapshot["received_age_ms"],
        fresh=snapshot["fresh"],
    )
    if result.fresh:
        active = snapshot["has_active_authority"]
        result.available = not active
        result.holding = snapshot["holding"]
        if identity is not None:
            result.held_by_client = active and (result.owner, result.owner_epoch) == identity
    return result
