"""Strict loopback transport for ScenarioSnapshot visual effects."""

from __future__ import annotations

import json
import socket
from collections.abc import Callable, Mapping
from contextlib import suppress
from typing import Any

from .dispatcher import ScenarioDispatchError
from .runtime import ScenarioSnapshot

MAX_SCENARIO_DATAGRAM_BYTES = 60_000


def encode_scenario_snapshot(snapshot: ScenarioSnapshot) -> bytes:
    """Encode the authoritative routed snapshot without recomputing state."""

    try:
        payload = json.dumps(
            snapshot.to_dict(),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ScenarioDispatchError(f"scenario snapshot is not JSON-serializable: {exc}") from exc
    if len(payload) > MAX_SCENARIO_DATAGRAM_BYTES:
        raise ScenarioDispatchError(
            f"scenario snapshot exceeds the loopback datagram limit: {len(payload)} > {MAX_SCENARIO_DATAGRAM_BYTES}"
        )
    return payload


class UdpScenarioVisualSink:
    """Fail-closed sender using RobotSimUE's existing snapshot UDP port.

    A successful ``sendto`` proves only that the complete datagram reached the
    local UDP sender. UE application is proven exclusively by
    ``scenario-visual-evidence.json`` from ``ue_registry_applied``.
    """

    def __init__(
        self,
        port: int,
        *,
        host: str = "127.0.0.1",
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        if host != "127.0.0.1":
            raise ValueError("scenario visual transport is restricted to 127.0.0.1")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("scenario snapshot port must be an integer from 1 to 65535")
        self._destination = (host, port)
        transport = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            transport.setblocking(False)
        except BaseException:
            with suppress(Exception):
                transport.close()
            raise
        self._socket = transport

    def apply_visual_entities(
        self,
        snapshot: ScenarioSnapshot,
    ) -> Mapping[str, Any]:
        """Queue one entire datagram without claiming remote application."""

        payload = encode_scenario_snapshot(snapshot)
        try:
            sent = self._socket.sendto(payload, self._destination)
        except OSError as exc:
            raise ScenarioDispatchError(f"scenario visual datagram send failed: {type(exc).__name__}: {exc}") from exc
        if sent != len(payload):
            raise ScenarioDispatchError(f"scenario visual datagram was truncated: {sent} of {len(payload)} bytes")
        return {
            "result": "queued",
            "delivery_stage": "udp_sender",
            "ue_application_verified": False,
            "required_application_evidence": {
                "artifact": "scenario-visual-evidence.json",
                "source": "ue_registry_applied",
                "input_source": "canonical_scenario_snapshot",
            },
            "transport": "udp_loopback_json_v1",
            "bytes": sent,
            "session_id": snapshot.session_id,
            "model_generation": snapshot.model_generation,
            "reset_generation": snapshot.reset_generation,
            "sequence": snapshot.sequence,
            "sim_time_ns": snapshot.sim_time_ns,
        }

    def close(self) -> None:
        """Release the sender socket."""

        self._socket.close()

    def __enter__(self) -> UdpScenarioVisualSink:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
