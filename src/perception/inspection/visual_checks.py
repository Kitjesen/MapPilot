"""Read indicator and cabinet-door observations through the configured VLM.

The reader reports visible state, never a pass/fail or a motion decision. The
evidence store compares that observation with the requested expected state.
"""

from __future__ import annotations

import asyncio
import base64
import json
from collections.abc import Mapping
from typing import Any

from runtime.contracts.inspection_evidence import InspectionFinding
from runtime.registry import get

VISUAL_ACTIONS = frozenset({
    "capture:indicator_on", "capture:indicator_off", "capture:indicator_green",
    "capture:indicator_red", "capture:indicator_yellow",
    "capture:door_open", "capture:door_closed",
})

_INDICATOR_STATES = {"on", "off", "green", "red", "yellow", "unknown"}
_DOOR_STATES = {"open", "closed", "unknown"}


class InspectionVisualReader:
    """Use an existing vision service or an injected vision-capable client."""

    def __init__(self, *, server_url: str = "", client: Any = None) -> None:
        self._server_url = str(server_url)
        self._client = client

    def read(self, action: str, rgb_bytes: bytes) -> InspectionFinding:
        if action not in VISUAL_ACTIONS:
            raise ValueError("unsupported_visual_check")
        if self._client is None:
            if not self._server_url:
                return self._unknown("vision_model_not_configured")
            try:
                # Seed the existing provider registry; the shared service owns
                # model lifetime and no model is downloaded or started here.
                from perception import backends

                self._client = get("vlm", "remote").create(server_url=self._server_url)
            except Exception:
                return self._unknown("vision_model_unavailable")
        client = self._client
        if getattr(client, "supports_vision", False) is not True:
            return self._unknown("vision_model_unavailable")
        try:
            if not client.is_available():
                load = getattr(client, "load_model", None)
                if callable(load):
                    load()
                if not client.is_available():
                    return self._unknown("vision_model_unavailable")
            target = "indicator light" if action.startswith("capture:indicator_") else "cabinet door"
            states = _INDICATOR_STATES if target == "indicator light" else _DOOR_STATES
            prompt = (
                f"Inspect the {target} in this photograph. Report only what is visible, "
                "without guessing the desired state. If more than one possible target is "
                "present, report the count and use unknown. If the target is hidden, too "
                "small, blurred, or its state cannot be determined, use unknown. Return "
                "one JSON object with target_count (integer), visible (boolean), "
                f"observed_state (one of {', '.join(sorted(states))}), and reason (text). "
                "Do not provide a verdict. A light's color means it is visibly lit; "
                "off means an identifiable unlit indicator, not an absent light. "
                "For a cabinet door, open requires a visible opening or leaf displacement."
            )
            messages = [{"role": "user", "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {
                    "url": "data:image/jpeg;base64," + base64.b64encode(rgb_bytes).decode("ascii"),
                    "detail": "high",
                }},
            ]}]
            response = asyncio.run(self._chat(client, messages))
        except TimeoutError:
            return self._unknown("vision_model_timeout")
        except Exception:
            return self._unknown("vision_model_unavailable")
        return self._observation(response, states)

    @staticmethod
    async def _chat(client: Any, messages: list[dict[str, Any]]) -> str:
        return await asyncio.wait_for(client.chat(messages, temperature=0.0), timeout=8.0)

    @staticmethod
    def _unknown(reason: str, **details: Any) -> InspectionFinding:
        return InspectionFinding(
            observed_state="unknown", source="inspection_vlm", reason=reason, details=details,
        )

    def _observation(self, response: Any, states: set[str]) -> InspectionFinding:
        try:
            if not isinstance(response, str):
                return self._unknown("invalid_visual_observation")
            text = response.strip()
            if text.startswith("```") and text.endswith("```"):
                text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()
            observation = json.loads(text)
            if not isinstance(observation, Mapping):
                return self._unknown("invalid_visual_observation")
            count = observation.get("target_count")
            visible = observation.get("visible")
            state = str(observation.get("observed_state", "unknown")).strip().lower()
            if type(count) is not int or count < 0 or not isinstance(visible, bool) or state not in states:
                return self._unknown("invalid_visual_observation")
            details = {"target_count": count, "visible": visible}
            if count > 1:
                return self._unknown("ambiguous_visual_target", **details)
            if count != 1 or not visible:
                return self._unknown("visual_target_not_visible", **details)
            if state == "unknown":
                return self._unknown("visual_state_unclear", **details)
            return InspectionFinding(
                observed_state=state,
                source="inspection_vlm",
                reason=str(observation.get("reason") or "visible_state_observed"),
                details=details,
            )
        except (ValueError, IndexError):
            return self._unknown("invalid_visual_observation")
