"""Small catalog of live field acceptance gates."""

from __future__ import annotations

from typing import Any

from diagnostics.field.evidence import REAL_RUNTIME_CONTRACT

REAL_RUNTIME_EVIDENCE_ARTIFACT = "artifacts/real_runtime/report.json"
REAL_RUNTIME_EVIDENCE_COLLECTOR_COMMAND = (
    "python -m diagnostics.field.runtime_evidence "
    "--gateway-url http://<robot>:5050 --duration-sec 20 "
    f"--expected-contract {REAL_RUNTIME_CONTRACT} --json-out {REAL_RUNTIME_EVIDENCE_ARTIFACT}"
)
SAVED_MAP_ARTIFACT_GATE_COMMAND = (
    "python -m diagnostics.field.map_artifacts <map-id> "
    "[--require-octomap | --require-occupancy] "
    "--json-out artifacts/saved_map_artifacts/report.json"
)


_GATES: dict[str, dict[str, Any]] = {
    "real_runtime_evidence": {
        "schema_version": "lingtu.real_runtime_evidence.v1",
        "scope": "observed_real_runtime",
        "acceptance_step": 2,
        "required_when": "before_claiming_real_runtime_or_navigation",
        "command": REAL_RUNTIME_EVIDENCE_COLLECTOR_COMMAND,
        "artifact": REAL_RUNTIME_EVIDENCE_ARTIFACT,
        "expected_runtime_contract": REAL_RUNTIME_CONTRACT,
    },
    "saved_map_artifact_gate": {
        "schema_version": "lingtu.saved_map_artifacts.gate.v1",
        "scope": "saved_map_artifact",
        "acceptance_step": 1,
        "required_when": "a saved map, octomap, or occupancy artifact is used",
        "command": SAVED_MAP_ARTIFACT_GATE_COMMAND,
        "artifact": "artifacts/saved_map_artifacts/report.json",
    },
}


def gate_catalog() -> dict[str, dict[str, Any]]:
    """Return the two gates that have live script consumers."""

    return {name: dict(gate) for name, gate in _GATES.items()}
