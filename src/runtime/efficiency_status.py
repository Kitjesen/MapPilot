"""Small helpers for structured efficiency benchmark status payloads."""

from __future__ import annotations

from typing import Any


BENCHMARK_PLATFORM = "developer_workstation_or_ci"
BENCHMARK_HARDWARE = "not_s100p_real_robot"
BENCHMARK_CLAIM_SCOPE = "planner_regression_only"


def benchmark_claim_metadata(
    *,
    generated_at: float,
    max_age_s: float = 86400.0,
) -> dict[str, Any]:
    return {
        "platform": BENCHMARK_PLATFORM,
        "hardware": BENCHMARK_HARDWARE,
        "synthetic": True,
        "claim_scope": BENCHMARK_CLAIM_SCOPE,
        "freshness": {"generated_at": generated_at, "max_age_s": max_age_s},
    }


def classify_benchmark_error(message: str) -> str:
    normalized = message.lower()
    if (
        "unavailable" in normalized
        or "not available" in normalized
        or "missing" in normalized
    ):
        return "skip"
    return "fail"
