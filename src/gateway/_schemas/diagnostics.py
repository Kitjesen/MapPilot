"""Internal Gateway diagnostics request and response models."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import Field

from gateway._schemas.common import (
    GatewayResponseModel,
)
from gateway._schemas.runtime_contracts import (
    RuntimeContractManifest,
)


class RoutecheckLatestResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool
    artifacts_root: str
    count: int = 0
    artifact_dir: str | None = None
    summary_path: str | None = None
    report_mtime: float | None = None
    report_age_s: float | None = None
    non_motion: bool | None = None
    simulation_only: bool | None = None
    real_robot_motion: bool | None = None
    cmd_vel_sent_to_hardware: bool | None = None
    gateway_used: bool | None = None
    driver_used: bool | None = None
    published: dict[str, Any] | None = None
    latest: dict[str, Any] | None = None
    reason: str | None = None
    ts: float


class RealRuntimeEvidenceLatestResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool
    artifacts_root: str
    count: int = 0
    artifact_dir: str | None = None
    report_path: str | None = None
    validation_path: str | None = None
    report_mtime: float | None = None
    report_age_s: float | None = None
    max_age_s: float
    runtime_contract: str | None = None
    runtime_evidence_ok: bool = False
    simulation_only: bool | None = None
    real_robot_motion: bool | None = None
    cmd_vel_sent_to_hardware: bool | None = None
    checked_real_motion_evidence: dict[str, Any] = Field(default_factory=dict)
    checked_hardware_boundary_evidence: dict[str, Any] = Field(default_factory=dict)
    checked_live_topic_freshness: dict[str, Any] = Field(default_factory=dict)
    checked_runtime_data_flow_evidence: dict[str, Any] = Field(default_factory=dict)
    blockers: list[str] = Field(default_factory=list)
    reason: str | None = None
    ts: float


class AlgorithmBenchmarkLatestResponse(GatewayResponseModel):
    schema_version: Literal["lingtu.algorithm_benchmark_latest.v1"] = "lingtu.algorithm_benchmark_latest.v1"
    ok: bool
    read_only: bool = True
    ros2_topic_required: bool = False
    publishes: list[str] = Field(default_factory=list)
    artifacts_root: str
    count: int = 0
    summary_path: str | None = None
    report_mtime: float | None = None
    report_age_s: float | None = None
    max_age_s: float
    preset: str = "dimos_benchmark"
    source: str = "server_sim_closure"
    active_product_profile: str = "inspection_mvp"
    strict_benchmark_profile: str = "dimos_benchmark"
    summary_schema_version: str | None = None
    claim_allowed: bool = False
    missing_or_failed: list[str] = Field(default_factory=list)
    required_gate_sequence: list[str] = Field(default_factory=list)
    validation_flow: list[dict[str, Any]] = Field(default_factory=list)
    claim_boundary: dict[str, Any] = Field(default_factory=dict)
    product_profiles: dict[str, dict[str, Any]] = Field(default_factory=dict)
    dimos_gap: dict[str, Any] = Field(default_factory=dict)
    blocking_categories: dict[str, list[str]] = Field(default_factory=dict)
    blockers: list[str] = Field(default_factory=list)
    reason: str | None = None
    latest: dict[str, Any] | None = None
    ts: float


class RuntimeContractResponse(GatewayResponseModel):
    schema_version: int = 1
    source: str
    manifest: RuntimeContractManifest
    ts: float


__all__ = (
    "AlgorithmBenchmarkLatestResponse",
    "RealRuntimeEvidenceLatestResponse",
    "RoutecheckLatestResponse",
    "RuntimeContractResponse",
)
