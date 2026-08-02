"""Validate runtime route contracts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from message.dds import dds_topic_name, topic_spec

from .loader import REPO_ROOT, load_route_contract
from .model import RouteBackend, RouteContract, TopicContract

_PORT_BINDING_DIRECTIONS = frozenset({"in", "out"})
_PORT_BINDING_BOUNDARIES = frozenset({"module", "endpoint", "native"})


@dataclass(frozen=True)
class RouteIssue:
    """A machine-readable route validation issue."""

    code: str
    message: str
    scope: str = "route"
    severity: str = "error"

    def as_dict(self) -> dict[str, str]:
        return {
            "code": self.code,
            "message": self.message,
            "scope": self.scope,
            "severity": self.severity,
        }


def validate_route_contract(contract: RouteContract | None = None, *, route: str = "robot") -> list[RouteIssue]:
    """Validate a route contract."""

    contract = contract or load_route_contract(route)
    issues: list[RouteIssue] = []
    topics = contract.topics
    allowed_backends = {backend.value for backend in RouteBackend}

    if not topics:
        return [_issue("topics_missing", "route contract has no topics")]

    for topic, spec in topics.items():
        if not _valid_topic_name(topic):
            issues.append(_issue("topic_name_invalid", f"topic {topic!r} must start with /", topic))
        if not spec.producer:
            issues.append(_issue("topic_producer_missing", f"topic {topic} has no producer", topic))
        if not spec.consumers and not _is_external_diagnostics_stream(spec):
            issues.append(_issue("topic_consumers_missing", f"topic {topic} has no consumers", topic))
        issues.extend(_validate_port_bindings(topic, spec.port_bindings))

    for topic, backend in contract.route.routes.items():
        if topic not in topics:
            issues.append(
                _issue("route_topic_missing", f"route {contract.route.name} references undeclared topic {topic}", topic)
            )
        if backend not in allowed_backends:
            issues.append(
                _issue("route_backend_unknown", f"route {contract.route.name} uses unknown backend {backend!r}", topic)
            )

    if contract.route.default not in allowed_backends:
        issues.append(
            _issue(
                "route_default_unknown",
                f"route {contract.route.name} uses unknown default backend {contract.route.default!r}",
                f"route:{contract.route.name}",
            )
        )

    issues.extend(_validate_protocol_bindings(contract))
    return issues


def assert_route_contract_valid(contract: RouteContract | None = None, *, route: str = "robot") -> RouteContract:
    """Return *contract* if valid, otherwise raise ValueError."""

    contract = contract or load_route_contract(route)
    issues = validate_route_contract(contract)
    if issues:
        joined = "; ".join(f"{issue.code}: {issue.message}" for issue in issues)
        raise ValueError(f"Route validation failed: {joined}")
    return contract


def _validate_protocol_bindings(contract: RouteContract) -> list[RouteIssue]:
    issues: list[RouteIssue] = []
    cpp_topics = _read_cpp_topic_header()
    for topic in sorted(contract.topics):
        backend = contract.route_for(topic)
        if backend == RouteBackend.DDS.value:
            if not contract.topic(topic).port_bindings:
                issues.append(
                    _issue(
                        "dds_port_bindings_missing",
                        f"DDS-routed topic {topic} has no runtime port/boundary bindings",
                        topic,
                    )
                )
            issues.extend(_validate_dds_binding(contract, topic, cpp_topics))
            if contract.route.endpoint_contract:
                issues.extend(_validate_dds_endpoint_binding(contract.route.endpoint_contract, topic))
        elif backend == RouteBackend.LCM.value:
            issues.extend(_validate_lcm_binding(contract, topic))
    return issues


def _validate_port_bindings(topic: str, bindings: Any) -> list[RouteIssue]:
    issues: list[RouteIssue] = []
    for index, binding in enumerate(bindings):
        scope = f"{topic}:port_bindings[{index}]"
        if not binding.owner:
            issues.append(_issue("port_binding_owner_missing", f"{scope} has no owner", topic))
        if not binding.port:
            issues.append(_issue("port_binding_port_missing", f"{scope} has no port", topic))
        if binding.direction not in _PORT_BINDING_DIRECTIONS:
            issues.append(
                _issue(
                    "port_binding_direction_invalid",
                    f"{scope} direction must be one of {sorted(_PORT_BINDING_DIRECTIONS)}, got {binding.direction!r}",
                    topic,
                )
            )
        if binding.boundary not in _PORT_BINDING_BOUNDARIES:
            issues.append(
                _issue(
                    "port_binding_boundary_invalid",
                    f"{scope} boundary must be one of {sorted(_PORT_BINDING_BOUNDARIES)}, got {binding.boundary!r}",
                    topic,
                )
            )
    return issues


def _validate_dds_binding(contract: RouteContract, topic: str, cpp_topics: str) -> list[RouteIssue]:
    issues: list[RouteIssue] = []
    spec = topic_spec(topic)
    if spec is None:
        return [_issue("dds_topic_spec_missing", f"DDS-routed topic {topic} has no typed DDS spec", topic)]

    expected_channel = dds_topic_name(topic, typed=True)
    binding = contract.route.binding_for(RouteBackend.DDS.value, topic)
    declared_channel = str(binding.get("channel") or expected_channel)
    if declared_channel != expected_channel:
        issues.append(
            _issue(
                "dds_channel_mismatch",
                f"{topic} DDS channel must be {expected_channel}, got {declared_channel}",
                topic,
            )
        )
    if not expected_channel.startswith("rt/"):
        issues.append(_issue("dds_channel_invalid", f"{topic} DDS channel must start with rt/", topic))

    expected_literal = f'"{topic}", "{expected_channel}"'
    if expected_literal not in cpp_topics:
        issues.append(
            _issue(
                "dds_cpp_topic_missing",
                f"C++ DDS topic registry is missing {topic} -> {expected_channel}",
                topic,
            )
        )
    if spec.idl_type not in cpp_topics:
        issues.append(
            _issue(
                "dds_cpp_idl_type_missing",
                f"C++ DDS topic registry is missing IDL type {spec.idl_type}",
                topic,
            )
        )
    return issues


def _validate_lcm_binding(contract: RouteContract, topic: str) -> list[RouteIssue]:
    binding = contract.route.binding_for(RouteBackend.LCM.value, topic)
    missing = [field for field in ("channel", "type") if not str(binding.get(field) or "").strip()]
    if not missing:
        return []
    return [
        _issue(
            "lcm_binding_missing",
            f"LCM-routed topic {topic} missing binding fields: {', '.join(missing)}",
            topic,
        )
    ]


def _validate_dds_endpoint_binding(contract_name: str, topic: str) -> list[RouteIssue]:
    try:
        from runtime.endpoints.dds.contracts import binding_for_topic

        binding_for_topic(contract_name, topic)
    except KeyError:
        return [
            _issue(
                "dds_endpoint_binding_missing",
                f"DDS route topic {topic} is missing from endpoint contract {contract_name}",
                topic,
            )
        ]
    return []


def _read_cpp_topic_header() -> str:
    path = REPO_ROOT / "src" / "message" / "cpp" / "dds_topics.hpp"
    try:
        return path.read_text(encoding="utf-8")
    except OSError:
        return ""


def _valid_topic_name(topic: str) -> bool:
    return topic.startswith("/") and "//" not in topic and len(topic) > 1


def _is_external_diagnostics_stream(spec: TopicContract) -> bool:
    """Allow intentionally output-only observability streams, never commands."""

    role = str(spec.role).strip().lower()
    schema = str(spec.schema).strip().lower()
    observability_role = role.endswith(
        ("_status", "_telemetry", "_diagnostic", "_diagnostics", "_event", "_events")
    )
    observability_schema = schema.endswith(
        ("_status", "_telemetry", "_diagnostic", "_diagnostics", "_event", "_events")
    )
    output_only_boundary = bool(spec.port_bindings) and all(
        binding.direction == "out" for binding in spec.port_bindings
    )
    return bool(
        spec.external_diagnostics_subscribable
        and observability_role
        and observability_schema
        and output_only_boundary
    )


def _issue(code: str, message: str, scope: str = "route", severity: str = "error") -> RouteIssue:
    return RouteIssue(code=code, message=message, scope=scope, severity=severity)
