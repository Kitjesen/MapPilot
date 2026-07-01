"""Stable runtime contract registry for portable adapters.

This module is the deep Interface around the legacy Python runtime contract
tables. It intentionally wraps ``runtime.runtime_interface`` instead of moving
those tables in the first pass, so existing imports and behavior stay intact.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
from typing import Any

from runtime import runtime_interface as runtime
from runtime.contracts.messages import (
    CONTRACTS,
    ContractError,
    MessageContract,
    MessageEnvelope,
    ValidationIssue,
    validate_message,
)


@dataclass(frozen=True)
class TopicContract:
    """Portable topic contract consumed by transports and non-Python adapters."""

    topic: str
    formats: tuple[str, ...]
    allowed_frame_ids: tuple[str, ...] = ()
    default_frame_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "topic": self.topic,
            "formats": list(self.formats),
            "allowed_frame_ids": list(self.allowed_frame_ids),
            "default_frame_id": self.default_frame_id,
        }


@dataclass(frozen=True)
class MessageContractSchema:
    """JSON-ready view of a dict-message contract."""

    name: str
    schema_version: int
    required_fields: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "schema_version": self.schema_version,
            "required_fields": list(self.required_fields),
            "envelope": {
                "type": self.name,
                "schema_version": self.schema_version,
                "encoding": "dict",
                "frame_id": "optional",
                "ts": "optional",
                "payload": {"required": list(self.required_fields)},
            },
        }


class RuntimeContractRegistry:
    """Read-only registry for runtime topics, sources, messages, and manifest data."""

    schema_version = "lingtu.runtime_contract_registry.v1"

    def manifest(self) -> dict[str, Any]:
        """Return the runtime contract manifest as JSON-ready plain data."""

        manifest = _json_ready(runtime.runtime_contract_manifest())
        manifest["contract_registry_schema_version"] = self.schema_version
        manifest["message_contracts"] = {
            name: self.message_schema(name).to_dict()
            for name in sorted(CONTRACTS)
        }
        return manifest

    def topic_contract(
        self,
        topic: str,
        *,
        runtime_contract: str | None = None,
    ) -> TopicContract:
        """Return the normalized contract for one canonical topic."""

        formats = runtime.topic_formats(topic)
        allowed = runtime.runtime_topic_allowed_frame_ids(runtime_contract).get(topic, ())
        default = allowed[0] if allowed else None
        return TopicContract(
            topic=topic,
            formats=tuple(formats),
            allowed_frame_ids=tuple(allowed),
            default_frame_id=default,
        )

    def topic_formats(self, topic: str) -> tuple[str, ...]:
        """Return declared message formats for a topic."""

        return runtime.topic_formats(topic)

    def topic_ros_types(self, topic: str) -> tuple[str, ...]:
        """Return concrete ROS message/service payload types for a topic."""

        return runtime.topic_ros_types(topic)

    def allowed_frame_ids(
        self,
        topic: str,
        *,
        runtime_contract: str | None = None,
    ) -> tuple[str, ...]:
        """Return declared frame ids for a topic in one runtime contract."""

        frames = runtime.runtime_topic_allowed_frame_ids(runtime_contract).get(topic)
        if not frames:
            raise ValueError(f"topic {topic!r} has no declared frame_id contract")
        return tuple(frames)

    def default_frame_id(
        self,
        topic: str,
        *,
        runtime_contract: str | None = None,
    ) -> str:
        """Return default frame id for a topic in one runtime contract."""

        return runtime.runtime_topic_default_frame_id(runtime_contract, topic)

    def topic_contracts(
        self,
        *,
        runtime_contract: str | None = None,
    ) -> dict[str, TopicContract]:
        """Return contracts for every declared topic format."""

        return {
            topic: self.topic_contract(topic, runtime_contract=runtime_contract)
            for topic in sorted(runtime.TOPIC_FORMATS)
        }

    def data_source_contract(
        self,
        name: str | runtime.DataSourceContract,
    ) -> runtime.DataSourceContract:
        """Return a data-source contract, resolving legacy aliases."""

        if isinstance(name, runtime.DataSourceContract):
            return name
        data_source = runtime.canonical_data_source_name(str(name)) or ""
        try:
            return runtime.DATA_SOURCE_CONTRACTS[data_source]
        except KeyError as exc:
            available = ", ".join(sorted(runtime.DATA_SOURCE_CONTRACTS))
            raise ValueError(
                f"unknown data source {data_source!r}; available: {available}"
            ) from exc

    def message_contract(self, name: str) -> MessageContract:
        """Return a high-risk dict message contract by name."""

        try:
            return CONTRACTS[name]
        except KeyError as exc:
            available = ", ".join(sorted(CONTRACTS))
            raise ValueError(f"unknown message contract {name!r}; available: {available}") from exc

    def message_schema(self, name: str) -> MessageContractSchema:
        """Return a JSON-ready schema view for one dict message contract."""

        contract = self.message_contract(name)
        return MessageContractSchema(
            name=contract.name,
            schema_version=runtime_contract_schema_version(),
            required_fields=tuple(contract.required_fields),
        )

    def validate_payload(self, name: str, payload: Any) -> list[ValidationIssue]:
        """Validate a dict payload through the existing message contract."""

        return validate_message(name, payload)

    def validate_envelope(
        self,
        envelope: MessageEnvelope | Mapping[str, Any],
        *,
        runtime_contract: str | None = None,
    ) -> list[ValidationIssue]:
        """Validate an envelope without assuming a transport-specific schema."""

        env = envelope if isinstance(envelope, MessageEnvelope) else MessageEnvelope.from_mapping(envelope)
        issues: list[ValidationIssue] = []

        if env.topic:
            try:
                self.topic_formats(env.topic)
            except ValueError as exc:
                issues.append(ValidationIssue("topic", "unknown_topic", str(exc)))
            if env.frame_id:
                allowed = runtime.runtime_topic_allowed_frame_ids(runtime_contract).get(env.topic, ())
                expanded = runtime.expand_frame_id_aliases(tuple(allowed))
                frame_id = runtime.normalize_frame_id(env.frame_id)
                if expanded and frame_id not in expanded:
                    issues.append(
                        ValidationIssue(
                            "frame_id",
                            "invalid_frame_id",
                            f"expected one of {expanded}, got {env.frame_id!r}",
                        )
                    )

        contract = env.message_contract or env.type
        if contract:
            if contract not in CONTRACTS:
                issues.append(
                    ValidationIssue(
                        "message_contract",
                        "unknown_contract",
                        f"unknown message contract: {contract}",
                    )
                )
            else:
                issues.extend(self.validate_payload(contract, env.payload))

        return issues

    def assert_valid_envelope(
        self,
        envelope: MessageEnvelope | Mapping[str, Any],
        *,
        runtime_contract: str | None = None,
    ) -> None:
        """Raise ContractError if an envelope violates the registry contract."""

        issues = self.validate_envelope(envelope, runtime_contract=runtime_contract)
        if issues:
            detail = "; ".join(f"{issue.path}: {issue.message}" for issue in issues)
            raise ContractError(f"envelope contract violation: {detail}")

    def validate_manifest(self, manifest: Mapping[str, Any] | None = None) -> list[str]:
        """Return manifest contract issues without raising."""

        data = self.manifest() if manifest is None else manifest
        issues: list[str] = []
        if data.get("schema_version") != "lingtu.runtime_interface.v1":
            issues.append("schema_version must be lingtu.runtime_interface.v1")
        for key in (
            "frames",
            "topics",
            "topic_formats",
            "data_sources",
            "algorithm_interfaces",
            "message_contracts",
        ):
            if not isinstance(data.get(key), Mapping):
                issues.append(f"{key} must be a mapping")

        topic_formats = data.get("topic_formats")
        if isinstance(topic_formats, Mapping):
            for topic, formats in topic_formats.items():
                if not isinstance(topic, str) or not topic:
                    issues.append("topic_formats contains an empty topic")
                if not _nonempty_string_sequence(formats):
                    issues.append(f"topic_formats[{topic!r}] must be a non-empty string list")

        data_sources = data.get("data_sources")
        if isinstance(data_sources, Mapping):
            for name, source in data_sources.items():
                if not isinstance(source, Mapping):
                    issues.append(f"data_sources[{name!r}] must be a mapping")
                    continue
                if source.get("name") != name:
                    issues.append(f"data_sources[{name!r}].name must match its key")
                for field in ("provider", "owns", "normalized_outputs", "command_sink"):
                    if field not in source:
                        issues.append(f"data_sources[{name!r}] missing {field}")

        message_contracts = data.get("message_contracts")
        if isinstance(message_contracts, Mapping):
            for name, contract in message_contracts.items():
                if not isinstance(contract, Mapping):
                    issues.append(f"message_contracts[{name!r}] must be a mapping")
                    continue
                if contract.get("name") != name:
                    issues.append(f"message_contracts[{name!r}].name must match its key")
                if not isinstance(contract.get("required_fields"), Sequence):
                    issues.append(f"message_contracts[{name!r}].required_fields must be a list")

        return issues


def default_runtime_contract_registry() -> RuntimeContractRegistry:
    """Return the process-local runtime contract registry."""

    return _DEFAULT_REGISTRY


def runtime_contract_schema_version() -> int:
    """Return the dict-message payload schema version."""

    from runtime.contracts.messages import CURRENT_SCHEMA_VERSION

    return CURRENT_SCHEMA_VERSION


def _nonempty_string_sequence(value: Any) -> bool:
    if isinstance(value, str) or not isinstance(value, Sequence):
        return False
    return bool(value) and all(isinstance(item, str) and item for item in value)


def _json_ready(value: Any) -> Any:
    if hasattr(value, "__dataclass_fields__"):
        return _json_ready(asdict(value))
    if isinstance(value, Mapping):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_json_ready(item) for item in value]
    if isinstance(value, list):
        return [_json_ready(item) for item in value]
    return value


_DEFAULT_REGISTRY = RuntimeContractRegistry()
