"""Message contract helpers for dict-based module ports."""

from .messages import (
    ContractError,
    CURRENT_SCHEMA_VERSION,
    MessageEnvelope,
    ValidationIssue,
    assert_valid_message,
    validate_message,
)
from .runtime import (
    RuntimeContractRegistry,
    TopicContract,
    default_runtime_contract_registry,
)

__all__ = [
    "CURRENT_SCHEMA_VERSION",
    "ContractError",
    "MessageEnvelope",
    "RuntimeContractRegistry",
    "TopicContract",
    "ValidationIssue",
    "assert_valid_message",
    "default_runtime_contract_registry",
    "validate_message",
]
