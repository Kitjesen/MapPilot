"""Decoder registry — protocol parsers shared across devices.

A Decoder takes raw bytes and yields typed messages. Each decoder is
registered globally by name; devices reference decoders by name in
their spec, so swapping a decoder doesn't require touching device code.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import Any, Callable, Iterator

logger = logging.getLogger(__name__)


class Decoder(ABC):
    """Protocol parser interface."""

    name: str = ""

    @abstractmethod
    def decode(self, raw: bytes) -> Iterator[Any]:
        """Parse raw bytes into typed messages. Yields zero or more."""


# ── Global registry ───────────────────────────────────────────────────

_decoders: dict[str, type[Decoder]] = {}


def register_decoder(name: str) -> Callable[[type[Decoder]], type[Decoder]]:
    """Decorator to register a Decoder class globally."""
    def wrap(cls: type[Decoder]) -> type[Decoder]:
        cls.name = name
        _decoders[name] = cls
        return cls
    return wrap


def decoder_registry() -> dict[str, type[Decoder]]:
    return dict(_decoders)


def get_decoder(name: str) -> type[Decoder] | None:
    return _decoders.get(name)
