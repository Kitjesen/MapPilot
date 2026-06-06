"""Encoder protocol for CLIP-style image/text encoders.

Decouples memory/ from semantic/ by defining the encoder contract in core/.
Both memory.modules.vector_memory_module and semantic.perception.* encoders
adhere to this Protocol instead of importing each other's concrete classes.

Usage:
    from core.encoder_protocol import EncoderProtocol

    class MyEncoder:
        def encode_image(self, image: np.ndarray) -> np.ndarray: ...
        def encode_text(self, text: str) -> np.ndarray: ...

    encoder: EncoderProtocol = MyEncoder()
    assert isinstance(encoder, EncoderProtocol)  # runtime_checkable
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class EncoderProtocol(Protocol):
    """Protocol for CLIP-style image/text encoders.

    Defines the minimum contract that an encoder must satisfy to be usable
    by VectorMemoryModule and other consumers in the memory/ package.

    Both CLIPEncoder and MobileCLIPEncoder in semantic.perception satisfy
    this protocol at runtime (method existence checked via runtime_checkable).
    """

    def encode_image(self, image: Any) -> Any:
        """Encode an image into a feature vector.

        Args:
            image: RGB or BGR image as (H, W, 3) uint8 array.

        Returns:
            L2-normalized feature vector of shape (D,), or empty array on failure.
        """
        ...

    def encode_text(self, text: str) -> Any:
        """Encode a text string into a feature vector.

        Args:
            text: Natural language query string.

        Returns:
            L2-normalized feature vector of shape (D,), or empty array on failure.
        """
        ...
