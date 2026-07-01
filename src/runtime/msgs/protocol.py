"""lingtu.runtime.msgs.protocol — LingtuMsg protocol and message type utilities.

Defines the canonical message interface that all LingTu inter-module messages
must implement.  Inspired by the ``DimosMsg`` protocol from dimos, adapted for
LingTu's binary ``encode()`` / ``decode()`` convention.

.. code-block:: python

    @runtime_checkable
    class LingtuMsg(Protocol):
        msg_name: str
        def encode(self) -> bytes: ...
        @classmethod
        def decode(cls, data: bytes) -> LingtuMsg: ...

All ``runtime.msgs.*`` types are expected to satisfy this protocol, making them
discoverable, self-describing, and transport-agnostic — no ROS 2 dependency.
"""

from __future__ import annotations

import importlib
from functools import lru_cache
from typing import TYPE_CHECKING, Protocol, runtime_checkable

if TYPE_CHECKING:
    from typing import Any


@runtime_checkable
class LingtuMsg(Protocol):
    """Protocol for LingTu's own transport-agnostic message types.

    Every message that flows through a Module ``In[T]`` / ``Out[T]`` port
    should implement this protocol.  The ``msg_name`` class attribute acts
    as a global identifier (e.g. ``"geometry_msgs.Pose"``), matching the
    ROS 2 naming convention so compat bridges can map without ambiguity.

    Minimal implementation::

        class MyMsg:
            msg_name = "my_msgs.MyMsg"

            def encode(self) -> bytes: ...
            @classmethod
            def decode(cls, data: bytes) -> MyMsg: ...
    """

    msg_name: str
    """Globally unique message type name (e.g. ``"nav_msgs.Odometry"``)."""

    def encode(self) -> bytes:
        """Serialise this message to a compact binary representation."""
        ...

    @classmethod
    def decode(cls, data: bytes) -> LingtuMsg:
        """Reconstruct a message instance from bytes produced by :meth:`encode`."""
        ...


def is_lingtu_msg(obj: Any) -> bool:
    """Return ``True`` if *obj* satisfies the :class:`LingtuMsg` protocol.

    Works with both instances and classes.
    """
    if isinstance(obj, type):
        return (
            hasattr(obj, "msg_name")
            and hasattr(obj, "encode")
            and hasattr(obj, "decode")
        )
    return (
        hasattr(obj, "msg_name")
        and callable(getattr(obj, "encode", None))
        and hasattr(type(obj), "decode")
        and callable(getattr(type(obj), "decode", None))
    )


@lru_cache(maxsize=256)
def resolve_msg_type(type_name: str) -> type[LingtuMsg] | None:
    """Resolve a message type name to its class.

    Args:
        type_name: Type name in format ``"module.ClassName"``
            (e.g. ``"geometry_msgs.Vector3"``, ``"nav_msgs.Odometry"``).

    Returns:
        The message class, or ``None`` if not found in ``runtime.msgs.*``.

    Example:
        >>> cls = resolve_msg_type("geometry_msgs.Pose")
        >>> cls  # runtime.msgs.geometry.Pose
    """
    try:
        module_name, class_name = type_name.rsplit(".", 1)
    except ValueError:
        return None

    import_paths = [
        f"runtime.msgs.{_MSG_MODULE_MAP.get(module_name, module_name)}",
    ]

    # Fallback: try direct module import
    if module_name not in _MSG_MODULE_MAP:
        import_paths.append(f"runtime.msgs.{module_name}")

    for path in import_paths:
        try:
            module = importlib.import_module(path)
            cls = getattr(module, class_name, None)
            if cls is not None and is_lingtu_msg(cls):
                return cls  # type: ignore[return-value]
        except ImportError:
            continue

    return None


# Map ROS2-style package names → LingTu msgs module names.
# Extended when new message families are added.
_MSG_MODULE_MAP: dict[str, str] = {
    "geometry_msgs": "geometry",
    "nav_msgs": "nav",
    "sensor_msgs": "sensor",
    "std_msgs": "nav",          # Header not used standalone; map for completeness
    "vision_msgs": "semantic",
    "lingtu_msgs": "semantic",  # LingTu custom semantic types
}


def json_message_encode(msg: Any) -> bytes:
    """JSON-then-length-prefix encoder for messages with ``to_dict()``.

    Suitable for low-bandwidth semantic / robot-state messages that already
    implement ``to_dict()`` / ``from_dict()``.  Layout::

        [length: uint32 LE] + [json_bytes...]

    Returns:
        Binary payload ready for :meth:`json_message_decode`.
    """
    import json as _json
    import struct as _struct

    payload = _json.dumps(msg.to_dict(), ensure_ascii=False).encode("utf-8")
    return _struct.pack("<I", len(payload)) + payload


def json_message_decode(data: bytes, cls: type) -> Any:
    """Inverse of :func:`json_message_encode`.

    Args:
        data: Bytes previously produced by ``json_message_encode``.
        cls: Message class that has a ``from_dict`` classmethod.

    Returns:
        Instance of *cls* reconstructed from the JSON payload.
    """
    import json as _json
    import struct as _struct

    length = _struct.unpack("<I", data[:4])[0]
    return cls.from_dict(_json.loads(data[4: 4 + length].decode("utf-8")))
