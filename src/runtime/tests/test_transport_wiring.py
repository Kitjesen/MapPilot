from __future__ import annotations

import pytest

from runtime.transport.local import LocalTransport
from runtime.wiring import WireDelivery, resolve_wire_delivery


def test_callback_and_local_wire_delivery() -> None:
    assert resolve_wire_delivery(None) is None
    assert resolve_wire_delivery("callback") is None
    assert isinstance(resolve_wire_delivery("local"), LocalTransport)


def test_python_dds_wire_delivery_is_removed() -> None:
    assert {item.value for item in WireDelivery} == {"callback", "local"}
    with pytest.raises(ValueError, match="Unknown wire delivery"):
        resolve_wire_delivery("shm")
    with pytest.raises(ValueError, match="Unknown wire delivery"):
        resolve_wire_delivery("dds")
