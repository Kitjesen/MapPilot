from __future__ import annotations

from lingtu.map_runtime_transaction import MapRuntimeTransaction


def test_all_map_runtime_transactions_share_one_mutation_lock() -> None:
    def command(_request):
        return {"success": True, "active": "demo"}

    def reload(_path):
        return {"ok": True}

    first = MapRuntimeTransaction(command, reload)
    second = MapRuntimeTransaction(command, reload)

    assert first.shared_lock() is second.shared_lock()
