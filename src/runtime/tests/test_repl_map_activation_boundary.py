from __future__ import annotations

from pathlib import Path


def test_repl_map_use_does_not_send_direct_set_active() -> None:
    source = (Path(__file__).parents[3] / "cli" / "repl.py").read_text(encoding="utf-8")
    map_handler = source.split("    def do_map(self, arg):", 1)[1].split("    def _map_cmd", 1)[0]

    assert 'self._map_cmd({"action": "set_active"' not in map_handler
    assert "use <name>" not in map_handler
    assert "/pgo/save_maps" not in map_handler
    assert "switch nav --map" in source
