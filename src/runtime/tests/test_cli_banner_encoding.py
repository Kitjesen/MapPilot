from __future__ import annotations


class _FakeSystem:
    modules = {"GatewayModule": object(), "TeleopModule": object()}
    connections = [object()]

    def get_module(self, name: str):
        if name == "TeleopModule":
            return self.modules[name]
        raise KeyError(name)


def test_print_banner_output_is_gbk_encodable(monkeypatch, capsys, tmp_path):
    import cli.ui as ui

    monkeypatch.setattr(ui, "_local_ip", lambda: "127.0.0.1")

    ui.print_banner(
        "stub",
        {
            "_desc": "Framework testing only",
            "robot": "stub",
            "planner": "astar",
            "enable_semantic": False,
            "enable_gateway": True,
            "gateway_port": 5051,
        },
        _FakeSystem(),
        str(tmp_path),
    )

    capsys.readouterr().out.encode("gbk")
