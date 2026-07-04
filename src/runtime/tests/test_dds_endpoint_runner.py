from __future__ import annotations

import json


def test_dds_endpoint_runner_fail_closed_on_missing_python_dds(monkeypatch, capsys) -> None:
    from runtime.adapters.dds import endpoint_runner

    def _raise_missing(_args):
        raise ImportError("cyclonedds-python is not installed")

    monkeypatch.setattr(endpoint_runner, "run_endpoint_service", _raise_missing)

    code = endpoint_runner.main(
        [
            "--fail-closed-on-missing-python-dds",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)
    assert code == 0
    assert payload["ok"] is True
    assert payload["mode"] == "disabled_missing_python_dds"
    assert payload["field_ready"] is False


def test_dds_endpoint_runner_does_not_fail_closed_other_errors(monkeypatch, capsys) -> None:
    from runtime.adapters.dds import endpoint_runner

    def _raise_runtime(_args):
        raise RuntimeError("brainstem unavailable")

    monkeypatch.setattr(endpoint_runner, "run_endpoint_service", _raise_runtime)

    code = endpoint_runner.main(
        [
            "--fail-closed-on-missing-python-dds",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)
    assert code == 1
    assert payload["ok"] is False
    assert payload["error_type"] == "RuntimeError"
