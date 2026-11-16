"""Real pipes and protocol reader, with an offline process in place of Codex."""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from decision.llm import codex_app_server


@pytest.fixture
def codex_server(tmp_path, monkeypatch):
    scenario = {}
    log = tmp_path / "protocol.jsonl"
    script = Path(__file__).parent / "fixtures/codex_app_server.py"
    popen = codex_app_server.subprocess.Popen

    def launch(command, **kwargs):
        assert command == ["fixture-codex", "app-server", "--stdio"]
        kwargs["env"] = {**kwargs["env"], "CODEX_FIXTURE_SCENARIO": json.dumps(scenario),
                         "CODEX_FIXTURE_LOG": str(log)}
        return popen([sys.executable, str(script)], **kwargs)

    monkeypatch.setattr(codex_app_server.subprocess, "Popen", launch)
    server = codex_app_server.CodexAppServer(command="fixture-codex")
    monkeypatch.setattr(codex_app_server, "app_server", server)
    yield SimpleNamespace(server=server, scenario=scenario,
                          messages=lambda: [json.loads(line) for line in log.read_text().splitlines()])
    server.close()
