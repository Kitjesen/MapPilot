from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_generated_topic_views_are_current() -> None:
    result = subprocess.run(
        [sys.executable, "tools/generate_topic_contracts.py", "--check"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_custom_product_graph_still_uses_the_message_catalogue(tmp_path: Path) -> None:
    from lingtu.assembly.graph.loader import load_runtime_graph
    from message.topics import TOPIC_SPECS

    graph = load_runtime_graph(tmp_path)
    assert graph.products == {}
    for topic, spec in TOPIC_SPECS.items():
        assert graph.topic_contracts[topic]["message_type"] == spec.message_type
