from __future__ import annotations

from pathlib import Path
from shutil import copytree

import pytest
import yaml

from lingtu.assembly.compile_plan import compile_run_plan, host_builder_from_run_plan
from lingtu.assembly.graph import RuntimeGraph, load_runtime_graph
from lingtu.run_plan import RunPlan


@pytest.fixture
def model_graph_root(tmp_path: Path) -> Path:
    root = tmp_path / "system"
    copytree(load_runtime_graph().root, root)
    return root


def _compile(graph: RuntimeGraph, product: str, **kwargs) -> RunPlan:
    return compile_run_plan(product, "real", robot="unitree/go2", graph=graph, **kwargs)


def test_shared_model_settings_are_snapshotted_in_each_plan(
    model_graph_root: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    graph = load_runtime_graph(model_graph_root)
    models_path = model_graph_root / "models.yaml"
    models = yaml.safe_load(models_path.read_text(encoding="utf-8"))
    baseline = _compile(graph, "nav", product_variant="semantic")
    snapshot = baseline.write(tmp_path / "run-plan.json")
    secret = "test-secret-should-never-be-serialized"
    monkeypatch.setenv(models["kimi"]["api_key_env"], secret)
    models["kimi"]["model"] = "updated-kimi-model"
    models_path.write_text(yaml.safe_dump(models), encoding="utf-8")

    for product, variant in (("nav", "semantic"), ("tracking", None)):
        plan = _compile(graph, product, product_variant=variant)
        assert plan.host_config["llm_model"] == "updated-kimi-model"
        assert plan.host_config["llm_api_key_env"] == models["kimi"]["api_key_env"]
        path = plan.write(tmp_path / f"{product}.json")
        assert secret not in path.read_text(encoding="utf-8")

    models_path.unlink()
    restored = RunPlan.load(snapshot)
    assert restored.host_config == baseline.host_config
    builder = host_builder_from_run_plan(restored)
    llm = next(entry for entry in builder._entries if entry.name == "LLMModule")
    assert llm.config["model"] == baseline.host_config["llm_model"]
    assert "llm_model" not in _compile(graph, "nav").host_config
    with pytest.raises(ValueError, match=r"models\.yaml"):
        _compile(graph, "nav", product_variant="semantic")


def test_inspection_model_settings_reach_all_llm_consumers(model_graph_root: Path) -> None:
    models = yaml.safe_load((model_graph_root / "models.yaml").read_text(encoding="utf-8"))
    qwen = models["qwen"]
    plan = _compile(load_runtime_graph(model_graph_root), "inspection")
    entries = {entry.name: entry.config for entry in host_builder_from_run_plan(plan)._entries}
    for entry, prefix in (
        ("LLMModule", ""),
        ("AgentPlannerModule", "llm_"),
        ("SemanticPlannerModule", "llm_"),
    ):
        config = entries[entry]
        assert config[f"{prefix}backend"] == "qwen"
        for key in ("model", "base_url", "api_key_env", "timeout_sec"):
            assert config[f"{prefix}{key}"] == qwen[key]


def test_model_defaults_follow_final_backend_and_yield_to_explicit_fields(model_graph_root: Path) -> None:
    models = yaml.safe_load((model_graph_root / "models.yaml").read_text(encoding="utf-8"))
    graph = load_runtime_graph(model_graph_root)
    qwen = _compile(graph, "tracking", overrides={"llm": "qwen"})
    for key in ("model", "base_url", "api_key_env", "timeout_sec"):
        assert qwen.host_config[f"llm_{key}"] == models["qwen"][key]

    graph.products["tracking"]["host"]["llm_model"] = "product-model"
    assert _compile(graph, "tracking").host_config["llm_model"] == "product-model"
    graph.envs["real"]["host_config"]["llm_model"] = "env-model"
    assert _compile(graph, "tracking").host_config["llm_model"] == "env-model"
    plan = _compile(graph, "tracking", overrides={"llm_model": "caller-model"})
    assert plan.host_config["llm_model"] == "caller-model"
    with pytest.raises(ValueError, match="kimmi"):
        _compile(graph, "tracking", overrides={"llm": "kimmi"})


@pytest.mark.parametrize(
    "connection, error_field",
    [
        ({"mode": "typo", "base_url": "", "api_key_env": "EXAMPLE_API_KEY", "timeout_sec": 30}, "model"),
        ({"model": "kimi", "base_url": "", "api_key_env": "EXAMPLE_API_KEY", "timeout_sec": "thirty"}, "timeout_sec"),
    ],
)
def test_malformed_model_settings_are_rejected(
    model_graph_root: Path, connection: dict[str, str | int], error_field: str
) -> None:
    models_path = model_graph_root / "models.yaml"
    models = yaml.safe_load(models_path.read_text(encoding="utf-8"))
    models["kimi"] = connection
    models_path.write_text(yaml.safe_dump(models), encoding="utf-8")
    with pytest.raises(ValueError, match=rf"models\.yaml.*kimi.*{error_field}"):
        _compile(load_runtime_graph(model_graph_root), "tracking")
