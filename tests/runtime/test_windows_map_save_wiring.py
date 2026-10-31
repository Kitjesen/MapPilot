"""Offline map-save process wiring; no Product or map identity is fabricated."""

from lingtu.assembly.graph import load_runtime_graph, resolve_processes
from lingtu.assembly.graph.processes import ProcessArtifact


def test_windows_mapd_exposes_existing_query_endpoint():
    graph = load_runtime_graph()
    windows = graph.envs["sim"]["backends"]["mujoco"]["processes"]["map_runtime"]["platforms"]["windows"]
    assert "--disable-query" not in windows["argv"]
    assert windows["argv"][0] == "build/maps-windows/Release/mapd.exe"


def test_resolved_map_command_keeps_query_without_inheriting_shell_pgo(monkeypatch):
    monkeypatch.setattr("lingtu.assembly.graph.processes._host_process_platform", lambda: "windows")
    monkeypatch.setattr(ProcessArtifact, "from_repository_path", classmethod(lambda cls, root, path: cls(str(path))))
    monkeypatch.setenv("LINGTU_PGO_BIN", "unrelated-shell-pgo.exe")
    selected, _, _, _ = resolve_processes("map", "sim", env_config={"backend": "mujoco"})
    command = next(p.command for p in selected if p.name == "map_runtime")
    assert dict(command.env).get("LINGTU_PGO_BIN") != "unrelated-shell-pgo.exe"
    assert "--disable-query" not in command.argv


def test_windows_map_save_uses_declared_current_optimizer(monkeypatch):
    monkeypatch.setattr("lingtu.assembly.graph.processes._host_process_platform", lambda: "windows")
    monkeypatch.setattr(ProcessArtifact, "from_repository_path", classmethod(lambda cls, root, path: cls(str(path))))
    selected, _, _, _ = resolve_processes("map", "sim", env_config={"backend": "mujoco"})
    command = next(p.command for p in selected if p.name == "map_runtime")
    expected = "build/map_optimization/Release/lt_pgo.exe"
    assert dict(command.env)["LINGTU_PGO_BIN"] == expected
    assert expected in {artifact.path for artifact in command.dependencies}
    assert "PATH" not in dict(command.env)


def test_linux_map_command_is_unchanged():
    linux = load_runtime_graph().envs["sim"]["backends"]["mujoco"]["processes"]["map_runtime"]["platforms"]["linux"]
    assert linux["argv"] == ["build/maps/mapd", "--domain-id", "231"]
    assert linux["env"] == {"LINGTU_DDS_DOMAIN_ID": "231"}
    assert "dependencies" not in linux
