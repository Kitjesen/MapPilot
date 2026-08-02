from __future__ import annotations

import builtins
from pathlib import Path


def _native_map_entry(
    name: str,
    *,
    octomap: str | None = None,
    pcd: str | None = None,
) -> dict:
    artifacts = []
    if octomap:
        artifacts.append({"type": "OCTOMAP_3D", "uri": octomap})
    if pcd:
        artifacts.append({"type": "POINTCLOUD", "uri": pcd})
    return {
        "name": name,
        "record": {"artifacts": artifacts},
    }


def test_cli_saved_map_discovery_uses_maps_service_not_directory_scanning(
    monkeypatch,
) -> None:
    import cli.runtime_extra as runtime_extra

    class FakeMapsService:
        def __init__(self, root: str):
            assert root == "maps-root"

        def list_maps(self):
            return {
                "success": True,
                "active": "ready",
                "maps": [
                    _native_map_entry(
                        "ready",
                        octomap="maps-root/ready/octomap.ot",
                        pcd="maps-root/ready/map.pcd",
                    ),
                    _native_map_entry(
                        "source_only",
                        pcd="maps-root/source_only/map.pcd",
                    ),
                ],
            }

        def get_bundle(self, name: str, capability: str):
            paths = {
                ("ready", "navigation_safety_3d"): "maps-root/ready/octomap.ot",
                ("ready", "source_pointcloud"): "maps-root/ready/map.pcd",
                ("source_only", "source_pointcloud"): "maps-root/source_only/map.pcd",
            }
            uri = paths.get((name, capability))
            return {
                "success": uri is not None,
                "artifact": {"uri": uri} if uri else None,
            }

    monkeypatch.setattr(
        runtime_extra,
        "NativeMapsService",
        FakeMapsService,
        raising=False,
    )

    def fail_directory_scan(*_args, **_kwargs):
        raise AssertionError("CLI must query maps through the maps service")

    monkeypatch.setattr(runtime_extra.os, "listdir", fail_directory_scan)

    assert runtime_extra._scan_maps("maps-root") == [("ready", "maps-root/ready/octomap.ot", True, True)]


def test_cli_saved_map_selection_activates_through_maps_service(
    monkeypatch,
) -> None:
    import cli.runtime_extra as runtime_extra

    activated: list[tuple[str, bool]] = []

    class FakeMapsService:
        def __init__(self, root: str):
            assert root == "maps-root"

        def list_maps(self):
            return {
                "success": True,
                "active": "",
                "maps": [
                    _native_map_entry(
                        "ready",
                        octomap="maps-root/ready/octomap.ot",
                        pcd="maps-root/ready/map.pcd",
                    )
                ],
            }

        def get_bundle(self, name: str, capability: str):
            paths = {
                ("ready", "navigation_safety_3d"): "maps-root/ready/octomap.ot",
                ("ready", "source_pointcloud"): "maps-root/ready/map.pcd",
            }
            uri = paths.get((name, capability))
            return {
                "success": uri is not None,
                "artifact": {"uri": uri} if uri else None,
            }

        def set_active_map(self, name: str, *, strict: bool = True):
            activated.append((name, strict))
            return {"success": True, "active": name}

    monkeypatch.setattr(
        runtime_extra,
        "NativeMapsService",
        FakeMapsService,
        raising=False,
    )
    monkeypatch.setattr(runtime_extra.sys.stdin, "isatty", lambda: True)
    monkeypatch.setattr(builtins, "input", lambda _prompt="": "1")
    monkeypatch.setattr(runtime_extra.os.path, "isfile", lambda _path: False)

    def fail_symlink(*_args, **_kwargs):
        raise AssertionError("CLI must not mutate the active-map symlink")

    monkeypatch.setattr(runtime_extra.os, "symlink", fail_symlink)
    cfg: dict = {}

    runtime_extra._select_map_interactive(cfg, "maps-root")

    assert activated == [("ready", True)]
    assert cfg["planner_map"] == "maps-root/ready/octomap.ot"


def test_repl_refuses_map_mutation_when_maps_service_is_unavailable(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from cli.repl import LingTuREPL

    class EmptySystem:
        def get_module(self, _name: str):
            raise KeyError(_name)

    map_dir = tmp_path / "maps"
    victim = map_dir / "keep_me"
    victim.mkdir(parents=True)
    (victim / "map.pcd").write_text("pcd", encoding="utf-8")

    import cli.runtime_extra as runtime_extra

    monkeypatch.setattr(runtime_extra, "_default_map_dir", lambda: str(map_dir))
    repl = LingTuREPL(EmptySystem(), {})

    repl._map_cmd({"action": "delete", "name": "keep_me"})

    assert victim.is_dir()
    assert "maps.service" in capsys.readouterr().out
