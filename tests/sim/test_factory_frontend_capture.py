"""Editor screenshot requests may pump Slate callbacks recursively."""

import runpy
import sys
import types
from pathlib import Path


def test_screenshot_is_requested_once_even_during_reentrant_slate_tick(monkeypatch, tmp_path):
    clock = [0.0]
    callbacks = []
    calls = []
    world = types.SimpleNamespace(get_path_name=lambda: "/Game/Maps/Demonstration")
    editor = types.SimpleNamespace(set_level_viewport_camera_info=lambda *_: None, get_editor_world=lambda: world)
    fake = types.ModuleType("unreal")
    fake.Vector = lambda x, y, z: types.SimpleNamespace(x=x, y=y, z=z)
    fake.Rotator = lambda **kwargs: types.SimpleNamespace(**kwargs)
    fake.UnrealEditorSubsystem = object
    fake.get_editor_subsystem = lambda _: editor
    fake.SystemLibrary = types.SimpleNamespace(execute_console_command=lambda *_: None)
    fake.register_slate_post_tick_callback = lambda callback: callbacks.append(callback) or 1
    fake.log = lambda *_: None

    def take(*_args, **_kwargs):
        calls.append(True)
        assert len(calls) == 1, "reentrant Slate tick requested another screenshot"
        callbacks[0](.016)
        return types.SimpleNamespace(is_task_done=lambda: False)

    fake.AutomationLibrary = types.SimpleNamespace(take_high_res_screenshot=take)
    monkeypatch.setitem(sys.modules, "unreal", fake)
    monkeypatch.setenv("LINGTU_FACTORY_CAPTURE_DIR", str(tmp_path))
    monkeypatch.setattr("time.monotonic", lambda: clock[0])
    source = Path(__file__).resolve().parents[2] / "sim/tools/assets/capture_factory_frontend.py"
    runpy.run_path(str(source))
    clock[0] = 20.0
    callbacks[0](.016)
    assert len(calls) == 1
