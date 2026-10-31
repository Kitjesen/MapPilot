"""Native DDS optical identity comes from the selected UE camera plan."""

from dataclasses import replace

import pytest
from tests.lingtu.test_measurement_compilation import baseline_plan as baseline_plan
from tests.lingtu.test_measurement_compilation import compilation as compilation


def select_unreal(snapshot):
    snapshot["session"]["runtime"]["mode"] = "unreal"
    for kind in ("rgb", "depth"):
        for stream in snapshot["sensor_plan"]["streams"][kind]:
            stream.update(owner="visual", source="unreal_camera", transport="camera_shm",
                          frame_id="thunder_01/camera1_link")


def test_compiler_binds_exact_optical_frame_on_selected_and_available_native_endpoint(compilation):
    baseline, snapshot, compile_plan = compilation
    select_unreal(snapshot)
    plan = compile_plan()
    previous = next(p for p in baseline.processes if "camera" in p.provides)
    selected = next(p for p in plan.processes if p.name == previous.name)
    available = next(p for p in plan.available_processes if p.name == previous.name)
    assert selected.command.argv == available.command.argv == (
        *previous.command.argv, "--camera-frame", "thunder_01/camera1_link",
    )
    assert plan.host_config == baseline.host_config


@pytest.mark.parametrize("argv", [
    ("--camera-frame", "wrong_frame"), ("--camera-frame=wrong_frame",),
    ("--camera-frame",),
    ("--camera-frame", "thunder_01/camera1_link", "--camera-frame", "thunder_01/camera1_link"),
])
def test_conflicting_or_ambiguous_endpoint_frame_fails_before_launch(compilation, argv):
    from lingtu.assembly.compile_plan import _bind_unreal_camera

    baseline, snapshot, _ = compilation
    select_unreal(snapshot)
    def change(process):
        return replace(process, command=replace(process.command, argv=(*process.command.argv, *argv))) if (
            "camera" in process.provides
        ) else process
    with pytest.raises(ValueError, match=r"camera.*frame"):
        _bind_unreal_camera(snapshot, tuple(map(change, baseline.processes)),
                            tuple(map(change, baseline.available_processes)))


def test_different_rgb_depth_optical_frames_fail_closed(compilation):
    _, snapshot, compile_plan = compilation
    select_unreal(snapshot)
    snapshot["sensor_plan"]["streams"]["depth"][0]["frame_id"] = "another/optical"
    with pytest.raises(ValueError, match="optical frame"):
        compile_plan()
