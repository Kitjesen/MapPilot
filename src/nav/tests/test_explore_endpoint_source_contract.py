from pathlib import Path

_ENDPOINT_SOURCE = Path("src/explore/cpp/endpoint/main.cpp")


def test_native_explore_dispatch_preserves_policy_goal_yaw() -> None:
    source = _ENDPOINT_SOURCE.read_text(encoding="utf-8")
    target_dispatch = source.split("const Pose2D target{", 1)[1].split("};", 1)[0]

    assert "last_decision.goal_x" in target_dispatch
    assert "last_decision.goal_y" in target_dispatch
    assert "last_decision.goal_yaw" in target_dispatch
