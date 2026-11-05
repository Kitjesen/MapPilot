from pathlib import Path


NAV_ENDPOINT = Path(__file__).resolve().parents[3] / "src/nav/cpp/endpoint/nav"


def test_navd_does_not_automatically_connect_stair_segments_or_profiles() -> None:
    sources = "\n".join(
        path.read_text(encoding="utf-8")
        for pattern in ("*.cpp", "*.hpp")
        for path in NAV_ENDPOINT.rglob(pattern)
        if path.name != "stairs.hpp"
    )
    assert "readStairFlights(" not in sources
    assert "resolveStairSegments(" not in sources
    assert "MotionProfileRequestOutput" not in sources
    assert "writeMotionProfileRequest" not in sources
    assert "kMotionProfileRequest" not in sources


def test_route_activation_still_reaches_executor() -> None:
    source = (NAV_ENDPOINT / "main.cpp").read_text(encoding="utf-8")
    activation = source.split("goal_plan_actions.activate_path =", 1)[1].split(
        "const auto write_start", 1
    )[0]
    assert "executor.setRoute(std::move(route))" in activation
    assert "route.stairs" not in activation
