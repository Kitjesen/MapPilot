from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.sim
def test_lingtu_gazebo_industrial_demo_has_single_visible_entrypoint():
    script = ROOT / "sim/scripts/launch_lingtu_gazebo_industrial_demo.sh"
    rviz = ROOT / "sim/planning/lingtu_industrial_demo.rviz"
    world = ROOT / "sim/worlds/gazebo/lingtu_gazebo_industrial_park.sdf"

    text = script.read_text(encoding="utf-8")
    rviz_text = rviz.read_text(encoding="utf-8")

    assert script.exists()
    assert rviz.exists()
    assert world.exists()
    assert "start [--rviz] [--gate]" in text
    assert "LINGTU_GAZEBO_DEMO_SKIP_PRESTOP" in text
    assert "lingtu_gazebo_industrial_park.sdf" in text
    assert "spawn_x:=-6.0" in text
    assert "ROS_DOMAIN_ID" in text
    assert "ROS_DOMAIN_ID\" -ge 232" in text
    assert "gazebo_frontier_exploration_smoke.py" in text
    assert "--frontier-safe-distance-m 0.80" in text
    assert "--max-local-path-unknown-ratio 1.0" in text
    assert "[i]gn gazebo server" in text
    assert "[i]gn gazebo gui" in text
    assert "sim_navigation.launch.py" in text
    assert "rviz2 -d" in text
    assert "/nav/registered_cloud" in rviz_text
    assert "/nav/map_cloud" in rviz_text
    assert "/nav/global_path" in rviz_text
    assert "/nav/local_path" in rviz_text
    assert "DebugCumulativeMapCloud" in rviz_text
    assert "Enabled: false" in rviz_text
