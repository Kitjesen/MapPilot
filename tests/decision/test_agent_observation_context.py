"""The Agent sees aligned current observations, not arbitrary cached frames."""

import numpy as np
import pytest

from decision.modules.agent_planner import AgentPlannerModule
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.nav import NavigationState
from runtime.msgs.semantic import Detection3D, SceneGraph
from runtime.msgs.sensor import Image, ImageFormat


@pytest.fixture
def agent(monkeypatch):
    monkeypatch.setattr(AgentPlannerModule, "_init_llm", lambda self: None)
    monkeypatch.setattr("decision.modules.agent_planner.time.time", lambda: 100.0)
    module = AgentPlannerModule(llm_backend="mock")
    module.setup()
    module.robot_pose._deliver(PoseStamped(Pose(Vector3(1, 2, 0.3)), frame_id="map", ts=100.0))
    module.scene_graph._deliver(SceneGraph(frame_id="map", ts=100.0, objects=[
        Detection3D(id="chair", label="chair", position=Vector3(3, 2, 1), ts=100.0)]))
    rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    module.observation_image._deliver(Image(rgb, format=ImageFormat.RGB, ts=100.0))
    yield module
    module.stop()


def test_aligned_color_and_scene_are_exposed_together(agent):
    context = agent._agent_context()
    assert context["camera_available"]
    assert context["visible_objects"] == "chair"
    assert context["camera_image"][0, 0].tolist() == [0, 0, 255]
    agent._observation_image.data[:] = 0
    assert context["camera_image"][0, 0].tolist() == [0, 0, 255]


@pytest.mark.parametrize("change", ["image_skew", "pose_skew", "stale", "wrong_frame", "map"])
def test_unaligned_or_invalidated_observations_are_not_model_images(agent, change, monkeypatch):
    if change == "image_skew":
        agent._observation_image.ts = 99.9
    elif change == "pose_skew":
        agent._current_robot_pose.ts = 99.9
    elif change == "stale":
        monkeypatch.setattr("decision.modules.agent_planner.time.time", lambda: 102.0)
    elif change == "wrong_frame":
        agent._current_scene_graph.frame_id = "odom"
    else:
        agent.navigation_state._deliver(NavigationState(boot_id="nav", map_id="a", sequence=1))
        agent.navigation_state._deliver(NavigationState(boot_id="nav", map_id="b", sequence=2))
    context = agent._agent_context()
    assert context["camera_image"] is None
    assert not context["camera_available"]


def test_retained_old_tracks_are_not_reported_as_current_detections(agent):
    agent._current_scene_graph.objects[0].ts = 98.0
    assert agent._agent_context()["visible_objects"] == "none"
    assert "not visible" in agent._tool_detect_object("chair")
