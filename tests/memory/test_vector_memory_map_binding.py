"""Spatial memory needs map provenance independently of encoder readiness."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

from memory.modules.vector_memory_module import VectorMemoryModule
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.nav import NavigationState
from runtime.msgs.semantic import Detection3D, SceneGraph


@pytest.fixture
def memory(monkeypatch):
    monkeypatch.setattr("memory.modules.vector_memory_module.time.time", lambda: 1000.0)
    monkeypatch.setattr(VectorMemoryModule, "_init_encoder", lambda self: None)
    monkeypatch.setattr(VectorMemoryModule, "_init_store", lambda self: None)
    module = VectorMemoryModule(store_interval=0)
    module._encoder_type = "clip"
    module._encoder = SimpleNamespace(encode_text=lambda texts: np.array([[1.0, 0.0]]))
    module._embedding_dim = 2
    module.setup()
    yield module
    module.stop()


def context(module, *, map_id="office", epoch=2, sequence=1):
    module.navigation_state._deliver(NavigationState(
        boot_id="nav-1", sequence=sequence, map_id=map_id, map_content_epoch=epoch,
    ))


def observation(module, *, frame_id="map", pose_ts=1000.0, scene_ts=1000.0):
    module.robot_pose._deliver(PoseStamped(
        Pose(Vector3(5.0, 6.0, 0.3)), frame_id=frame_id, ts=pose_ts,
    ))
    module.scene_graph._deliver(SceneGraph(
        frame_id="map", ts=scene_ts,
        objects=[Detection3D(id="chair", label="chair", ts=scene_ts)],
    ))


def test_encoder_readiness_does_not_make_unbound_memory_navigable(memory):
    memory._robot_xy = (100.0, 200.0)
    memory._store_snapshot(["chair"])
    result = json.loads(memory.query_location("chair"))
    assert result["found"]
    assert result["semantic_encoder_ready"]
    assert result["navigable"] is False


def test_bound_snapshot_uses_the_observation_pose_not_raw_odometry(memory):
    context(memory)
    memory._robot_xy = (100.0, 200.0)
    observation(memory)
    result = json.loads(memory.query_location("chair"))
    assert result["navigable"] is True
    assert result["best"]["x"] == 5.0
    assert result["best"]["z"] == 0.3
    assert result["best"]["map_id"] == "office"
    assert result["best"]["map_content_epoch"] == 2
    assert result["best"]["frame_id"] == "map"


@pytest.mark.parametrize("change", [{"map_id": "yard"}, {"epoch": 3}])
def test_old_map_memory_remains_searchable_but_cannot_drive(memory, change):
    context(memory)
    observation(memory)
    context(memory, sequence=2, **change)
    result = json.loads(memory.query_location("chair"))
    assert result["found"]
    assert result["navigable"] is False
    assert result["best"]["navigable"] is False


@pytest.mark.parametrize("options", [
    {"frame_id": "odom"}, {"pose_ts": 999.0},
    {"pose_ts": 970.0, "scene_ts": 970.0},
])
def test_unsynchronized_or_stale_pose_is_query_only(memory, options):
    context(memory)
    observation(memory, **options)
    assert json.loads(memory.query_location("chair"))["navigable"] is False


def test_new_runtime_does_not_overwrite_previous_chroma_snapshots():
    rows = {}
    def upsert(*, ids, metadatas, **kwargs):
        rows.update(zip(ids, metadatas))
    for position in (1.0, 2.0):
        module = VectorMemoryModule()
        module._use_chromadb = True
        module._collection = SimpleNamespace(upsert=upsert)
        module._robot_xy = (position, 0.0)
        module._store_snapshot(["chair"])
    assert len(rows) == 2
    assert sorted(row["x"] for row in rows.values()) == [1.0, 2.0]


def test_chroma_query_preserves_map_provenance(memory):
    context(memory)
    observation(memory)
    record = dict(memory._np_metadata[0])
    memory._collection = SimpleNamespace(
        count=lambda: 1,
        query=lambda **kwargs: {"ids": [["persisted"]], "metadatas": [[record]], "distances": [[0.0]]},
    )
    memory._use_chromadb = True
    result = json.loads(memory.query_location("chair"))
    assert result["navigable"] is True
    assert result["best"]["map_id"] == "office"
    context(memory, epoch=3, sequence=2)
    assert json.loads(memory.query_location("chair"))["navigable"] is False


def test_mcp_search_reads_the_concrete_memory_response(memory):
    from gateway.mcp_server import MCPServerModule

    server = MCPServerModule()
    server.on_system_modules({"VectorMemoryModule": memory})
    memory._store_snapshot(["chair"])
    result = json.loads(server.query_memory("chair"))
    assert result["count"] == 1
    assert result["results"][0]["query_only"] is True
    assert "position" not in result["results"][0]

    context(memory)
    observation(memory)
    result = json.loads(server.query_memory("chair"))
    assert result["results"][0]["navigable"] is True
    assert result["results"][0]["position"] == [5.0, 6.0, 0.3]
    assert result["results"][0]["map_id"] == "office"
