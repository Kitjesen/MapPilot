"""Tests for FrontierModule, TopologicalMemoryModule, EpisodicMemoryModule,
TaggedLocationsModule -- frontier exploration and memory Module conversions.
"""

import numpy as np
import pytest

from runtime import Module, Out, autoconnect
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import Detection3D, SceneGraph
from memory.modules.episodic_module import EpisodicMemoryModule
from memory.modules.tagged_locations_module import TaggedLocationsModule
from memory.modules.topological_module import TopologicalMemoryModule
from decision.modules.frontier_module import FrontierModule


def _make_odom(x, y, z=0.0):
    return Odometry(pose=Pose(position=Vector3(x, y, z)))

def _make_sg(labels, positions=None):
    objects = []
    for i, label in enumerate(labels):
        pos = positions[i] if positions else (float(i), float(i), 0.0)
        objects.append(Detection3D(id=f"obj_{i}", label=label, confidence=0.9, position=Vector3(*pos)))
    return SceneGraph(objects=objects)

def _collect(module, port_name):
    collected = []
    getattr(module, port_name)._add_callback(collected.append)
    return collected


class TestFrontierModulePorts:
    def test_ports_in_detected(self):
        mod = FrontierModule()
        assert "scene_graph" in mod.ports_in
        assert "odometry" in mod.ports_in
        assert "instruction" in mod.ports_in

    def test_ports_out_detected(self):
        mod = FrontierModule()
        assert "frontier_goal" in mod.ports_out
        assert "frontier_scores" in mod.ports_out

    def test_port_types(self):
        mod = FrontierModule()
        assert mod.ports_in["scene_graph"].msg_type is SceneGraph
        assert mod.ports_in["odometry"].msg_type is Odometry
        assert mod.ports_in["instruction"].msg_type is str
        assert mod.ports_out["frontier_goal"].msg_type is PoseStamped
        assert mod.ports_out["frontier_scores"].msg_type is dict

    def test_layer(self):
        assert FrontierModule._layer == 4
        assert FrontierModule().layer == 4

    def test_scorer_accessible(self):
        assert FrontierModule().scorer is not None

    def test_no_output_without_costmap(self):
        mod = FrontierModule()
        mod.setup()
        goals = _collect(mod, "frontier_goal")
        mod.odometry._deliver(_make_odom(0, 0))
        mod.instruction._deliver("find the kitchen")
        mod.scene_graph._deliver(_make_sg(["chair", "table"]))
        assert len(goals) == 0

    def test_costmap_integration(self):
        mod = FrontierModule(min_frontier_size=1, max_frontiers=5, score_threshold=0.0)
        mod.setup()
        goals = _collect(mod, "frontier_goal")
        scores = _collect(mod, "frontier_scores")
        grid = np.zeros((20, 20), dtype=np.int8)
        grid[:, 10:] = -1
        mod.update_costmap(grid, resolution=1.0, origin_x=0.0, origin_y=0.0)
        mod.odometry._deliver(_make_odom(5, 5))
        mod.instruction._deliver("find the door")
        mod.scene_graph._deliver(_make_sg(["door"], positions=[(9.0, 5.0, 0.0)]))
        assert len(goals) >= 1
        assert len(scores) >= 1
        assert isinstance(goals[0], PoseStamped)
        assert "frontier_count" in scores[0]


class TestTopologicalMemoryModulePorts:
    def test_ports_detected(self):
        mod = TopologicalMemoryModule()
        assert "scene_graph" in mod.ports_in
        assert "odometry" in mod.ports_in
        assert "topo_summary" in mod.ports_out
        assert "topo_graph" in mod.ports_out

    def test_port_types(self):
        mod = TopologicalMemoryModule()
        assert mod.ports_in["scene_graph"].msg_type is SceneGraph
        assert mod.ports_in["odometry"].msg_type is Odometry
        assert mod.ports_out["topo_summary"].msg_type is str
        assert mod.ports_out["topo_graph"].msg_type is dict

    def test_layer(self):
        assert TopologicalMemoryModule._layer == 3
        assert TopologicalMemoryModule().layer == 3


class TestTopologicalMemoryModuleData:
    def test_feed_odom_sg_get_summary(self):
        mod = TopologicalMemoryModule(new_node_distance=1.0)
        mod.setup()
        summaries = _collect(mod, "topo_summary")
        graphs = _collect(mod, "topo_graph")
        mod.odometry._deliver(_make_odom(0, 0))
        mod.scene_graph._deliver(_make_sg(["chair", "table"]))
        assert len(summaries) >= 1
        assert len(graphs) >= 1
        assert isinstance(summaries[-1], str)
        assert isinstance(graphs[-1], dict)

    def test_new_node_creates_summary(self):
        mod = TopologicalMemoryModule(new_node_distance=1.0)
        mod.setup()
        summaries = _collect(mod, "topo_summary")
        mod.odometry._deliver(_make_odom(0, 0))
        mod.scene_graph._deliver(_make_sg(["chair"]))
        mod.odometry._deliver(_make_odom(5, 5))
        mod.scene_graph._deliver(_make_sg(["bed", "lamp"]))
        assert len(summaries) >= 2

    def test_graph_has_nodes(self):
        mod = TopologicalMemoryModule(new_node_distance=1.0)
        mod.setup()
        graphs = _collect(mod, "topo_graph")
        mod.odometry._deliver(_make_odom(0, 0))
        mod.scene_graph._deliver(_make_sg(["desk"]))
        assert len(graphs) >= 1
        assert "nodes" in graphs[-1]
        assert len(graphs[-1]["nodes"]) >= 1

    def test_no_output_without_odom(self):
        mod = TopologicalMemoryModule()
        mod.setup()
        summaries = _collect(mod, "topo_summary")
        mod.scene_graph._deliver(_make_sg(["chair"]))
        assert len(summaries) == 0

    def test_nan_odom_rejected(self):
        mod = TopologicalMemoryModule()
        mod.setup()
        summaries = _collect(mod, "topo_summary")
        mod.odometry._deliver(Odometry(pose=Pose(position=Vector3(float("nan"), 0, 0))))
        mod.scene_graph._deliver(_make_sg(["chair"]))
        assert len(summaries) == 0


class TestEpisodicMemoryModulePorts:
    def test_ports_detected(self):
        mod = EpisodicMemoryModule()
        assert "scene_graph" in mod.ports_in
        assert "odometry" in mod.ports_in
        assert "memory_context" in mod.ports_out

    def test_port_types(self):
        mod = EpisodicMemoryModule()
        assert mod.ports_in["scene_graph"].msg_type is SceneGraph
        assert mod.ports_in["odometry"].msg_type is Odometry
        assert mod.ports_out["memory_context"].msg_type is str

    def test_layer(self):
        assert EpisodicMemoryModule._layer == 3


class TestEpisodicMemoryModuleData:
    def test_feed_data_get_context(self):
        mod = EpisodicMemoryModule(min_distance_m=0.5)
        mod.setup()
        contexts = _collect(mod, "memory_context")
        mod.odometry._deliver(_make_odom(0, 0))
        mod.scene_graph._deliver(_make_sg(["chair", "table"]))
        assert len(contexts) >= 1
        assert isinstance(contexts[-1], str)

    def test_multiple_positions_accumulate(self):
        mod = EpisodicMemoryModule(min_distance_m=0.5)
        mod.setup()
        contexts = _collect(mod, "memory_context")
        mod.odometry._deliver(_make_odom(0, 0))
        mod.scene_graph._deliver(_make_sg(["chair"]))
        mod.odometry._deliver(_make_odom(10, 10))
        mod.scene_graph._deliver(_make_sg(["bed"]))
        assert len(contexts) >= 2
        assert len(mod.memory) >= 2

    def test_no_output_without_odom(self):
        mod = EpisodicMemoryModule()
        mod.setup()
        contexts = _collect(mod, "memory_context")
        mod.scene_graph._deliver(_make_sg(["chair"]))
        assert len(contexts) == 0

    def test_spatial_dedup(self):
        mod = EpisodicMemoryModule(min_distance_m=2.0)
        mod.setup()
        mod.odometry._deliver(_make_odom(0, 0))
        mod.scene_graph._deliver(_make_sg(["chair"]))
        mod.scene_graph._deliver(_make_sg(["table"]))
        assert len(mod.memory) == 1

    def test_context_is_formatted_string(self):
        mod = EpisodicMemoryModule(min_distance_m=0.1)
        mod.setup()
        contexts = _collect(mod, "memory_context")
        mod.odometry._deliver(_make_odom(5, 10))
        mod.scene_graph._deliver(_make_sg(["sofa", "tv"]))
        assert len(contexts) >= 1


class TestTaggedLocationsModulePorts:
    def test_ports_detected(self):
        mod = TaggedLocationsModule()
        assert "odometry" in mod.ports_in
        assert "tag_command" in mod.ports_in
        assert "saved_location" in mod.ports_out
        assert "tag_status" in mod.ports_out

    def test_port_types(self):
        mod = TaggedLocationsModule()
        assert mod.ports_in["odometry"].msg_type is Odometry
        assert mod.ports_in["tag_command"].msg_type is str
        assert mod.ports_out["saved_location"].msg_type is PoseStamped
        assert mod.ports_out["tag_status"].msg_type is str

    def test_layer(self):
        assert TaggedLocationsModule._layer == 3


class TestTaggedLocationsModuleSaveRecall:
    def test_save_and_goto(self):
        mod = TaggedLocationsModule()
        mod.setup()
        statuses = _collect(mod, "tag_status")
        locations = _collect(mod, "saved_location")
        mod.odometry._deliver(_make_odom(10, 20, 0.5))
        mod.tag_command._deliver("save:kitchen")
        assert len(statuses) == 1
        assert statuses[-1] == "saved:kitchen"
        mod.tag_command._deliver("goto:kitchen")
        assert len(locations) == 1
        assert locations[0].x == pytest.approx(10.0)
        assert locations[0].y == pytest.approx(20.0)
        assert statuses[-1] == "recalled:kitchen"

    def test_goto_not_found(self):
        mod = TaggedLocationsModule()
        mod.setup()
        statuses = _collect(mod, "tag_status")
        mod.tag_command._deliver("goto:nonexistent")
        assert statuses[-1] == "not_found:nonexistent"

    def test_remove(self):
        mod = TaggedLocationsModule()
        mod.setup()
        statuses = _collect(mod, "tag_status")
        mod.odometry._deliver(_make_odom(1, 2))
        mod.tag_command._deliver("save:office")
        mod.tag_command._deliver("remove:office")
        assert statuses[-1] == "removed:office"
        mod.tag_command._deliver("goto:office")
        assert statuses[-1] == "not_found:office"

    def test_remove_not_found(self):
        mod = TaggedLocationsModule()
        mod.setup()
        statuses = _collect(mod, "tag_status")
        mod.tag_command._deliver("remove:ghost")
        assert statuses[-1] == "not_found:ghost"

    def test_list(self):
        mod = TaggedLocationsModule()
        mod.setup()
        statuses = _collect(mod, "tag_status")
        mod.odometry._deliver(_make_odom(1, 2))
        mod.tag_command._deliver("save:a")
        mod.tag_command._deliver("save:b")
        mod.tag_command._deliver("list")
        last = statuses[-1]
        assert last.startswith("locations:")
        assert "a" in last
        assert "b" in last

    def test_list_empty(self):
        mod = TaggedLocationsModule()
        mod.setup()
        statuses = _collect(mod, "tag_status")
        mod.tag_command._deliver("list")
        assert statuses[-1] == "locations:"

    def test_save_without_odom(self):
        mod = TaggedLocationsModule()
        mod.setup()
        statuses = _collect(mod, "tag_status")
        mod.tag_command._deliver("save:kitchen")
        assert statuses[-1] == "error:no_odometry"

    def test_unknown_command(self):
        mod = TaggedLocationsModule()
        mod.setup()
        statuses = _collect(mod, "tag_status")
        mod.tag_command._deliver("invalid_cmd")
        assert statuses[-1].startswith("unknown_command:")

    def test_empty_command_ignored(self):
        mod = TaggedLocationsModule()
        mod.setup()
        statuses = _collect(mod, "tag_status")
        mod.tag_command._deliver("")
        assert len(statuses) == 0

    def test_fuzzy_goto(self):
        mod = TaggedLocationsModule()
        mod.setup()
        statuses = _collect(mod, "tag_status")
        locations = _collect(mod, "saved_location")
        mod.odometry._deliver(_make_odom(5, 5))
        mod.tag_command._deliver("save:conference_room")
        mod.tag_command._deliver("goto:conference")
        assert len(locations) == 1
        assert statuses[-1] == "recalled:conference_room"


class TestAutoconnectPerceptionMemory:
    def _make_source_module(self):
        class SourceModule(Module, layer=2):
            scene_graph: Out[SceneGraph]
            odometry: Out[Odometry]
        return SourceModule

    def test_autoconnect_topo_memory(self):
        SourceModule = self._make_source_module()
        system = autoconnect(
            SourceModule.blueprint(),
            TopologicalMemoryModule.blueprint(new_node_distance=1.0),
        ).build()
        system.start()
        src = system.get_module("SourceModule")
        topo = system.get_module("TopologicalMemoryModule")
        summaries = _collect(topo, "topo_summary")
        src.odometry.publish(_make_odom(0, 0))
        src.scene_graph.publish(_make_sg(["chair"]))
        assert len(summaries) >= 1
        system.stop()

    def test_autoconnect_episodic_memory(self):
        SourceModule = self._make_source_module()
        system = autoconnect(
            SourceModule.blueprint(),
            EpisodicMemoryModule.blueprint(min_distance_m=0.1),
        ).build()
        system.start()
        src = system.get_module("SourceModule")
        ep = system.get_module("EpisodicMemoryModule")
        contexts = _collect(ep, "memory_context")
        src.odometry.publish(_make_odom(0, 0))
        src.scene_graph.publish(_make_sg(["desk"]))
        assert len(contexts) >= 1
        system.stop()

    def test_autoconnect_multiple_memory_modules(self):
        SourceModule = self._make_source_module()
        system = autoconnect(
            SourceModule.blueprint(),
            TopologicalMemoryModule.blueprint(new_node_distance=1.0),
            EpisodicMemoryModule.blueprint(min_distance_m=0.1),
        ).build()
        system.start()
        src = system.get_module("SourceModule")
        topo = system.get_module("TopologicalMemoryModule")
        ep = system.get_module("EpisodicMemoryModule")
        topo_summaries = _collect(topo, "topo_summary")
        ep_contexts = _collect(ep, "memory_context")
        src.odometry.publish(_make_odom(3, 4))
        src.scene_graph.publish(_make_sg(["lamp"]))
        total = len(topo_summaries) + len(ep_contexts)
        assert total >= 1
        system.stop()
