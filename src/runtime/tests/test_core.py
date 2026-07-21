"""Tests for lingtu.core — Module / Stream / Transport / Blueprint 框架。

覆盖:
- Module 端口扫描 (In/Out from annotations)
- Out.publish → In._deliver 通过 _add_callback
- LocalTransport pub/sub
- Blueprint auto_wire 按 (name, type) 匹配
- 端到端: Source.Out → Sink.In 数据流
- autoconnect 合并多个 Blueprint
- Module.blueprint() 工厂方法
- 层级依赖校验
- SystemHandle 生命周期
- 消息统计 (msg_count, last_ts, latest)
"""

import json
import logging
import sys
from typing import Any

import pytest

from runtime.blueprint import Blueprint, autoconnect
from runtime.module import Module, rpc, skill
from runtime.msgs.geometry import PoseStamped, Vector3
from runtime.msgs.semantic import Detection3D, GoalResult, SceneGraph
from runtime.runtime_interface import TOPICS
from runtime.stream import In, Out
from runtime.transport.local import LocalTransport

# Windows spawns worker processes slowly (multiprocessing 'spawn' re-imports the
# module tree); allow more time for cross-process replies there.
_WORKER_TIMEOUT = 20.0 if sys.platform == "win32" else 5.0

# ============================================================================
# Test Fixtures — 测试用模块定义
# ============================================================================

class SourceModule(Module, layer=1):
    """模拟感知源: 输出场景图和位姿。"""
    scene_graph: Out[SceneGraph]
    pose: Out[PoseStamped]


class SinkModule(Module, layer=4):
    """模拟规划消费者: 接收场景图和位姿。"""
    scene_graph: In[SceneGraph]
    pose: In[PoseStamped]


class DetectorModule(Module, layer=3):
    """模拟检测器: 输入位姿 → 输出检测结果和场景图。"""
    pose: In[PoseStamped]
    detections: Out[Detection3D]
    scene_graph: Out[SceneGraph]


class PlannerModule(Module, layer=5):
    """模拟规划器: 输入场景图 → 输出目标。"""
    scene_graph: In[SceneGraph]
    goal: Out[GoalResult]


class NoLayerModule(Module):
    """无层级标签的模块。"""
    data: Out[Vector3]


class SetupTrackModule(Module, layer=2):
    """跟踪 setup/start/stop 调用的模块。"""
    output: Out[Vector3]
    input_port: In[Vector3]

    def __init__(self, **config):
        super().__init__(**config)
        self.setup_called = False
        self.start_called = False
        self.stop_called = False

    def setup(self):
        self.setup_called = True
        self.input_port.subscribe(self._on_data)

    def start(self):
        super().start()
        self.start_called = True

    def stop(self):
        super().stop()
        self.stop_called = True

    def _on_data(self, msg: Vector3):
        # Echo: 收到数据后发布
        self.output.publish(msg)


# ============================================================================
# TestOut — 输出端口
# ============================================================================

class TestOut:
    def test_basic_publish(self):
        """Out.publish 调用已注册的回调。"""
        port = Out(SceneGraph, "test")
        received = []
        port._add_callback(received.append)

        sg = SceneGraph()
        port.publish(sg)

        assert len(received) == 1
        assert received[0] is sg  # 零拷贝

    def test_multiple_callbacks(self):
        """Out 支持多个回调 (扇出)。"""
        port = Out(int, "test")
        r1, r2 = [], []
        port._add_callback(r1.append)
        port._add_callback(r2.append)

        port.publish(42)

        assert r1 == [42]
        assert r2 == [42]

    def test_msg_count_and_ts(self):
        """发布后 msg_count 递增，last_ts 更新。"""
        port = Out(int, "counter")
        assert port.msg_count == 0
        assert port.last_ts == 0.0

        port.publish(1)
        assert port.msg_count == 1
        assert port.last_ts > 0

        ts1 = port.last_ts
        port.publish(2)
        assert port.msg_count == 2
        assert port.last_ts >= ts1

    def test_remove_callback(self):
        """_remove_callback 后不再收到消息。"""
        port = Out(int, "test")
        received = []
        port._add_callback(received.append)
        port.publish(1)
        assert len(received) == 1

        port._remove_callback(received.append)
        port.publish(2)
        assert len(received) == 1  # 没有新增

    def test_properties(self):
        """Out 属性正确。"""
        port = Out(SceneGraph, "my_port")
        assert port.name == "my_port"
        assert port.msg_type is SceneGraph
        assert port.callback_count == 0

        port._add_callback(lambda x: None)
        assert port.callback_count == 1

    def test_callback_exception_doesnt_stop_delivery(self):
        """一个回调异常不影响其他回调。"""
        port = Out(int, "test")
        received = []

        def bad_cb(msg):
            raise RuntimeError("boom")

        port._add_callback(bad_cb)
        port._add_callback(received.append)

        port.publish(99)
        assert received == [99]


# ============================================================================
# TestIn — 输入端口
# ============================================================================

class TestIn:
    def test_deliver(self):
        """_deliver 调用已注册的回调。"""
        port = In(SceneGraph, "test")
        received = []
        port.subscribe(received.append)

        sg = SceneGraph()
        port._deliver(sg)

        assert len(received) == 1
        assert received[0] is sg

    def test_latest(self):
        """latest 保存最新消息。"""
        port = In(int, "test")
        assert port.latest is None

        port._deliver(10)
        assert port.latest == 10

        port._deliver(20)
        assert port.latest == 20

    def test_msg_count(self):
        """_deliver 递增 msg_count。"""
        port = In(int, "test")
        assert port.msg_count == 0

        port._deliver(1)
        port._deliver(2)
        assert port.msg_count == 2

    def test_connected_property(self):
        """connected 反映是否有注册回调。"""
        port = In(int, "test")
        assert not port.connected

        port.subscribe(lambda x: None)
        assert port.connected

    def test_deliver_without_subscriber(self):
        """无订阅者时 _deliver 不报错，仍更新 latest。"""
        port = In(int, "test")
        port._deliver(42)  # 不应报错
        assert port.latest == 42
        assert port.msg_count == 1

    def test_callback_exception_doesnt_crash(self):
        """回调异常不影响 In 状态更新。"""
        port = In(int, "test")

        def bad_cb(msg):
            raise ValueError("oops")

        port.subscribe(bad_cb)
        port._deliver(5)
        assert port.msg_count == 1
        assert port.latest == 5


# ============================================================================
# TestOutToIn — Out → In 直连
# ============================================================================

class TestOutToIn:
    def test_direct_connection(self):
        """Out._add_callback(In._deliver) 实现直连。"""
        out = Out(SceneGraph, "scene_graph")
        inp = In(SceneGraph, "scene_graph")

        out._add_callback(inp._deliver)

        sg = SceneGraph(objects=[Detection3D(label="chair")])
        out.publish(sg)

        assert inp.latest is sg
        assert inp.msg_count == 1
        assert out.msg_count == 1

    def test_fan_out(self):
        """一个 Out 连接多个 In。"""
        out = Out(PoseStamped, "pose")
        in1 = In(PoseStamped, "pose")
        in2 = In(PoseStamped, "pose")

        out._add_callback(in1._deliver)
        out._add_callback(in2._deliver)

        p = PoseStamped()
        out.publish(p)

        assert in1.latest is p
        assert in2.latest is p


# ============================================================================
# TestLocalTransport
# ============================================================================

class TestLocalTransport:
    def test_pub_sub(self):
        """基本发布/订阅。"""
        t = LocalTransport()
        received = []
        t.subscribe("topic_a", received.append)

        t.publish("topic_a", 42)
        assert received == [42]

    def test_multiple_subscribers(self):
        """同一 topic 多个订阅者。"""
        t = LocalTransport()
        r1, r2 = [], []
        t.subscribe("t", r1.append)
        t.subscribe("t", r2.append)

        t.publish("t", "hello")
        assert r1 == ["hello"]
        assert r2 == ["hello"]

    def test_isolated_topics(self):
        """不同 topic 互不影响。"""
        t = LocalTransport()
        ra, rb = [], []
        t.subscribe("a", ra.append)
        t.subscribe("b", rb.append)

        t.publish("a", 1)
        t.publish("b", 2)

        assert ra == [1]
        assert rb == [2]

    def test_no_subscriber(self):
        """无订阅者发布不报错。"""
        t = LocalTransport()
        t.publish("empty", "data")  # 不应报错

    def test_close_clears_all(self):
        """close 清空所有订阅。"""
        t = LocalTransport()
        t.subscribe("x", lambda m: None)
        assert t.subscriber_count("x") == 1

        t.close()
        assert t.subscriber_count("x") == 0
        assert t.topics == []

    def test_unsubscribe(self):
        """取消订阅后不再收到消息。"""
        t = LocalTransport()
        received = []
        t.subscribe("t", received.append)
        t.publish("t", 1)
        assert len(received) == 1

        t.unsubscribe("t", received.append)
        t.publish("t", 2)
        assert len(received) == 1

    def test_subscriber_count(self):
        """subscriber_count 正确统计。"""
        t = LocalTransport()
        assert t.subscriber_count("t") == 0

        def cb1(m): return None
        def cb2(m): return None
        t.subscribe("t", cb1)
        t.subscribe("t", cb2)
        assert t.subscriber_count("t") == 2

    def test_topics_list(self):
        """topics 返回活跃 topic。"""
        t = LocalTransport()
        t.subscribe("a", lambda m: None)
        t.subscribe("b", lambda m: None)
        assert set(t.topics) == {"a", "b"}

    def test_transport_protocol(self):
        """LocalTransport 满足 Transport 协议。"""
        from runtime.stream import Transport
        t = LocalTransport()
        assert isinstance(t, Transport)

    def test_callback_exception_isolated(self):
        """一个回调异常不影响其他回调。"""
        t = LocalTransport()
        received = []

        def bad(msg):
            raise RuntimeError("fail")

        t.subscribe("t", bad)
        t.subscribe("t", received.append)

        t.publish("t", "ok")
        assert received == ["ok"]


# ============================================================================
# TestOutWithTransport — Out 绑定 Transport
# ============================================================================

class TestOutWithTransport:
    def test_bind_transport(self):
        """Out 绑定 Transport 后，publish 同时发送到传输层。"""
        transport = LocalTransport()
        out = Out(SceneGraph, "scene_graph")
        out._bind_transport(transport, TOPICS.semantic_scene_graph)

        received_local = []
        received_transport = []
        out._add_callback(received_local.append)
        transport.subscribe(TOPICS.semantic_scene_graph, received_transport.append)

        sg = SceneGraph()
        out.publish(sg)

        assert len(received_local) == 1
        assert len(received_transport) == 1
        assert received_local[0] is sg
        assert received_transport[0] is sg


# ============================================================================
# TestModule — 模块端口扫描
# ============================================================================

class TestModule:
    def test_port_scanning_out(self):
        """Module 自动扫描 Out[T] 标注。"""
        mod = SourceModule()
        assert "scene_graph" in mod.ports_out
        assert "pose" in mod.ports_out
        assert mod.ports_out["scene_graph"].msg_type is SceneGraph
        assert mod.ports_out["pose"].msg_type is PoseStamped

    def test_port_scanning_in(self):
        """Module 自动扫描 In[T] 标注。"""
        mod = SinkModule()
        assert "scene_graph" in mod.ports_in
        assert "pose" in mod.ports_in
        assert mod.ports_in["scene_graph"].msg_type is SceneGraph
        assert mod.ports_in["pose"].msg_type is PoseStamped

    def test_mixed_ports(self):
        """Module 同时有 In 和 Out。"""
        mod = DetectorModule()
        assert "pose" in mod.ports_in
        assert "detections" in mod.ports_out
        assert "scene_graph" in mod.ports_out

    def test_port_instances_are_attrs(self):
        """端口实例可通过属性访问。"""
        mod = SourceModule()
        assert isinstance(mod.scene_graph, Out)
        assert isinstance(mod.pose, Out)
        assert mod.scene_graph.name == "scene_graph"

    def test_layer_attribute(self):
        """layer 参数正确设置。"""
        assert SourceModule._layer == 1
        assert SinkModule._layer == 4
        assert NoLayerModule._layer is None

        mod = SourceModule()
        assert mod.layer == 1

    def test_config_passthrough(self):
        """配置参数正确传递。"""
        mod = SourceModule(camera_id=0, fps=30)
        assert mod._config == {"camera_id": 0, "fps": 30}

    def test_lifecycle(self):
        """Module 生命周期: setup → start → stop。"""
        mod = SourceModule()
        assert not mod.running

        mod.setup()
        mod.start()
        assert mod.running

        mod.stop()
        assert not mod.running

    def test_all_ports(self):
        """all_ports 返回所有端口。"""
        mod = DetectorModule()
        all_p = mod.all_ports
        assert "pose" in all_p
        assert "detections" in all_p
        assert "scene_graph" in all_p

    def test_port_summary(self):
        """port_summary 返回结构化信息。"""
        mod = SourceModule()
        summary = mod.port_summary()
        assert summary["module"] == "SourceModule"
        assert summary["layer"] == 1
        assert "scene_graph" in summary["ports_out"]
        assert summary["ports_out"]["scene_graph"]["type"] == "SceneGraph"

    def test_repr(self):
        """Module repr 包含端口名。"""
        mod = SourceModule()
        r = repr(mod)
        assert "SourceModule" in r
        assert "scene_graph" in r

    def test_blueprint_factory(self):
        """Module.blueprint() 返回 Blueprint。"""
        bp = SourceModule.blueprint(camera_id=0)
        assert isinstance(bp, Blueprint)


# ============================================================================
# TestBlueprint — 蓝图编排
# ============================================================================

class TestBlueprint:
    def test_add_and_build(self):
        """add + build 创建模块实例。"""
        system = Blueprint().add(SourceModule).build()
        assert "SourceModule" in system.modules

    def test_duplicate_module_name_raises(self):
        """重复模块名报错。"""
        with pytest.raises(ValueError, match="already contains"):
            Blueprint().add(SourceModule).add(SourceModule)

    def test_alias(self):
        """alias 允许同类型多实例。"""
        system = (
            Blueprint()
            .add(SourceModule, alias="source_1")
            .add(SourceModule, alias="source_2")
            .build()
        )
        assert "source_1" in system.modules
        assert "source_2" in system.modules

    def test_explicit_wire(self):
        """wire 显式连接 Out → In。"""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .wire("SourceModule", "scene_graph", "SinkModule", "scene_graph")
            .build()
        )

        src = system.get_module("SourceModule")
        sink = system.get_module("SinkModule")

        sg = SceneGraph(objects=[Detection3D(label="table")])
        src.scene_graph.publish(sg)

        assert sink.scene_graph.latest is sg

    def test_wire_unknown_module_raises(self):
        """wire 引用不存在的模块报错。"""
        bp = Blueprint().add(SourceModule)
        with pytest.raises(ValueError, match="unknown"):
            bp.wire("SourceModule", "scene_graph", "NonExistent", "scene_graph").build()

    def test_wire_unknown_port_raises(self):
        """wire 引用不存在的端口报错。"""
        bp = Blueprint().add(SourceModule).add(SinkModule)
        with pytest.raises(ValueError, match="no Out port"):
            bp.wire("SourceModule", "nonexistent", "SinkModule", "scene_graph").build()

    def test_wire_type_mismatch_raises(self):
        """wire 类型不匹配报错。"""
        bp = Blueprint().add(SourceModule).add(SinkModule)
        with pytest.raises(TypeError, match="type mismatch"):
            bp.wire("SourceModule", "scene_graph", "SinkModule", "pose").build()

    def test_auto_wire_by_name_and_type(self):
        """auto_wire 按 (name, msg_type) 自动匹配。"""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .auto_wire()
            .build()
        )

        src = system.get_module("SourceModule")
        sink = system.get_module("SinkModule")

        # scene_graph: Out[SceneGraph] ↔ In[SceneGraph] — 同名同类型，应连接
        sg = SceneGraph()
        src.scene_graph.publish(sg)
        assert sink.scene_graph.latest is sg

        # pose: Out[PoseStamped] ↔ In[PoseStamped] — 同名同类型，应连接
        p = PoseStamped()
        src.pose.publish(p)
        assert sink.pose.latest is p

    def test_auto_wire_typed_out_to_any_in(self):
        """typing.Any input accepts a typed output with the same port name."""

        class TypedSource(Module, layer=1):
            data: Out[Vector3]

        class AnySink(Module, layer=2):
            data: In[Any]

        system = (
            Blueprint()
            .add(TypedSource)
            .add(AnySink)
            .auto_wire()
            .build()
        )

        msg = Vector3(1.0, 2.0, 3.0)
        system.get_module("TypedSource").data.publish(msg)
        assert system.get_module("AnySink").data.latest is msg

    def test_auto_wire_any_out_to_typed_in(self):
        """typing.Any output accepts a typed input with the same port name."""

        class AnySource(Module, layer=1):
            data: Out[Any]

        class TypedSink(Module, layer=2):
            data: In[Vector3]

        system = (
            Blueprint()
            .add(AnySource)
            .add(TypedSink)
            .auto_wire()
            .build()
        )

        msg = Vector3(4.0, 5.0, 6.0)
        system.get_module("AnySource").data.publish(msg)
        assert system.get_module("TypedSink").data.latest is msg

    def test_auto_wire_prefers_exact_type_over_any(self):
        """Exact type matches stay preferred when Any outputs are also present."""

        class TypedSource(Module, layer=1):
            data: Out[Vector3]

        class AnySource(Module, layer=1):
            data: Out[Any]

        class TypedSink(Module, layer=2):
            data: In[Vector3]

        system = (
            Blueprint()
            .add(TypedSource, alias="typed")
            .add(AnySource, alias="any_source")
            .add(TypedSink)
            .auto_wire()
            .build()
        )

        any_msg = Vector3(9.0, 9.0, 9.0)
        typed_msg = Vector3(1.0, 2.0, 3.0)
        sink = system.get_module("TypedSink")

        system.get_module("any_source").data.publish(any_msg)
        assert sink.data.latest is None

        system.get_module("typed").data.publish(typed_msg)
        assert sink.data.latest is typed_msg

    def test_auto_wire_no_self_connect(self):
        """auto_wire 不会自连接 (同一模块的 Out→In)。"""
        system = (
            Blueprint()
            .add(DetectorModule)
            .auto_wire()
            .build()
        )
        det = system.get_module("DetectorModule")
        # DetectorModule 有 In[PoseStamped] 和 Out[SceneGraph]
        # 名称不同或同模块，不应自连接
        det.scene_graph.publish(SceneGraph())
        # 没有其他模块的 In[SceneGraph]，所以不会被连到自己

    def test_auto_wire_skips_explicit(self):
        """auto_wire 跳过已有显式连接的 In。"""

        system = (
            Blueprint()
            .add(SourceModule, alias="src1")
            .add(SourceModule, alias="src2")
            .add(SinkModule)
            .wire("src1", "scene_graph", "SinkModule", "scene_graph")
            .auto_wire()
            .build()
        )

        sink = system.get_module("SinkModule")
        src1 = system.get_module("src1")
        system.get_module("src2")

        # scene_graph 已显式连到 src1
        sg1 = SceneGraph(objects=[Detection3D(label="from_src1")])
        src1.scene_graph.publish(sg1)
        assert sink.scene_graph.latest is sg1

    def test_worker_build_failure_shuts_down_coordinator(self, monkeypatch):
        """Worker-mode build failures release the started coordinator."""

        class WorkerModule(Module, layer=1):
            _run_in_worker = True
            data: Out[Vector3]

        instances = []

        class FakeCoordinator:
            def __init__(self, n_workers):
                self.n_workers = n_workers
                self.shutdown_called = False
                instances.append(self)

            def start(self):
                pass

            def deploy(self, *args, **kwargs):
                raise RuntimeError("deploy failed")

            def shutdown(self):
                self.shutdown_called = True

        import runtime.coordinator as coordinator_mod

        monkeypatch.setattr(coordinator_mod, "ModuleCoordinator", FakeCoordinator)
        with pytest.raises(RuntimeError, match="deploy failed"):
            Blueprint().add(WorkerModule).build(n_workers=1)

        assert instances
        assert instances[0].shutdown_called

    def test_build_with_custom_transport(self):
        """build 接受自定义 Transport。"""
        transport = LocalTransport()
        system = Blueprint().add(SourceModule).build(transport=transport)

        src = system.get_module("SourceModule")
        # Transport 应被绑定到 Out 端口
        assert src.scene_graph._transport is transport

    def test_connections_recorded(self):
        """build 后 connections 记录所有连接。"""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .wire("SourceModule", "scene_graph", "SinkModule", "scene_graph")
            .build()
        )
        conns = system.connections
        assert len(conns) >= 1
        assert ("SourceModule", "scene_graph", "SinkModule", "scene_graph") in conns

    def test_merge(self):
        """merge 合并两个 Blueprint。"""
        bp1 = Blueprint().add(SourceModule)
        bp2 = Blueprint().add(SinkModule)
        bp1.merge(bp2)

        system = bp1.auto_wire().build()
        assert "SourceModule" in system.modules
        assert "SinkModule" in system.modules

    def test_merge_conflict_raises(self):
        """merge 名称冲突报错。"""
        bp1 = Blueprint().add(SourceModule)
        bp2 = Blueprint().add(SourceModule)
        with pytest.raises(ValueError, match="exists in both"):
            bp1.merge(bp2)


# ============================================================================
# TestAutoconnect — 多蓝图合并
# ============================================================================

class TestAutoconnect:
    def test_basic_autoconnect(self):
        """autoconnect 合并并自动连接。"""
        bp = autoconnect(
            SourceModule.blueprint(),
            SinkModule.blueprint(),
        )
        system = bp.build()

        src = system.get_module("SourceModule")
        sink = system.get_module("SinkModule")

        sg = SceneGraph(objects=[Detection3D(label="desk")])
        src.scene_graph.publish(sg)
        assert sink.scene_graph.latest is sg

    def test_autoconnect_three_modules(self):
        """autoconnect 三个模块的链路。"""
        bp = autoconnect(
            SourceModule.blueprint(),
            DetectorModule.blueprint(),
            PlannerModule.blueprint(),
        )
        system = bp.build()

        src = system.get_module("SourceModule")
        system.get_module("PlannerModule")
        detector = system.get_module("DetectorModule")

        # SourceModule.pose → DetectorModule.pose (同名同类型)
        p = PoseStamped()
        src.pose.publish(p)
        assert detector.pose.latest is p


# ============================================================================
# TestSystemHandle — 运行时管理
# ============================================================================

class TestSystemHandle:
    def test_start_stop(self):
        """start/stop 调用所有模块的生命周期方法。"""
        system = (
            Blueprint()
            .add(SetupTrackModule, alias="tracker")
            .build()
        )
        mod = system.get_module("tracker")

        assert not mod.setup_called
        assert not mod.start_called

        system.start()
        assert mod.setup_called
        assert mod.start_called
        assert system.started

        system.stop()
        assert mod.stop_called
        assert not system.started

    def test_double_start_no_error(self):
        """重复 start 不报错。"""
        system = Blueprint().add(SourceModule).build()
        system.start()
        system.start()  # 不应报错
        system.stop()

    def test_stop_without_start(self):
        """未 start 直接 stop 不报错。"""
        system = Blueprint().add(SourceModule).build()
        system.stop()  # 不应报错

    def test_get_module(self):
        """get_module 按名称获取。"""
        system = Blueprint().add(SourceModule).build()
        mod = system.get_module("SourceModule")
        assert isinstance(mod, SourceModule)

    def test_get_module_unknown_raises(self):
        """get_module 未知名称报错。"""
        system = Blueprint().add(SourceModule).build()
        with pytest.raises(KeyError, match="Unknown module"):
            system.get_module("NonExistent")

    def test_health(self):
        """health 返回系统健康信息。"""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .auto_wire()
            .build()
        )
        system.start()

        src = system.get_module("SourceModule")
        src.scene_graph.publish(SceneGraph())

        h = system.health()
        assert h["started"] is True
        assert h["module_count"] == 2
        assert h["connection_count"] >= 1
        assert h["total_messages_out"] >= 1
        assert "SourceModule" in h["modules"]

        system.stop()

    def test_startup_order_respects_deps(self):
        """启动顺序: 上游模块 (低层) 先于下游模块 (高层)。"""
        system = (
            Blueprint()
            .add(SinkModule)     # L4
            .add(SourceModule)   # L1
            .auto_wire()
            .build()
        )
        order = system.health()["startup_order"]
        src_idx = order.index("SourceModule")
        sink_idx = order.index("SinkModule")
        assert src_idx < sink_idx  # L1 先于 L4

    def test_repr(self):
        """SystemHandle repr。"""
        system = Blueprint().add(SourceModule).build()
        r = repr(system)
        assert "stopped" in r
        system.start()
        r = repr(system)
        assert "running" in r
        system.stop()


# ============================================================================
# TestEndToEnd — 端到端数据流
# ============================================================================

class TestEndToEnd:
    def test_source_to_sink_full_pipeline(self):
        """完整流水线: Source → Sink，使用真实消息类型。"""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .auto_wire()
            .build()
        )
        system.start()

        src = system.get_module("SourceModule")
        sink = system.get_module("SinkModule")

        # 发送带内容的 SceneGraph
        sg = SceneGraph(
            objects=[
                Detection3D(id="obj_1", label="chair", confidence=0.9,
                            position=Vector3(1.0, 2.0, 0.5)),
                Detection3D(id="obj_2", label="table", confidence=0.85,
                            position=Vector3(3.0, 4.0, 0.7)),
            ],
            frame_id="map",
        )
        src.scene_graph.publish(sg)

        # 验证到达
        assert sink.scene_graph.latest is sg
        assert sink.scene_graph.latest.objects[0].label == "chair"
        assert len(sink.scene_graph.latest.objects) == 2

        # 发送 PoseStamped
        pose = PoseStamped(frame_id="map")
        src.pose.publish(pose)
        assert sink.pose.latest is pose

        system.stop()

    def test_echo_module_data_flow(self):
        """SetupTrackModule 收到数据后 echo 输出。"""
        class VecSource(Module):
            output: Out[Vector3]

        class VecSink(Module):
            input_port: In[Vector3]

        system = (
            Blueprint()
            .add(VecSource, alias="src")
            .add(SetupTrackModule, alias="echo")
            .add(VecSink, alias="sink")
            .wire("src", "output", "echo", "input_port")
            .wire("echo", "output", "sink", "input_port")
            .build()
        )
        system.start()

        src = system.get_module("src")
        sink = system.get_module("sink")

        v = Vector3(1.0, 2.0, 3.0)
        src.output.publish(v)

        # echo 应该将消息转发到 sink
        assert sink.input_port.latest is v

        system.stop()

    def test_multi_hop_pipeline(self):
        """多跳流水线: Source → Detector → Planner (混合 auto_wire + 显式 wire)。

        SourceModule 和 DetectorModule 都有 Out[SceneGraph] 名为 scene_graph，
        auto_wire 会因歧义跳过 PlannerModule.scene_graph，因此需要显式连接。
        """
        system = (
            Blueprint()
            .add(SourceModule)
            .add(DetectorModule)
            .add(PlannerModule)
            # 显式: Detector.scene_graph → Planner.scene_graph (解决歧义)
            .wire("DetectorModule", "scene_graph", "PlannerModule", "scene_graph")
            # auto_wire 处理无歧义的: Source.pose → Detector.pose
            .auto_wire()
            .build()
        )
        system.start()

        src = system.get_module("SourceModule")
        detector = system.get_module("DetectorModule")
        planner = system.get_module("PlannerModule")

        # Source.pose → Detector.pose (auto-wired, 唯一匹配)
        p = PoseStamped(frame_id="odom")
        src.pose.publish(p)
        assert detector.pose.latest is p

        # Detector.scene_graph → Planner.scene_graph (显式连接)
        sg = SceneGraph(objects=[Detection3D(label="door")])
        detector.scene_graph.publish(sg)
        assert planner.scene_graph.latest is sg

        system.stop()

    def test_msg_counts_accumulate(self):
        """消息计数在整个流水线中正确累积。"""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .auto_wire()
            .build()
        )
        system.start()

        src = system.get_module("SourceModule")
        sink = system.get_module("SinkModule")

        for _i in range(10):
            src.scene_graph.publish(SceneGraph())

        assert src.scene_graph.msg_count == 10
        assert sink.scene_graph.msg_count == 10

        system.stop()


# ============================================================================
# TestLayerDeps — 层级依赖
# ============================================================================

class TestLayerDeps:
    def test_no_layer_module_no_violation(self):
        """无层级标签模块不触发校验。"""
        system = (
            Blueprint()
            .add(NoLayerModule)
            .add(SinkModule)
            .auto_wire()
            .build()
        )
        h = system.health()
        assert h["layer_violations"] == []

    def test_feedback_loop_does_not_warn_and_preserves_fallback_order(self, caplog):
        class PlantModule(Module, layer=1):
            cmd: In[Vector3]
            state: Out[Vector3]

        class ControllerModule(Module, layer=2):
            state: In[Vector3]
            cmd: Out[Vector3]

        with caplog.at_level(logging.WARNING):
            system = (
                Blueprint()
                .add(ControllerModule)
                .add(PlantModule)
                .wire("PlantModule", "state", "ControllerModule", "state")
                .wire("ControllerModule", "cmd", "PlantModule", "cmd")
                .build()
            )

        h = system.health()
        assert h["layer_violations"] == []
        assert h["startup_order"] == ["ControllerModule", "PlantModule"]
        assert not any(
            "Layer dependency violation" in record.getMessage()
            for record in caplog.records
        )

    def test_normal_layer_flow(self):
        """正常层级流 (L1 → L4) 无违规。"""
        system = (
            Blueprint()
            .add(SourceModule)    # L1
            .add(SinkModule)      # L4
            .auto_wire()
            .build()
        )
        h = system.health()
        assert h["layer_violations"] == []


# ============================================================================
# TestOnSystemModules — on_system_modules hook
# ============================================================================

class TestOnSystemModules:
    """on_system_modules is called by Blueprint.build() after all modules
    are instantiated and wired, allowing cross-module discovery."""

    def test_hook_called_during_build(self):
        """on_system_modules is called on every module during build()."""
        received: dict = {}

        class ObserverModule(Module):
            data: Out[Vector3]

            def on_system_modules(self, modules):
                received.update(modules)

        system = Blueprint().add(ObserverModule).build()
        assert "ObserverModule" in received
        assert received["ObserverModule"] is system.get_module("ObserverModule")

    def test_hook_receives_all_modules(self):
        """Each module's on_system_modules receives the full module dict."""
        snapshots: list = []

        class ModA(Module):
            scene_graph: Out[SceneGraph]

            def on_system_modules(self, modules):
                snapshots.append(set(modules.keys()))

        class ModB(Module):
            scene_graph: In[SceneGraph]

            def on_system_modules(self, modules):
                snapshots.append(set(modules.keys()))

        Blueprint().add(ModA).add(ModB).auto_wire().build()

        assert len(snapshots) == 2
        for snap in snapshots:
            assert snap == {"ModA", "ModB"}

    def test_hook_can_discover_rpc_methods(self):
        """on_system_modules can inspect @rpc methods on peer modules."""
        from runtime.module import rpc

        found_rpcs: list = []

        class ServiceModule(Module):
            data: Out[Vector3]

            @rpc
            def ping(self) -> str:
                return "pong"

        class ClientModule(Module):
            data: In[Vector3]

            def on_system_modules(self, modules):
                svc = modules.get("ServiceModule")
                if svc is not None:
                    found_rpcs.extend(svc.rpcs.keys())

        Blueprint().add(ServiceModule).add(ClientModule).auto_wire().build()
        assert "ping" in found_rpcs

    def test_hook_called_before_system_handle_returned(self):
        """on_system_modules is called before build() returns, so the
        SystemHandle's modules dict is fully populated by the time
        start() is called."""
        call_order: list = []

        class WatchModule(Module):
            data: Out[Vector3]

            def on_system_modules(self, modules):
                call_order.append("hook")

            def setup(self):
                call_order.append("setup")

        system = Blueprint().add(WatchModule).build()
        system.start()
        # hook is called during build(), before setup() during start()
        assert call_order.index("hook") < call_order.index("setup")
        system.stop()

    def test_default_hook_is_noop(self):
        """Default on_system_modules does nothing — no exception raised."""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .auto_wire()
            .build()
        )
        # If we got here without exception, the noop default worked fine.
        assert len(system.modules) == 2


# ============================================================================
# TestIoDynamicPorts — io() dynamic port creation
# ============================================================================

class TestIoDynamicPorts:
    """io() creates In/Out ports at runtime and registers them in
    _ports_in / _ports_out so Blueprint wiring and cleanup both work."""

    def test_io_creates_out_port(self):
        """io(..., Out, T) creates an Out port registered in ports_out."""
        mod = SourceModule()
        port = mod.io("dynamic_out", Out, Vector3)

        assert isinstance(port, Out)
        assert "dynamic_out" in mod.ports_out
        assert mod.dynamic_out is port
        assert port.msg_type is Vector3
        assert port.name == "dynamic_out"

    def test_io_creates_in_port(self):
        """io(..., In, T) creates an In port registered in ports_in."""
        mod = SinkModule()
        port = mod.io("dynamic_in", In, Vector3)

        assert isinstance(port, In)
        assert "dynamic_in" in mod.ports_in
        assert mod.dynamic_in is port
        assert port.msg_type is Vector3
        assert port.name == "dynamic_in"

    def test_io_without_msg_type_uses_Any(self):
        """io() with no msg_type defaults to typing.Any."""
        from typing import Any
        mod = SourceModule()
        port = mod.io("untyped_out", Out)
        assert port.msg_type is Any

    def test_io_invalid_direction_raises(self):
        """io() with an invalid direction raises ValueError."""
        mod = SourceModule()
        with pytest.raises(ValueError, match="direction must be In or Out"):
            mod.io("bad", str, Vector3)

    def test_io_port_is_functional(self):
        """Dynamic Out port publishes to subscribers correctly."""
        mod = SourceModule()
        port = mod.io("dyn_vec", Out, Vector3)

        received = []
        port._add_callback(received.append)

        v = Vector3(1.0, 2.0, 3.0)
        port.publish(v)

        assert received == [v]
        assert port.msg_count == 1

    def test_io_in_port_receives_messages(self):
        """Dynamic In port receives messages via _deliver."""
        mod = SinkModule()
        port = mod.io("dyn_in", In, Vector3)

        received = []
        port.subscribe(received.append)

        v = Vector3(4.0, 5.0, 6.0)
        port._deliver(v)

        assert received == [v]
        assert port.latest is v

    def test_io_dynamic_port_wirable(self):
        """Dynamic ports created in __init__ can be wired via Blueprint.

        Ports must exist at build() time so Blueprint can resolve them.
        io() is called from __init__ (or before build()) for this use case.
        """

        class DynSource(Module):
            def __init__(self, **config):
                super().__init__(**config)
                self.io("dyn_vec", Out, Vector3)

        class DynSink(Module):
            def __init__(self, **config):
                super().__init__(**config)
                self.io("dyn_vec", In, Vector3)

        system = (
            Blueprint()
            .add(DynSource)
            .add(DynSink)
            .wire("DynSource", "dyn_vec", "DynSink", "dyn_vec")
            .build()
        )
        system.start()

        src = system.get_module("DynSource")
        sink = system.get_module("DynSink")

        v = Vector3(7.0, 8.0, 9.0)
        src.dyn_vec.publish(v)
        assert sink.dyn_vec.latest is v

        system.stop()

    def test_io_cleanup_on_stop(self):
        """Dynamic ports are cleaned up when the module stops."""
        mod = SourceModule()
        out_port = mod.io("dyn_out", Out, Vector3)
        in_port = mod.io("dyn_in", In, Vector3)

        received = []
        out_port._add_callback(received.append)
        in_port.subscribe(received.append)

        mod.stop()

        # After stop, callbacks and subscribers are cleared
        assert out_port.callback_count == 0
        assert in_port._callback is None

# ============================================================================
# TestBackpressure — In[T] delivery policies
# ============================================================================

class TestBackpressure:
    """Test In[T] backpressure with 'latest' policy."""

    def test_default_policy_is_all(self):
        """Default policy delivers every message."""
        inp = In(int, "val")
        assert inp.policy == "all"
        assert inp.drop_count == 0

    def test_set_policy_latest(self):
        inp = In(int, "val")
        inp.set_policy("latest")
        assert inp.policy == "latest"

    def test_set_policy_invalid_raises(self):
        inp = In(int, "val")
        with pytest.raises(ValueError, match="Unknown policy"):
            inp.set_policy("random")

    def test_all_policy_delivers_every_message(self):
        """With 'all' policy, every _deliver() triggers callback."""
        inp = In(int, "val")
        received = []
        inp.subscribe(received.append)
        for i in range(10):
            inp._deliver(i)
        assert received == list(range(10))
        assert inp.drop_count == 0

    def test_latest_policy_drops_when_busy(self):
        """With 'latest' policy, messages are dropped if callback is running."""
        import threading

        inp = In(int, "val")
        inp.set_policy("latest")

        barrier = threading.Event()
        received = []

        def slow_callback(msg):
            received.append(msg)
            if msg == 1:
                barrier.wait(timeout=2.0)  # block on first real message

        inp.subscribe(slow_callback)

        # Deliver from a background thread so it blocks
        t = threading.Thread(target=inp._deliver, args=(1,))
        t.start()
        import time
        time.sleep(0.02)  # let it enter callback

        # These should be dropped (callback busy)
        inp._deliver(2)
        inp._deliver(3)
        inp._deliver(4)

        # Release the blocked callback
        barrier.set()
        t.join(timeout=2.0)

        # Only message 1 was processed, 2-4 dropped
        assert received == [1]
        assert inp.drop_count == 3
        assert inp.latest == 4  # latest always updated
        assert inp.msg_count == 4  # all counted

    def test_latest_policy_normal_when_not_busy(self):
        """With 'latest' policy, sequential messages all deliver normally."""
        inp = In(int, "val")
        inp.set_policy("latest")
        received = []
        inp.subscribe(received.append)

        for i in range(5):
            inp._deliver(i)

        assert received == list(range(5))
        assert inp.drop_count == 0

    def test_latest_always_updates(self):
        """Even when dropping, .latest property always has the newest value."""
        inp = In(str, "val")
        inp.set_policy("latest")
        inp._deliver("a")
        inp._deliver("b")
        inp._deliver("c")
        assert inp.latest == "c"
        assert inp.msg_count == 3

    def test_drop_count_in_repr(self):
        """Drop count appears in repr when non-zero."""
        inp = In(int, "val")
        inp.set_policy("latest")
        assert "latest" in repr(inp)
        assert "dropped" not in repr(inp)


# ============================================================================
# TestRpc — @rpc decorator and Module.rpcs / call_rpc
# ============================================================================

class TestRpc:
    """Tests for the @rpc decorator, Module.rpcs property, and call_rpc method."""

    def test_rpc_marks_method(self):
        """@rpc sets __rpc__ = True on the decorated function."""
        def my_fn(self):
            pass

        decorated = rpc(my_fn)
        assert getattr(decorated, '__rpc__', False) is True

    def test_rpc_preserves_function(self):
        """@rpc returns the same callable (no wrapper)."""
        def my_fn(self):
            return 42

        decorated = rpc(my_fn)
        assert decorated is my_fn

    def test_rpcs_discovers_decorated_methods(self):
        """Module.rpcs returns all @rpc-decorated methods."""
        class MyModule(Module):
            @rpc
            def navigate_to(self, x: float, y: float) -> bool:
                return True

            @rpc
            def get_status(self) -> str:
                return "ok"

        mod = MyModule()
        discovered = mod.rpcs
        assert "navigate_to" in discovered
        assert "get_status" in discovered

    def test_rpcs_excludes_non_rpc_methods(self):
        """Module.rpcs does not include methods without @rpc."""
        class MyModule(Module):
            @rpc
            def rpc_method(self) -> str:
                return "rpc"

            def plain_method(self) -> str:
                return "plain"

        mod = MyModule()
        discovered = mod.rpcs
        assert "rpc_method" in discovered
        assert "plain_method" not in discovered

    def test_rpcs_excludes_private_methods(self):
        """Module.rpcs never returns names starting with undersruntime."""
        class MyModule(Module):
            @rpc
            def public_rpc(self) -> int:
                return 1

        mod = MyModule()
        for name in mod.rpcs:
            assert not name.startswith('_'), f"Private name leaked: {name}"

    def test_rpcs_empty_when_no_decorators(self):
        """Module.rpcs returns empty dict when no @rpc methods exist."""
        mod = SourceModule()
        assert mod.rpcs == {}

    def test_call_rpc_invokes_method(self):
        """call_rpc calls the named @rpc method and returns its result."""
        class MyModule(Module):
            @rpc
            def add(self, a: int, b: int) -> int:
                return a + b

        mod = MyModule()
        result = mod.call_rpc("add", a=3, b=4)
        assert result == 7

    def test_call_rpc_unknown_method_raises(self):
        """call_rpc raises ValueError for unknown method names."""
        mod = SourceModule()
        with pytest.raises(ValueError, match="RPC method 'nonexistent' not found"):
            mod.call_rpc("nonexistent")

    def test_call_rpc_non_rpc_method_raises(self):
        """call_rpc raises ValueError for methods that exist but lack @rpc."""
        class MyModule(Module):
            def plain(self) -> str:
                return "plain"

        mod = MyModule()
        with pytest.raises(ValueError, match="RPC method 'plain' not found"):
            mod.call_rpc("plain")

    def test_rpcs_returns_bound_methods(self):
        """rpcs values are bound methods callable without explicit self."""
        class MyModule(Module):
            @rpc
            def ping(self) -> str:
                return "pong"

        mod = MyModule()
        ping_fn = mod.rpcs["ping"]
        assert ping_fn() == "pong"

    def test_rpc_importable_from_core(self):
        """rpc is exported from the core package."""
        from runtime import rpc as core_rpc
        assert core_rpc is rpc


# ============================================================================
# TestWorker — Worker process and WorkerManager
# ============================================================================

# ============================================================================
# TestSkillSchema - @skill schema generation
# ============================================================================

class TestSkillSchema:
    def test_optional_pep604_numeric_arg_stays_numeric_and_optional(self):
        class MyModule(Module):
            @skill
            def navigate_to(self, x: float, y: float, z: float | None = None) -> str:
                return "ok"

        schema = json.loads(MyModule().get_skill_infos()[0].args_schema)

        assert schema["properties"]["z"]["type"] == "number"
        assert "z" not in schema["required"]


class _SkillModDoThing(Module):
    """Module with a @skill method — must be module-level for multiprocessing pickle."""
    @skill
    def do_thing(self, x: int) -> str:
        """Do a thing with x."""
        return str(x)


class _SkillModPing(Module):
    """Module with a @skill method — must be module-level for multiprocessing pickle."""
    @skill
    def ping(self) -> str:
        """Ping the module."""
        return "pong"


class _WorkerTestModule(Module):
    """Minimal module used to verify cross-process RPC calls."""
    value: Out[int]

    def __init__(self, initial=0, **kw):
        super().__init__(**kw)
        self._val = initial

    @rpc
    def get_value(self) -> int:
        return self._val

    @rpc
    def set_value(self, v: int) -> None:
        self._val = v


class TestWorker:
    """Tests for Worker and WorkerManager IPC."""

    def test_worker_list_empty(self):
        """Fresh worker returns empty module list."""
        import multiprocessing

        from runtime.worker import Worker

        cmd_q: multiprocessing.Queue = multiprocessing.Queue()
        resp_q: multiprocessing.Queue = multiprocessing.Queue()
        w = Worker(worker_id="test", cmd_queue=cmd_q, resp_queue=resp_q)
        w.start()

        cmd_q.put(("LIST",))
        resp = resp_q.get(timeout=_WORKER_TIMEOUT)
        assert resp[0] == "RESULT"
        assert resp[1] == []

        cmd_q.put(("SHUTDOWN",))
        resp_q.get(timeout=_WORKER_TIMEOUT)
        w.join(timeout=_WORKER_TIMEOUT)

    def test_worker_deploy_module(self):
        """DEPLOY instantiates a module inside the worker."""
        import multiprocessing

        from runtime.worker import Worker

        cmd_q: multiprocessing.Queue = multiprocessing.Queue()
        resp_q: multiprocessing.Queue = multiprocessing.Queue()
        w = Worker(worker_id="test", cmd_queue=cmd_q, resp_queue=resp_q)
        w.start()

        cmd_q.put(("DEPLOY", _WorkerTestModule, "mod1", (), {"initial": 42}))
        resp = resp_q.get(timeout=_WORKER_TIMEOUT)
        assert resp[0] == "OK"
        assert resp[1] == "mod1"

        # Confirm the module appears in the list
        cmd_q.put(("LIST",))
        resp = resp_q.get(timeout=_WORKER_TIMEOUT)
        assert "mod1" in resp[1]

        cmd_q.put(("SHUTDOWN",))
        resp_q.get(timeout=_WORKER_TIMEOUT)
        w.join(timeout=_WORKER_TIMEOUT)

    def test_worker_rpc_call_across_process(self):
        """RPC call crosses the process boundary and returns the correct value."""
        import multiprocessing

        from runtime.worker import Worker

        cmd_q: multiprocessing.Queue = multiprocessing.Queue()
        resp_q: multiprocessing.Queue = multiprocessing.Queue()
        w = Worker(worker_id="test", cmd_queue=cmd_q, resp_queue=resp_q)
        w.start()

        # Deploy with initial=7
        cmd_q.put(("DEPLOY", _WorkerTestModule, "mod1", (), {"initial": 7}))
        resp_q.get(timeout=_WORKER_TIMEOUT)

        # get_value should return 7
        cmd_q.put(("RPC_CALL", "mod1", "get_value", (), {}))
        resp = resp_q.get(timeout=_WORKER_TIMEOUT)
        assert resp[0] == "RESULT"
        assert resp[1] == 7

        # set_value then get_value
        cmd_q.put(("RPC_CALL", "mod1", "set_value", (), {"v": 99}))
        resp_q.get(timeout=_WORKER_TIMEOUT)

        cmd_q.put(("RPC_CALL", "mod1", "get_value", (), {}))
        resp = resp_q.get(timeout=_WORKER_TIMEOUT)
        assert resp[1] == 99

        cmd_q.put(("SHUTDOWN",))
        resp_q.get(timeout=_WORKER_TIMEOUT)
        w.join(timeout=_WORKER_TIMEOUT)

    def test_worker_shutdown_graceful(self):
        """SHUTDOWN stops all modules and the worker process exits cleanly."""
        import multiprocessing

        from runtime.worker import Worker

        cmd_q: multiprocessing.Queue = multiprocessing.Queue()
        resp_q: multiprocessing.Queue = multiprocessing.Queue()
        w = Worker(worker_id="test", cmd_queue=cmd_q, resp_queue=resp_q)
        w.start()
        assert w.is_alive()

        cmd_q.put(("DEPLOY", _WorkerTestModule, "m", (), {}))
        resp_q.get(timeout=_WORKER_TIMEOUT)

        cmd_q.put(("SHUTDOWN",))
        resp = resp_q.get(timeout=_WORKER_TIMEOUT)
        assert resp[0] == "OK"

        w.join(timeout=_WORKER_TIMEOUT)
        assert not w.is_alive()

    def test_worker_manager_multiple_workers(self):
        """WorkerManager starts multiple workers and routes commands correctly."""
        from runtime.worker_manager import WorkerManager

        mgr = WorkerManager(n_workers=2)
        mgr.start()
        assert mgr.is_started
        assert mgr.n_workers == 2

        mgr.deploy(0, _WorkerTestModule, "m0", kwargs={"initial": 10})
        mgr.deploy(1, _WorkerTestModule, "m1", kwargs={"initial": 20})

        assert mgr.list_modules(0) == ["m0"]
        assert mgr.list_modules(1) == ["m1"]

        assert mgr.rpc_call(0, "m0", "get_value") == 10
        assert mgr.rpc_call(1, "m1", "get_value") == 20

        mgr.shutdown()
        assert not mgr.is_started

    def test_worker_manager_setup_start_stop_lifecycle(self):
        """WorkerManager setup/start_module/stop_module calls module lifecycle."""
        from runtime.worker_manager import WorkerManager

        mgr = WorkerManager(n_workers=1)
        mgr.start()

        mgr.deploy(0, _WorkerTestModule, "mod", kwargs={"initial": 5})
        mgr.setup(0, "mod")
        mgr.start_module(0, "mod")

        # Module is running — get_value still works
        assert mgr.rpc_call(0, "mod", "get_value") == 5

        mgr.stop_module(0, "mod")
        mgr.shutdown()

    def test_worker_get_skills_command(self):
        """GET_SKILLS returns serialized SkillInfo list for a deployed module."""
        import multiprocessing

        from runtime.worker import Worker

        cmd_q: multiprocessing.Queue = multiprocessing.Queue()
        resp_q: multiprocessing.Queue = multiprocessing.Queue()
        w = Worker(worker_id="sk", cmd_queue=cmd_q, resp_queue=resp_q)
        w.start()

        cmd_q.put(("DEPLOY", _SkillModDoThing, "skmod", (), {}))
        resp_q.get(timeout=_WORKER_TIMEOUT)

        cmd_q.put(("GET_SKILLS", "skmod"))
        resp = resp_q.get(timeout=_WORKER_TIMEOUT)
        assert resp[0] == "RESULT"
        skills = resp[1]
        assert isinstance(skills, list)
        assert len(skills) == 1
        assert skills[0]["func_name"] == "do_thing"
        assert skills[0]["class_name"] == "_SkillModDoThing"
        assert "args_schema" in skills[0]

        cmd_q.put(("SHUTDOWN",))
        resp_q.get(timeout=_WORKER_TIMEOUT)
        w.join(timeout=_WORKER_TIMEOUT)

    def test_worker_manager_get_skills(self):
        """WorkerManager.get_skills() aggregates skill info from a worker module."""
        from runtime.worker_manager import WorkerManager

        mgr = WorkerManager(n_workers=1)
        mgr.start()
        mgr.deploy(0, _SkillModPing, "sm", kwargs={})

        skills = mgr.get_skills(0, "sm")
        assert isinstance(skills, list)
        assert len(skills) == 1
        assert skills[0]["func_name"] == "ping"
        assert skills[0]["class_name"] == "_SkillModPing"

        mgr.shutdown()
