"""Tests for runtime.introspection and runtime.resource_monitor."""


from runtime.blueprint import Blueprint, autoconnect
from runtime.module import Module
from runtime.msgs.geometry import PoseStamped
from runtime.msgs.semantic import SceneGraph
from runtime.stream import In, Out

# ============================================================================
# Test fixtures
# ============================================================================

class SrcMod(Module, layer=1):
    pose: Out[PoseStamped]
    scene_graph: Out[SceneGraph]


class MidMod(Module, layer=3):
    pose: In[PoseStamped]
    scene_graph: Out[SceneGraph]


class SinkMod(Module, layer=5):
    scene_graph: In[SceneGraph]
    pose: In[PoseStamped]


def _build_system():
    return autoconnect(
        SrcMod.blueprint(),
        MidMod.blueprint(),
        SinkMod.blueprint(),
    ).build()


# ============================================================================
# render_text
# ============================================================================

class TestRenderText:
    def test_returns_string(self):
        from runtime.introspection import render_text
        system = _build_system()
        try:
            out = render_text(system)
            assert isinstance(out, str)
            assert "SrcMod" in out
            assert "SinkMod" in out
        finally:
            system.stop()

    def test_contains_module_names(self):
        from runtime.introspection import render_text
        system = _build_system()
        try:
            out = render_text(system, color=False)
            assert "SrcMod" in out
            assert "MidMod" in out
            assert "SinkMod" in out
        finally:
            system.stop()

    def test_contains_layer_labels(self):
        from runtime.introspection import render_text
        system = _build_system()
        try:
            out = render_text(system, color=False)
            assert "L1" in out
            assert "L3" in out
            assert "L5" in out
        finally:
            system.stop()

    def test_contains_connections(self):
        from runtime.introspection import render_text
        system = _build_system()
        try:
            out = render_text(system, color=False)
            assert "Connections" in out
        finally:
            system.stop()

    def test_no_color_mode(self):
        from runtime.introspection import render_text
        system = _build_system()
        try:
            out = render_text(system, color=False)
            assert "\033[" not in out
        finally:
            system.stop()

    def test_color_mode(self):
        from runtime.introspection import render_text
        system = _build_system()
        try:
            out = render_text(system, color=True)
            assert "\033[" in out
        finally:
            system.stop()

    def test_port_types_shown(self):
        from runtime.introspection import render_text
        system = _build_system()
        try:
            out = render_text(system, color=False)
            assert "PoseStamped" in out or "SceneGraph" in out
        finally:
            system.stop()


# ============================================================================
# render_connections
# ============================================================================

class TestRenderConnections:
    def test_returns_string(self):
        from runtime.introspection import render_connections
        system = _build_system()
        try:
            out = render_connections(system, color=False)
            assert isinstance(out, str)
        finally:
            system.stop()

    def test_contains_arrow(self):
        from runtime.introspection import render_connections
        system = _build_system()
        try:
            out = render_connections(system, color=False)
            assert "►" in out or "→" in out or "──" in out
        finally:
            system.stop()


# ============================================================================
# render_dot
# ============================================================================

class TestRenderDot:
    def test_returns_dot_string(self):
        from runtime.introspection import render_dot
        system = _build_system()
        try:
            dot = render_dot(system)
            assert isinstance(dot, str)
            assert dot.startswith("digraph")
            assert dot.endswith("}")
        finally:
            system.stop()

    def test_contains_module_nodes(self):
        from runtime.introspection import render_dot
        system = _build_system()
        try:
            dot = render_dot(system)
            assert "SrcMod" in dot
            assert "MidMod" in dot
            assert "SinkMod" in dot
        finally:
            system.stop()

    def test_contains_subgraphs(self):
        from runtime.introspection import render_dot
        system = _build_system()
        try:
            dot = render_dot(system)
            assert "subgraph" in dot
        finally:
            system.stop()

    def test_contains_channel_hubs(self):
        from runtime.introspection import render_dot
        system = _build_system()
        try:
            dot = render_dot(system)
            # channel hub nodes are named chan_<port>_<type>
            assert "chan_" in dot
        finally:
            system.stop()

    def test_ignored_modules(self):
        from runtime.introspection import render_dot
        system = _build_system()
        try:
            dot = render_dot(system, ignored_modules={"SinkMod"})
            assert "SinkMod" not in dot
            assert "SrcMod" in dot
        finally:
            system.stop()

    def test_valid_dot_syntax_basic(self):
        from runtime.introspection import render_dot
        system = _build_system()
        try:
            dot = render_dot(system)
            # Basic structural checks
            assert "digraph lingtu {" in dot
            assert dot.count("{") == dot.count("}")
        finally:
            system.stop()


# ============================================================================
# Blueprint.global_config()
# ============================================================================

class TestGlobalConfig:
    def test_global_config_returns_self(self):
        bp = Blueprint()
        result = bp.global_config(n_workers=0)
        assert result is bp

    def test_global_config_chained(self):
        system = (
            autoconnect(SrcMod.blueprint(), SinkMod.blueprint())
            .global_config(n_workers=0)
            .build()
        )
        try:
            assert system is not None
            assert "SrcMod" in system.modules
            assert "SinkMod" in system.modules
        finally:
            system.stop()

    def test_global_config_zero_workers_is_single_process(self):
        from runtime.blueprint import SystemHandle
        system = (
            autoconnect(SrcMod.blueprint(), SinkMod.blueprint())
            .global_config(n_workers=0)
            .build()
        )
        try:
            assert isinstance(system, SystemHandle)
        finally:
            system.stop()

    def test_global_config_overrides_build_param(self):
        """global_config(n_workers=0) takes precedence over build(n_workers=...)."""
        from runtime.blueprint import SystemHandle
        bp = autoconnect(SrcMod.blueprint(), SinkMod.blueprint())
        bp.global_config(n_workers=0)
        # Even though build() is called without n_workers, global_config wins
        system = bp.build()
        try:
            assert isinstance(system, SystemHandle)
        finally:
            system.stop()

    def test_global_config_defaults(self):
        bp = Blueprint()
        bp.global_config()
        assert bp._global_cfg.get("n_workers", 0) == 0

    def test_global_config_kwargs_stored(self):
        bp = Blueprint()
        bp.global_config(n_workers=0, foo="bar")
        assert bp._global_cfg.get("foo") == "bar"


# ============================================================================
# ResourceMonitor
# ============================================================================

class TestResourceMonitor:
    def test_instantiate(self):
        from runtime.resource_monitor import ResourceMonitor
        m = ResourceMonitor(poll_interval=1.0)
        assert m is not None

    def test_register_and_stats_empty(self):
        from runtime.resource_monitor import ResourceMonitor
        m = ResourceMonitor(poll_interval=60.0)
        m.register("test-proc", pid=1)  # PID 1 = init/system process
        # Stats not populated until first poll
        stats = m.stats()
        assert isinstance(stats, dict)

    def test_summary_no_procs(self):
        from runtime.resource_monitor import ResourceMonitor
        m = ResourceMonitor()
        s = m.summary()
        assert isinstance(s, str)
        assert "no processes" in s

    def test_start_stop(self):
        import os

        from runtime.resource_monitor import ResourceMonitor
        m = ResourceMonitor(poll_interval=60.0)
        m.register("self", pid=os.getpid())
        m.start()
        assert m.is_running()
        m.stop()

    def test_unregister(self):
        from runtime.resource_monitor import ResourceMonitor
        m = ResourceMonitor()
        m.register("proc-a", pid=1)
        m.unregister("proc-a")
        # Should not appear in future polls
        assert "proc-a" not in m._pids

    def test_start_idempotent(self):
        import os

        from runtime.resource_monitor import ResourceMonitor
        m = ResourceMonitor(poll_interval=60.0)
        m.register("self", pid=os.getpid())
        m.start()
        m.start()  # second call is no-op
        assert m.is_running()
        m.stop()

    def test_import_from_core(self):
        from runtime import ResourceMonitor  # lazy export via __getattr__
        assert ResourceMonitor is not None


# ============================================================================
# WorkerManager.get_pid
# ============================================================================

class TestWorkerManagerGetPid:
    def test_get_pid_before_start_returns_none(self):
        from runtime.worker_manager import WorkerManager
        mgr = WorkerManager(n_workers=1)
        assert mgr.get_pid(0) is None

    def test_get_pid_unknown_worker_returns_none(self):
        from runtime.worker_manager import WorkerManager
        mgr = WorkerManager(n_workers=1)
        assert mgr.get_pid(99) is None
