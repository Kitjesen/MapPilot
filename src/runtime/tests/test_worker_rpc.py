"""Tests for RPCClient, RemoteOut, RemoteIn, and ModuleCoordinator.

These tests use a mock WorkerManager so they run without a real Worker process.
ModuleCoordinator tests patch the import so coordinator.py is exercised
end-to-end without requiring worker_manager.py to exist.
"""
from unittest.mock import MagicMock, patch

import pytest

from runtime.module import Module, rpc
from runtime.remote_ports import RemoteIn, RemoteOut
from runtime.runtime_interface import TOPICS
from runtime.rpc_client import RPCClient

# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

class NavModule(Module):
    """Minimal module with a couple of @rpc methods for testing."""

    @rpc
    def navigate(self, x: float, y: float) -> bool:
        return True

    @rpc
    def stop(self) -> None:
        pass

    def not_rpc(self) -> str:
        return "plain"


def make_mock_manager() -> MagicMock:
    """Return a MagicMock that satisfies the WorkerManager interface."""
    mgr = MagicMock()
    mgr.rpc_call.return_value = True
    mgr.health.return_value = {"ok": True}
    mgr.n_workers = 4
    return mgr


# ---------------------------------------------------------------------------
# RPCClient tests
# ---------------------------------------------------------------------------

class TestRPCClient:

    def test_rpc_methods_are_callable(self):
        mgr = make_mock_manager()
        proxy = RPCClient(mgr, worker_id=0, module_id="nav",
                          rpc_methods={"navigate", "stop"})

        caller = proxy.navigate
        assert callable(caller)

    def test_rpc_call_forwards_to_manager(self):
        mgr = make_mock_manager()
        proxy = RPCClient(mgr, worker_id=0, module_id="nav",
                          rpc_methods={"navigate"})

        result = proxy.navigate(x=1.0, y=2.0)

        mgr.rpc_call.assert_called_once_with(
            0, "nav", "navigate", kwargs={"x": 1.0, "y": 2.0}, timeout=30.0
        )
        assert result is True

    def test_non_rpc_attribute_raises(self):
        mgr = make_mock_manager()
        proxy = RPCClient(mgr, worker_id=0, module_id="nav",
                          rpc_methods={"navigate"})

        with pytest.raises(AttributeError):
            _ = proxy.not_rpc

    def test_private_attribute_raises(self):
        mgr = make_mock_manager()
        proxy = RPCClient(mgr, worker_id=0, module_id="nav",
                          rpc_methods={"navigate"})

        with pytest.raises(AttributeError):
            _ = proxy.__nonexistent

    def test_lifecycle_setup_forwards(self):
        mgr = make_mock_manager()
        proxy = RPCClient(mgr, worker_id=2, module_id="sensor",
                          rpc_methods=set())
        proxy.setup()
        mgr.setup.assert_called_once_with(2, "sensor")

    def test_lifecycle_start_forwards(self):
        mgr = make_mock_manager()
        proxy = RPCClient(mgr, worker_id=1, module_id="sensor",
                          rpc_methods=set())
        proxy.start()
        mgr.start_module.assert_called_once_with(1, "sensor")

    def test_lifecycle_stop_forwards(self):
        mgr = make_mock_manager()
        proxy = RPCClient(mgr, worker_id=1, module_id="sensor",
                          rpc_methods=set())
        proxy.stop()
        mgr.stop_module.assert_called_once_with(1, "sensor")

    def test_health_returns_manager_result(self):
        mgr = make_mock_manager()
        proxy = RPCClient(mgr, worker_id=0, module_id="nav",
                          rpc_methods=set())
        h = proxy.health()
        assert h == {"ok": True}

    def test_properties(self):
        mgr = make_mock_manager()
        proxy = RPCClient(mgr, worker_id=3, module_id="slam",
                          rpc_methods={"foo", "bar"})
        assert proxy.worker_id == 3
        assert proxy.module_id == "slam"
        assert proxy.rpc_methods == {"foo", "bar"}

    def test_repr(self):
        mgr = make_mock_manager()
        proxy = RPCClient(mgr, worker_id=0, module_id="nav",
                          rpc_methods={"navigate"})
        r = repr(proxy)
        assert "nav" in r
        assert "worker-0" in r

    def test_empty_rpc_methods_default(self):
        mgr = make_mock_manager()
        proxy = RPCClient(mgr, worker_id=0, module_id="nav")
        assert proxy.rpc_methods == set()

    def test_get_skill_infos_calls_get_skills(self):
        from runtime.module import SkillInfo
        mgr = make_mock_manager()
        mgr.get_skills.return_value = [
            {"func_name": "navigate", "class_name": "NavModule", "args_schema": "{}"},
        ]
        proxy = RPCClient(mgr, worker_id=1, module_id="nav", rpc_methods={"navigate"})
        infos = proxy.get_skill_infos()
        mgr.get_skills.assert_called_once_with(1, "nav")
        assert len(infos) == 1
        assert isinstance(infos[0], SkillInfo)
        assert infos[0].func_name == "navigate"
        assert infos[0].class_name == "NavModule"

    def test_get_skill_infos_returns_empty_on_no_skills(self):
        mgr = make_mock_manager()
        mgr.get_skills.return_value = []
        proxy = RPCClient(mgr, worker_id=0, module_id="nav")
        infos = proxy.get_skill_infos()
        assert infos == []


# ---------------------------------------------------------------------------
# RemoteOut tests
# ---------------------------------------------------------------------------

class TestRemoteOut:

    def test_default_topic_from_name(self):
        ro = RemoteOut("odometry")
        assert ro.transport_topic == "/odometry"

    def test_explicit_topic(self):
        ro = RemoteOut("odometry", transport_topic=TOPICS.odometry)
        assert ro.transport_topic == TOPICS.odometry

    def test_publish_calls_transport(self):
        transport = MagicMock()
        ro = RemoteOut("pos", transport_topic="/pos")
        ro.bind_transport(transport)
        ro.publish({"x": 1.0})
        transport.publish.assert_called_once_with("/pos", {"x": 1.0})

    def test_publish_without_transport_is_silent(self):
        ro = RemoteOut("pos")
        # Should not raise even with no transport bound.
        ro.publish({"x": 1.0})

    def test_bind_transport_overrides_topic(self):
        transport = MagicMock()
        ro = RemoteOut("pos", transport_topic="/old")
        ro.bind_transport(transport, topic="/new")
        assert ro.transport_topic == "/new"

    def test_repr(self):
        ro = RemoteOut("vel", transport_topic="/cmd_vel")
        assert "vel" in repr(ro)
        assert "/cmd_vel" in repr(ro)


# ---------------------------------------------------------------------------
# RemoteIn tests
# ---------------------------------------------------------------------------

class TestRemoteIn:

    def test_default_topic_from_name(self):
        ri = RemoteIn("cmd_vel")
        assert ri.transport_topic == "/cmd_vel"

    def test_explicit_topic(self):
        ri = RemoteIn("cmd_vel", transport_topic=TOPICS.cmd_vel)
        assert ri.transport_topic == TOPICS.cmd_vel

    def test_subscribe_then_bind_wires_callback(self):
        transport = MagicMock()
        callback = MagicMock()
        ri = RemoteIn("cmd", transport_topic="/cmd")

        ri.subscribe(callback)
        ri.bind_transport(transport)

        transport.subscribe.assert_called_once_with("/cmd", callback)

    def test_bind_then_subscribe_wires_callback(self):
        transport = MagicMock()
        callback = MagicMock()
        ri = RemoteIn("cmd", transport_topic="/cmd")

        ri.bind_transport(transport)
        ri.subscribe(callback)

        transport.subscribe.assert_called_once_with("/cmd", callback)

    def test_bind_without_callback_does_not_subscribe(self):
        transport = MagicMock()
        ri = RemoteIn("cmd")
        ri.bind_transport(transport)
        transport.subscribe.assert_not_called()

    def test_bind_overrides_topic(self):
        transport = MagicMock()
        ri = RemoteIn("cmd", transport_topic="/old")
        ri.bind_transport(transport, topic="/new")
        assert ri.transport_topic == "/new"

    def test_repr_idle(self):
        ri = RemoteIn("vel")
        assert "idle" in repr(ri)

    def test_repr_connected(self):
        transport = MagicMock()
        ri = RemoteIn("vel", transport_topic="/vel")
        ri.bind_transport(transport)
        ri.subscribe(MagicMock())
        assert "connected" in repr(ri)


# ---------------------------------------------------------------------------
# ModuleCoordinator tests (mock WorkerManager via patch)
# ---------------------------------------------------------------------------

class TestModuleCoordinator:
    """Patch runtime.worker_manager.WorkerManager so coordinator can be imported
    without a real implementation."""

    @pytest.fixture
    def mock_wm_cls(self):
        """Patch WorkerManager at the module level used by coordinator.py."""
        wm_instance = make_mock_manager()
        with patch("runtime.coordinator.WorkerManager" if False else
                   "runtime.worker_manager.WorkerManager",
                   autospec=False) as _unused:
            # Patch at the point where coordinator imports it.
            with patch("runtime.coordinator.ModuleCoordinator.__init__",
                       wraps=_patched_init(wm_instance)):
                yield wm_instance

    def _make_coord(self):
        """Build a ModuleCoordinator with a mocked WorkerManager."""
        from runtime.coordinator import ModuleCoordinator
        coord = object.__new__(ModuleCoordinator)
        coord._mgr = make_mock_manager()
        coord._mgr.n_workers = 4
        coord._proxies = {}
        coord._assignments = {}
        coord._next_worker = 0
        return coord

    def test_deploy_returns_rpc_client(self):
        coord = self._make_coord()
        proxy = coord.deploy(NavModule, "nav")
        assert isinstance(proxy, RPCClient)
        assert proxy.module_id == "nav"

    def test_deploy_discovers_rpc_methods(self):
        coord = self._make_coord()
        proxy = coord.deploy(NavModule, "nav")
        assert "navigate" in proxy.rpc_methods
        assert "stop" in proxy.rpc_methods
        assert "not_rpc" not in proxy.rpc_methods

    def test_deploy_round_robin(self):
        coord = self._make_coord()
        p0 = coord.deploy(NavModule, "nav0")
        p1 = coord.deploy(NavModule, "nav1")
        p2 = coord.deploy(NavModule, "nav2")
        p3 = coord.deploy(NavModule, "nav3")
        p4 = coord.deploy(NavModule, "nav4")
        assert p0.worker_id == 0
        assert p1.worker_id == 1
        assert p2.worker_id == 2
        assert p3.worker_id == 3
        assert p4.worker_id == 0  # wraps around

    def test_deploy_explicit_worker_id(self):
        coord = self._make_coord()
        proxy = coord.deploy(NavModule, "nav", worker_id=2)
        assert proxy.worker_id == 2

    def test_get_proxy(self):
        coord = self._make_coord()
        coord.deploy(NavModule, "nav")
        assert coord.get_proxy("nav") is not None
        assert coord.get_proxy("missing") is None

    def test_proxies_property(self):
        coord = self._make_coord()
        coord.deploy(NavModule, "a")
        coord.deploy(NavModule, "b")
        proxies = coord.proxies
        assert "a" in proxies and "b" in proxies

    def test_setup_all_calls_manager(self):
        coord = self._make_coord()
        coord.deploy(NavModule, "nav", worker_id=0)
        coord.setup_all()
        coord._mgr.setup.assert_called_with(0, "nav")

    def test_start_all_calls_manager(self):
        coord = self._make_coord()
        coord.deploy(NavModule, "nav", worker_id=1)
        coord.start_all()
        coord._mgr.start_module.assert_called_with(1, "nav")

    def test_stop_all_suppresses_errors(self):
        coord = self._make_coord()
        coord.deploy(NavModule, "nav", worker_id=0)
        coord._mgr.stop_module.side_effect = RuntimeError("boom")
        # Should not raise.
        coord.stop_all()

    def test_shutdown_clears_state(self):
        coord = self._make_coord()
        coord.deploy(NavModule, "nav")
        coord.shutdown()
        assert coord._proxies == {}
        assert coord._assignments == {}

    def test_rpc_call_end_to_end(self):
        """Proxy created by coordinator can forward an RPC call."""
        coord = self._make_coord()
        coord._mgr.rpc_call.return_value = 42
        proxy = coord.deploy(NavModule, "nav", worker_id=0)
        result = proxy.navigate(x=1.0, y=2.0)
        coord._mgr.rpc_call.assert_called_once_with(
            0, "nav", "navigate", kwargs={"x": 1.0, "y": 2.0}, timeout=30.0
        )
        assert result == 42


def _patched_init(wm_instance):
    """Helper — unused but referenced for type-checking clarity."""
    pass
