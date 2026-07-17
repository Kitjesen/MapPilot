"""Tests for Worker DDS transport selection, DomainRouter, and WorkerDeployment.

Covers:
  - WorkerDeployment: is_remote, resolve_transport, to_metadata
  - WorkerDeploymentRegistry: register, get, resolve_transport, resolve_domain_id
  - DomainRouter: exact match, wildcard, default, has_routes, domain_id_for
  - Global router: set/get/resolve_domain_for_topic
  - Blueprint.worker(): builder stores deployment config
  - Backward compatibility: no worker() call → SHM default
"""

import pytest

from runtime.transport.domain_router import (
    DomainRoute,
    DomainRouter,
    get_global_router,
    resolve_domain_for_topic,
    set_global_router,
)
from runtime.worker_config import WorkerDeployment, WorkerDeploymentRegistry

# ---------------------------------------------------------------------------
# WorkerDeployment
# ---------------------------------------------------------------------------


class TestWorkerDeployment:
    def test_default_is_local(self):
        dep = WorkerDeployment(module_name="Det")
        assert dep.host == "localhost"
        assert dep.is_remote is False

    def test_remote_host(self):
        dep = WorkerDeployment(module_name="SLAM", host="192.168.66.190")
        assert dep.is_remote is True

    def test_loopback_is_not_remote(self):
        dep = WorkerDeployment(module_name="A", host="127.0.0.1")
        assert dep.is_remote is False

    def test_empty_host_is_not_remote(self):
        dep = WorkerDeployment(module_name="A", host="")
        assert dep.is_remote is False

    # resolve_transport --------------------------------------------------

    def test_resolve_auto_local_shm(self):
        dep = WorkerDeployment(module_name="Det", transport="auto")
        assert dep.resolve_transport() == "shm"

    def test_resolve_auto_remote_dds(self):
        dep = WorkerDeployment(module_name="SLAM", host="192.168.66.190", transport="auto")
        assert dep.resolve_transport() == "dds"

    def test_resolve_none_local_shm(self):
        dep = WorkerDeployment(module_name="Det")
        assert dep.resolve_transport() == "shm"

    def test_resolve_none_remote_dds(self):
        dep = WorkerDeployment(module_name="SLAM", host="10.0.0.1")
        assert dep.resolve_transport() == "dds"

    def test_resolve_explicit_shm(self):
        dep = WorkerDeployment(module_name="X", host="10.0.0.1", transport="shm")
        assert dep.resolve_transport() == "shm"

    def test_resolve_explicit_dds_local(self):
        dep = WorkerDeployment(module_name="X", transport="dds")
        assert dep.resolve_transport() == "dds"

    # to_metadata --------------------------------------------------------

    def test_to_metadata(self):
        dep = WorkerDeployment(
            module_name="SLAM",
            host="192.168.66.190",
            transport="dds",
            domain_id=42,
            qos_profile="sensor_data",
        )
        meta = dep.to_metadata()
        assert meta["host"] == "192.168.66.190"
        assert meta["transport"] == "dds"
        assert meta["domain_id"] == 42
        assert meta["qos_profile"] == "sensor_data"

    def test_to_metadata_defaults(self):
        dep = WorkerDeployment(module_name="Det")
        meta = dep.to_metadata()
        assert meta["transport"] == "shm"
        assert meta["domain_id"] is None


# ---------------------------------------------------------------------------
# WorkerDeploymentRegistry
# ---------------------------------------------------------------------------


class TestWorkerDeploymentRegistry:
    def test_register_and_get(self):
        reg = WorkerDeploymentRegistry()
        dep = WorkerDeployment(module_name="SLAM", host="10.0.0.1")
        reg.register(dep)
        assert reg.get("SLAM") is dep

    def test_get_missing_returns_none(self):
        reg = WorkerDeploymentRegistry()
        assert reg.get("missing") is None

    def test_resolve_transport_default_shm(self):
        reg = WorkerDeploymentRegistry()
        assert reg.resolve_transport("unknown") == "shm"

    def test_resolve_transport_with_deployment(self):
        reg = WorkerDeploymentRegistry()
        reg.register(WorkerDeployment(module_name="SLAM", host="10.0.0.1"))
        assert reg.resolve_transport("SLAM") == "dds"

    def test_resolve_transport_custom_default(self):
        reg = WorkerDeploymentRegistry()
        assert reg.resolve_transport("unknown", default="dds") == "dds"

    def test_resolve_domain_id(self):
        reg = WorkerDeploymentRegistry()
        reg.register(WorkerDeployment(module_name="SLAM", domain_id=42))
        assert reg.resolve_domain_id("SLAM") == 42

    def test_resolve_domain_id_missing(self):
        reg = WorkerDeploymentRegistry()
        assert reg.resolve_domain_id("missing") is None

    def test_all(self):
        reg = WorkerDeploymentRegistry()
        dep1 = WorkerDeployment(module_name="A")
        dep2 = WorkerDeployment(module_name="B")
        reg.register(dep1)
        reg.register(dep2)
        all_deps = reg.all()
        assert set(all_deps.keys()) == {"A", "B"}


# ---------------------------------------------------------------------------
# DomainRoute
# ---------------------------------------------------------------------------


class TestDomainRoute:
    def test_default_values(self):
        r = DomainRoute()
        assert r.domain_id == 0
        assert r.host is None
        assert r.is_remote is False

    def test_remote(self):
        r = DomainRoute(domain_id=42, host="192.168.66.190")
        assert r.is_remote is True

    def test_localhost_not_remote(self):
        r = DomainRoute(host="localhost")
        assert r.is_remote is False

    def test_frozen(self):
        r = DomainRoute(domain_id=1)
        with pytest.raises(AttributeError):
            r.domain_id = 2  # type: ignore[misc]


# ---------------------------------------------------------------------------
# DomainRouter
# ---------------------------------------------------------------------------


class TestDomainRouter:
    def test_empty_router_returns_default(self):
        router = DomainRouter(default_domain=5)
        assert router.resolve("/any/topic") == DomainRoute(domain_id=5)

    def test_exact_match(self):
        router = DomainRouter(
            {
                "/slam/map_cloud": {"domain_id": 42},
            }
        )
        route = router.resolve("/slam/map_cloud")
        assert route.domain_id == 42

    def test_no_match_returns_default(self):
        router = DomainRouter(
            {
                "/slam/*": {"domain_id": 42},
            },
            default_domain=0,
        )
        route = router.resolve("/perception/detections")
        assert route.domain_id == 0

    def test_wildcard_match(self):
        router = DomainRouter(
            {
                "/slam/*": {"domain_id": 42, "host": "192.168.66.190"},
                "*": {"domain_id": 0},
            }
        )
        route = router.resolve("/slam/map_cloud")
        assert route.domain_id == 42
        assert route.host == "192.168.66.190"

    def test_wildcard_fallback(self):
        router = DomainRouter(
            {
                "/slam/*": {"domain_id": 42},
                "*": {"domain_id": 0},
            }
        )
        route = router.resolve("/perception/detections")
        assert route.domain_id == 0

    def test_first_match_wins(self):
        router = DomainRouter(
            {
                "/slam/pose": {"domain_id": 10},
                "/slam/*": {"domain_id": 42},
                "*": {"domain_id": 0},
            }
        )
        # Exact pattern matches first
        assert router.domain_id_for("/slam/pose") == 10
        # Wildcard matches next
        assert router.domain_id_for("/slam/map_cloud") == 42

    def test_domain_id_for_shortcut(self):
        router = DomainRouter({"/cmd_vel": {"domain_id": 7}})
        assert router.domain_id_for("/cmd_vel") == 7

    def test_has_routes_false(self):
        router = DomainRouter()
        assert router.has_routes is False

    def test_has_routes_true(self):
        router = DomainRouter({"*": {"domain_id": 0}})
        assert router.has_routes is True

    def test_patterns_returns_copy(self):
        router = DomainRouter({"/a": {"domain_id": 1}})
        pats = router.patterns()
        assert len(pats) == 1
        pats.append(("dummy", DomainRoute()))  # mutating copy should not affect router
        assert len(router.patterns()) == 1

    def test_qos_profile_in_route(self):
        router = DomainRouter(
            {
                "/slam/*": {"domain_id": 42, "qos_profile": "sensor_data"},
            }
        )
        route = router.resolve("/slam/odom")
        assert route.qos_profile == "sensor_data"

    def test_repr(self):
        router = DomainRouter({"*": {"domain_id": 0}})
        r = repr(router)
        assert "DomainRouter" in r
        assert "patterns=1" in r


# ---------------------------------------------------------------------------
# Global router helpers
# ---------------------------------------------------------------------------


class TestGlobalRouter:
    def setup_method(self):
        set_global_router(None)

    def teardown_method(self):
        set_global_router(None)

    def test_get_returns_none_by_default(self):
        assert get_global_router() is None

    def test_set_and_get(self):
        router = DomainRouter({"*": {"domain_id": 0}})
        set_global_router(router)
        assert get_global_router() is router

    def test_resolve_without_router(self):
        assert resolve_domain_for_topic("/any") == 0

    def test_resolve_with_router(self):
        set_global_router(DomainRouter({"/slam/*": {"domain_id": 42}}))
        assert resolve_domain_for_topic("/slam/pose") == 42

    def test_resolve_fallback(self):
        set_global_router(DomainRouter({"/slam/*": {"domain_id": 42}}))
        # When the global router exists but has no matching pattern,
        # it returns its own default domain (0), not the function fallback.
        assert resolve_domain_for_topic("/other", fallback=99) == 0

    def test_resolve_empty_router_fallback(self):
        set_global_router(DomainRouter())
        assert resolve_domain_for_topic("/any", fallback=5) == 5


# ---------------------------------------------------------------------------
# Blueprint.worker() integration
# ---------------------------------------------------------------------------


class TestBlueprintWorker:
    def test_worker_stores_deployment(self):
        from runtime.blueprint import Blueprint
        from runtime.module import Module

        class DummyModule(Module):
            pass

        bp = Blueprint()
        bp.add(DummyModule, alias="Det")
        bp.worker("Det", host="10.0.0.1", transport="dds", domain_id=42)

        dep = bp._worker_deployments.get("Det")
        assert dep is not None
        assert dep.host == "10.0.0.1"
        assert dep.transport == "dds"
        assert dep.domain_id == 42

    def test_worker_returns_self(self):
        from runtime.blueprint import Blueprint

        bp = Blueprint()
        result = bp.worker("SomeModule")
        assert result is bp

    def test_no_worker_call_defaults_to_shm(self):
        """Backward compatibility: no worker() → resolve_transport returns shm."""
        from runtime.blueprint import Blueprint
        from runtime.module import Module

        class DummyModule(Module):
            pass

        bp = Blueprint()
        bp.add(DummyModule, alias="Det")
        assert bp._worker_deployments.resolve_transport("Det") == "shm"

    def test_worker_auto_local_shm(self):
        from runtime.blueprint import Blueprint

        bp = Blueprint()
        bp.worker("Det", transport="auto")
        assert bp._worker_deployments.resolve_transport("Det") == "shm"

    def test_worker_auto_remote_dds(self):
        from runtime.blueprint import Blueprint

        bp = Blueprint()
        bp.worker("SLAM", host="192.168.66.190", transport="auto")
        assert bp._worker_deployments.resolve_transport("SLAM") == "dds"

    def test_worker_explicit_dds_on_localhost(self):
        from runtime.blueprint import Blueprint

        bp = Blueprint()
        bp.worker("Det", transport="dds")
        assert bp._worker_deployments.resolve_transport("Det") == "dds"

    def test_multiple_workers(self):
        from runtime.blueprint import Blueprint

        bp = Blueprint()
        bp.worker("SLAM", host="10.0.0.1", domain_id=42)
        bp.worker("Det", host="10.0.0.2", domain_id=43)

        assert bp._worker_deployments.resolve_transport("SLAM") == "dds"
        assert bp._worker_deployments.resolve_domain_id("SLAM") == 42
        assert bp._worker_deployments.resolve_transport("Det") == "dds"
        assert bp._worker_deployments.resolve_domain_id("Det") == 43

    def test_worker_qos_profile(self):
        from runtime.blueprint import Blueprint

        bp = Blueprint()
        bp.worker("SLAM", host="10.0.0.1", qos_profile="sensor_data")
        dep = bp._worker_deployments.get("SLAM")
        assert dep is not None
        assert dep.qos_profile == "sensor_data"


# ---------------------------------------------------------------------------
# Factory domain_id forwarding (unit-level, no real DDS)
# ---------------------------------------------------------------------------


class TestFactoryDomainId:
    """Verify create_transport / create_transport_adapter accept domain_id."""

    def test_create_transport_local_ignores_domain(self):
        from runtime.transport.factory import create_transport

        t = create_transport("local", domain_id=42)
        assert t is not None

    def test_create_transport_shm_ignores_domain(self):
        from runtime.transport.factory import create_transport

        t = create_transport("shm", domain_id=42)
        assert t is not None
        t.close()
