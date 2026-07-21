"""Tests for planner backend registry (no C++ .so required)."""

import pytest

from runtime.registry import clear, get, get_metadata, list_plugins, restore, snapshot
from lingtu.plugin_seed import seed_builtin_plugins
from runtime.profiles.planner_backends import normalize_planner_name
from nav.services.plan.contracts import GlobalPlannerBackend

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def _registry_isolation():
    """Save/restore global registry around each test."""
    state = snapshot()
    yield
    restore(state)


@pytest.fixture
def seeded_registry():
    """Seed planner_backend plugins and return registry state.

    Uses reload_loaded=True because the module may already be imported
    from other tests in this session; without reload the @register
    decorators would not re-fire and the registry would remain empty.
    """
    clear()
    seed_builtin_plugins(
        groups=("planner_backend",),
        reload_loaded=True,
    )
    return snapshot()


# ---------------------------------------------------------------------------
# Registry Tests
# ---------------------------------------------------------------------------

class TestPlannerRegistry:
    """Verify planner backends register correctly."""

    def test_planner_name_normalization_preserves_canonical_names(self):
        """Canonical planner names remain unchanged."""
        assert normalize_planner_name("octoplanner3d") == "octoplanner3d"
        assert normalize_planner_name("far") == "far"

    def test_planner_name_normalization_maps_octplanner_alias(self):
        """Historical octplanner spelling maps to the OctoPlanner3D backend."""
        assert normalize_planner_name("octplanner") == "octoplanner3d"

    def test_planner_backend_category_exists(self, seeded_registry):
        """planner_backend category exists after seeding."""
        assert "planner_backend" in seeded_registry

    def test_planner_backend_list(self, seeded_registry):
        """Registry exposes the default planner and the explicit optional FAR backend."""
        backends = list_plugins("planner_backend")
        assert backends == ["far", "octoplanner3d"]

    def test_planner_backend_get_pct_rejected(self, seeded_registry):
        """PCT is legacy-only and is not seeded as a product planner backend."""
        with pytest.raises(KeyError):
            get("planner_backend", "pct")

    def test_planner_backend_get_astar_rejected(self, seeded_registry):
        """A* is not a product global-planner backend."""
        with pytest.raises(KeyError):
            get("planner_backend", "astar")

    def test_planner_backend_get_direct_rejected(self, seeded_registry):
        """Direct path is not a product global-planner backend."""
        with pytest.raises(KeyError):
            get("planner_backend", "direct")

    def test_planner_backend_get_raises_for_unknown(self, seeded_registry):
        """registry.get with unknown name raises KeyError."""
        with pytest.raises(KeyError):
            get("planner_backend", "nonexistent_planner")

    def test_planner_backend_metadata_octoplanner3d(self, seeded_registry):
        """OctoPlanner3D backend has descriptive metadata."""
        meta = get_metadata("planner_backend", "octoplanner3d")
        assert "description" in meta
        assert "OctoPlanner3D" in meta["description"]

    def test_planner_backend_metadata_far(self, seeded_registry):
        """FAR advertises its optional native and occupancy-map contract."""
        meta = get_metadata("planner_backend", "far")
        assert meta["optional"] is True
        assert meta["supported_map_extensions"] == [".npz"]
        assert "map_generation" in meta["input_schema"]["required"]

    def test_far_registry_lookup_does_not_load_native_library(self, seeded_registry):
        """Plugin discovery remains available on hosts without the optional binary."""
        backend_cls = get("planner_backend", "far")
        assert backend_cls.__name__ == "FarBackend"
        assert callable(backend_cls.plan_request)

    def test_planner_backend_seeding_idempotent(self, seeded_registry):
        """Seeding plugins twice does not duplicate entries."""
        before = list_plugins("planner_backend")
        seed_builtin_plugins(groups=("planner_backend",))
        after = list_plugins("planner_backend")
        assert before == after

    def test_backend_switching_via_registry(self, seeded_registry):
        """Registry-based lookup returns the product planner."""
        backend_names = ["octoplanner3d"]
        for name in backend_names:
            BackendCls = get("planner_backend", name)
            instance = BackendCls(map_path="")
            assert instance is not None
            assert isinstance(instance, GlobalPlannerBackend)
            assert callable(instance.plan_request)
            assert callable(instance.plan)
            assert callable(instance.update_map)
