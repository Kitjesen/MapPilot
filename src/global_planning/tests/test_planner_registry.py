"""Tests for planner backend registry (no C++ .so required)."""

import pytest

from core.plugin_seed import seed_builtin_plugins
from core.registry import clear, get, get_metadata, list_plugins, restore, snapshot


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

    def test_planner_backend_category_exists(self, seeded_registry):
        """planner_backend category exists after seeding."""
        assert "planner_backend" in seeded_registry

    def test_planner_backend_list(self, seeded_registry):
        """list_plugins returns both PCT and A*."""
        backends = list_plugins("planner_backend")
        assert "pct" in backends
        assert "astar" in backends

    def test_planner_backend_get_pct(self, seeded_registry):
        """registry.get('planner_backend', 'pct') returns the PCT class."""
        BackendCls = get("planner_backend", "pct")
        assert BackendCls is not None
        # Verify it can be instantiated
        instance = BackendCls(tomogram_path="")
        assert hasattr(instance, "plan")
        assert hasattr(instance, "available")
        assert hasattr(instance, "update_map")

    def test_planner_backend_get_astar(self, seeded_registry):
        """registry.get('planner_backend', 'astar') returns the A* class."""
        BackendCls = get("planner_backend", "astar")
        assert BackendCls is not None
        instance = BackendCls(tomogram_path="")
        assert hasattr(instance, "plan")
        assert hasattr(instance, "update_map")

    def test_planner_backend_get_raises_for_unknown(self, seeded_registry):
        """registry.get with unknown name raises KeyError."""
        with pytest.raises(KeyError):
            get("planner_backend", "nonexistent_planner")

    def test_planner_backend_metadata_pct(self, seeded_registry):
        """PCT backend has descriptive metadata."""
        meta = get_metadata("planner_backend", "pct")
        assert "description" in meta
        assert "C++" in meta["description"] or "ele_planner" in meta["description"]

    def test_planner_backend_metadata_astar(self, seeded_registry):
        """A* backend has descriptive metadata."""
        meta = get_metadata("planner_backend", "astar")
        assert "description" in meta
        assert "Python" in meta["description"] or "fallback" in meta["description"]

    def test_planner_backend_class_identity(self, seeded_registry):
        """PCT and A* backends are distinct classes."""
        PCTCls = get("planner_backend", "pct")
        AstarCls = get("planner_backend", "astar")
        assert PCTCls is not AstarCls

    def test_planner_backend_seeding_idempotent(self, seeded_registry):
        """Seeding plugins twice does not duplicate entries."""
        before = list_plugins("planner_backend")
        seed_builtin_plugins(groups=("planner_backend",))
        after = list_plugins("planner_backend")
        assert before == after

    def test_backend_switching_via_registry(self, seeded_registry):
        """Registry-based lookup enables backend switching."""
        backend_names = ["pct", "astar"]
        for name in backend_names:
            BackendCls = get("planner_backend", name)
            instance = BackendCls(tomogram_path="")
            assert instance is not None
            # Both backends have plan() and update_map()
            assert callable(instance.plan)
            assert callable(instance.update_map)
