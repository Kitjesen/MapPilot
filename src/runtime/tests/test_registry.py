"""Tests for runtime.registry — plugin registration and lookup."""


import pytest

from runtime.module import Module
from runtime.registry import (
    auto_select,
    clear,
    get,
    get_metadata,
    list_categories,
    list_plugins,
    register,
    restore,
    snapshot,
)


def _snapshot_registry():
    return snapshot()


def _restore_registry(snap):
    restore(snap)


class TestRegister:
    """Test @register decorator and get() lookup."""

    def setup_method(self):
        self._saved = snapshot()
        clear()

    def teardown_method(self):
        restore(self._saved)

    def test_register_and_get(self):
        @register("driver", "test_bot")
        class TestBot(Module, layer=1):
            pass

        result = get("driver", "test_bot")
        assert result is TestBot

    def test_get_unknown_category_raises(self):
        with pytest.raises(KeyError, match="Unknown category"):
            get("nonexistent", "foo")

    def test_get_unknown_name_raises(self):
        @register("driver", "alpha")
        class Alpha(Module, layer=1):
            pass

        with pytest.raises(KeyError, match="Unknown plugin"):
            get("driver", "beta")

    def test_list_plugins(self):
        @register("encoder", "clip")
        class Clip:
            pass

        @register("encoder", "dino")
        class Dino:
            pass

        names = list_plugins("encoder")
        assert names == ["clip", "dino"]

    def test_list_plugins_empty_category(self):
        assert list_plugins("nonexistent") == []

    def test_list_categories(self):
        @register("driver", "d1")
        class D1:
            pass

        @register("planner", "p1")
        class P1:
            pass

        cats = list_categories()
        assert "driver" in cats
        assert "planner" in cats


class TestAutoSelect:
    """Test auto_select with priority and platform filtering."""

    def setup_method(self):
        self._saved = snapshot()
        clear()

    def teardown_method(self):
        restore(self._saved)

    def test_auto_select_highest_priority(self):
        @register("driver", "low", priority=1)
        class Low:
            pass

        @register("driver", "high", priority=10)
        class High:
            pass

        assert auto_select("driver") == "high"

    def test_auto_select_platform_filter(self):
        @register("driver", "arm_only", priority=10, platforms={"aarch64"})
        class ArmOnly:
            pass

        @register("driver", "any_platform", priority=5)
        class AnyPlatform:
            pass

        # On x86_64, arm_only is filtered out
        assert auto_select("driver", platform="x86_64") == "any_platform"
        # On aarch64, arm_only wins (higher priority)
        assert auto_select("driver", platform="aarch64") == "arm_only"

    def test_auto_select_no_match(self):
        @register("driver", "arm_only", platforms={"aarch64"})
        class ArmOnly:
            pass

        assert auto_select("driver", platform="riscv") is None

    def test_auto_select_unknown_category(self):
        assert auto_select("nonexistent") is None


class TestMetadata:
    """Test metadata storage and retrieval."""

    def setup_method(self):
        self._saved = snapshot()
        clear()

    def teardown_method(self):
        restore(self._saved)

    def test_get_metadata(self):
        @register("driver", "bot", priority=5, platforms={"aarch64"}, description="Test bot")
        class Bot:
            pass

        meta = get_metadata("driver", "bot")
        assert meta["priority"] == 5
        assert "aarch64" in meta["platforms"]
        assert meta["description"] == "Test bot"

    def test_get_metadata_missing(self):
        assert get_metadata("driver", "nonexistent") == {}
