"""Decision module."""

import json
import os
import tempfile

from memory.spatial.tagged_locations import TaggedLocationStore

# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------


class TestTagAndQuery:
    def setup_method(self):
        self.store = TaggedLocationStore()

    def test_tag_and_query_exact(self):
        """Test tag and query exact."""
        self.store.tag("体育馆", x=10.0, y=20.0, z=0.0)
        result = self.store.query("体育馆")
        assert result is not None
        assert result["name"] == "体育馆"
        assert result["position"] == [10.0, 20.0, 0.0]

    def test_query_not_found_returns_none(self):
        """Test query not found returns none."""
        assert self.store.query("不存在的地点") is None

    def test_tag_stores_yaw(self):
        """Test tag stores yaw."""
        self.store.tag("入口", x=1.0, y=2.0, z=0.0, yaw=1.57)
        result = self.store.query("入口")
        assert result["yaw"] is not None
        assert abs(result["yaw"] - 1.57) < 1e-6

    def test_tag_default_yaw_is_none(self):
        """Test tag default yaw is none."""
        self.store.tag("餐厅", x=5.0, y=6.0)
        result = self.store.query("餐厅")
        assert result["yaw"] is None

    def test_tag_default_z_is_zero(self):
        """Test tag default z is zero."""
        self.store.tag("走廊", x=3.0, y=4.0)
        result = self.store.query("走廊")
        assert result["position"][2] == 0.0


class TestFuzzyQuery:
    def setup_method(self):
        self.store = TaggedLocationStore()
        self.store.tag("体育馆", x=25.0, y=30.0)
        self.store.tag("体育馆入口", x=22.0, y=28.0)
        self.store.tag("图书馆", x=10.0, y=15.0)

    def test_fuzzy_exact_match(self):
        """Test fuzzy exact match."""
        result = self.store.query_fuzzy("体育馆")
        assert result is not None

    def test_fuzzy_partial_match(self):
        """Test fuzzy partial match."""
        result = self.store.query_fuzzy("体育")
        assert result is not None
        assert "体育" in result["name"]

    def test_fuzzy_in_instruction(self):
        """Test fuzzy in instruction."""
        result = self.store.query_fuzzy("导航到体育馆")
        assert result is not None
        assert "体育馆" in result["name"]

    def test_fuzzy_prefers_longer_match(self):
        """Test fuzzy prefers longer match."""
        result = self.store.query_fuzzy("体育馆入口附近")
        assert result is not None
        assert result["name"] == "体育馆入口"

    def test_fuzzy_no_match_returns_none(self):
        """Test fuzzy no match returns none."""
        assert self.store.query_fuzzy("操场") is None

    def test_fuzzy_empty_store(self):
        """Test fuzzy empty store."""
        empty = TaggedLocationStore()
        assert empty.query_fuzzy("任意") is None


class TestRemove:
    def setup_method(self):
        self.store = TaggedLocationStore()

    def test_remove_existing(self):
        """Test remove existing."""
        self.store.tag("办公室", x=1.0, y=2.0)
        removed = self.store.remove("办公室")
        assert removed is True
        assert self.store.query("办公室") is None

    def test_remove_nonexistent(self):
        """Test remove nonexistent."""
        removed = self.store.remove("不存在")
        assert removed is False

    def test_remove_does_not_affect_others(self):
        """Test remove does not affect others."""
        self.store.tag("A", x=1.0, y=2.0)
        self.store.tag("B", x=3.0, y=4.0)
        self.store.remove("A")
        assert self.store.query("B") is not None


class TestListAll:
    def setup_method(self):
        self.store = TaggedLocationStore()

    def test_list_all_empty(self):
        assert self.store.list_all() == []

    def test_list_all_returns_all(self):
        self.store.tag("A", x=1.0, y=0.0)
        self.store.tag("B", x=2.0, y=0.0)
        self.store.tag("C", x=3.0, y=0.0)
        all_entries = self.store.list_all()
        assert len(all_entries) == 3
        names = {e["name"] for e in all_entries}
        assert names == {"A", "B", "C"}


class TestChineseNames:
    def setup_method(self):
        self.store = TaggedLocationStore()

    def test_chinese_name_stored_correctly(self):
        """Test chinese name stored correctly."""
        self.store.tag("会议室", x=5.0, y=10.0)
        result = self.store.query("会议室")
        assert result is not None
        assert result["name"] == "会议室"

    def test_multiple_chinese_names(self):
        names = ["大厅", "走廊", "楼梯间", "办公区"]
        for i, name in enumerate(names):
            self.store.tag(name, x=float(i), y=0.0)
        for name in names:
            assert self.store.query(name) is not None

    def test_mixed_chinese_english_name(self):
        self.store.tag("Room 101会议室", x=1.0, y=1.0)
        result = self.store.query("Room 101会议室")
        assert result is not None


class TestOverwrite:
    def setup_method(self):
        self.store = TaggedLocationStore()

    def test_overwrite_same_name_updates_position(self):
        """Test overwrite same name updates position."""
        self.store.tag("地点A", x=1.0, y=2.0)
        self.store.tag("地点A", x=99.0, y=88.0)
        result = self.store.query("地点A")
        assert result["position"] == [99.0, 88.0, 0.0]

    def test_overwrite_count_stays_same(self):
        """Test overwrite count stays same."""
        self.store.tag("地点A", x=1.0, y=2.0)
        self.store.tag("地点A", x=5.0, y=6.0)
        assert len(self.store.list_all()) == 1


# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------


class TestSaveLoad:
    def test_save_and_load(self):
        """Test save and load."""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "tags.json")

            store_write = TaggedLocationStore(path)
            store_write.tag("图书馆", x=10.0, y=20.0, z=0.5, yaw=1.0)
            store_write.tag("食堂", x=30.0, y=40.0)
            store_write.save()

            store_read = TaggedLocationStore(path)
            lib = store_read.query("图书馆")
            canteen = store_read.query("食堂")

            assert lib is not None
            assert lib["position"] == [10.0, 20.0, 0.5]
            assert abs(lib["yaw"] - 1.0) < 1e-6

            assert canteen is not None
            assert canteen["position"] == [30.0, 40.0, 0.0]

    def test_load_nonexistent_file_ok(self):
        """Test load nonexistent file ok."""
        store = TaggedLocationStore("/tmp/nonexistent_lingtu_tags_abc123.json")
        assert store.list_all() == []

    def test_save_empty_store(self):
        """Test save empty store."""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "empty.json")
            store = TaggedLocationStore(path)
            store.save()
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            assert data == []

    def test_save_preserves_chinese(self):
        """Test save preserves chinese."""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "zh.json")
            store = TaggedLocationStore(path)
            store.tag("操场", x=1.0, y=2.0)
            store.save()
            with open(path, encoding="utf-8") as f:
                content = f.read()
            assert "操场" in content

    def test_in_memory_save_is_noop(self):
        """Test in memory save is noop."""
        store = TaggedLocationStore()
        store.tag("A", x=1.0, y=2.0)
        store.save()
