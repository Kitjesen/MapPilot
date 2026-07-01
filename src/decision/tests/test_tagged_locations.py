"""Tagged Locations 单元测试。
不依赖 ROS2，纯 Python 可运行。

运行:
    cd src/decision && python -m pytest test/test_tagged_locations.py -v
"""

import json
import os
import tempfile


from memory.spatial.tagged_locations import TaggedLocationStore

# ---------------------------------------------------------------------------
# 基本增删查测试
# ---------------------------------------------------------------------------

class TestTagAndQuery:
    def setup_method(self):
        self.store = TaggedLocationStore()  # 不持久化，内存模式

    def test_tag_and_query_exact(self):
        """tag 后能精确 query 到。"""
        self.store.tag("体育馆", x=10.0, y=20.0, z=0.0)
        result = self.store.query("体育馆")
        assert result is not None
        assert result["name"] == "体育馆"
        assert result["position"] == [10.0, 20.0, 0.0]

    def test_query_not_found_returns_none(self):
        """查询不存在的名称返回 None。"""
        assert self.store.query("不存在的地点") is None

    def test_tag_stores_yaw(self):
        """yaw 参数应被正确存储。"""
        self.store.tag("入口", x=1.0, y=2.0, z=0.0, yaw=1.57)
        result = self.store.query("入口")
        assert result["yaw"] is not None
        assert abs(result["yaw"] - 1.57) < 1e-6

    def test_tag_default_yaw_is_none(self):
        """未提供 yaw 时，存储值应为 None。"""
        self.store.tag("餐厅", x=5.0, y=6.0)
        result = self.store.query("餐厅")
        assert result["yaw"] is None

    def test_tag_default_z_is_zero(self):
        """未提供 z 时，默认为 0.0。"""
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
        """精确名称应命中。"""
        result = self.store.query_fuzzy("体育馆")
        assert result is not None

    def test_fuzzy_partial_match(self):
        """模糊匹配 '体育' → 包含 '体育' 的地点。"""
        result = self.store.query_fuzzy("体育")
        assert result is not None
        assert "体育" in result["name"]

    def test_fuzzy_in_instruction(self):
        """指令包含标签名 '导航到体育馆' → 匹配 '体育馆'。"""
        result = self.store.query_fuzzy("导航到体育馆")
        assert result is not None
        assert "体育馆" in result["name"]

    def test_fuzzy_prefers_longer_match(self):
        """多个匹配时取最长标签名（'体育馆入口' 比 '体育馆' 长）。"""
        result = self.store.query_fuzzy("体育馆入口附近")
        assert result is not None
        assert result["name"] == "体育馆入口"

    def test_fuzzy_no_match_returns_none(self):
        """完全不匹配返回 None。"""
        assert self.store.query_fuzzy("操场") is None

    def test_fuzzy_empty_store(self):
        """空 store 的模糊查询应返回 None。"""
        empty = TaggedLocationStore()
        assert empty.query_fuzzy("任意") is None


class TestRemove:
    def setup_method(self):
        self.store = TaggedLocationStore()

    def test_remove_existing(self):
        """删除存在的标签应返回 True，之后 query 返回 None。"""
        self.store.tag("办公室", x=1.0, y=2.0)
        removed = self.store.remove("办公室")
        assert removed is True
        assert self.store.query("办公室") is None

    def test_remove_nonexistent(self):
        """删除不存在的标签应返回 False。"""
        removed = self.store.remove("不存在")
        assert removed is False

    def test_remove_does_not_affect_others(self):
        """删除一个标签不影响其他标签。"""
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
        """中文名称应完整保存和检索。"""
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
        """同名 tag 两次，取最新坐标。"""
        self.store.tag("地点A", x=1.0, y=2.0)
        self.store.tag("地点A", x=99.0, y=88.0)
        result = self.store.query("地点A")
        assert result["position"] == [99.0, 88.0, 0.0]

    def test_overwrite_count_stays_same(self):
        """同名覆盖后，总数不增加。"""
        self.store.tag("地点A", x=1.0, y=2.0)
        self.store.tag("地点A", x=5.0, y=6.0)
        assert len(self.store.list_all()) == 1


# ---------------------------------------------------------------------------
# 持久化测试
# ---------------------------------------------------------------------------

class TestSaveLoad:
    def test_save_and_load(self):
        """保存到文件再加载回来，数据应一致。"""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "tags.json")

            # 写
            store_write = TaggedLocationStore(path)
            store_write.tag("图书馆", x=10.0, y=20.0, z=0.5, yaw=1.0)
            store_write.tag("食堂", x=30.0, y=40.0)
            store_write.save()

            # 读
            store_read = TaggedLocationStore(path)
            lib = store_read.query("图书馆")
            canteen = store_read.query("食堂")

            assert lib is not None
            assert lib["position"] == [10.0, 20.0, 0.5]
            assert abs(lib["yaw"] - 1.0) < 1e-6

            assert canteen is not None
            assert canteen["position"] == [30.0, 40.0, 0.0]

    def test_load_nonexistent_file_ok(self):
        """加载不存在的文件不应抛出异常，store 应为空。"""
        store = TaggedLocationStore("/tmp/nonexistent_lingtu_tags_abc123.json")
        assert store.list_all() == []

    def test_save_empty_store(self):
        """空 store 保存为文件后，文件应为有效 JSON 数组。"""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "empty.json")
            store = TaggedLocationStore(path)
            store.save()
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            assert data == []

    def test_save_preserves_chinese(self):
        """JSON 文件应正确保存中文字符（ensure_ascii=False）。"""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "zh.json")
            store = TaggedLocationStore(path)
            store.tag("操场", x=1.0, y=2.0)
            store.save()
            with open(path, encoding="utf-8") as f:
                content = f.read()
            assert "操场" in content

    def test_in_memory_save_is_noop(self):
        """路径为空时，save() 不创建文件，也不抛异常。"""
        store = TaggedLocationStore()  # 无路径
        store.tag("A", x=1.0, y=2.0)
        store.save()  # 应静默无操作
