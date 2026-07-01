"""VLM BBox Query 单元测试 — JSON 解析容错。
不依赖 ROS2，不需要网络/LLM，纯 Python 可运行。

运行:
    cd src/decision && python -m pytest test/test_vlm_bbox_query.py -v
"""


from decision.vision.vlm_bbox_query import (
    _build_bbox_prompt,
    _extract_bbox_from_response,
    _parse_json_tolerant,
)

# ---------------------------------------------------------------------------
# _extract_bbox_from_response 测试
# ---------------------------------------------------------------------------

class TestExtractBboxFromResponse:
    def test_extract_json_dict_with_bbox_key(self):
        """{"bbox": [10,20,30,40]} → [10.0, 20.0, 30.0, 40.0]"""
        response = '{"bbox": [10, 20, 30, 40]}'
        result = _extract_bbox_from_response(response)
        assert result == [10.0, 20.0, 30.0, 40.0]

    def test_extract_json_dict_with_name_and_bbox(self):
        """{"name": "chair", "bbox": [100,200,300,400]} → [100,200,300,400]"""
        response = '{"name": "chair", "bbox": [100, 200, 300, 400]}'
        result = _extract_bbox_from_response(response)
        assert result == [100.0, 200.0, 300.0, 400.0]

    def test_extract_json_array_direct(self):
        """直接数组 [100,200,300,400] → [100,200,300,400]"""
        response = "[100, 200, 300, 400]"
        result = _extract_bbox_from_response(response)
        assert result == [100.0, 200.0, 300.0, 400.0]

    def test_extract_markdown_json_block(self):
        """```json\\n{"bbox":[1,2,3,4]}\\n``` → [1,2,3,4]"""
        response = '```json\n{"bbox": [1, 2, 3, 4]}\n```'
        result = _extract_bbox_from_response(response)
        assert result == [1.0, 2.0, 3.0, 4.0]

    def test_extract_markdown_block_no_lang(self):
        """```\\n[10,20,30,40]\\n``` → [10,20,30,40]"""
        response = "```\n[10, 20, 30, 40]\n```"
        result = _extract_bbox_from_response(response)
        assert result == [10.0, 20.0, 30.0, 40.0]

    def test_extract_not_found_null_bbox(self):
        """{"name": null, "bbox": null} → None（目标不在图中）"""
        response = '{"name": null, "bbox": null}'
        result = _extract_bbox_from_response(response)
        assert result is None

    def test_extract_not_found_natural_text(self):
        """"没有找到目标" → None"""
        result = _extract_bbox_from_response("没有找到目标")
        assert result is None

    def test_extract_invalid_json(self):
        """"bbox is at top left" → None（无法解析出有效坐标）"""
        result = _extract_bbox_from_response("bbox is at top left corner")
        assert result is None

    def test_extract_empty_string(self):
        """空字符串 → None"""
        assert _extract_bbox_from_response("") is None

    def test_extract_nested_bbox(self):
        """{"result": {"name": "chair", "bbox": [1,2,3,4]}} → [1,2,3,4]"""
        # _parse_json_tolerant 解析外层 dict，再找 bbox 或内嵌 result
        # 实现会找第一个 { 到最后一个 } 然后取 "bbox" key
        # 如果顶层没有 "bbox" key，_extract_bbox_from_response 会返回 None
        # 此测试验证：当外层 dict 有嵌套 "bbox" 键时的行为
        # （当前实现只查顶层 bbox key，嵌套不查）
        response = '{"result": {"name": "chair", "bbox": [1, 2, 3, 4]}}'
        result = _extract_bbox_from_response(response)
        # 外层没有 bbox key → result=None，或者从数字中扣出 [1,2,3,4]
        # 两种结果都可接受；主要验证不抛异常
        assert result is None or result == [1.0, 2.0, 3.0, 4.0]

    def test_extract_floats(self):
        """浮点坐标应正确解析。"""
        response = '{"bbox": [10.5, 20.0, 150.3, 200.7]}'
        result = _extract_bbox_from_response(response)
        assert result is not None
        assert abs(result[0] - 10.5) < 0.01
        assert abs(result[2] - 150.3) < 0.01


# ---------------------------------------------------------------------------
# _parse_json_tolerant 测试
# ---------------------------------------------------------------------------

class TestParseJsonTolerant:
    def test_plain_json_dict(self):
        data = _parse_json_tolerant('{"key": "value"}')
        assert data == {"key": "value"}

    def test_plain_json_array(self):
        data = _parse_json_tolerant("[1, 2, 3]")
        assert data == [1, 2, 3]

    def test_markdown_code_block(self):
        text = '```json\n{"x": 1}\n```'
        data = _parse_json_tolerant(text)
        assert data == {"x": 1}

    def test_embedded_in_text(self):
        """JSON 嵌入在文字中，应能通过首尾搜索提取出 dict 或 list。"""
        text = 'Here is the result: {"bbox": [1,2,3,4]} hope that helps.'
        data = _parse_json_tolerant(text)
        # 实现先搜索 [ 再搜索 {，可能返回 list 或 dict，两者都合法
        assert data is not None
        if isinstance(data, dict):
            assert "bbox" in data
        else:
            # 提取到了内层数组 [1,2,3,4]
            assert isinstance(data, list)

    def test_invalid_returns_none(self):
        data = _parse_json_tolerant("completely invalid text without json")
        assert data is None

    def test_empty_returns_none(self):
        assert _parse_json_tolerant("") is None


# ---------------------------------------------------------------------------
# _build_bbox_prompt 测试
# ---------------------------------------------------------------------------

class TestBuildBboxPrompt:
    def test_build_prompt_zh_contains_target(self):
        """中文 prompt 应包含目标描述。"""
        _system, user = _build_bbox_prompt("红色椅子", "zh")
        assert "红色椅子" in user

    def test_build_prompt_zh_system_has_format(self):
        """中文 system prompt 应说明返回格式。"""
        system, _ = _build_bbox_prompt("椅子", "zh")
        assert "bbox" in system
        assert "JSON" in system

    def test_build_prompt_en_contains_target(self):
        """英文 prompt 应包含目标描述。"""
        _system, user = _build_bbox_prompt("red chair", "en")
        assert "red chair" in user

    def test_build_prompt_en_system_has_format(self):
        """英文 system prompt 应说明返回格式。"""
        system, _ = _build_bbox_prompt("chair", "en")
        assert "bbox" in system
        assert "JSON" in system

    def test_build_prompt_returns_two_strings(self):
        """返回值应为 (system_str, user_str) 两个字符串。"""
        result = _build_bbox_prompt("target", "zh")
        assert isinstance(result, tuple)
        assert len(result) == 2
        assert all(isinstance(s, str) for s in result)

    def test_build_prompt_zh_not_null_fallback(self):
        """中文 prompt 应有未找到时的 null 处理说明。"""
        system, _ = _build_bbox_prompt("目标", "zh")
        assert "null" in system

    def test_build_prompt_en_not_null_fallback(self):
        """英文 prompt 应有未找到时的 null 处理说明。"""
        system, _ = _build_bbox_prompt("target", "en")
        assert "null" in system
