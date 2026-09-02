"""Decision module."""

import asyncio
from unittest import mock

import decision.vision.vlm_bbox as vlm_bbox

from decision.vision.vlm_bbox import (
    _build_bbox_prompt,
    _extract_bbox_from_response,
    _parse_json_tolerant,
)

# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------


class TestExtractBboxFromResponse:
    def test_extract_json_dict_with_bbox_key(self):
        """Test extract json dict with bbox key."""
        response = '{"bbox": [10, 20, 30, 40]}'
        result = _extract_bbox_from_response(response)
        assert result == [10.0, 20.0, 30.0, 40.0]

    def test_extract_json_dict_with_name_and_bbox(self):
        """Test extract json dict with name and bbox."""
        response = '{"name": "chair", "bbox": [100, 200, 300, 400]}'
        result = _extract_bbox_from_response(response)
        assert result == [100.0, 200.0, 300.0, 400.0]

    def test_extract_json_array_direct(self):
        """Test extract json array direct."""
        response = "[100, 200, 300, 400]"
        result = _extract_bbox_from_response(response)
        assert result == [100.0, 200.0, 300.0, 400.0]

    def test_extract_markdown_json_block(self):
        """Test extract markdown json block."""
        response = '```json\n{"bbox": [1, 2, 3, 4]}\n```'
        result = _extract_bbox_from_response(response)
        assert result == [1.0, 2.0, 3.0, 4.0]

    def test_extract_markdown_block_no_lang(self):
        """Test extract markdown block no lang."""
        response = "```\n[10, 20, 30, 40]\n```"
        result = _extract_bbox_from_response(response)
        assert result == [10.0, 20.0, 30.0, 40.0]

    def test_extract_not_found_null_bbox(self):
        """Test extract not found null bbox."""
        response = '{"name": null, "bbox": null}'
        result = _extract_bbox_from_response(response)
        assert result is None

    def test_extract_not_found_natural_text(self):
        """Test extract not found natural text."""
        result = _extract_bbox_from_response("没有找到目标")
        assert result is None

    def test_extract_invalid_json(self):
        """Test extract invalid json."""
        result = _extract_bbox_from_response("bbox is at top left corner")
        assert result is None

    def test_extract_empty_string(self):
        """Test extract empty string."""
        assert _extract_bbox_from_response("") is None

    def test_extract_nested_bbox(self):
        """Test extract nested bbox."""

        response = '{"result": {"name": "chair", "bbox": [1, 2, 3, 4]}}'
        result = _extract_bbox_from_response(response)

        assert result is None or result == [1.0, 2.0, 3.0, 4.0]

    def test_extract_floats(self):
        """Test extract floats."""
        response = '{"bbox": [10.5, 20.0, 150.3, 200.7]}'
        result = _extract_bbox_from_response(response)
        assert result is not None
        assert abs(result[0] - 10.5) < 0.01
        assert abs(result[2] - 150.3) < 0.01


# ---------------------------------------------------------------------------

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
        """Test embedded in text."""
        text = 'Here is the result: {"bbox": [1,2,3,4]} hope that helps.'
        data = _parse_json_tolerant(text)

        assert data is not None
        if isinstance(data, dict):
            assert "bbox" in data
        else:
            assert isinstance(data, list)

    def test_invalid_returns_none(self):
        data = _parse_json_tolerant("completely invalid text without json")
        assert data is None

    def test_empty_returns_none(self):
        assert _parse_json_tolerant("") is None


# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------


class TestBuildBboxPrompt:
    def test_build_prompt_zh_contains_target(self):
        """Test build prompt zh contains target."""
        _system, user = _build_bbox_prompt("红色椅子", "zh")
        assert "红色椅子" in user

    def test_build_prompt_zh_system_has_format(self):
        """Test build prompt zh system has format."""
        system, _ = _build_bbox_prompt("椅子", "zh")
        assert "bbox" in system
        assert "JSON" in system

    def test_build_prompt_en_contains_target(self):
        """Test build prompt en contains target."""
        _system, user = _build_bbox_prompt("red chair", "en")
        assert "red chair" in user

    def test_build_prompt_en_system_has_format(self):
        """Test build prompt en system has format."""
        system, _ = _build_bbox_prompt("chair", "en")
        assert "bbox" in system
        assert "JSON" in system

    def test_build_prompt_returns_two_strings(self):
        """Test build prompt returns two strings."""
        result = _build_bbox_prompt("target", "zh")
        assert isinstance(result, tuple)
        assert len(result) == 2
        assert all(isinstance(s, str) for s in result)

    def test_build_prompt_zh_not_null_fallback(self):
        """Test build prompt zh not null fallback."""
        system, _ = _build_bbox_prompt("目标", "zh")
        assert "null" in system

    def test_build_prompt_en_not_null_fallback(self):
        """Test build prompt en not null fallback."""
        system, _ = _build_bbox_prompt("target", "en")
        assert "null" in system


def _mock_client(responses):
    client = mock.MagicMock()

    async def chat_with_image(**_kwargs):
        response = responses.pop(0)
        if isinstance(response, Exception):
            raise response
        return response

    client.chat_with_image = chat_with_image
    return client


class TestQueryObjectBboxRetryCache:
    def setup_method(self):
        vlm_bbox._bbox_cache.clear()

    def test_retry_succeeds_on_second_attempt(self):
        client = _mock_client([RuntimeError("timeout"), '{"bbox": [10, 20, 100, 200]}'])
        with mock.patch("asyncio.sleep", new_callable=mock.AsyncMock):
            result = asyncio.run(vlm_bbox.query_object_bbox(client, "img_b64", "chair"))
        assert result == [10.0, 20.0, 100.0, 200.0]

    def test_stale_cache_is_returned_when_retries_fail(self):
        key = vlm_bbox._cache_key("chair", "img_b64")
        vlm_bbox._cache_put(key, [5.0, 6.0, 55.0, 66.0], confidence=1.0)
        client = _mock_client([RuntimeError("one"), RuntimeError("two"), RuntimeError("three")])

        with mock.patch("asyncio.sleep", new_callable=mock.AsyncMock):
            result = asyncio.run(vlm_bbox.query_object_bbox(client, "img_b64", "chair"))
        assert result == [5.0, 6.0, 55.0, 66.0]

    def test_no_cache_returns_none_when_retries_fail(self):
        client = _mock_client([RuntimeError("one"), RuntimeError("two"), RuntimeError("three")])
        with mock.patch("asyncio.sleep", new_callable=mock.AsyncMock):
            result = asyncio.run(vlm_bbox.query_object_bbox(client, "fresh_img", "sofa"))
        assert result is None
