import asyncio
import json
from functools import partial

import httpx
import pytest

from decision.tasks.web import execute_web_tool


@pytest.fixture
def provider(monkeypatch):
    monkeypatch.setenv("TAVILY_API_KEY", "test-secret")
    original = httpx.AsyncClient

    def install(handler):
        monkeypatch.setattr(httpx, "AsyncClient", partial(original, transport=httpx.MockTransport(handler)))

    return install


def test_unconfigured_does_not_send_network_request(monkeypatch):
    monkeypatch.delenv("TAVILY_API_KEY", raising=False)
    assert "TAVILY_API_KEY" in asyncio.run(execute_web_tool("web_search", {"query": "robot"}))["error"]


def test_search_sends_bounded_query_and_normalizes_sources(provider):
    def request(req):
        assert str(req.url) == "https://api.tavily.com/search"
        assert req.headers["Authorization"] == "Bearer test-secret"
        assert json.loads(req.content)["max_results"] == 5
        return httpx.Response(200, json={"results": [
            {"title": "Robot", "url": "https://example.com/robot", "content": "a" * 1500},
            {"url": "javascript:alert(1)", "content": "bad"}, {"url": "https://example.com", "content": []}]})
    provider(request)
    result = asyncio.run(execute_web_tool("web_search", {"query": "robot"}))
    assert len(result["sources"]) == 2
    assert result["sources"][0]["truncated"] and len(result["sources"][0]["content"]) == 1200


def test_extract_returns_actual_provider_url_and_marks_truncation(provider):
    def request(req):
        assert json.loads(req.content)["urls"] == ["https://example.com/start"]
        return httpx.Response(200, json={"results": [
            {"url": "https://example.com/canonical", "raw_content": "x" * 17000}]})
    provider(request)
    result = asyncio.run(execute_web_tool("web_extract", {"url": "https://example.com/start"}))
    assert result["sources"][0]["url"].endswith("canonical")
    assert len(result["sources"][0]["content"]) == 16000


@pytest.mark.parametrize("url", ["file:///tmp/key", "javascript:alert(1)", "https://a:b@example.com", "http://[", ""])
def test_invalid_extract_url(url):
    assert "error" in asyncio.run(execute_web_tool("web_extract", {"url": url}))


@pytest.mark.parametrize("response", [httpx.Response(429, text="test-secret"), httpx.Response(200, text="bad"),
                                    httpx.Response(200, json={"results": [], "failed_results": [{"error": "private"}]})])
def test_extract_failures_do_not_look_like_success_or_expose_provider_body(provider, response):
    provider(lambda req: response)
    result = asyncio.run(execute_web_tool("web_extract", {"url": "https://example.com"}))
    assert "error" in result
    assert "test-secret" not in json.dumps(result) and "private" not in json.dumps(result)


def test_timeout_and_cancel(provider):
    async def timeout(req):
        raise httpx.ReadTimeout("timeout")
    provider(timeout)
    assert "超时" in asyncio.run(execute_web_tool("web_search", {"query": "robot"}))["error"]

    async def cancelled(req):
        raise asyncio.CancelledError
    provider(cancelled)
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(execute_web_tool("web_search", {"query": "robot"}))
