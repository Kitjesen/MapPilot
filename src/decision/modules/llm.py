"""LLM backend Module."""

from __future__ import annotations

import asyncio
import logging
import os
import threading
import time
from dataclasses import dataclass
from typing import Any

from runtime.backend_status import BackendStatus
from runtime.module import Module
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Request / Response types
# ---------------------------------------------------------------------------


@dataclass
class LLMRequest:
    """L L M Request."""

    messages: list  # OpenAI format: [{"role": "system", "content": "..."}]
    request_id: str = ""  # caller-assigned ID for routing responses
    temperature: float = 0.2
    caller: str = ""  # which module sent this

    @staticmethod
    def simple(prompt: str, system: str = "", request_id: str = "", caller: str = ""):
        """Convenience: single user prompt."""
        msgs = []
        if system:
            msgs.append({"role": "system", "content": system})
        msgs.append({"role": "user", "content": prompt})
        return LLMRequest(messages=msgs, request_id=request_id, caller=caller)


@dataclass
class LLMResponse:
    """L L M Response."""

    text: str
    request_id: str = ""
    model: str = ""
    latency_ms: float = 0.0
    error: str = ""  # non-empty if failed

    @property
    def ok(self) -> bool:
        return not self.error


# ---------------------------------------------------------------------------
# LLMModule
# ---------------------------------------------------------------------------


@register("llm", "pluggable", description="Pluggable LLM backend module")
class LLMModule(Module, layer=4):
    """Pluggable LLM Module - async chat via In/Out ports.

    In:  request  (LLMRequest)   - prompt from any module
    Out: response (LLMResponse)  - LLM reply routed back

    The async event loop runs in a background thread so LLM calls
    don't block the Module system's synchronous callback chain.
    """

    request: In[LLMRequest]
    response: Out[LLMResponse]

    def __init__(
        self,
        backend: str = "kimi",
        model: str = "",
        api_key_env: str = "",
        timeout_sec: float = 10.0,
        temperature: float = 0.2,
        base_url: str = "",
        **kw,
    ):
        super().__init__(**kw)
        self._backend_status = BackendStatus.configured_as(backend)
        self._backend_name = backend
        self._canonical_backend_name = backend
        self._model = model
        self._api_key_env = api_key_env
        self._timeout_sec = timeout_sec
        self._temperature = temperature
        self._base_url = base_url
        self._client = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_thread: threading.Thread | None = None
        self._call_count = 0
        self._total_latency_ms = 0.0
        self._error_count = 0
        self._backend_lock = threading.RLock()
        self._in_flight = 0

        self._cb_threshold: int = kw.get("circuit_breaker_threshold", 5)
        self._cb_cooldown: float = kw.get("circuit_breaker_cooldown", 60.0)
        self._consecutive_failures: int = 0
        self._circuit_open_until: float = 0.0  # time.time() when circuit re-closes

    def preflight(self):
        """Check API key availability before startup."""
        if self._backend_name == "mock":
            return None
        defaults = {
            "kimi": "MOONSHOT_API_KEY",
            "openai": "OPENAI_API_KEY",
            "claude": "ANTHROPIC_API_KEY",
            "qwen": "DASHSCOPE_API_KEY",
        }
        env_var = self._api_key_env or defaults.get(self._backend_name, "")
        if env_var and not os.environ.get(env_var):
            return (
                f"API key env var '{env_var}' not set for backend '{self._backend_name}'. Set it or use backend='mock'."
            )
        return None

    def setup(self) -> None:
        """Create LLM client and start async event loop."""
        self._client = self._create_client()
        self.request.subscribe(self._on_request)

        # Start async loop in background thread for non-blocking LLM calls
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever, daemon=True, name=f"llm-{self._backend_name}"
        )
        self._loop_thread.start()
        logger.info("LLMModule: backend='%s' model='%s'", self._backend_name, self._model)

    @property
    def client(self):
        """Return the initialized client for same-layer multimodal helpers."""
        return self._client

    def _create_client(self):
        """Factory: instantiate the selected LLM backend."""
        return self._create_client_for(
            self._backend_name,
            model=self._model,
            api_key_env=self._api_key_env,
            timeout_sec=self._timeout_sec,
            temperature=self._temperature,
            base_url=self._base_url,
        )

    def _create_client_for(
        self,
        backend: str,
        *,
        model: str = "",
        api_key_env: str = "",
        timeout_sec: float | None = None,
        temperature: float | None = None,
        base_url: str = "",
    ):
        """Instantiate an LLM backend without mutating active state."""
        from decision.llm.client import LLMConfig, create_llm_client, resolve_llm_backend

        # Resolve defaults per backend
        defaults = {
            "kimi": {"model": "kimi-k2.5", "api_key_env": "MOONSHOT_API_KEY", "base_url": "https://api.moonshot.cn/v1"},
            "openai": {"model": "gpt-4o-mini", "api_key_env": "OPENAI_API_KEY"},
            "claude": {"model": "claude-3-5-sonnet-20241022", "api_key_env": "ANTHROPIC_API_KEY"},
            "qwen": {"model": "qwen-turbo", "api_key_env": "DASHSCOPE_API_KEY"},
            "mock": {"model": "mock", "api_key_env": ""},
        }
        canonical = resolve_llm_backend(backend)
        d = defaults.get(backend, defaults.get(canonical, {}))

        config = LLMConfig(
            backend=backend,
            model=model or d.get("model", ""),
            api_key_env=api_key_env or d.get("api_key_env", ""),
            timeout_sec=timeout_sec if timeout_sec is not None else self._timeout_sec,
            temperature=temperature if temperature is not None else self._temperature,
            base_url=base_url or d.get("base_url", ""),
        )
        return create_llm_client(config)

    def _on_request(self, req: LLMRequest):
        """Dispatch LLM call to async loop, publish response when done."""
        if self._client is None or self._loop is None:
            self.response.publish(LLMResponse(text="", request_id=req.request_id, error="LLM client not initialized"))
            return

        now = time.time()
        if self._consecutive_failures >= self._cb_threshold:
            if now < self._circuit_open_until:
                self.response.publish(
                    LLMResponse(
                        text="",
                        request_id=req.request_id,
                        model=self._model,
                        error=f"Circuit breaker open ({self._consecutive_failures} "
                        f"consecutive failures, retry in "
                        f"{self._circuit_open_until - now:.0f}s)",
                    )
                )
                return

            logger.info("LLMModule: circuit half-open, probing backend")

        with self._backend_lock:
            self._in_flight += 1
        future = asyncio.run_coroutine_threadsafe(self._async_chat(req), self._loop)
        # Non-blocking: response published from async callback
        future.add_done_callback(lambda f: self._handle_result(f, req))

    def _is_transient_error(self, error: Exception) -> bool:
        """Transient errors should count toward circuit breaker but with lower weight."""
        msg = str(error).lower()
        transient_patterns = (
            "timeout",
            "timed out",
            "429",
            "rate limit",
            "connection",
            "temporary",
            "unavailable",
            "503",
        )
        return any(p in msg for p in transient_patterns)

    async def _async_chat(self, req: LLMRequest) -> str:
        """Run the actual LLM call with enforced timeout."""
        return await asyncio.wait_for(
            self._client.chat(req.messages, temperature=req.temperature),
            timeout=self._timeout_sec,
        )

    def _handle_result(self, future, req: LLMRequest):
        """Callback when async LLM call completes."""
        t0 = time.time()
        try:
            text = future.result(timeout=self._timeout_sec + 5)
            latency_ms = (time.time() - t0) * 1000
            self._call_count += 1
            self._total_latency_ms += latency_ms

            if self._consecutive_failures > 0:
                logger.info("LLMModule: backend recovered after %d failures", self._consecutive_failures)
            self._consecutive_failures = 0
            self._circuit_open_until = 0.0
            self.response.publish(
                LLMResponse(
                    text=text,
                    request_id=req.request_id,
                    model=self._model,
                    latency_ms=latency_ms,
                )
            )
        except Exception as e:
            self._error_count += 1
            if self._is_transient_error(e):
                self._consecutive_failures += 1
            else:
                # Permanent errors (401, 403, bad model) open circuit immediately
                self._consecutive_failures = self._cb_threshold
            # Open circuit after threshold consecutive failures
            if self._consecutive_failures >= self._cb_threshold:
                self._circuit_open_until = time.time() + self._cb_cooldown
                logger.warning(
                    "LLMModule: circuit breaker OPEN after %d consecutive failures (cooldown %.0fs)",
                    self._consecutive_failures,
                    self._cb_cooldown,
                )
            logger.error("LLMModule: call failed (%d/%d): %s", self._consecutive_failures, self._cb_threshold, e)
            self.response.publish(
                LLMResponse(
                    text="",
                    request_id=req.request_id,
                    model=self._model,
                    error=str(e),
                )
            )
        finally:
            with self._backend_lock:
                self._in_flight = max(0, self._in_flight - 1)

    def stop(self):
        """Shutdown async loop and client."""
        loop = self._loop
        if loop is not None:
            loop.call_soon_threadsafe(loop.stop)
        if self._loop_thread:
            self._loop_thread.join(timeout=3.0)
        if loop is not None:
            try:
                loop.close()
            except RuntimeError:
                pass
        self._loop = None
        self._loop_thread = None
        super().stop()

    def reconfigure_backend(
        self,
        category: str,
        backend: str,
        **config: Any,
    ) -> dict[str, Any]:
        if category not in {"llm", "llm_client"}:
            return super().reconfigure_backend(category, backend, **config)

        from decision.llm.client import available_llm_backends, resolve_llm_backend

        available = available_llm_backends()
        if backend not in available:
            return {
                "ok": False,
                "category": "llm",
                "requested_backend": backend,
                "reason": "unknown_backend",
                "available": available,
            }

        with self._backend_lock:
            if self._in_flight > 0:
                return {
                    "ok": False,
                    "category": "llm",
                    "requested_backend": backend,
                    "reason": "backend_reconfigure_in_flight",
                    "in_flight": self._in_flight,
                }
            previous_backend = self._backend_name
            previous_client = self._client
            previous_status = self._backend_status
            model = str(config.get("model", self._model) or "")
            api_key_env = str(config.get("api_key_env", self._api_key_env) or "")
            base_url = str(config.get("base_url", self._base_url) or "")
            timeout_sec = float(config.get("timeout_sec", self._timeout_sec))
            temperature = float(config.get("temperature", self._temperature))
            try:
                client = self._create_client_for(
                    backend,
                    model=model,
                    api_key_env=api_key_env,
                    timeout_sec=timeout_sec,
                    temperature=temperature,
                    base_url=base_url,
                )
            except ValueError:
                return {
                    "ok": False,
                    "category": "llm",
                    "requested_backend": backend,
                    "reason": "unknown_backend",
                    "available": available_llm_backends(),
                }
            except Exception as exc:
                self._client = previous_client
                self._backend_status = previous_status
                return {
                    "ok": False,
                    "category": "llm",
                    "requested_backend": backend,
                    "reason": "backend_reconfigure_failed",
                    "error": str(exc),
                }

            self._backend_name = backend
            self._canonical_backend_name = resolve_llm_backend(backend)
            client_config = getattr(client, "config", None)
            self._model = model or getattr(client_config, "model", self._model)
            self._api_key_env = api_key_env
            self._base_url = base_url
            self._timeout_sec = timeout_sec
            self._temperature = temperature
            self._client = client
            self._backend_status = BackendStatus.configured_as(backend)
            self._backend_status.use(backend, degraded=False)
            self._consecutive_failures = 0
            self._circuit_open_until = 0.0
            return {
                "ok": True,
                "category": "llm",
                "previous_backend": previous_backend,
                "backend": backend,
                "degraded": False,
            }

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        avg_ms = self._total_latency_ms / self._call_count if self._call_count > 0 else 0.0
        circuit_state = "closed"
        if self._consecutive_failures >= self._cb_threshold:
            circuit_state = "half-open" if time.time() >= self._circuit_open_until else "open"
        info["llm"] = {
            **self._backend_status.as_health_fields(),
            "canonical_backend": self._canonical_backend_name,
            "model": self._model,
            "calls": self._call_count,
            "errors": self._error_count,
            "avg_ms": round(avg_ms, 1),
            "circuit_breaker": circuit_state,
            "consecutive_failures": self._consecutive_failures,
        }
        return info
