"""
robustness.py — 超时 / 重试 / GPU 安全装饰器

Canonical location: runtime.utils.robustness

用法:
    from runtime.utils.robustness import async_timeout, gpu_safe, retry

    @async_timeout(30.0)
    async def resolve_goal(self, ...):
        ...

    @gpu_safe(fallback=[])
    def detect(self, rgb):
        ...

    @retry(max_attempts=2, backoff=1.0, on=(ConnectionError,))
    def fetch_data(self):
        ...
"""

import asyncio
import functools
import logging
import random
import time
from collections.abc import Callable, Sequence
from typing import Any, TypeVar

logger = logging.getLogger(__name__)

F = TypeVar("F", bound=Callable[..., Any])
_sleep = time.sleep


# ──────────────────────────────────────────────
#  async_timeout — 异步方法超时装饰器
# ──────────────────────────────────────────────

def async_timeout(
    seconds: float = 30.0,
    *,
    timeout_return: Any = None,
    log_level: int = logging.ERROR,
):
    """为 async 方法添加 asyncio.wait_for 超时保护。

    Args:
        seconds: 超时秒数
        timeout_return: 超时后的返回值 (默认 None)
        log_level: 超时日志级别

    Example:
        @async_timeout(60.0)
        async def _resolve_goal(self):
            return await self._resolver.resolve(...)
    """

    def decorator(func: F) -> F:
        @functools.wraps(func)
        async def wrapper(*args, **kwargs):
            try:
                return await asyncio.wait_for(
                    func(*args, **kwargs), timeout=seconds
                )
            except asyncio.TimeoutError:
                logger.log(
                    log_level,
                    "%s timed out after %.1fs",
                    func.__qualname__,
                    seconds,
                )
                return timeout_return

        return wrapper  # type: ignore[return-value]

    return decorator


# ──────────────────────────────────────────────
#  gpu_safe — GPU 操作安全包装
# ──────────────────────────────────────────────

def gpu_safe(
    fallback: Any = None,
    *,
    clear_cache: bool = True,
    log_level: int = logging.ERROR,
):
    """捕获 CUDA OOM 和 RuntimeError，清理 GPU 缓存并返回 fallback。

    Args:
        fallback: GPU 错误后的返回值
        clear_cache: 是否调用 torch.cuda.empty_cache()
        log_level: 错误日志级别

    Example:
        @gpu_safe(fallback=[])
        def detect(self, rgb):
            results = self._model.predict(rgb)
            return results
    """

    def decorator(func: F) -> F:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except RuntimeError as e:
                err_msg = str(e).lower()
                is_cuda = "cuda" in err_msg or "out of memory" in err_msg
                logger.log(
                    log_level,
                    "%s failed%s: %s",
                    func.__qualname__,
                    " (CUDA OOM)" if is_cuda else "",
                    e,
                )
                if clear_cache and is_cuda:
                    _try_empty_cuda_cache()
                if callable(fallback):
                    return fallback()
                return fallback

        return wrapper  # type: ignore[return-value]

    return decorator


def _try_empty_cuda_cache():
    """安全地清理 CUDA 缓存。"""
    try:
        from runtime.msgs.numpy_compat import numpy_import_is_safe

        if not numpy_import_is_safe():
            logger.debug("Skipping torch CUDA cache cleanup: NumPy import is unsafe")
            return

        import torch

        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    except Exception:
        pass


# ──────────────────────────────────────────────
#  retry / retry_async — 重试装饰器
# ──────────────────────────────────────────────

def retry(
    max_attempts: int = 2,
    backoff: float = 1.0,
    on: Sequence[type[Exception]] = (Exception,),
    *,
    jitter: bool = True,
    log_level: int = logging.WARNING,
):
    """同步函数重试装饰器，指数退避 + 随机抖动。

    Args:
        max_attempts: 最大尝试次数 (含首次)
        backoff: 首次重试等待秒数 (后续 ×2)
        on: 捕获的异常类型
        jitter: 是否加随机抖动 (±25%) 防 thundering herd
        log_level: 重试日志级别
    """

    def decorator(func: F) -> F:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            last_exc = None
            for attempt in range(max_attempts):
                try:
                    return func(*args, **kwargs)
                except tuple(on) as e:
                    last_exc = e
                    if attempt < max_attempts - 1:
                        wait = backoff * (2**attempt)
                        if jitter:
                            wait *= 0.75 + random.random() * 0.5  # ±25%
                        logger.log(
                            log_level,
                            "%s attempt %d/%d failed: %s — retrying in %.1fs",
                            func.__qualname__,
                            attempt + 1,
                            max_attempts,
                            e,
                            wait,
                        )
                        _sleep(wait)
            raise last_exc  # type: ignore[misc]

        return wrapper  # type: ignore[return-value]

    return decorator


def retry_async(
    max_attempts: int = 2,
    backoff: float = 1.0,
    on: Sequence[type[Exception]] = (Exception,),
    *,
    jitter: bool = True,
    log_level: int = logging.WARNING,
):
    """异步函数重试装饰器，指数退避 + 随机抖动。"""

    def decorator(func: F) -> F:
        @functools.wraps(func)
        async def wrapper(*args, **kwargs):
            last_exc = None
            for attempt in range(max_attempts):
                try:
                    return await func(*args, **kwargs)
                except tuple(on) as e:
                    last_exc = e
                    if attempt < max_attempts - 1:
                        wait = backoff * (2**attempt)
                        if jitter:
                            wait *= 0.75 + random.random() * 0.5  # ±25%
                        logger.log(
                            log_level,
                            "%s attempt %d/%d failed: %s — retrying in %.1fs",
                            func.__qualname__,
                            attempt + 1,
                            max_attempts,
                            e,
                            wait,
                        )
                        await asyncio.sleep(wait)
            raise last_exc  # type: ignore[misc]

        return wrapper  # type: ignore[return-value]

    return decorator
