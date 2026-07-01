"""Quick Kimi-k2.5 API connectivity test."""
import asyncio
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from decision.llm_client import LLMConfig, create_llm_client


async def main():
    key = os.environ.get("MOONSHOT_API_KEY", "")
    print(f"API key: {key[:8]}...{key[-4:]}" if len(key) > 12 else "NOT SET")

    cfg = LLMConfig(
        backend="openai",
        model="kimi-k2.5",
        api_key_env="MOONSHOT_API_KEY",
        timeout_sec=60.0,
        max_retries=1,
        temperature=1.0,
        base_url="https://api.moonshot.cn/v1",
    )
    client = create_llm_client(cfg)
    print(f"Client available: {client.is_available()}")
    print(f"Model: {cfg.model}, Base: {cfg.base_url}")

    print("\n--- Test 1: Simple chat ---")
    resp = await client.chat([
        {"role": "system", "content": "Reply in JSON only."},
        {"role": "user", "content": 'Objects: [chair, door]. Instruction: find the chair. Output: {"target":"...","confidence":0.0-1.0}'},
    ])
    print(f"Response ({len(resp)} chars): {resp[:200]}")

    print("\n--- Test 2: Chinese cross-lingual ---")
    resp2 = await client.chat([
        {"role": "system", "content": "You are a navigation planner. Reply in JSON: {\"target_label\":\"...\",\"reasoning\":\"...\"}"},
        {"role": "user", "content": "Scene objects: fire extinguisher (id=2, near door), exit sign (id=3). Instruction: \u627e\u706d\u706b\u5668 (find fire extinguisher)"},
    ])
    print(f"Response ({len(resp2)} chars): {resp2[:200]}")

    print("\nAll tests passed!")


if __name__ == "__main__":
    asyncio.run(main())
