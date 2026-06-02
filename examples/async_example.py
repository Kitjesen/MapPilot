"""Async LingTu SDK example.

Demonstrates ``AsyncLingTuClient`` for non-blocking robot control.

Requirements:
    pip install lingtu-sdk[aiohttp]

Usage:
    python examples/async_example.py              # connect to localhost
    python examples/async_example.py 192.168.66.190  # connect to robot
"""

from __future__ import annotations

import asyncio
import sys

from lingtu_sdk import AsyncLingTuClient
from lingtu_sdk.mcp import LingTuMCP


async def main(host: str = "127.0.0.1") -> None:
    """Run the async SDK demonstration."""
    async with AsyncLingTuClient(host, port=5050) as robot:
        # -- Concurrent state queries ------------------------------------
        health_task = asyncio.create_task(robot.health())
        state_task = asyncio.create_task(robot.state())

        health = await health_task
        state = await state_task

        print(f"Modules: {health.modules_ok}/{health.modules_total} ok")
        print(f"Mode: {state.mode}")

        # -- Navigation --------------------------------------------------
        await robot.go(10.0, 5.0)
        pos = await robot.position()
        print(f"Position: x={pos.x:.2f}, y={pos.y:.2f}")

        # -- Maps & locations --------------------------------------------
        ml = await robot.maps()
        for m in ml.maps:
            print(f"  Map: {m.name}  Active: {m.is_active}")

        await robot.tag_location("出发点", use_current_pose=True)

        # -- Session lifecycle -------------------------------------------
        await robot.start_session("navigating", map_name="factory_01")
        sess = await robot.session()
        print(f"Session: {sess.mode} / map: {sess.active_map}")
        await robot.end_session()

        # -- Explore -----------------------------------------------------
        await robot.start_session("exploring")
        sess = await robot.session()
        print(f"Explore session: {sess.mode}")
        await robot.end_session()


async def mcp_example(host: str = "127.0.0.1") -> None:
    """Demonstrate MCP tools from an async context."""
    robot = AsyncLingTuClient(host, port=5050)
    await robot.__aenter__()
    try:
        mcp = LingTuMCP(host=host, port=8090)

        tools = mcp.list_tools()
        print(f"Available MCP tools: {len(tools)}")

        health = mcp.call("get_health")
        print(f"Health: {health}")

        mcp.navigate_to(5.0, 3.0)
    finally:
        await robot.close()


if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    asyncio.run(main(host))
    asyncio.run(mcp_example(host))
