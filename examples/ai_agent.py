"""AI Agent controlling LingTu via MCP tools.

Demonstrates the ``LingTuMCP`` client pattern -- the same interface an LLM
(Claude, GPT) would use through the MCP protocol.

Requirements:
    pip install lingtu-sdk httpx

Usage:
    python examples/ai_agent.py              # connect to localhost
    python examples/ai_agent.py 192.168.66.190  # connect to robot
"""

from __future__ import annotations

import sys

from lingtu.sdk import LingTuClient
from lingtu.sdk.mcp import LingTuMCP


def main(host: str = "127.0.0.1") -> None:
    """Run the AI agent demonstration."""
    robot = LingTuClient(host, port=5050)
    mcp = LingTuMCP(robot)

    # -- System health ----------------------------------------------------
    health = mcp.call("get_health")
    print(f"Health: {health}")

    # -- Scene understanding ---------------------------------------------
    scene = mcp.call("get_scene_graph")
    print(f"Scene objects: {scene.get('count', 0)}")

    # -- Memory & locations ----------------------------------------------
    tags = mcp.call("list_tagged_locations")
    print(f"Tagged locations: {tags}")

    # -- Navigate using coordinates (MCP tool: navigate_to) --------------
    mcp.call("navigate_to", {"x": 10.0, "y": 5.0})

    # -- Navigate using semantic description (MCP tool: navigate_to_object)
    mcp.call("navigate_to_object", {"instruction": "go to the charger station"})

    # -- Query memory ----------------------------------------------------
    memory = mcp.call("query_memory", {"query": "背包"})
    print(f"Memory results: {memory.get('count', 0)}")

    # -- Detect objects --------------------------------------------------
    objects = mcp.call("detect_objects", {"query": "person"})
    print(f"Detected: {objects}")

    # -- Tag current position --------------------------------------------
    result = mcp.call("tag_location", {"name": "实验室入口"})
    print(f"Tagged: {result}")

    # -- Convenience methods (same tools, typed) ------------------------
    mcp.navigate_to(5.0, 3.0)
    mcp.find_object("fire extinguisher")
    mcp.query_memory("where did I see the backpack")
    mcp.stop()

    # -- Patrol ----------------------------------------------------------
    mcp.start_patrol([
        {"x": 0.0, "y": 0.0},
        {"x": 5.0, "y": 3.0, "yaw": 1.57},
    ])

    # -- Exploration ----------------------------------------------------
    print(mcp.begin_exploration())
    print(mcp.get_frontiers())

    # -- TARE exploration -----------------------------------------------
    print(mcp.start_tare_exploration())
    print(mcp.get_tare_status())

    # -- Voxel query ----------------------------------------------------
    print(mcp.get_voxel_stats())

    # -- GNSS fusion ----------------------------------------------------
    print(mcp.get_gnss_fusion_status())
    mcp.set_gnss_fusion(True)

    # -- Cleanup ---------------------------------------------------------
    robot.close()


if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    main(host)
