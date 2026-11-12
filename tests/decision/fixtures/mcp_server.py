"""Real local MCP protocol fixture; it has no robot or external network access."""

import asyncio

from mcp.server.mcpserver import MCPServer

server = MCPServer("lingtu-test", log_level="ERROR")


@server.tool()
def lookup_manual(query: str) -> dict:
    return {"query": query, "answer": "Fixture manual: inspect the map before navigation.", "source": "manual://start"}


@server.tool()
def save_note(text: str) -> dict:
    return {"saved": text}


@server.tool()
def unavailable_manual() -> str:
    raise ValueError("Fixture manual unavailable")


@server.tool()
async def slow_lookup() -> str:
    await asyncio.sleep(60)
    return "late"


@server.resource("manual://start")
def manual() -> str:
    return "Fixture reference only. No robot actions were executed."


@server.prompt()
def explain(topic: str) -> str:
    return f"Explain {topic}; this is a fixture prompt, not execution permission."


if __name__ == "__main__":
    import sys

    transport = sys.argv[1] if len(sys.argv) > 1 else "stdio"
    server.run(transport=transport, **({"host": "127.0.0.1", "port": int(sys.argv[2])} if transport != "stdio" else {}))
