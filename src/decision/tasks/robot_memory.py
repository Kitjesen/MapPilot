"""Durable robot memory layers exposed through the conversation tool boundary."""

MEMORY_LAYERS = ("operator", "spatial", "episodic", "procedural", "working")
BOT_MEMORY_LAYERS = ("operator", "procedural", "working")
ROBOT_MEMORY_LAYERS = ("spatial", "episodic")

ROBOT_MEMORY_TOOLS = [
    {"type": "function", "function": {
        "name": "read_robot_memory",
        "description": "Read durable robot memory by layer or text query. Layers are operator (profile), spatial, episodic, procedural and working. This is read-only; do not treat memories as current sensor truth or motion permission.",
        "parameters": {"type": "object", "properties": {
            "layer": {"type": "string", "enum": list(MEMORY_LAYERS)},
            "domain": {"type": "string", "enum": ["bot", "robot"]},
            "owner_id": {"type": "string", "maxLength": 120},
            "query": {"type": "string", "maxLength": 200},
            "limit": {"type": "integer", "minimum": 1, "maximum": 50}},
            "additionalProperties": False},
    }},
    {"type": "function", "writes": True, "function": {
        "name": "remember_robot_memory",
        "description": "Save one robot memory only when the operator explicitly asks to remember it. Provide the layer and provenance. Never store credentials, motion authorization, or an unverified observation as fact. Working memories should use a short TTL.",
        "parameters": {"type": "object", "properties": {
            "layer": {"type": "string", "enum": list(MEMORY_LAYERS)},
            "domain": {"type": "string", "enum": ["bot", "robot"]},
            "owner_id": {"type": "string", "maxLength": 120},
            "content": {"type": "string", "minLength": 1, "maxLength": 2000},
            "source": {"type": "string", "maxLength": 200},
            "confidence": {"type": "number", "minimum": 0, "maximum": 1},
            "ttl_seconds": {"type": "integer", "minimum": 60, "maximum": 31536000}},
            "required": ["layer", "content"], "additionalProperties": False},
    }},
    {"type": "function", "writes": True, "function": {
        "name": "update_robot_memory",
        "description": "Update one robot memory after reading it first. Supply the exact memory ID and old content to prevent overwriting a concurrent change.",
        "parameters": {"type": "object", "properties": {
            "memory_id": {"type": "string"},
            "old_content": {"type": "string"},
            "domain": {"type": "string", "enum": ["bot", "robot"]},
            "owner_id": {"type": "string", "maxLength": 120},
            "content": {"type": "string", "minLength": 1, "maxLength": 2000},
            "source": {"type": "string", "maxLength": 200},
            "confidence": {"type": "number", "minimum": 0, "maximum": 1},
            "ttl_seconds": {"type": "integer", "minimum": 60, "maximum": 31536000}},
            "required": ["memory_id", "old_content", "content"], "additionalProperties": False},
    }},
    {"type": "function", "writes": True, "function": {
        "name": "forget_robot_memory",
        "description": "Forget one robot memory only after reading it and receiving an explicit operator request. This removes the memory record but does not rewrite conversation history or inspection evidence.",
        "parameters": {"type": "object", "properties": {
            "memory_id": {"type": "string"},
            "old_content": {"type": "string"},
            "domain": {"type": "string", "enum": ["bot", "robot"]},
            "owner_id": {"type": "string", "maxLength": 120}},
            "required": ["memory_id", "old_content"], "additionalProperties": False},
    }},
]

# Bot memory aliases make ownership explicit while the legacy robot-memory
# tools remain a compatible cross-domain read/write surface.
BOT_MEMORY_TOOLS = [{**tool, "function": {**tool["function"],
    "name": tool["function"]["name"].replace("_robot_memory", "_bot_memory"),
    "description": tool["function"]["description"].replace("robot memory", "Bot memory").replace("robot_memory", "bot_memory"),
}} for tool in ROBOT_MEMORY_TOOLS]
