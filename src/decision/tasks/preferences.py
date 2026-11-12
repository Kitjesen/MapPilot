"""Curated user preferences, adapted from Hermes memory-tool behavior.

See decision/llm/MEMORY_NOTICE.md for provenance and intentional differences.
"""

PREFERENCE_TOOLS = [
    {"type": "function", "function": {
        "name": "read_assistant_memory",
        "description": "List saved assistant/user preferences across conversations. This is not robot spatial memory.",
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    }},
    {"type": "function", "writes": True, "function": {
        "name": "remember_preference",
        "description": "Save one durable preference only when the user explicitly asks to remember it. Never save credentials, temporary task state, motion authorization or unverified observations. Maximum 1000 characters.",
        "parameters": {"type": "object", "properties": {"content": {"type": "string"}},
                       "required": ["content"], "additionalProperties": False},
    }},
    {"type": "function", "writes": True, "function": {
        "name": "update_preference",
        "description": "Update a saved preference at the user's request. Read memory first and supply the exact entry ID and old content. Maximum 1000 characters.",
        "parameters": {"type": "object", "properties": {
            "entry_id": {"type": "string"}, "old_content": {"type": "string"}, "content": {"type": "string"}},
            "required": ["entry_id", "old_content", "content"], "additionalProperties": False},
    }},
    {"type": "function", "writes": True, "function": {
        "name": "forget_preference",
        "description": "Remove one saved preference at the user's request. Read memory first and supply the exact entry ID and old content. This does not erase conversation history or shares.",
        "parameters": {"type": "object", "properties": {
            "entry_id": {"type": "string"}, "old_content": {"type": "string"}},
            "required": ["entry_id", "old_content"], "additionalProperties": False},
    }},
]
