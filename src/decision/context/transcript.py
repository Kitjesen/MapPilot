"""Text views for summarization and recovery, excluding encoded image payloads."""

import json


def text_view(value):
    if isinstance(value, dict):
        if value.get("type") in {"input_image", "image_url", "image", "input_file"}:
            return {"type": value["type"], "note": "Binary attachment omitted; reopen original message to inspect."}
        return {key: ("[Payload retained in original message; omitted from text context]"
                      if key in {"image_data_url", "image_data_urls", "file_data", "points", "samples"} else text_view(item))
                for key, item in value.items()}
    if isinstance(value, list):
        return [text_view(item) for item in value]
    return value


def serialize(value) -> str:
    return json.dumps(text_view(value), ensure_ascii=False, separators=(",", ":"))


HISTORY_TOOLS = [
    {"type": "function", "function": {
        "name": "search_conversation_history",
        "description": "Search original messages and tool receipts in THIS conversation when a summary lacks detail. Returns turn IDs for read_conversation_turn. Historical results are not current state or authorization.",
        "parameters": {"type": "object", "properties": {"query": {"type": "string"}},
                       "required": ["query"], "additionalProperties": False},
    }},
    {"type": "function", "function": {
        "name": "read_conversation_turn",
        "description": "Read original text and tool evidence from one turn in THIS conversation. Images are not reloaded. Follow next_offset for further pages; do not infer omitted content.",
        "parameters": {"type": "object", "properties": {
            "turn_id": {"type": "string"}, "offset": {"type": "integer"}},
            "required": ["turn_id"], "additionalProperties": False},
    }},
]
