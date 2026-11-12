from types import SimpleNamespace

from decision.context.budget import ToolContextBudget


def output(identity, text):
    return {"type": "function_call_output", "call_id": identity, "output": text}


def test_provider_anchor_reclaims_old_bodies_without_mutating_original_or_pairs():
    events = []
    budget = ToolContextBudget("turn-123", 1, events.append, token_budget=100)
    history = [output("old-history", "h" * 5000)]
    items = [*history, {"type": "function_call", "call_id": "one", "name": "lookup", "arguments": "{}"},
             output("one", "x" * 5000), {"type": "function_call", "call_id": "two", "name": "lookup", "arguments": "{}"},
             output("two", "y" * 5000)]
    assert budget.prepare(items) == items  # No fabricated provider count.
    budget.observe(SimpleNamespace(input_tokens=150))
    prepared = budget.prepare(items)
    assert prepared[0] == history[0] and prepared[-1] == items[-1]
    assert prepared[1] == items[1] and prepared[3] == items[3]
    assert "read_conversation_turn" in prepared[2]["output"] and "turn-123" in prepared[2]["output"]
    assert items[2]["output"] == "x" * 5000
    assert len(events) == 1 and events[0]["provider_input_tokens"] == 150
    assert budget.prepare(items) == prepared and len(events) == 1


def test_new_tool_text_is_estimated_separately_from_observed_tokens():
    events = []
    budget = ToolContextBudget("turn-123", 0, events.append, token_budget=2000)
    budget.prepare([])
    budget.observe(SimpleNamespace(input_tokens=100))
    budget.prepare([output("one", "x" * 5000), output("two", "y" * 5000)])
    assert events[0]["provider_input_tokens"] == 100
    assert events[0]["new_text_token_estimate"] > 2000
