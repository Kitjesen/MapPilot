"""Provider-anchored in-loop reclamation of old tool bodies, with exact read-back."""

import math

from decision.context.transcript import serialize

PROMPT_TOKEN_BUDGET = 24000  # LingTu working budget, not a provider's maximum window.


class ToolContextBudget:
    def __init__(self, turn_id, history_length, on_event, token_budget=PROMPT_TOKEN_BUDGET):
        self.turn_id = turn_id
        self.history_length = history_length
        self.on_event = on_event
        self.token_budget = token_budget
        self.input_tokens = None
        self.sent_characters = 0
        self.anchored_characters = 0
        self.demoted = {}

    def observe(self, usage):
        self.input_tokens = usage.input_tokens
        self.anchored_characters = self.sent_characters

    def prepare(self, items):
        prepared = [{**item, "output": self.demoted[item["call_id"]]}
                    if item.get("type") == "function_call_output" and item.get("call_id") in self.demoted else item
                    for item in items]
        characters = len(serialize(prepared))
        added_estimate = math.ceil(max(0, characters - self.anchored_characters) / 3)
        if self.input_tokens is not None and self.input_tokens + added_estimate >= self.token_budget:
            outputs = [index for index, item in enumerate(prepared) if index >= self.history_length
                       and item.get("type") == "function_call_output"]
            newly_demoted = []
            for index in outputs[:-1]:
                item = prepared[index]
                if item["call_id"] in self.demoted or len(serialize(item["output"])) < 4000:
                    continue
                reference = (f"Historical tool body retained in original conversation turn {self.turn_id}; "
                             f"call_id={item['call_id']}. Use read_conversation_turn with this turn_id and page offsets "
                             "to recover the exact text before relying on missing details. The tool was already called; do not repeat its side effects.")
                self.demoted[item["call_id"]] = reference
                prepared[index] = {**item, "output": reference}
                newly_demoted.append(item["call_id"])
            if newly_demoted:
                self.on_event({"type": "context_budget", "status": "completed", "scope": "in_loop",
                    "basis": "provider_input_tokens_plus_new_text_estimate", "provider_input_tokens": self.input_tokens,
                    "new_text_token_estimate": added_estimate, "budget_tokens": self.token_budget,
                    "demoted_call_ids": newly_demoted, "turn_id": self.turn_id})
        self.sent_characters = len(serialize(prepared))
        return prepared
