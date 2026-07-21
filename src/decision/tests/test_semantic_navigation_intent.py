from __future__ import annotations

import pytest

from decision.semantic_navigation import (
    HybridSemanticIntentParser,
    SemanticAction,
    SymbolicIntentError,
    TravelMode,
    normalize_floor_id,
)


@pytest.mark.parametrize(
    ("text", "target", "floor", "mode"),
    [
        ("带我到6楼某公司", "某公司", "floor-6", TravelMode.ANY),
        ("去六层某公司", "某公司", "floor-6", TravelMode.ANY),
        ("走楼梯去6楼某公司", "某公司", "floor-6", TravelMode.STAIRS),
        ("坐电梯去6楼某公司", "某公司", "floor-6", TravelMode.ELEVATOR),
        ("请带我到第六层，某公司。", "某公司", "floor-6", TravelMode.ANY),
    ],
)
def test_first_release_navigation_phrases(text, target, floor, mode):
    result = HybridSemanticIntentParser().parse(text)

    assert result is not None
    assert result.action is SemanticAction.NAVIGATE
    assert result.target_query == target
    assert result.floor_id == floor
    assert result.travel_mode is mode


@pytest.mark.parametrize(
    ("text", "target"),
    [
        ("去电梯厅", "电梯厅"),
        ("去楼梯间", "楼梯间"),
        ("去安全梯口", "安全梯口"),
    ],
)
def test_connector_place_names_are_not_mistaken_for_travel_preferences(text, target):
    result = HybridSemanticIntentParser().parse(text)

    assert result is not None
    assert result.action is SemanticAction.NAVIGATE
    assert result.target_query == target
    assert result.travel_mode is TravelMode.ANY


@pytest.mark.parametrize(
    ("text", "action"),
    [
        ("暂停导览", SemanticAction.PAUSE_TOUR),
        ("继续", SemanticAction.RESUME_TOUR),
        ("取消导览", SemanticAction.CANCEL_TOUR),
    ],
)
def test_tour_lifecycle_phrases(text, action):
    result = HybridSemanticIntentParser().parse(text)

    assert result is not None
    assert result.action is action


def test_start_named_tour_keeps_symbolic_identifier():
    result = HybridSemanticIntentParser().parse("开始展厅导览A")

    assert result is not None
    assert result.action is SemanticAction.START_TOUR
    assert result.tour_id == "展厅导览A"
    assert result.target_query == "展厅导览A"


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        ("6楼", "floor-6"),
        ("六层", "floor-6"),
        ("第六层", "floor-6"),
        ("负一楼", "floor-b1"),
        ("二十三层", "floor-23"),
        ("floor_6", "floor-6"),
        ("floor-6", "floor-6"),
        ("floor-b1", "floor-b1"),
    ],
)
def test_floor_normalization(raw, expected):
    assert normalize_floor_id(raw) == expected


def test_fast_path_does_not_require_llm():
    calls = []
    parser = HybridSemanticIntentParser(lambda text: calls.append(text) or None)

    result = parser.parse("坐电梯去6楼某公司")

    assert result is not None
    assert result.source == "rules"
    assert calls == []


def test_unknown_text_can_use_symbolic_llm_fallback():
    parser = HybridSemanticIntentParser(
        lambda _text: {
            "action": "navigate",
            "target_query": "某公司",
            "floor_id": "六楼",
            "travel_mode": "stairs",
            "confidence": 0.81,
        }
    )

    result = parser.parse("劳驾领路去那家六楼的公司")

    assert result is not None
    assert result.source == "llm"
    assert result.target_query == "某公司"
    assert result.floor_id == "floor-6"
    assert result.travel_mode is TravelMode.STAIRS


@pytest.mark.parametrize(
    "payload",
    [
        {"action": "navigate", "target_query": "某公司", "x": 1.0},
        {"action": "navigate", "target_query": "某公司", "target": {"pose": [1, 2]}},
        {"action": "navigate", "target_query": "某公司", "coordinates": [1, 2, 3]},
    ],
)
def test_llm_coordinates_are_rejected(payload):
    parser = HybridSemanticIntentParser(lambda _text: payload)

    with pytest.raises(SymbolicIntentError, match="coordinate field is forbidden"):
        parser.parse("请理解这句话")


@pytest.mark.parametrize(
    "payload",
    [
        {"action": "navigate", "target_query": "某公司", "waypoint": [1, 2, 3]},
        {"action": "navigate", "target_query": {"name": "某公司"}},
        {"action": "navigate", "target_query": "某公司", "floor_id": True},
        {
            "action": "navigate",
            "target_query": "某公司",
            "needs_clarification": "false",
        },
        {"action": "navigate", "target_query": "某公司", "confidence": True},
    ],
)
def test_llm_result_accepts_only_strict_symbolic_schema(payload):
    parser = HybridSemanticIntentParser(lambda _text: payload)

    with pytest.raises(SymbolicIntentError):
        parser.parse("请理解这句话")


def test_floor_only_command_requests_clarification_instead_of_inventing_place():
    result = HybridSemanticIntentParser().parse("去六楼")

    assert result is not None
    assert result.action is SemanticAction.NAVIGATE
    assert result.target_query == ""
    assert result.needs_clarification is True
    assert result.reason == "place_required"


@pytest.mark.parametrize("action", ["navigate", "start_tour"])
def test_llm_may_request_clarification_without_inventing_an_identifier(action: str):
    intent = HybridSemanticIntentParser.from_symbolic_mapping(
        {
            "action": action,
            "needs_clarification": True,
            "reason": "identifier_required",
        },
        raw_text="请继续",
    )

    assert intent.needs_clarification is True
    assert intent.target_query == ""
    assert intent.tour_id == ""


def test_unrelated_text_is_not_forced_into_navigation():
    assert HybridSemanticIntentParser().parse("今天天气怎么样") is None
