from __future__ import annotations

from decision.semantic_navigation.query import parse_object_query


def test_water_request_becomes_one_canonical_geometry_query() -> None:
    query = parse_object_query("帮我找一瓶水")

    assert query is not None
    assert query.label == "water_bottle"
    assert query.quantity == 1
    assert "水瓶" in query.aliases
    assert "water bottle" in query.aliases


def test_water_aliases_share_the_same_target_label() -> None:
    labels = {parse_object_query(text).label for text in ("找水瓶", "find a bottle of water", "找到矿泉水")}

    assert labels == {"water_bottle"}


def test_place_command_is_not_an_object_query() -> None:
    assert parse_object_query("去会议室") is None
