from __future__ import annotations

from perception.reconstruction.taxonomy import load_semantic_taxonomy


def test_semantic_taxonomy_is_stable_uint16_contract() -> None:
    taxonomy = load_semantic_taxonomy()

    assert taxonomy.name == "lingtu.semantic"
    assert taxonomy.version == 1
    assert taxonomy.ids["background"] == 0
    assert taxonomy.ids["chair"] == 6
    assert taxonomy.ids["desk"] == taxonomy.ids["table"] == 7
    assert taxonomy.encode(["chair", "unknown-class"]).tolist() == [6, 0]
    assert taxonomy.palette["6"]["name"] == "chair"
