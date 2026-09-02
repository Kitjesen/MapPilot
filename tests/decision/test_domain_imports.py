"""Contract tests: verify perception and decision packages import without error."""


def test_import_perception():
    """perception package is importable."""
    import perception  # noqa: F401


def test_import_decision():
    """decision package is importable."""
    import decision  # noqa: F401


def test_import_semantic_reconstruction():
    """perception.reconstruction package is importable."""
    import perception.reconstruction  # noqa: F401


def test_import_all_subpackages():
    """The split perception/decision packages can be imported in one pass."""
    import decision
    import perception
    import perception.reconstruction as reconstruction

    assert perception is not None
    assert decision is not None
    assert reconstruction is not None
