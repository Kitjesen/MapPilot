from tools.validate import validate_topics


def test_extract_nav_topics_from_config_detects_unquoted_yaml_topic_values(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(validate_topics, "ROOT_DIR", str(tmp_path))
    config = tmp_path / "runtime_topics.yaml"
    config.write_text("route:\n  topic: /nav/traversability\n", encoding="utf-8")

    found = validate_topics.extract_nav_topics_from_config(str(tmp_path))

    assert "/nav/traversability" in found


def test_extract_nav_topics_from_config_ignores_non_topic_absolute_paths(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(validate_topics, "ROOT_DIR", str(tmp_path))
    config = tmp_path / "paths.yaml"
    config.write_text(
        'install_prefix: "/nav/runtime"\ncache_dir: "/slam/cache"\n',
        encoding="utf-8",
    )

    found = validate_topics.extract_nav_topics_from_config(str(tmp_path))

    assert found == {}


def test_extract_nav_topics_from_config_ignores_non_topic_mapping_keys(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(validate_topics, "ROOT_DIR", str(tmp_path))
    config = tmp_path / "paths.yaml"
    config.write_text('"/nav/runtime": cache_directory\n', encoding="utf-8")

    found = validate_topics.extract_nav_topics_from_config(str(tmp_path))

    assert found == {}
