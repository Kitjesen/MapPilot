import pytest

from tools.validate import validate_topics


@pytest.mark.parametrize(
    "topic",
    (
        "/maps/local_collision",
        "/driver/control_state",
        "/sensors/control/request",
        "/sim/clock",
        "/robot/joint_states",
        "/gnss/fix",
        "/tf",
        "/tf_static",
    ),
)
def test_extract_current_topics(tmp_path, monkeypatch, topic):
    monkeypatch.setattr(validate_topics, "ROOT_DIR", str(tmp_path))
    config = tmp_path / "topics.yaml"
    config.write_text(f"route:\n  topic: {topic}\n", encoding="utf-8")
    source = tmp_path / "publisher.py"
    source.write_text(f'publish("{topic}")\n', encoding="utf-8")

    assert set(validate_topics.extract_nav_topics_from_config(str(tmp_path))) == {topic}
    assert set(validate_topics.extract_nav_topics_from_sources((str(source),))) == {topic}


def test_unknown_map_topic_is_reported(tmp_path, monkeypatch):
    monkeypatch.setattr(validate_topics, "ROOT_DIR", str(tmp_path))
    config = tmp_path / "topics.yaml"
    config.write_text("route:\n  topic: /maps/local_collison\n", encoding="utf-8")
    errors = []

    validate_topics._check_topic_locations(
        label="test config",
        found=validate_topics.extract_nav_topics_from_config(str(tmp_path)),
        contract_topics={"/maps/local_collision"},
        used_topics=set(),
        errors=errors,
    )

    assert len(errors) == 1
    assert "/maps/local_collison" in errors[0]


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
        'install_prefix: "/nav/runtime"\ncache_dir: "/slam/cache"\n'
        'map_dir: "/maps/cache"\ndriver_dir: "/driver/cache"\n'
        'sensor_dir: "/sensors/cache"\nsim_dir: "/sim/cache"\n',
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
