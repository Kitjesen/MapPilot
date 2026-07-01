from pathlib import Path


def test_message_package_is_the_wire_contract_registry() -> None:
    from message.dds import topic_spec
    from runtime.runtime_interface import TOPICS

    assert Path("src/message").is_dir()
    assert Path("src/message/cpp/dds_topics.hpp").is_file()
    assert Path("src/message/idl/README.md").is_file()
    assert Path("src/runtime/msgs").is_dir()
    assert topic_spec(TOPICS.odometry).cpp_type == "lingtu::dds::Odometry"
    assert topic_spec(TOPICS.map_cloud).idl_type == "lingtu.dds.PointCloud2"


def test_python_dds_types_are_owned_by_message_package() -> None:
    readme = Path("src/message/idl/README.md").read_text(encoding="utf-8")
    from message.dds import topic_spec
    from runtime.runtime_interface import TOPICS

    assert "Python native DDS product types live in `message.dds_types`" in readme
    assert topic_spec(TOPICS.raw_lidar_points).import_path.startswith("message.dds_types.")
    assert topic_spec(TOPICS.odometry).import_path.startswith("message.dds_types.")


def test_cpp_topic_contracts_cover_registered_product_dds_topics() -> None:
    from message.dds import TOPIC_SPECS, dds_topic_name

    header = Path("src/message/cpp/dds_topics.hpp").read_text(encoding="utf-8")

    assert "RawMessage" not in header
    for spec in TOPIC_SPECS.values():
        assert f'"{spec.topic}"' in header
        assert f'"{dds_topic_name(spec.topic, typed=True)}"' in header
        assert f'"{spec.idl_type}"' in header
        assert f'"{spec.cpp_type}"' in header
