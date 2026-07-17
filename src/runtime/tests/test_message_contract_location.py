import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
TOPIC_CONTRACT_SOURCE = "src/runtime/runtime_interface.py"
ALLOWED_PRODUCT_TOPIC_LITERAL_PATHS = (
    "src/runtime/runtime_interface.py",
    "src/message/",
    "src/runtime/route_contract/",
    "src/runtime/endpoints/",
    "src/runtime/adapters/",
    "src/runtime/transport/",
    "src/runtime/tf/",
    "src/runtime/remote_ports.py",
    "src/localization/adapters/",
    "src/localization/launch/",
    "src/perception/adapters/",
    "src/nav/adapters/",
    "src/drivers/adapters/",
    "src/drivers/real/lidar/",
)
CHECKED_PRODUCT_TOPIC_ROOTS = (
    "cli",
    "src/decision",
    "src/drivers",
    "src/gateway",
    "src/localization",
    "src/memory",
    "src/nav",
    "src/perception",
    "src/runtime",
)
SKIPPED_PRODUCT_TOPIC_PATH_PARTS = frozenset(
    {
        "3rdparty",
        "3rd-party",
        "_deps",
        "build",
        "vendor",
        "livox_ros_driver2",
        "OrbbecSDK_ROS2",
    }
)


def test_message_package_is_the_wire_contract_registry() -> None:
    from message.dds import topic_spec
    from runtime.runtime_interface import TOPICS

    assert Path("src/message").is_dir()
    assert Path("src/message/cpp/dds_topics.hpp").is_file()
    assert Path("src/message/idl/README.md").is_file()
    assert Path("src/runtime/msgs").is_dir()
    assert topic_spec(TOPICS.odometry).cpp_type == "lingtu::dds::Odometry"
    assert topic_spec(TOPICS.map_cloud).idl_type == "lingtu.dds.PointCloud2"


def test_driver_odometry_has_one_canonical_runtime_topic() -> None:
    from message.dds import dds_topic_name, topic_spec
    from runtime.runtime_interface import TOPICS, topic_allowed_frame_ids, topic_formats

    assert TOPICS.driver_odometry == "/driver/odometry"
    assert TOPICS.dog_odometry == TOPICS.driver_odometry
    assert topic_formats(TOPICS.driver_odometry) == ("odometry",)
    assert topic_allowed_frame_ids(TOPICS.driver_odometry) == ("odom",)
    assert dds_topic_name(TOPICS.driver_odometry, typed=True) == "rt/driver/odometry"
    assert topic_spec(TOPICS.driver_odometry).cpp_type == "lingtu::dds::Odometry"


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


def test_camera_dds_topics_match_native_camera_publisher() -> None:
    from message.dds import dds_topic_name, dds_type_for_topic, topic_spec
    from message.dds_types.camera import CameraInfo, Image
    from runtime.runtime_interface import TOPICS

    expected = {
        TOPICS.camera_color: ("Image", Image, "rt/camera/color", "kCameraColor"),
        TOPICS.camera_depth: ("Image", Image, "rt/camera/depth", "kCameraDepth"),
        TOPICS.camera_info: ("CameraInfo", CameraInfo, "rt/camera/info", "kCameraInfo"),
    }

    idl = Path("src/message/idl/lingtu_slam.idl").read_text(encoding="utf-8")
    cpp_topics = Path("src/message/cpp/dds_topics.hpp").read_text(encoding="utf-8")
    publisher = Path("src/drivers/real/camera/native/camera_dds.cpp").read_text(encoding="utf-8")
    reader = Path("src/drivers/real/camera/dds_module.py").read_text(encoding="utf-8")

    assert "struct Image" in idl
    assert "sequence<octet> data;" in idl
    assert "struct CameraInfo" in idl
    assert "sequence<double> d;" in idl
    assert "double k[9];" in idl
    assert "double p[12];" in idl

    for topic, (type_name, dds_type, dds_topic, cpp_constant) in expected.items():
        spec = topic_spec(topic)
        assert spec is not None
        assert spec.type_name == type_name
        assert spec.dds_topic == dds_topic
        assert spec.import_path.startswith("message.dds_types.camera.")
        assert spec.idl_type == f"lingtu.dds.{type_name}"
        assert spec.cpp_type == f"lingtu::dds::{type_name}"
        assert dds_topic_name(topic, typed=True) == dds_topic
        assert dds_type_for_topic(topic) is dds_type
        assert f"inline constexpr TopicContract {cpp_constant}" in cpp_topics
        assert f'"{topic}", "{dds_topic}"' in cpp_topics
        assert f"{cpp_constant}.dds_topic" in publisher
        if topic == TOPICS.camera_info:
            assert f"TOPICS.{_topic_attr(topic)}" in reader or "ShmFrameReader" in reader

    assert "lingtu_dds_Image_desc" in publisher
    assert "lingtu_dds_CameraInfo_desc" in publisher
    assert "ShmFrameReader" in reader
    assert "runtime.adapters.dds" not in reader
    assert "cyclonedds" not in reader


def test_gnss_dds_topics_match_native_gnss_publisher() -> None:
    from message.dds import dds_topic_name, dds_type_for_topic, topic_spec
    from message.dds_types.gnss import GnssFix, GnssStatus
    from message.dds_types.nav import Odometry
    from runtime.runtime_interface import TOPICS, topic_allowed_frame_ids, topic_formats

    expected = {
        TOPICS.gnss_fix: ("GnssFix", GnssFix, "rt/gnss/fix", "kGnssFix"),
        TOPICS.gnss_status: ("GnssStatus", GnssStatus, "rt/gnss/status", "kGnssStatus"),
        TOPICS.gnss_odom: ("Odometry", Odometry, "rt/gnss/odom", "kGnssOdom"),
    }

    idl = Path("src/message/idl/lingtu_slam.idl").read_text(encoding="utf-8")
    cpp_topics = Path("src/message/cpp/dds_topics.hpp").read_text(encoding="utf-8")
    module_source = Path("src/drivers/real/gnss/native/module.cpp").read_text(encoding="utf-8")
    dds_module_source = Path("src/drivers/real/gnss/native/dds_module.cpp").read_text(encoding="utf-8")

    assert "struct GnssFix" in idl
    assert "double position_covariance[9];" in idl
    assert "struct GnssStatus" in idl
    assert topic_formats(TOPICS.gnss_fix) == ("lingtu.dds.GnssFix",)
    assert topic_formats(TOPICS.gnss_status) == ("lingtu.dds.GnssStatus",)
    assert topic_formats(TOPICS.gnss_odom) == ("odometry",)
    assert topic_allowed_frame_ids(TOPICS.gnss_fix) == ("gnss_antenna",)
    assert topic_allowed_frame_ids(TOPICS.gnss_status) == ("gnss_antenna",)
    assert topic_allowed_frame_ids(TOPICS.gnss_odom) == ("map", "odom")

    for topic, (type_name, dds_type, dds_topic, cpp_constant) in expected.items():
        spec = topic_spec(topic)
        assert spec is not None
        assert spec.type_name == type_name
        assert spec.dds_topic == dds_topic
        assert dds_topic_name(topic, typed=True) == dds_topic
        assert dds_type_for_topic(topic) is dds_type
        assert f"inline constexpr TopicContract {cpp_constant}" in cpp_topics
        assert f'"{topic}", "{dds_topic}"' in cpp_topics
        assert f"{cpp_constant}.dds_topic" in module_source

    assert "lingtu_dds_GnssFix_desc" in dds_module_source
    assert "lingtu_dds_GnssStatus_desc" in dds_module_source


def _topic_attr(topic: str) -> str:
    from runtime.runtime_interface import TOPICS

    for name, value in vars(TOPICS).items():
        if value == topic:
            return name
    raise AssertionError(f"topic is not declared on TOPICS: {topic}")


def test_product_topic_literals_are_owned_by_runtime_topics_contract() -> None:
    product_topics = _canonical_runtime_topics()
    offenders: list[str] = []
    for path in _iter_checked_python_sources():
        relative_path = path.relative_to(REPO_ROOT).as_posix()
        if _is_allowed_product_topic_literal_path(relative_path):
            continue
        for line_no, value in _topic_string_literals(path):
            matched_topic = _matched_canonical_topic(value, product_topics)
            if matched_topic:
                offenders.append(f"{relative_path}:{line_no} hardcodes {matched_topic!r} in {value!r}")

    assert offenders == [], (
        "Canonical runtime topics must come from runtime.runtime_interface.TOPICS "
        "in ordinary Python modules. Move protocol literals to an adapter/endpoint/"
        "contract file, or replace the string with TOPICS.<name>. Offenders: " + "; ".join(offenders)
    )


def _iter_checked_python_sources() -> list[Path]:
    paths: list[Path] = []
    for root_name in CHECKED_PRODUCT_TOPIC_ROOTS:
        root = REPO_ROOT / root_name
        if not root.exists():
            continue
        if root.is_file() and root.suffix == ".py":
            paths.append(root)
            continue
        for path in root.rglob("*.py"):
            if (
                "__pycache__" in path.parts
                or "tests" in path.parts
                or SKIPPED_PRODUCT_TOPIC_PATH_PARTS.intersection(path.parts)
            ):
                continue
            paths.append(path)
    return sorted(set(paths))


def _is_allowed_product_topic_literal_path(relative_path: str) -> bool:
    return relative_path == TOPIC_CONTRACT_SOURCE or any(
        relative_path.startswith(prefix) for prefix in ALLOWED_PRODUCT_TOPIC_LITERAL_PATHS
    )


def _canonical_runtime_topics() -> frozenset[str]:
    from runtime.runtime_interface import TOPICS

    return frozenset(
        value for value in vars(TOPICS).values() if isinstance(value, str) and value.startswith("/")
    ) | frozenset({"/tf", "/tf_static"})


def _matched_canonical_topic(value: str, topics: frozenset[str]) -> str | None:
    if value in topics:
        return value
    return next(
        (topic for topic in topics if _contains_topic_token(value, topic)),
        None,
    )


def _contains_topic_token(value: str, topic: str) -> bool:
    index = value.find(topic)
    while index >= 0:
        before = value[index - 1] if index > 0 else ""
        after_index = index + len(topic)
        after = value[after_index] if after_index < len(value) else ""
        if not _is_topic_word_char(before) and not _is_topic_word_char(after):
            return True
        index = value.find(topic, index + 1)
    return False


def _is_topic_word_char(value: str) -> bool:
    return value.isalnum() or value in "_.-/"


def _topic_string_literals(path: Path) -> list[tuple[int, str]]:
    tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    docstring_nodes = {
        node.body[0]
        for node in ast.walk(tree)
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
        and node.body
        and isinstance(node.body[0], ast.Expr)
        and isinstance(node.body[0].value, ast.Constant)
        and isinstance(node.body[0].value.value, str)
    }
    values: list[tuple[int, str]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
            continue
        if any(getattr(parent, "value", None) is node for parent in docstring_nodes):
            continue
        values.append((int(getattr(node, "lineno", 0)), node.value))
    return values
