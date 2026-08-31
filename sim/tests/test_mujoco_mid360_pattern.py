import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from runtime.tests.numpy_guard import import_numpy_or_skip

pytestmark = [pytest.mark.sim]

np = import_numpy_or_skip()

from sim.engine.mujoco.lidar import MuJoCoLidar  # noqa: E402
from sim.sensors.livox_mid360 import read_plugin_lidar  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
MID360_PATTERN = ROOT / "sim/assets/livox/mid360.npy"


def test_repo_mid360_pattern_asset_is_official_converted_scan_mode() -> None:
    assert MID360_PATTERN.is_file()

    angles = np.load(MID360_PATTERN, mmap_mode="r")

    assert angles.shape == (800000, 2)
    assert angles.dtype == np.float32
    assert 0.0 <= float(np.min(angles[:, 0])) < 1e-4
    assert np.isclose(float(np.max(angles[:, 0])), 2.0 * np.pi, atol=1e-5)
    assert np.isclose(float(np.min(angles[:, 1])), -0.1258784, atol=1e-5)
    assert np.isclose(float(np.max(angles[:, 1])), 0.9104336, atol=1e-5)


def test_mid360_nominal_config_uses_conservative_official_envelope() -> None:
    from drivers.sim.mujoco.runtime import DEFAULT_MID360_SAMPLES_PER_FRAME
    from sim.engine.core.sensor import LidarConfig

    config = LidarConfig()

    assert DEFAULT_MID360_SAMPLES_PER_FRAME == 20000
    assert config.samples_per_frame == 20000
    assert config.fps == pytest.approx(10.0)
    assert config.range_min == pytest.approx(0.1)
    assert config.range_max == pytest.approx(40.0)
    assert config.noise_std == pytest.approx(0.02)
    assert config.range_noise_near_std_m == pytest.approx(0.03)
    assert config.range_noise_far_std_m == pytest.approx(0.02)
    assert config.range_noise_near_m == pytest.approx(0.2)
    assert config.range_noise_far_m == pytest.approx(10.0)
    assert config.angle_noise_std_rad < math.radians(0.15)
    assert config.pixel_dropout_prob == 0.0
    assert config.distance_dropout_prob_at_max == 0.0
    assert config.intensity_base == pytest.approx(15.0)
    assert config.intensity_noise_std == 0.0


def test_lidar_config_preserves_the_existing_positional_surface() -> None:
    from sim.engine.core.sensor import LidarConfig

    config = LidarConfig("sensor_mount", "sensor_contract")

    assert config.body_name == "sensor_mount"
    assert config.sensor_name == "sensor_contract"
    assert config.exclude_body_name is None


def test_failed_official_lidar_init_restores_robot_geom_groups(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    mujoco = pytest.importorskip("mujoco")
    from sim.engine.core.sensor import LidarConfig
    from sim.engine.mujoco import lidar as lidar_module

    model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <body name="base_link">
              <geom name="body_geom" type="box" size="0.2 0.1 0.1"/>
              <body name="lidar_link">
                <geom name="lidar_geom" type="sphere" size="0.02"/>
                <site name="lidar_site" size="0.001"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    data = mujoco.MjData(model)
    original_groups = model.geom_group.copy()

    class FailingWrapper:
        def __init__(self, *_args, **_kwargs) -> None:
            raise RuntimeError("synthetic backend failure")

    monkeypatch.setattr(
        lidar_module,
        "_load_official_mujoco_lidar_wrapper",
        lambda: FailingWrapper,
    )
    config = LidarConfig(
        body_name="lidar_link",
        exclude_body_name="base_link",
        site_name="lidar_site",
        backend="mujoco_lidar",
        require_product_backend=False,
        mid360_npy_path=str(MID360_PATTERN),
    )

    with pytest.raises(RuntimeError, match="backend initialization failed"):
        MuJoCoLidar(model, data, config)

    np.testing.assert_array_equal(model.geom_group, original_groups)


def test_lidar_self_exclusion_keeps_robot_visual_and_collision_groups_distinct() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <body name="base_link">
              <geom name="visual" type="box" size="0.2 0.1 0.1"
                    group="1" contype="0" conaffinity="0"/>
              <geom name="collision" type="box" size="0.2 0.1 0.1"
                    group="3" contype="1" conaffinity="1"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._model = model
    lidar._exclude_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    lidar._geomgroup = np.ones(6, dtype=np.uint8)
    lidar._self_geom_count = 0

    lidar._exclude_robot_geoms()

    visual_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "visual")
    collision_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "collision")
    assert int(model.geom_group[visual_id]) == 5
    assert int(model.geom_group[collision_id]) == 3
    assert int(lidar._geomgroup[5]) == 0
    assert int(lidar._geomgroup[3]) == 0


def test_formal_mid360_uses_declared_site_and_excludes_the_robot_root() -> None:
    pytest.importorskip("mujoco")
    pytest.importorskip("mujoco_lidar")
    from drivers.sim.mujoco.runtime import build_engine

    engine = build_engine(
        world=ROOT / "sim/packages/worlds/open_field/1.1.0/physics/open_field.xml",
        robot_xml=ROOT / "sim/robots/doso/thunder_v4/mjcf/thunderv4.xml",
        drive_mode="kinematic",
        start=[0.0, 0.0, 0.0],
        start_orientation_wxyz=[1.0, 0.0, 0.0, 0.0],
        initial_keyframe="v4_nominal_stand",
        mujoco_memory="",
        base_body_name="base_link",
        lidar_body_name="lidar1_link",
        lidar_site_name="lidar1_link_site",
        physics_timestep_s=0.001,
        mid360_pattern=MID360_PATTERN,
        mid360_samples_per_frame=20_000,
        lidar_backend="mujoco_lidar",
        mujoco_lidar_backend="cpu",
        require_product_lidar_backend=True,
    )
    try:
        report = engine.get_lidar_backend_report()
        points = engine.get_lidar_points(sample_count=1_000)
        physics_dt = engine.dt
    finally:
        engine.close()

    assert report["body_name"] == "lidar1_link"
    assert report["exclude_body_name"] == "base_link"
    assert report["site_name"] == "lidar1_link_site"
    assert physics_dt == pytest.approx(0.001)
    assert len(points) > 0


def test_product_mujoco_runtime_enables_realistic_lidar_returns() -> None:
    text = Path("src/drivers/sim/mujoco/runtime.py").read_text(encoding="utf-8")

    assert "add_noise=True" in text
    assert "mid360_npy_path=str(pattern_path)" in text


def test_saved_map_relocalize_discovers_generic_native_same_source_map(
    tmp_path,
    monkeypatch,
) -> None:
    from sim.scripts import saved_map_relocalize_runtime_gate as gate

    map_pcd = tmp_path / "artifacts/sim_diagnostics/native_slam_capture/run-1/same_source_map/map.pcd"
    map_pcd.parent.mkdir(parents=True)
    map_pcd.write_text(
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n",
        encoding="ascii",
    )

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    assert gate._resolve_latest_map() == map_pcd


def test_mujoco_backend_catalogs_all_products_without_legacy_flat_acceptance() -> None:
    from runtime.graph.loader import load_runtime_graph

    backend = load_runtime_graph().envs["sim"]["backends"]["mujoco"]

    assert backend["supported_products"] == [
        "teleop",
        "teleop_avoid",
        "map",
        "nav",
        "tracking",
        "inspection",
        "explore",
    ]
    assert set(backend["acceptance"]["products"]) == {
        "teleop",
        "teleop_avoid",
        "map",
        "nav",
        "tracking",
        "inspection",
        "explore",
    }
    assert "acceptance_runner" not in backend
    assert "acceptance_runners" not in backend


def test_load_mid360_csv_pattern_converts_zenith_to_elevation(tmp_path) -> None:
    csv_path = tmp_path / "mid360.csv"
    csv_path.write_text(
        "Time/s,Azimuth/deg,Zenith/deg\n1,0,90\n2,90,0\n3,180,180\n",
        encoding="utf-8",
    )

    angles = MuJoCoLidar._load_scan_mode_angles(str(csv_path))

    assert angles.shape == (3, 2)
    np.testing.assert_allclose(angles[:, 0], [0.0, np.pi / 2, np.pi], atol=1e-6)
    np.testing.assert_allclose(angles[:, 1], [0.0, np.pi / 2, -np.pi / 2], atol=1e-6)


def test_mid360_pattern_sampler_advances_and_wraps() -> None:
    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._config = SimpleNamespace(samples_per_frame=3)
    lidar._ray_angles = np.array(
        [
            [0.0, 0.0],
            [np.pi / 2, 0.0],
            [np.pi, 0.0],
            [3 * np.pi / 2, 0.0],
            [0.0, np.pi / 2],
        ],
        dtype=np.float32,
    )
    lidar._ray_cursor = 0

    first = lidar._next_pattern_angles()
    second = lidar._next_pattern_angles()

    np.testing.assert_allclose(first[0], [0.0, np.pi / 2, np.pi], atol=1e-6)
    np.testing.assert_allclose(first[1], 0.0, atol=0.0)
    np.testing.assert_allclose(second[0], [3 * np.pi / 2, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(second[1], [0.0, np.pi / 2, 0.0], atol=1e-6)
    assert lidar._ray_cursor == 1


def test_lidar_return_model_uses_distance_weighted_intensity() -> None:
    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._config = SimpleNamespace(
        add_noise=False,
        range_min=0.1,
        range_max=70.0,
        pixel_dropout_prob=0.0,
        distance_dropout_prob_at_max=0.0,
        intensity_base=180.0,
        intensity_range_scale_m=25.0,
        intensity_noise_std=0.0,
        intensity_min=1.0,
        intensity_max=255.0,
        site_name="lidar_site",
    )
    lidar._rng = np.random.default_rng(0)
    lidar._mujoco_lidar = None
    lidar._data = None
    lidar._body_id = 0

    cloud = lidar._points_with_return_model(np.array([[1.0, 0.0, 0.0], [40.0, 0.0, 0.0]], dtype=np.float32))

    assert cloud.shape == (2, 4)
    assert cloud[0, 3] > cloud[1, 3]
    assert not np.allclose(cloud[:, 3], 100.0)


def test_lidar_return_model_applies_radial_range_noise_without_lateral_drift() -> None:
    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._config = SimpleNamespace(
        add_noise=True,
        noise_std=0.05,
        range_min=0.1,
        range_max=70.0,
        pixel_dropout_prob=0.0,
        distance_dropout_prob_at_max=0.0,
        intensity_base=180.0,
        intensity_range_scale_m=25.0,
        intensity_noise_std=0.0,
        intensity_min=1.0,
        intensity_max=255.0,
        site_name="lidar_site",
    )
    lidar._rng = np.random.default_rng(7)
    lidar._mujoco_lidar = None
    lidar._data = None
    lidar._body_id = 0
    ideal = np.array([[10.0, 0.0, 0.0], [0.0, 20.0, 0.0]], dtype=np.float32)

    cloud = lidar._points_with_return_model(ideal)

    assert cloud.shape == (2, 4)
    assert cloud[0, 1] == 0.0
    assert cloud[0, 2] == 0.0
    assert cloud[1, 0] == 0.0
    assert cloud[1, 2] == 0.0
    assert not np.allclose(np.linalg.norm(cloud[:, :3], axis=1), [10.0, 20.0])


def test_lidar_return_model_interpolates_mid360_near_and_far_range_noise() -> None:
    class OneSigmaRng:
        @staticmethod
        def normal(_loc, scale, size=None):
            values = np.asarray(scale, dtype=np.float32)
            if values.ndim == 0:
                return np.full(int(size), float(values), dtype=np.float32)
            return values

    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._config = SimpleNamespace(
        add_noise=True,
        noise_std=0.02,
        range_noise_near_std_m=0.03,
        range_noise_far_std_m=0.02,
        range_noise_near_m=0.2,
        range_noise_far_m=10.0,
        range_min=0.1,
        range_max=40.0,
        pixel_dropout_prob=0.0,
        distance_dropout_prob_at_max=0.0,
        intensity_base=180.0,
        intensity_range_scale_m=25.0,
        intensity_noise_std=0.0,
        intensity_min=1.0,
        intensity_max=255.0,
        site_name="lidar_site",
    )
    lidar._rng = OneSigmaRng()
    lidar._mujoco_lidar = None
    lidar._data = None
    lidar._body_id = 0

    cloud = lidar._points_with_return_model(np.array([[0.2, 0.0, 0.0], [10.0, 0.0, 0.0]], dtype=np.float32))

    np.testing.assert_allclose(cloud[:, 0], [0.23, 10.02], atol=1e-6)
    np.testing.assert_allclose(cloud[:, 1:3], 0.0, atol=0.0)


def test_ray_caster_plugin_applies_post_return_angle_noise_approximation() -> None:
    class FixedAngleRng:
        @staticmethod
        def normal(_loc, _scale, size=None):
            assert size == (1, 2)
            return np.array([[0.01, 0.0]], dtype=np.float32)

    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._backend = "ray_caster_lidar"
    lidar._config = SimpleNamespace(
        add_noise=True,
        noise_std=0.0,
        angle_noise_std_rad=0.01,
        range_min=0.1,
        range_max=40.0,
        pixel_dropout_prob=0.0,
        distance_dropout_prob_at_max=0.0,
        intensity_base=180.0,
        intensity_range_scale_m=25.0,
        intensity_noise_std=0.0,
        intensity_min=1.0,
        intensity_max=255.0,
        site_name="lidar_site",
    )
    lidar._rng = FixedAngleRng()
    lidar._mujoco_lidar = None
    lidar._data = None
    lidar._body_id = 0

    cloud = lidar._points_with_return_model(np.array([[10.0, 0.0, 0.0]], dtype=np.float32))

    assert math.atan2(float(cloud[0, 1]), float(cloud[0, 0])) == pytest.approx(0.01)
    assert np.linalg.norm(cloud[0, :3]) == pytest.approx(10.0)


def test_mid360_pattern_angle_noise_keeps_theta_wrapped() -> None:
    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._config = SimpleNamespace(
        samples_per_frame=4,
        add_noise=True,
        angle_noise_std_rad=0.01,
    )
    lidar._ray_angles = np.array([[0.0, 0.1], [6.2, 0.2]], dtype=np.float32)
    lidar._ray_cursor = 0
    lidar._rng = np.random.default_rng(0)

    theta, phi = lidar._next_pattern_angles()

    assert theta.shape == (4,)
    assert phi.shape == (4,)
    assert np.all(theta >= 0.0)
    assert np.all(theta <= 2.0 * np.pi)


def test_mujoco_lidar_dependency_is_exactly_pinned() -> None:
    try:
        import tomllib
    except ModuleNotFoundError:
        import tomli as tomllib

    project = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))
    dependencies = project["project"]["optional-dependencies"]["sim-mujoco"]

    assert "mujoco-lidar==0.3.3" in dependencies
    assert ('name = "mujoco-lidar", marker = "extra == \'sim-mujoco\'", specifier = "==0.3.3"') in (
        ROOT / "uv.lock"
    ).read_text(encoding="utf-8")


def test_official_mujoco_lidar_distribution_is_not_shadowed() -> None:
    import importlib.metadata

    from sim.engine.mujoco.lidar import _load_official_mujoco_lidar_wrapper

    wrapper = _load_official_mujoco_lidar_wrapper()
    distribution = importlib.metadata.distribution("mujoco-lidar")
    package_root = Path(distribution.locate_file("mujoco_lidar")).resolve()

    assert importlib.metadata.version("mujoco-lidar") == "0.3.3"
    assert wrapper.__module__ == "mujoco_lidar.lidar_wrapper"
    assert package_root.is_dir()
    assert ROOT / "src" not in package_root.parents
    assert "site-packages" in {part.lower() for part in package_root.parts}
    assert not (ROOT / "src/drivers/sim/lidar/mujoco_lidar").exists()


def test_product_lidar_requires_explicit_canonical_pattern() -> None:
    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._config = SimpleNamespace(
        mid360_npy_path=None,
        require_product_backend=True,
    )

    with pytest.raises(RuntimeError, match="explicit canonical MID-360 pattern"):
        lidar._load_configured_mid360_angles()


def test_product_lidar_accepts_canonical_pattern() -> None:
    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._config = SimpleNamespace(
        mid360_npy_path=str(MID360_PATTERN),
        require_product_backend=True,
    )

    angles = lidar._load_configured_mid360_angles()
    assert angles.shape == (800_000, 2)


def test_plugin_lidar_reports_malformed_native_state(monkeypatch: pytest.MonkeyPatch) -> None:
    fake_mujoco = SimpleNamespace(
        mjtObj=SimpleNamespace(mjOBJ_SENSOR=1),
        mj_name2id=lambda *_args: 0,
    )
    monkeypatch.setitem(sys.modules, "mujoco", fake_mujoco)
    model = SimpleNamespace(
        sensor_plugin=np.array([0]),
        plugin_stateadr=np.array([0]),
        sensor_adr=np.array([0]),
    )
    data = SimpleNamespace(
        plugin_state=np.array([1, 2]),
        sensordata=np.arange(5, dtype=np.float32),
    )

    with pytest.raises(RuntimeError, match="invalid MuJoCo LiDAR state.*lidar_mid360"):
        read_plugin_lidar(model, data)
