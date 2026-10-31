import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from tests.runtime.numpy_guard import import_numpy_or_skip

pytestmark = [pytest.mark.sim]

np = import_numpy_or_skip()

from sim.compat.engine.mujoco.lidar import MuJoCoLidar
from sim.compat.sensors.livox_mid360 import read_plugin_lidar

ROOT = Path(__file__).resolve().parents[2]
MID360_PATTERN = ROOT / "sim/packages/sensors/livox/mid360/assets/mid360.npy"


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
    from sim.compat.engine.core.sensor import LidarConfig

    from drivers.sim.mujoco.runtime import DEFAULT_MID360_SAMPLES_PER_FRAME

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
    from sim.compat.engine.core.sensor import LidarConfig

    config = LidarConfig("sensor_mount", "sensor_contract")

    assert config.body_name == "sensor_mount"
    assert config.sensor_name == "sensor_contract"
    assert config.exclude_body_name is None


def test_failed_official_lidar_init_restores_robot_geom_groups(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    mujoco = pytest.importorskip("mujoco")
    from sim.compat.engine.core.sensor import LidarConfig
    from sim.compat.engine.mujoco import lidar as lidar_module

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


def test_lidar_self_geometry_preparation_preserves_physical_collision_flags() -> None:
    mujoco = pytest.importorskip("mujoco")
    pytest.importorskip("mujoco_lidar")
    from sim.compat.engine.core.sensor import LidarConfig

    model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <body name="base_link">
              <geom name="visual" type="box" size="0.2 0.1 0.1"
                    group="1" contype="0" conaffinity="0"/>
              <geom name="collision" type="box" size="0.2 0.1 0.1"
                    group="3" contype="1" conaffinity="1"/>
              <body name="lidar_link"><site name="lidar_site"/></body>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    original_contype = model.geom_contype.copy()
    original_conaffinity = model.geom_conaffinity.copy()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    MuJoCoLidar(model, data, LidarConfig(
        body_name="lidar_link",
        exclude_body_name="base_link",
        site_name="lidar_site",
        backend="mujoco_lidar",
        mujoco_lidar_backend="cpu",
        mid360_npy_path=str(MID360_PATTERN),
        add_noise=False,
    ))

    np.testing.assert_array_equal(model.geom_contype, original_contype)
    np.testing.assert_array_equal(model.geom_conaffinity, original_conaffinity)


def _occlusion_lidar(
    monkeypatch: pytest.MonkeyPatch, *, blocker: str = "", assets: str = "",
    legacy_root_site: bool = False,
) -> MuJoCoLidar:
    mujoco = pytest.importorskip("mujoco")
    pytest.importorskip("mujoco_lidar")
    from sim.compat.engine.core.sensor import LidarConfig

    site = '<site name="lidar_site" size="0.001"/>'
    model = mujoco.MjModel.from_xml_string(
        f"""
        <mujoco>
          <asset>{assets}</asset>
          <worldbody>
            <geom name="front_target" type="sphere" pos="3 0 0" size="0.25"/>
            <geom name="side_target" type="sphere" pos="0 3 0" size="0.25"/>
            <geom name="rear_target" type="sphere" pos="-3 0 0" size="0.25"/>
            <body name="base_link">
              <geom name="torso" type="box" pos="-0.5 0 0" size="0.1 0.1 0.1"/>
              {site if legacy_root_site else ""}
              <body name="lidar_link">
                <geom name="sensor_housing" type="sphere" size="0.03"
                      contype="0" conaffinity="0"/>
                {"" if legacy_root_site else site}
              </body>
              {blocker}
            </body>
          </worldbody>
        </mujoco>
        """
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lidar = MuJoCoLidar(model, data, LidarConfig(
        body_name="base_link" if legacy_root_site else "lidar_link",
        exclude_body_name="base_link",
        site_name="lidar_site",
        backend="mujoco_lidar",
        mujoco_lidar_backend="cpu",
        mid360_npy_path=str(MID360_PATTERN),
        add_noise=False,
    ))
    monkeypatch.setattr(
        lidar,
        "_next_pattern_angles",
        lambda *_args, **_kwargs: (
            np.array([0.0, np.pi / 2], dtype=np.float32),
            np.zeros(2, dtype=np.float32),
        ),
    )
    return lidar


def test_mid360_sensor_housing_does_not_hide_visible_environment(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lidar = _occlusion_lidar(monkeypatch)

    points = lidar.scan_xyz()

    np.testing.assert_allclose(points, [[2.75, 0, 0], [0, 2.75, 0]], atol=1e-5)


def test_mid360_legacy_root_site_ignores_only_its_colocated_sensor_housing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lidar = _occlusion_lidar(
        monkeypatch,
        legacy_root_site=True,
        blocker='<body name="front_bracket"><geom type="box" pos="1 0 0" '
        'size="0.15 0.35 0.35"/></body>',
    )

    points = lidar.scan_xyz()

    np.testing.assert_allclose(points, [[0, 2.75, 0]], atol=1e-5)


def test_mid360_legacy_root_site_does_not_ignore_a_colocated_mounting_bracket(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lidar = _occlusion_lidar(
        monkeypatch,
        legacy_root_site=True,
        blocker='<body name="mounting_bracket"><geom type="box" '
        'size="0.05 0.05 0.05"/></body>',
    )

    points = lidar.scan_xyz()

    assert points.shape == (0, 3)


@pytest.mark.parametrize("collidable", [True, False], ids=["collision", "visual"])
def test_mid360_robot_geometry_blocks_environment_without_returning_self_points(
    monkeypatch: pytest.MonkeyPatch,
    collidable: bool,
) -> None:
    blocker = (
        '<body name="front_bracket"><geom name="blocker" type="box" '
        f'pos="1 0 0" size="0.15 0.35 0.35" contype="{int(collidable)}" '
        f'conaffinity="{int(collidable)}"/></body>'
    )
    lidar = _occlusion_lidar(monkeypatch, blocker=blocker)

    points = lidar.scan_xyz()

    np.testing.assert_allclose(points, [[0, 2.75, 0]], atol=1e-5)


def test_mid360_robot_torso_blocks_a_rear_ray(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lidar = _occlusion_lidar(monkeypatch)
    monkeypatch.setattr(
        lidar,
        "_next_pattern_angles",
        lambda *_args, **_kwargs: (
            np.array([np.pi], dtype=np.float32),
            np.zeros(1, dtype=np.float32),
        ),
    )

    points = lidar.scan_xyz()

    assert points.shape == (0, 3)


def test_plugin_returns_are_filtered_by_robot_occlusion(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lidar = _occlusion_lidar(
        monkeypatch,
        blocker='<body name="front_bracket"><geom type="box" pos="1 0 0" '
        'size="0.15 0.35 0.35"/></body>',
    )
    lidar._plugin_reader = lambda *_args: np.array([[2.75, 0, 0], [0, 2.75, 0]])
    np.testing.assert_allclose(lidar._scan_plugin(), [[0, 2.75, 0]], atol=1e-5)


def test_mid360_self_geometry_inside_minimum_range_still_blocks_environment(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lidar = _occlusion_lidar(
        monkeypatch,
        blocker='<body name="close_bracket"><geom type="box" pos="0.05 0 0" '
        'size="0.02 0.03 0.03"/></body>',
    )

    points = lidar.scan_xyz()

    np.testing.assert_allclose(points, [[0, 2.75, 0]], atol=1e-5)


def test_mid360_moving_robot_part_updates_the_occluded_direction(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    mujoco = pytest.importorskip("mujoco")
    lidar = _occlusion_lidar(
        monkeypatch,
        blocker='<body name="moving_bracket" pos="1 0 0">'
        '<joint name="bracket_slide" type="slide" axis="0 1 0"/>'
        '<geom type="box" size="0.15 0.35 0.35"/></body>',
    )
    np.testing.assert_allclose(lidar.scan_xyz(), [[0, 2.75, 0]], atol=1e-5)
    lidar._data.qpos[0] = 1.0
    mujoco.mj_forward(lidar._model, lidar._data)

    points = lidar.scan_xyz()

    np.testing.assert_allclose(points, [[2.75, 0, 0], [0, 2.75, 0]], atol=1e-5)


def test_mid360_self_occlusion_uses_the_rotated_sensor_frame(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    mujoco = pytest.importorskip("mujoco")
    lidar = _occlusion_lidar(
        monkeypatch,
        blocker='<body name="side_bracket"><geom type="box" pos="0 1 0" '
        'size="0.35 0.15 0.35"/></body>',
    )
    lidar._model.body_quat[lidar._body_id] = [math.sqrt(.5), 0, 0, math.sqrt(.5)]
    mujoco.mj_forward(lidar._model, lidar._data)
    monkeypatch.setattr(
        lidar,
        "_next_pattern_angles",
        lambda *_args, **_kwargs: (np.zeros(1, dtype=np.float32), np.zeros(1, dtype=np.float32)),
    )

    points = lidar.scan_xyz()

    assert points.shape == (0, 3)


@pytest.mark.parametrize("mesh_x", [-1.0, 1.0], ids=["clear", "blocked"])
def test_mid360_visual_mesh_controls_occlusion_instead_of_coarse_contact_proxy(
    monkeypatch: pytest.MonkeyPatch,
    mesh_x: float,
) -> None:
    lidar = _occlusion_lidar(
        monkeypatch,
        assets='<mesh name="shell" vertex="-.1 -.2 -.2  -.1 -.2 .2  -.1 .2 -.2 '
        '-.1 .2 .2  .1 -.2 -.2  .1 -.2 .2  .1 .2 -.2  .1 .2 .2"/>',
        blocker='<body name="body_shell"><geom name="contact_proxy" type="box" '
        'size="1 .5 .5"/><geom name="optical_shell" type="mesh" mesh="shell" '
        f'pos="{mesh_x} 0 0" contype="0" conaffinity="0"/></body>',
    )

    points = lidar.scan_xyz()

    expected = [[0, 2.75, 0]] if mesh_x > 0 else [[2.75, 0, 0], [0, 2.75, 0]]
    np.testing.assert_allclose(points, expected, atol=1e-5)


def test_formal_mid360_uses_declared_site_and_excludes_the_robot_root() -> None:
    pytest.importorskip("mujoco")
    pytest.importorskip("mujoco_lidar")
    from drivers.sim.mujoco.runtime import build_engine

    engine = build_engine(
        world=ROOT / "sim/packages/worlds/open_field/1.1.0/physics/open_field.xml",
        robot_xml=ROOT / "sim/packages/robots/doso/thunder_v4/mjcf/thunderv4.xml",
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
    from sim.scripts.mujoco import saved_map_relocalization as gate

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
    from lingtu.assembly.graph.loader import load_runtime_graph

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
    assert "acceptance" not in backend
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


def test_mid360_pattern_sampler_advances_with_snapshot_clock_and_wraps() -> None:
    lidar = MuJoCoLidar.__new__(MuJoCoLidar)
    lidar._config = SimpleNamespace(samples_per_frame=3, fps=200_000 / 3)
    lidar._data = SimpleNamespace(time=3 / 200_000)
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
    lidar._data.time = 6 / 200_000
    second = lidar._next_pattern_angles()

    np.testing.assert_allclose(first[0], [0.0, np.pi / 2, np.pi], atol=1e-6)
    np.testing.assert_allclose(first[1], 0.0, atol=0.0)
    np.testing.assert_allclose(second[0], [3 * np.pi / 2, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(second[1], [0.0, np.pi / 2, 0.0], atol=1e-6)
    # Repeating a snapshot preserves its scan interval instead of advancing it.
    repeated = lidar._next_pattern_angles()
    np.testing.assert_array_equal(repeated, second)


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
        fps=200_000 / 4,
        add_noise=True,
        angle_noise_std_rad=0.01,
    )
    lidar._ray_angles = np.array([[0.0, 0.1], [6.2, 0.2]], dtype=np.float32)
    lidar._data = SimpleNamespace(time=4 / 200_000)
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

    from sim.compat.engine.mujoco.lidar import _load_official_mujoco_lidar_wrapper

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

    with pytest.raises(RuntimeError, match=r"invalid MuJoCo LiDAR state.*lidar_mid360"):
        read_plugin_lidar(model, data)
