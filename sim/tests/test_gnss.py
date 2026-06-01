"""Unit tests for GNSS messages + GnssModule + WTRTK-980 simulator."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import pytest

pytestmark = [pytest.mark.sim]

# Ensure src/ on path for standalone pytest runs
_ROOT = Path(__file__).resolve().parent.parent.parent
_SRC = _ROOT / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

# And repo root (for sim/)
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.msgs.gnss import (
    GnssFix,
    GnssFixType,
    GnssOdom,
    GnssStatus,
)
from slam.gnss_module import (
    GnssModule,
    MapOrigin,
    QualityConfig,
    fix_weight,
    lla_to_ecef,
    lla_to_enu,
)

# ═══════════════════════════════════════════════════════════════════
# GnssFixType enum
# ═══════════════════════════════════════════════════════════════════


class TestGnssFixType:
    def test_rtk_flags(self):
        assert GnssFixType.RTK_FIXED.is_rtk
        assert GnssFixType.RTK_FLOAT.is_rtk
        assert not GnssFixType.SINGLE.is_rtk
        assert not GnssFixType.NO_FIX.is_rtk

    def test_usable(self):
        assert not GnssFixType.NO_FIX.is_usable
        assert GnssFixType.SINGLE.is_usable
        assert GnssFixType.RTK_FIXED.is_usable

    def test_accuracy_ordering(self):
        assert (
            GnssFixType.RTK_FIXED.typical_accuracy_m
            < GnssFixType.RTK_FLOAT.typical_accuracy_m
            < GnssFixType.DGPS.typical_accuracy_m
            < GnssFixType.SINGLE.typical_accuracy_m
        )


# ═══════════════════════════════════════════════════════════════════
# GnssFix dataclass
# ═══════════════════════════════════════════════════════════════════


class TestGnssFix:
    def _fix(self, **over):
        base = dict(
            lat=31.2304, lon=121.4737, alt=10.5,
            fix_type=GnssFixType.RTK_FIXED,
            covariance=(0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.002),
            num_sat=18, num_sat_used=15, seq=42,
        )
        base.update(over)
        return GnssFix(**base)

    def test_encode_decode_roundtrip(self):
        fix = self._fix()
        decoded = GnssFix.decode(fix.encode())
        assert decoded.lat == pytest.approx(fix.lat)
        assert decoded.lon == pytest.approx(fix.lon)
        assert decoded.alt == pytest.approx(fix.alt)
        assert decoded.fix_type == GnssFixType.RTK_FIXED
        assert decoded.num_sat == 18
        assert decoded.num_sat_used == 15
        assert decoded.seq == 42

    def test_horizontal_std(self):
        fix = self._fix(covariance=(0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.04))
        assert fix.horizontal_std_m == pytest.approx(0.1, rel=0.01)

    def test_vertical_std(self):
        fix = self._fix(covariance=(0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.04))
        assert fix.vertical_std_m == pytest.approx(0.2, rel=0.01)

    def test_int_to_enum_coercion(self):
        fix = GnssFix(fix_type=4)
        assert fix.fix_type == GnssFixType.RTK_FIXED

    def test_invalid_covariance_length(self):
        with pytest.raises(ValueError, match="covariance must have 9"):
            GnssFix(covariance=(0.0, 0.0))

    def test_is_rtk_property(self):
        assert self._fix(fix_type=GnssFixType.RTK_FIXED).is_rtk
        assert self._fix(fix_type=GnssFixType.RTK_FLOAT).is_rtk
        assert not self._fix(fix_type=GnssFixType.SINGLE).is_rtk

    def test_to_dict_round_trip(self):
        d = self._fix().to_dict()
        assert d["fix_type"] == "RTK_FIXED"
        assert d["fix_type_int"] == 4
        assert d["is_rtk"] is True


# ═══════════════════════════════════════════════════════════════════
# GnssStatus
# ═══════════════════════════════════════════════════════════════════


class TestGnssStatus:
    def test_healthy_requires_link_and_sat(self):
        ok = GnssStatus(
            fix_type=GnssFixType.RTK_FIXED, num_sat_used=10,
            age_s=0.1, link_ok=True,
        )
        assert ok.is_healthy

    def test_unhealthy_no_link(self):
        bad = GnssStatus(
            fix_type=GnssFixType.RTK_FIXED, num_sat_used=10,
            age_s=0.1, link_ok=False,
        )
        assert not bad.is_healthy

    def test_unhealthy_stale(self):
        bad = GnssStatus(
            fix_type=GnssFixType.RTK_FIXED, num_sat_used=10,
            age_s=5.0, link_ok=True,
        )
        assert not bad.is_healthy

    def test_unhealthy_few_sat(self):
        bad = GnssStatus(
            fix_type=GnssFixType.RTK_FIXED, num_sat_used=3,
            age_s=0.1, link_ok=True,
        )
        assert not bad.is_healthy


# ═══════════════════════════════════════════════════════════════════
# LLA ↔ ENU math
# ═══════════════════════════════════════════════════════════════════


class TestCoordTransforms:
    def test_ecef_self_consistency(self):
        # Shanghai
        x, y, z = lla_to_ecef(31.2304, 121.4737, 10.0)
        assert x == pytest.approx(-2851000, rel=0.001)  # rough ECEF
        # Magnitude check — should be near Earth radius
        r = (x * x + y * y + z * z) ** 0.5
        assert 6.3e6 < r < 6.4e6

    def test_enu_origin_is_zero(self):
        e, n, u = lla_to_enu(
            31.2304, 121.4737, 10.0,
            31.2304, 121.4737, 10.0,
        )
        assert abs(e) < 1e-6
        assert abs(n) < 1e-6
        assert abs(u) < 1e-6

    def test_enu_east_delta(self):
        # 0.0001° longitude at Shanghai ≈ 9.5 m East
        e, n, u = lla_to_enu(
            31.2304, 121.4738, 10.0,
            31.2304, 121.4737, 10.0,
        )
        assert 9.0 < e < 10.0
        assert abs(n) < 0.1
        assert abs(u) < 0.01

    def test_enu_north_delta(self):
        # 0.0001° latitude ≈ 11.1 m North
        e, n, u = lla_to_enu(
            31.2305, 121.4737, 10.0,
            31.2304, 121.4737, 10.0,
        )
        assert abs(e) < 0.1
        assert 10.5 < n < 11.5
        assert abs(u) < 0.01

    def test_enu_altitude_delta(self):
        e, n, u = lla_to_enu(
            31.2304, 121.4737, 15.0,
            31.2304, 121.4737, 10.0,
        )
        assert abs(e) < 0.01
        assert abs(n) < 0.01
        assert u == pytest.approx(5.0, abs=0.01)


# ═══════════════════════════════════════════════════════════════════
# Quality filter
# ═══════════════════════════════════════════════════════════════════


class TestQualityFilter:
    def _cfg(self, **over):
        base = dict(min_sat_used=8, max_hdop=2.5, require_fix_type=1)
        base.update(over)
        return QualityConfig(**base)

    def _good_fix(self, **over):
        base = dict(
            fix_type=GnssFixType.RTK_FIXED, num_sat_used=15,
            covariance=(0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.002),
        )
        base.update(over)
        return GnssFix(**base)

    def test_rtk_fixed_full_weight(self):
        assert fix_weight(self._good_fix(), self._cfg()) == 1.0

    def test_rtk_float_reduced_weight(self):
        w = fix_weight(
            self._good_fix(fix_type=GnssFixType.RTK_FLOAT,
                          covariance=(0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.02)),
            self._cfg(),
        )
        assert w == 0.3

    def test_rtk_float_disabled(self):
        w = fix_weight(
            self._good_fix(fix_type=GnssFixType.RTK_FLOAT,
                          covariance=(0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.02)),
            self._cfg(allow_float=False),
        )
        assert w == 0.0

    def test_single_low_weight(self):
        w = fix_weight(
            self._good_fix(fix_type=GnssFixType.SINGLE,
                          covariance=(1, 0, 0, 0, 1, 0, 0, 0, 4)),
            self._cfg(max_hdop=10),  # SINGLE has high hdop — let it pass
        )
        assert w == 0.05

    def test_sat_below_threshold(self):
        assert fix_weight(self._good_fix(num_sat_used=3), self._cfg()) == 0.0

    def test_fix_type_below_threshold(self):
        assert fix_weight(
            self._good_fix(fix_type=GnssFixType.SINGLE,
                          covariance=(1, 0, 0, 0, 1, 0, 0, 0, 4)),
            self._cfg(require_fix_type=4, max_hdop=10),
        ) == 0.0

    def test_no_fix_rejected(self):
        assert fix_weight(
            self._good_fix(fix_type=GnssFixType.NO_FIX),
            self._cfg(),
        ) == 0.0


# ═══════════════════════════════════════════════════════════════════
# MapOrigin
# ═══════════════════════════════════════════════════════════════════


class TestMapOrigin:
    def test_not_initialised(self):
        o = MapOrigin()
        assert not o.is_initialised
        with pytest.raises(RuntimeError):
            o.as_tuple()

    def test_initialised(self):
        o = MapOrigin(lat=1, lon=2, alt=3)
        assert o.is_initialised
        assert o.as_tuple() == (1, 2, 3)

    def test_auto_init(self):
        o = MapOrigin(auto_init=True)
        o.initialise(31.0, 121.0, 10.0)
        assert o.as_tuple() == (31.0, 121.0, 10.0)


# ═══════════════════════════════════════════════════════════════════
# GnssModule
# ═══════════════════════════════════════════════════════════════════


class _FakePort:
    def __init__(self):
        self.items = []

    def publish(self, v):
        self.items.append(v)


@pytest.fixture
def gnss_mod():
    m = GnssModule(
        origin_lat=31.2304, origin_lon=121.4737, origin_alt=10.0,
        auto_init_origin=False,
    )
    m.gnss_fix = _FakePort()
    m.gnss_odom = _FakePort()
    m.gnss_status = _FakePort()
    m.alive = _FakePort()
    return m


def _fix(lat=31.2304, lon=121.4737, alt=10.0,
         fix_type=GnssFixType.RTK_FIXED, num_sat_used=15):
    return GnssFix(
        lat=lat, lon=lon, alt=alt, fix_type=fix_type,
        covariance=(0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.002),
        num_sat=18, num_sat_used=num_sat_used,
    )


class TestGnssModule:
    def test_good_fix_publishes_both(self, gnss_mod):
        fix = _fix()
        gnss_mod.inject_fix(fix)
        assert len(gnss_mod.gnss_fix.items) == 1
        assert len(gnss_mod.gnss_odom.items) == 1
        odom = gnss_mod.gnss_odom.items[0]
        assert odom.fix_type == GnssFixType.RTK_FIXED
        # At origin: ENU should be near zero
        assert abs(odom.east) < 0.01
        assert abs(odom.north) < 0.01

    def test_no_fix_publishes_fix_only(self, gnss_mod):
        fix = _fix(fix_type=GnssFixType.NO_FIX)
        gnss_mod.inject_fix(fix)
        assert len(gnss_mod.gnss_fix.items) == 1
        assert len(gnss_mod.gnss_odom.items) == 0

    def test_few_sat_publishes_fix_only(self, gnss_mod):
        fix = _fix(num_sat_used=3)
        gnss_mod.inject_fix(fix)
        assert len(gnss_mod.gnss_fix.items) == 1
        assert len(gnss_mod.gnss_odom.items) == 0

    def test_enu_east_motion(self, gnss_mod):
        gnss_mod.inject_fix(_fix(lon=121.4738))  # 0.0001° east
        odom = gnss_mod.gnss_odom.items[0]
        assert 9.0 < odom.east < 10.0

    def test_auto_init_origin(self):
        m = GnssModule(auto_init_origin=True)
        m.gnss_fix = _FakePort()
        m.gnss_odom = _FakePort()
        m.gnss_status = _FakePort()
        m.alive = _FakePort()
        assert not m._origin.is_initialised
        m.inject_fix(_fix(lat=40.0, lon=116.0))
        assert m._origin.is_initialised
        assert m._origin.lat == 40.0

    def test_no_auto_init_no_origin_no_odom(self):
        m = GnssModule(auto_init_origin=False)
        m.gnss_fix = _FakePort()
        m.gnss_odom = _FakePort()
        m.gnss_status = _FakePort()
        m.alive = _FakePort()
        m.inject_fix(_fix())
        assert len(m.gnss_fix.items) == 1
        assert len(m.gnss_odom.items) == 0  # no origin → no odom

    def test_float_cov_scaled_up(self, gnss_mod):
        fix = _fix(fix_type=GnssFixType.RTK_FLOAT)
        # scale cov to satisfy hdop
        fix = GnssFix(
            lat=fix.lat, lon=fix.lon, alt=fix.alt,
            fix_type=GnssFixType.RTK_FLOAT,
            covariance=(0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.02),
            num_sat=fix.num_sat, num_sat_used=fix.num_sat_used,
        )
        gnss_mod.inject_fix(fix)
        odom = gnss_mod.gnss_odom.items[0]
        # weight=0.3 → cov / 0.3 = 3.33x
        assert odom.cov_e == pytest.approx(0.01 / 0.3, rel=0.01)


# ═══════════════════════════════════════════════════════════════════
# WTRTK-980 simulator
# ═══════════════════════════════════════════════════════════════════


class TestWtrtkSim:
    """Full-stack sim → GnssModule → odom output."""

    def _build_chain(self, noise_sigma=0.01, fix_type=GnssFixType.RTK_FIXED,
                     loss_windows=None):
        from sim.sensors.wtrtk980_sim import NoiseProfile, WtrtkSim

        gnss = GnssModule(
            origin_lat=31.2304, origin_lon=121.4737, origin_alt=10.0,
            auto_init_origin=False,
        )
        gnss.gnss_fix = _FakePort()
        gnss.gnss_odom = _FakePort()
        gnss.gnss_status = _FakePort()
        gnss.alive = _FakePort()

        sim = WtrtkSim(
            gnss=gnss,
            origin_lla=(31.2304, 121.4737, 10.0),
            get_body_xy=lambda: (0.0, 0.0),
            noise=NoiseProfile(rtk_fixed=noise_sigma),
            fix_type=fix_type,
            loss_windows=loss_windows,
            seed=42,
        )
        return sim, gnss

    def test_sim_publishes_fix(self):
        sim, gnss = self._build_chain()
        sim.publish_once()
        assert len(gnss.gnss_fix.items) == 1
        assert len(gnss.gnss_odom.items) == 1

    def test_sim_noise_in_range(self):
        """Average noise over many samples should be near zero."""
        sim, gnss = self._build_chain(noise_sigma=0.01)
        for _ in range(100):
            sim.publish_once()
        # mean of 100 odoms should be ~0 at origin with 1cm sigma
        mean_e = sum(o.east for o in gnss.gnss_odom.items) / len(gnss.gnss_odom.items)
        assert abs(mean_e) < 0.005  # well within CLT bound

    def test_loss_window_gives_no_fix(self):
        sim, gnss = self._build_chain(loss_windows=[(0.0, 999.0)])
        sim.publish_once()
        assert gnss.gnss_fix.items[0].fix_type == GnssFixType.NO_FIX
        assert len(gnss.gnss_odom.items) == 0

    def test_sim_frame_id(self):
        sim, gnss = self._build_chain()
        sim.publish_once()
        assert gnss.gnss_fix.items[0].frame_id == "gnss_antenna"
