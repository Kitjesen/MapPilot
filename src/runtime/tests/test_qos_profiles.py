"""Unit tests for the centralized DDS QoS profile loader.

These tests are ROS-free and do not require cyclonedds: the translation path
is exercised with lightweight fakes injected into the module, and the graceful
degradation path is verified by forcing cyclonedds unavailable.
"""

import pytest

from runtime.transport import qos as qos_mod


@pytest.fixture(autouse=True)
def _clear_qos_cache():
    qos_mod.reset_cache()
    yield
    qos_mod.reset_cache()


# ── domain id resolution ─────────────────────────────────────────────────


class TestResolveDomainId:
    def test_explicit_wins(self, monkeypatch):
        monkeypatch.setenv("LINGTU_DDS_DOMAIN_ID", "42")
        assert qos_mod.resolve_domain_id(7) == 7

    def test_reads_env_when_none(self, monkeypatch):
        monkeypatch.setenv("LINGTU_DDS_DOMAIN_ID", "5")
        assert qos_mod.resolve_domain_id() == 5

    def test_defaults_to_zero(self, monkeypatch):
        monkeypatch.delenv("LINGTU_DDS_DOMAIN_ID", raising=False)
        assert qos_mod.resolve_domain_id() == 0

    def test_invalid_env_falls_back_to_zero(self, monkeypatch):
        monkeypatch.setenv("LINGTU_DDS_DOMAIN_ID", "not-a-number")
        assert qos_mod.resolve_domain_id() == 0


# ── duration parsing ─────────────────────────────────────────────────────


class TestDurationParsing:
    def test_milliseconds(self):
        assert qos_mod._duration_ns("200ms") == 200_000_000

    def test_seconds(self):
        assert qos_mod._duration_ns("1s") == 1_000_000_000

    def test_microseconds(self):
        assert qos_mod._duration_ns("50us") == 50_000

    def test_nanoseconds(self):
        assert qos_mod._duration_ns("5ns") == 5

    def test_bare_number_is_seconds(self):
        assert qos_mod._duration_ns(2) == 2_000_000_000

    def test_none_returns_none(self):
        assert qos_mod._duration_ns(None) is None

    def test_garbage_returns_none(self):
        assert qos_mod._duration_ns("abcms") is None
        assert qos_mod._duration_ns("") is None


# ── raw config + topic mapping (no cyclonedds needed) ────────────────────


class TestRawConfigAndMapping:
    def test_profiles_load_from_real_yaml(self):
        profiles = qos_mod._load_raw_profiles()
        assert "high_freq_state" in profiles
        assert "lidar_pointcloud" in profiles

    def test_topic_to_profile_reverse_mapping(self):
        mapping = qos_mod._topic_to_profile()
        assert mapping.get("/driver/odometry") == "high_freq_state"
        assert mapping.get("rt/driver/odometry") == "high_freq_state"
        assert mapping.get("/slam/odometry") == "high_freq_state"
        assert mapping.get("/nav/cmd_vel") == "final_velocity_command"

    def test_missing_config_degrades_to_empty(self, monkeypatch, tmp_path):
        monkeypatch.setattr(qos_mod, "_QOS_CONFIG_PATH", tmp_path / "nope.yaml")
        qos_mod.reset_cache()
        assert qos_mod._load_raw_profiles() == {}
        assert qos_mod._topic_to_profile() == {}

    def test_bad_yaml_degrades_to_empty(self, monkeypatch, tmp_path):
        bad = tmp_path / "bad.yaml"
        bad.write_text("profiles: [this is not: a mapping", encoding="utf-8")
        monkeypatch.setattr(qos_mod, "_QOS_CONFIG_PATH", bad)
        qos_mod.reset_cache()
        assert qos_mod._load_raw_profiles() == {}


# ── graceful degradation when cyclonedds is unavailable ──────────────────


class TestCycloneUnavailable:
    def test_qos_for_profile_returns_none(self, monkeypatch):
        monkeypatch.setattr(qos_mod, "_CYCLONE_AVAILABLE", False)
        qos_mod.reset_cache()
        assert qos_mod.qos_for_profile("high_freq_state") is None

    def test_qos_for_topic_returns_none(self, monkeypatch):
        monkeypatch.setattr(qos_mod, "_CYCLONE_AVAILABLE", False)
        qos_mod.reset_cache()
        assert qos_mod.qos_for_topic("/slam/odometry") is None


# ── translation path with injected fakes (deterministic, ROS-free) ───────


class _FakeSentinel:
    def __init__(self, label):
        self.label = label

    def __repr__(self):  # pragma: no cover - debug aid
        return f"<{self.label}>"


class _FakeReliability:
    BestEffort = _FakeSentinel("BestEffort")

    @staticmethod
    def Reliable(blocking):
        return ("Reliable", blocking)


class _FakeHistory:
    KeepAll = _FakeSentinel("KeepAll")

    @staticmethod
    def KeepLast(depth):
        return ("KeepLast", depth)


class _FakeDurability:
    Volatile = _FakeSentinel("Volatile")
    TransientLocal = _FakeSentinel("TransientLocal")


class _FakePolicy:
    Reliability = _FakeReliability
    History = _FakeHistory
    Durability = _FakeDurability

    @staticmethod
    def Deadline(value):
        return ("Deadline", value)

    @staticmethod
    def Lifespan(value):
        return ("Lifespan", value)


class _FakeQos:
    def __init__(self, *policies):
        self.policies = list(policies)


def _fake_duration(**kwargs):
    return kwargs


@pytest.fixture
def fake_cyclone(monkeypatch):
    monkeypatch.setattr(qos_mod, "_CYCLONE_AVAILABLE", True)
    monkeypatch.setattr(qos_mod, "Policy", _FakePolicy)
    monkeypatch.setattr(qos_mod, "Qos", _FakeQos)
    monkeypatch.setattr(qos_mod, "duration", _fake_duration)
    qos_mod.reset_cache()
    yield
    qos_mod.reset_cache()


class TestTranslation:
    def test_best_effort_profile(self, fake_cyclone):
        q = qos_mod.qos_for_profile("lidar_pointcloud")
        assert isinstance(q, _FakeQos)
        assert _FakePolicy.Reliability.BestEffort in q.policies
        assert ("KeepLast", 2) in q.policies
        assert _FakePolicy.Durability.Volatile in q.policies
        # lifespan 200ms present
        assert ("Lifespan", {"nanoseconds": 200_000_000}) in q.policies

    def test_reliable_transient_local_profile(self, fake_cyclone):
        q = qos_mod.qos_for_profile("global_path")
        assert ("Reliable", {"seconds": 1.0}) in q.policies
        assert _FakePolicy.Durability.TransientLocal in q.policies
        assert ("KeepLast", 1) in q.policies

    def test_deadline_present(self, fake_cyclone):
        q = qos_mod.qos_for_profile("high_freq_state")
        assert ("Deadline", {"nanoseconds": 20_000_000}) in q.policies

    def test_final_velocity_expires_by_driver_watchdog(self, fake_cyclone):
        q = qos_mod.qos_for_topic("/nav/cmd_vel")
        assert ("Reliable", {"seconds": 1.0}) in q.policies
        assert ("KeepLast", 1) in q.policies
        assert ("Deadline", {"nanoseconds": 50_000_000}) in q.policies
        assert ("Lifespan", {"nanoseconds": 200_000_000}) in q.policies

    def test_qos_for_topic_uses_mapping(self, fake_cyclone):
        q = qos_mod.qos_for_topic("/slam/odometry")
        assert isinstance(q, _FakeQos)
        assert isinstance(qos_mod.qos_for_topic("rt/driver/odometry"), _FakeQos)

    def test_tf_profile_returns_none(self, fake_cyclone):
        # tf profile only carries a note; it must never be overridden.
        assert qos_mod.qos_for_profile("tf") is None

    def test_unknown_profile_returns_none(self, fake_cyclone):
        assert qos_mod.qos_for_profile("does_not_exist") is None

    def test_unmapped_topic_returns_none(self, fake_cyclone):
        assert qos_mod.qos_for_topic("/nowhere/unmapped") is None

    def test_result_is_cached(self, fake_cyclone):
        first = qos_mod.qos_for_profile("control_commands")
        second = qos_mod.qos_for_profile("control_commands")
        assert first is second
