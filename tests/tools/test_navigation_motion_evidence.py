from copy import deepcopy

from tools.diagnostics.navigation_motion_evidence import summarize


def sample():
    return {
        "dds": {"nav_endpoint": {
            "stamp_s": 10,
            "last_local": {"dynamic_avoidance": "detour", "prediction_count": 1},
            "final_cmd_vel": {"vx": .3, "vy": 0, "wz": 0},
            "final_output": {"published": True, "output_sequence": 12, "producer_boot_id": "a"},
            "driver_control": {"fresh": True, "last_command_accepted": True,
                               "accepted_output_sequence": 12, "accepted_producer_boot_id": "a"},
            "input_gate": {"odom_age_s": .02, "odom_max_age_s": .5},
        }},
        "state": {"localization": {"odometry": {"ts": 100, "vx": .2, "wz": 0}}},
    }


def test_motion_requires_matching_ack_and_new_fresh_odometry():
    first = sample()
    repeated_odom = deepcopy(first)
    repeated_odom["dds"]["nav_endpoint"]["stamp_s"] = 11
    result = summarize([first, first, repeated_odom])
    assert result["unique_navigation_samples"] == 2
    assert result["acknowledged_command_with_observed_motion_samples"] == 1
    for field, value in (("accepted_output_sequence", 11), ("accepted_producer_boot_id", "old"),
                         ("fresh", False)):
        wrong = sample()
        wrong["dds"]["nav_endpoint"]["driver_control"][field] = value
        assert summarize([wrong])["acknowledged_command_with_observed_motion_samples"] == 0


def test_stale_or_missing_odom_is_not_a_confirmed_stop():
    row = sample()
    nav = row["dds"]["nav_endpoint"]
    nav["last_local"]["dynamic_avoidance"] = "waiting"
    nav["final_cmd_vel"]["vx"] = 0
    row["state"]["localization"]["odometry"]["vx"] = 0
    assert summarize([row])["wait_with_fresh_quiet_odometry_samples"] == 1
    nav["input_gate"]["odom_age_s"] = .9
    assert summarize([row])["wait_with_fresh_quiet_odometry_samples"] == 0
    row["state"]["localization"]["odometry"] = {}
    assert summarize([row])["wait_with_fresh_quiet_odometry_samples"] == 0


def test_nonzero_command_during_wait_is_reported_without_claiming_acceptance():
    row = sample()
    row["dds"]["nav_endpoint"]["last_local"]["dynamic_avoidance"] = "waiting_for_detour"
    result = summarize([row, {"error": "connection lost"}])
    assert result["wait_with_nonzero_command_samples"] == 1
    assert result["capture_errors"] == 1


def test_startup_without_native_status_is_not_evidence():
    result = summarize([{"dds": {"nav_endpoint": None}}])
    assert result["unavailable_navigation_samples"] == 1
    assert result["unique_navigation_samples"] == 0
