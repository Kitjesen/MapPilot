from decision.tasks.activity import activity_type, normalize_activity_state


def test_activity_types_keep_robot_domains_distinct():
    assert activity_type("navigate_to") == "navigation"
    assert activity_type("follow_person") == "tracking"
    assert activity_type("start_inspection_route") == "inspection"
    assert activity_type("get_inspection_status") == "inspection"
    assert activity_type("start_navigation") == "navigation"
    assert activity_type("get_navigation_status") == "navigation"
    assert activity_type("web_search") == "tool"
    assert activity_type("reply", role="assistant") == "model"


def test_activity_state_normalizes_legacy_and_waiting_values():
    assert normalize_activity_state("streaming") == "running"
    assert normalize_activity_state("pending_approval") == "awaiting"
    assert normalize_activity_state("interrupted") == "cancelled"
    assert normalize_activity_state(None) == "started"
