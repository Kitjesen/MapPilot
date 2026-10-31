"""Domain skill contracts describe the existing execution and result boundaries."""

import json

from decision.modules.semantic_planner import SemanticPlannerModule
from decision.modules.visual_servo import VisualServoModule
from nav.skills.skills_module import NavSkills


def test_follow_contract_describes_receipts_and_exact_task_queries():
    servo = VisualServoModule(product_session_id="skill-session")
    tools = {info.func_name: info.to_tool() for info in servo.get_skill_infos()}
    try:
        start = tools["start_follow"]
        assert start["annotations"] == {"readOnlyHint": False}
        assert "get_follow_task" in start["description"]
        assert "not successful following" in start["description"]
        required = set(start["outputSchema"]["required"])
        rejected = servo.start_follow()
        assert required <= rejected.keys()
        assert rejected["accepted"] is False

        receipt = servo.start_follow(description="person in a gray shirt")
        assert required <= receipt.keys()
        assert receipt["accepted"] and not receipt["execution_confirmed"]
        query = tools["get_follow_task"]
        assert query["annotations"] == {"readOnlyHint": True}
        assert query["inputSchema"]["required"] == ["follow_task_id", "product_session_id"]
        assert "not a tracker ID" in query["inputSchema"]["properties"]["follow_task_id"]["description"]
        current = servo.get_follow_task(receipt["follow_task_id"], receipt["product_session_id"])
        assert set(query["outputSchema"]["required"]) <= current.keys()
        assert current["found"] and not current["terminal"] and current["result"] is None
        wrong_session = servo.get_follow_task(receipt["follow_task_id"], "previous-session")
        assert set(query["outputSchema"]["required"]) <= wrong_session.keys()
        assert wrong_session == {"found": False, "reason": "product_session_mismatch"}

        cancel = tools["cancel_follow_task"]
        assert cancel["annotations"] == {"readOnlyHint": False}
        assert "does not prove motion has stopped" in cancel["description"]
        cancelled = servo.cancel_follow_task(receipt["follow_task_id"], receipt["product_session_id"])
        assert set(cancel["outputSchema"]["required"]) <= cancelled.keys()
        final = servo.get_follow_task(receipt["follow_task_id"], receipt["product_session_id"])
        assert final["terminal"] and final["result"] == "cancelled"
    finally:
        servo.stop()


def test_navigation_contract_separates_coordinates_requests_and_tasks():
    nav = NavSkills()
    tools = {info.func_name: info.to_tool() for info in nav.get_skill_infos()}
    coordinates = tools["navigate_to"]["inputSchema"]["properties"]
    assert "metres" in coordinates["x"]["description"]
    assert "radians" in coordinates["yaw"]["description"]
    assert "fresh map-frame" in coordinates["z"]["description"]
    assert "not the submission request_id" in tools["cancel_mission"]["description"]
    result_tool = tools["get_navigation_result"]
    result = json.loads(nav.get_navigation_result("unknown-request"))
    assert result_tool["annotations"] == {"readOnlyHint": True}
    assert set(result_tool["outputSchema"]["required"]) <= result.keys()
    assert not result["found"]
    assert result["reason"] == "request_not_owned_by_nav_skills"


def test_semantic_contract_distinguishes_parsing_grounding_and_execution():
    planner = SemanticPlannerModule()
    tools = {info.func_name: info.to_tool() for info in planner.get_skill_infos()}
    for name in ("decompose_task", "resolve_task_step", "get_scene_objects", "get_planner_status"):
        assert tools[name]["annotations"] == {"readOnlyHint": True}
    assert "send_instruction" not in tools
    assert "not a completed model request" in tools["decompose_task"]["description"]
    assert "neither checks an executable route nor publishes motion" in tools["resolve_task_step"]["description"]


def test_documentation_extractor_retains_configured_and_bare_skills():
    from tools.docs.extract_api_docs import extract_skills

    inventory = {(entry["class_name"], entry["method_name"]): entry for entry in extract_skills()}
    assert ("VisualServoModule", "start_follow") in inventory
    assert ("VisualServoModule", "find_object") in inventory
    assert ("NavSkills", "get_navigation_result") in inventory
    assert "accepted=true is only admission" in inventory["VisualServoModule", "start_follow"]["description"]
