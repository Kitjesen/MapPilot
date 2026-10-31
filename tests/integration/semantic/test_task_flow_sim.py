"""The MuJoCo-truth grounding contract remains read-only and motion-free."""

from sim.evaluation.task_flow import run


def test_task_flow_grounding_contract_previews_both_subgoals_without_execution() -> None:
    report = run()

    assert report["passed"] is True
    assert report["physics"]["backend"] == "mujoco"
    assert report["physics"]["source"] == "model_truth"
    assert [grounding["status"] for grounding in report["groundings"]] == ["candidate", "resolved"]
    assert report["groundings"][0]["target_label"] == "meeting_room"
    assert report["groundings"][1]["candidate_id"] == "box-1"
    assert report["plan"]["is_complete"] is False
    assert report["plan"]["current_step"] == 0
    assert [step["status"] for step in report["plan"]["subgoals"]] == ["pending", "pending"]
    assert report["scene"]["hierarchy_errors"] == []
    assert report["motion_commands"] == []
