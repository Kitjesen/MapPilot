from decision.modules.visual_servo import VisualServoModule


def test_visual_servo_exposes_structured_bot_contracts():
    module = VisualServoModule()
    infos = {info.func_name: info for info in module.get_skill_infos()}

    assert infos["follow_person"].output_schema["type"] == "object"
    assert infos["stop_servo"].output_schema["type"] == "object"
    assert infos["get_servo_status"].output_schema["type"] == "object"
    assert infos["get_servo_status"].read_only is True

    status = module.get_servo_status()
    assert status["available_operations"] == ["find", "follow"]
