import json

from decision.tasks.sdk_agent import model_tool_output


def test_camera_is_image_input_and_viewer_points_do_not_become_model_text():
    result = {"success": True, "image_data_url": "data:image/jpeg;base64,YWJj", "points": [1, 2, 3], "point_count": 1}
    output = model_tool_output(result)
    assert output[1] == {"type": "image", "image_url": result["image_data_url"], "detail": "auto"}
    assert json.loads(output[0]["text"]) == {"success": True, "point_count": 1}
    assert result["points"] == [1, 2, 3]
    text = json.loads(model_tool_output(result, image_input=False))
    assert not text["image_input_available"] and "image_data_url" not in text


def test_mcp_multiple_images_are_not_serialized_into_model_text():
    images = ["data:image/png;base64,YQ==", "data:image/png;base64,Yg=="]
    result = model_tool_output({"server": "manual", "image_data_urls": images})
    assert [part["image_url"] for part in result[1:]] == images
    assert "base64" not in result[0]["text"]
    text_only = json.loads(model_tool_output({"image_data_urls": images}, image_input=False))
    assert not text_only["image_input_available"] and "image_data_urls" not in text_only


def test_old_camera_observations_adapt_when_switching_to_a_text_only_model():
    from decision.tasks.sdk_agent import model_camera_input

    original = [{"type": "function_call_output", "call_id": "photo", "output": [
        {"type": "input_text", "text": "cached camera snapshot"},
        {"type": "input_image", "image_url": "data:image/jpeg;base64,YWJj"},
    ]}]
    converted = model_camera_input(original, image_input=False, chat_completions=True)
    assert "cannot view this image" in converted[0]["output"]
    assert len(original[0]["output"]) == 2
