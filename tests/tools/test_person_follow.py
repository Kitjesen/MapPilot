from __future__ import annotations

import asyncio
from types import SimpleNamespace

import numpy as np
import pytest
import tools.datasets.person_follow as person_follow
from tools.datasets.person_follow import (
    UnsupportedInstruction,
    _close_chat,
    _create_chat,
    build_parser,
    crop_tracks,
    identity_verdict,
    parse_instruction,
    reacquisition_due,
    select_person,
    select_track_id,
    selected_track,
    tracker_input,
    validate_plan,
    verify_candidate,
)

from decision.tasks.decomposition import SubGoalAction, TaskPlan
from perception.detection.detector_base import Detection2D


def _parser_args(*model_args: str):
    return build_parser().parse_args(
        [
            "--video",
            "input.mp4",
            "--yolo-weights",
            "yolo.pt",
            *model_args,
            "--instruction",
            "跟踪穿红衣服的人",
            "--output",
            "output",
        ]
    )


@pytest.mark.parametrize("backend", ["kimi", "moonshot"])
def test_api_mode_uses_k3_for_moonshot_replay_without_network(
    monkeypatch, backend: str
) -> None:
    monkeypatch.setenv("MOONSHOT_API_KEY", "test-key-long-enough")

    client, evidence = _create_chat(_parser_args("--backend", backend))
    try:
        assert client.config.backend == backend
        assert client.config.model == "kimi-k3"
        assert client.config.api_key_env == "MOONSHOT_API_KEY"
        assert client.config.base_url == "https://api.moonshot.cn/v1"
        assert client.config.timeout_sec == 30.0
        assert evidence == {
            "backend": backend,
            "model": "kimi-k3",
            "base_url": "https://api.moonshot.cn/v1",
            "real_api_requested": True,
        }
    finally:
        asyncio.run(_close_chat(client))


def test_api_mode_retains_explicit_k2_6_model(monkeypatch) -> None:
    monkeypatch.setenv("MOONSHOT_API_KEY", "test-key-long-enough")

    client, evidence = _create_chat(
        _parser_args("--backend", "kimi", "--model", "kimi-k2.6")
    )
    try:
        assert client.config.model == "kimi-k2.6"
        assert evidence["model"] == "kimi-k2.6"
    finally:
        asyncio.run(_close_chat(client))


def test_non_moonshot_backend_keeps_registered_model_default(monkeypatch) -> None:
    from decision.modules.llm import LLMModule

    received: list[str] = []

    class DummyClient:
        supports_vision = True
        config = SimpleNamespace(
            model="provider-default",
            api_key_env="OPENAI_API_KEY",
            base_url="",
        )

        @staticmethod
        def is_available() -> bool:
            return True

    def create_client(module: LLMModule):
        received.append(module._model)
        return DummyClient()

    monkeypatch.setattr(LLMModule, "_create_client", create_client)

    _client, evidence = _create_chat(_parser_args("--backend", "openai"))

    assert received == [""]
    assert evidence["model"] == "provider-default"


def test_api_mode_missing_key_fails_without_mock_fallback(monkeypatch) -> None:
    monkeypatch.delenv("MOONSHOT_API_KEY", raising=False)

    with pytest.raises(RuntimeError, match="missing API key env var 'MOONSHOT_API_KEY'"):
        _create_chat(_parser_args("--backend", "kimi"))


def test_mock_backend_cannot_be_recorded_as_real_visual_api() -> None:
    with pytest.raises(RuntimeError, match=r"no_verified_vision_support.*backend 'mock'"):
        _create_chat(_parser_args("--backend", "mock"))


def test_local_qwen_mode_keeps_model_path_and_device(monkeypatch) -> None:
    sentinel = object()
    received = []
    monkeypatch.setattr(
        person_follow,
        "_create_local_chat",
        lambda model_path, device: (received.append((model_path, device)), sentinel)[1],
    )

    model, evidence = _create_chat(_parser_args("--model-path", "qwen-local", "--device", "cpu"))

    assert model is sentinel
    assert received == [(person_follow.Path("qwen-local"), "cpu")]
    assert evidence == {
        "backend": "local_qwen",
        "model": "qwen-local",
        "base_url": "",
        "real_api_requested": False,
    }


def test_model_path_and_api_backend_are_mutually_exclusive() -> None:
    with pytest.raises(SystemExit):
        _parser_args("--model-path", "qwen", "--backend", "kimi")


def test_api_client_is_closed_after_stop(monkeypatch) -> None:
    class DummyClient:
        closed = False

        async def close(self):
            self.closed = True

    client = DummyClient()
    monkeypatch.setattr(
        person_follow,
        "_create_chat",
        lambda _args: (
            client,
            {"backend": "kimi", "model": "kimi-k2.6", "base_url": "", "real_api_requested": True},
        ),
    )

    async def stopped(_args, _model, _evidence):
        return {"status": "stopped_by_instruction"}

    monkeypatch.setattr(person_follow, "_run_with_model", stopped)

    result = asyncio.run(person_follow.run(SimpleNamespace()))

    assert result == {"status": "stopped_by_instruction"}
    assert client.closed


def test_api_client_is_closed_when_replay_fails(monkeypatch) -> None:
    class DummyClient:
        closed = False

        async def close(self):
            self.closed = True

    client = DummyClient()
    monkeypatch.setattr(
        person_follow,
        "_create_chat",
        lambda _args: (
            client,
            {"backend": "kimi", "model": "kimi-k2.6", "base_url": "", "real_api_requested": True},
        ),
    )

    async def fail(_args, _model, _evidence):
        raise RuntimeError("replay failed")

    monkeypatch.setattr(person_follow, "_run_with_model", fail)

    with pytest.raises(RuntimeError, match="replay failed"):
        asyncio.run(person_follow.run(SimpleNamespace()))
    assert client.closed


def test_llm_plan_is_actually_requested_and_validated() -> None:
    calls = []

    async def chat(messages):
        calls.append(messages)
        return '{"subgoals":[{"action":"follow","target":"穿红衣服的人","parameters":{}}]}'

    evidence = asyncio.run(parse_instruction("跟踪穿红衣服的人", chat))

    assert calls == [evidence.messages]
    assert evidence.raw_response.startswith("{")
    assert validate_plan(evidence.plan) == (SubGoalAction.FOLLOW, "穿红衣服的人")


@pytest.mark.parametrize(
    "response,reason",
    [
        ("not json", "unsupported_action:status"),
        (
            '{"subgoals":[{"action":"follow","target":"甲"},{"action":"follow","target":"乙"}]}',
            "expected_exactly_one_follow_or_stop_action",
        ),
        ('{"subgoals":[{"action":"navigate","target":"人"}]}', "unsupported_action:navigate"),
    ],
)
def test_unsupported_llm_plan_fails_explicitly(response: str, reason: str) -> None:
    async def chat(_messages):
        return response

    with pytest.raises(UnsupportedInstruction, match=reason):
        asyncio.run(parse_instruction("跟踪一个人", chat))


def test_stop_is_the_only_other_supported_action() -> None:
    async def chat(_messages):
        return '{"subgoals":[{"action":"stop","target":"person_follow"}]}'

    evidence = asyncio.run(parse_instruction("停止跟踪", chat))
    assert validate_plan(evidence.plan) == (SubGoalAction.STOP, "person_follow")


def test_tracker_input_preserves_detector_boxes_scores_and_indices() -> None:
    detections = [
        Detection2D(np.asarray([1, 2, 30, 40]), 0.8, "person", class_id=-1),
        Detection2D(np.asarray([50, 6, 90, 100]), 0.12, "person", class_id=99),
    ]

    result = tracker_input(detections)

    np.testing.assert_array_equal(result.xyxy, [[1, 2, 30, 40], [50, 6, 90, 100]])
    np.testing.assert_allclose(result.conf, [0.8, 0.12])
    np.testing.assert_array_equal(result.cls, [0, 0])
    np.testing.assert_array_equal(result._orig_indices, [0, 1])
    assert result.to_records() == [
        {
            "bbox_xyxy": [1.0, 2.0, 30.0, 40.0],
            "score": pytest.approx(0.8),
            "class_id": 0,
            "detection_index": 0,
        },
        {
            "bbox_xyxy": [50.0, 6.0, 90.0, 100.0],
            "score": pytest.approx(0.12),
            "class_id": 0,
            "detection_index": 1,
        },
    ]


def test_missing_selected_id_never_switches_to_visible_person() -> None:
    tracks = np.asarray(
        [
            [10, 10, 40, 100, 3, 0.9, 0, 0],
            [60, 10, 100, 100, 8, 0.8, 0, 1],
        ],
        dtype=np.float32,
    )

    assert int(selected_track(tracks, 8)[4]) == 8
    assert selected_track(tracks, 5) is None
    assert selected_track(tracks, None) is None


def test_crop_order_matches_numbered_track_order() -> None:
    frame = np.zeros((12, 20, 3), dtype=np.uint8)
    frame[:, :10] = 10
    frame[:, 10:] = 200
    tracks = np.asarray(
        [[0, 0, 10, 12, 4, 0.9, 0, 0], [10, 0, 20, 12, 9, 0.8, 0, 1]],
        dtype=np.float32,
    )

    crops = crop_tracks(frame, tracks)

    assert len(crops) == 2
    assert int(crops[0].mean()) == 10
    assert int(crops[1].mean()) == 200


def test_vlm_zero_means_no_target_without_arbitrary_fallback() -> None:
    calls = []

    async def chat(messages):
        calls.append(messages)
        return "0"

    crops = [np.zeros((20, 10, 3), dtype=np.uint8), np.ones((20, 10, 3), dtype=np.uint8)]
    selected = asyncio.run(select_person("穿红衣服的人", crops, chat))

    assert selected == -1
    assert len(calls) == 1


def test_missing_target_waits_half_second_then_reacquisition_is_due() -> None:
    assert not reacquisition_due(10, 30.0, missing_since=10, last_attempt=None)
    assert not reacquisition_due(24, 30.0, missing_since=10, last_attempt=None)
    assert reacquisition_due(25, 30.0, missing_since=10, last_attempt=None)
    assert not reacquisition_due(30, 30.0, missing_since=10, last_attempt=25)
    assert reacquisition_due(40, 30.0, missing_since=10, last_attempt=25)


def test_reacquisition_zero_keeps_old_id_missing_and_does_not_bind_other_person() -> None:
    messages_seen = []

    async def chat(messages):
        messages_seen.append(messages)
        return "0"

    frame = np.full((30, 40, 3), 50, dtype=np.uint8)
    tracks = np.asarray(
        [[0, 0, 20, 30, 12, 0.9, 0, 0], [20, 0, 40, 30, 18, 0.8, 0, 1]],
        dtype=np.float32,
    )
    new_id, selected_index = asyncio.run(
        select_track_id(
            "灰色上衣、粉色包的人",
            frame,
            tracks,
            chat,
            reference_crop_bgr=np.full((20, 10, 3), 100, dtype=np.uint8),
        )
    )

    assert new_id is None
    assert selected_index == -1
    assert len(messages_seen) == 1
    content = messages_seen[0][0]["content"]
    assert content[0]["text"].startswith("Reference image")
    assert content[1]["type"] == "image_url"
    assert "Numbered current candidates" in content[2]["text"]


def test_reacquisition_can_bind_only_the_id_selected_by_vlm() -> None:
    async def chat(_messages):
        return "2"

    frame = np.zeros((30, 40, 3), dtype=np.uint8)
    tracks = np.asarray(
        [[0, 0, 20, 30, 12, 0.9, 0, 0], [20, 0, 40, 30, 18, 0.8, 0, 1]],
        dtype=np.float32,
    )

    new_id, selected_index = asyncio.run(
        select_track_id(
            "灰色上衣、粉色包的人",
            frame,
            tracks,
            chat,
            reference_crop_bgr=np.ones((20, 10, 3), dtype=np.uint8),
        )
    )

    assert new_id == 18
    assert selected_index == 1


@pytest.mark.parametrize("response", ["OTHER", "UNKNOWN", "MATCH because clothes look similar", "match"])
def test_pair_verification_rejects_other_unknown_and_non_strict_match(response: str) -> None:
    calls = []

    async def chat(messages):
        calls.append(messages)
        return response

    verdict, new_id = asyncio.run(
        verify_candidate(
            "灰色上衣、粉色包的人",
            np.full((30, 20, 3), 200, dtype=np.uint8),
            17,
            chat,
            reference_crop_bgr=np.full((30, 20, 3), 40, dtype=np.uint8),
        )
    )

    assert verdict == identity_verdict(response)
    assert verdict != "MATCH"
    assert new_id is None
    assert len(calls) == 1
    prompt = calls[0][0]["content"][0]["text"]
    assert "灰色上衣、粉色包的人" in prompt
    assert "Reply with exactly MATCH" in prompt


def test_pair_verification_strict_match_binds_the_selected_candidate_id() -> None:
    async def chat(_messages):
        return "MATCH"

    verdict, new_id = asyncio.run(
        verify_candidate(
            "灰色上衣、粉色包的人",
            np.full((30, 20, 3), 42, dtype=np.uint8),
            17,
            chat,
            reference_crop_bgr=np.full((30, 20, 3), 40, dtype=np.uint8),
        )
    )

    assert verdict == "MATCH"
    assert new_id == 17


def test_initial_candidate_that_conflicts_with_description_is_not_bound() -> None:
    calls = []

    async def chat(messages):
        calls.append(messages)
        return "OTHER"

    verdict, new_id = asyncio.run(
        verify_candidate(
            "穿亮紫色羽绒服、戴黄色帽子的人",
            np.full((30, 20, 3), 80, dtype=np.uint8),
            6,
            chat,
        )
    )

    assert verdict == "OTHER"
    assert new_id is None
    content = calls[0][0]["content"]
    assert "穿亮紫色羽绒服、戴黄色帽子的人" in content[0]["text"]
    assert all(part.get("text") != "Original target crop:" for part in content)
    assert any(part.get("text") == "Current selected candidate crop:" for part in content)
