"""Replay a real RGB video for language-selected person tracking evidence.

This experiment proves only the RGB perception path: an LLM parses one follow
or stop instruction, a VLM proposes and verifies a visible target, and
NativeByteTracker supplies 2-D identities.  Inference can use local Qwen
weights or a registered API client with verified vision support.  When an
identity disappears, a sampled VLM comparison can verify a new track against
the original target.  It does not infer metric depth, move a robot, or
authorize native navigation.
"""

from __future__ import annotations

import argparse
import asyncio
import inspect
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Awaitable, Callable, Sequence

import numpy as np

from decision.tasks.decomposition import SubGoalAction, TaskDecomposer, TaskPlan
from decision.vision.person import PersonTracker, identity_verdict, verify_candidate
from perception.backends import DetectorSpec
from perception.detection.detector_base import Detection2D
from perception.tracking.native_byte_tracker import NativeByteTracker
from runtime.registry import get

Chat = Callable[[list[dict[str, Any]]], Awaitable[str]]


class UnsupportedInstruction(ValueError):
    """The model response did not describe one supported experiment action."""


@dataclass(frozen=True)
class PlanEvidence:
    """Exact LLM exchange plus the validated symbolic plan."""

    messages: list[dict[str, Any]]
    raw_response: str
    plan: TaskPlan

    def to_dict(self) -> dict[str, Any]:
        return {
            "messages": self.messages,
            "raw_response": self.raw_response,
            "plan": self.plan.to_dict(),
        }


class TrackerInput:
    """Minimal detector result surface consumed by NativeByteTracker."""

    def __init__(self, detections: Sequence[Detection2D]) -> None:
        if detections:
            self.xyxy = np.asarray([detection.bbox for detection in detections], dtype=np.float32)
            self.conf = np.asarray([detection.score for detection in detections], dtype=np.float32)
        else:
            self.xyxy = np.empty((0, 4), dtype=np.float32)
            self.conf = np.empty((0,), dtype=np.float32)
        # This experiment has one detector prompt, so all rows share one class.
        self.cls = np.zeros(len(detections), dtype=np.int32)
        self._orig_indices = np.arange(len(detections), dtype=np.int32)

    def to_records(self) -> list[dict[str, Any]]:
        """Return the exact detector rows passed to NativeByteTracker."""
        return [
            {
                "bbox_xyxy": [float(value) for value in self.xyxy[index]],
                "score": float(self.conf[index]),
                "class_id": int(self.cls[index]),
                "detection_index": int(self._orig_indices[index]),
            }
            for index in range(len(self.xyxy))
        ]


def validate_plan(plan: TaskPlan) -> tuple[SubGoalAction, str]:
    """Require exactly one follow or stop action from the model response."""
    errors = plan.validate()
    if errors:
        raise UnsupportedInstruction(f"invalid_task_plan:{','.join(errors)}")
    if len(plan.subgoals) != 1:
        raise UnsupportedInstruction("expected_exactly_one_follow_or_stop_action")
    goal = plan.subgoals[0]
    if goal.action not in (SubGoalAction.FOLLOW, SubGoalAction.STOP):
        raise UnsupportedInstruction(f"unsupported_action:{goal.action.value}")
    if goal.action is SubGoalAction.FOLLOW and not goal.target.strip():
        raise UnsupportedInstruction("follow_target_is_empty")
    return goal.action, goal.target.strip()


async def parse_instruction(instruction: str, chat: Chat) -> PlanEvidence:
    """Actually ask the language model to produce and validate the task plan."""
    decomposer = TaskDecomposer()
    messages = decomposer.build_decomposition_prompt(instruction)
    raw_response = str(await chat(messages))
    plan = decomposer.parse_decomposition_response(instruction, raw_response)
    validate_plan(plan)
    return PlanEvidence(messages=messages, raw_response=raw_response, plan=plan)


def tracker_input(detections: Sequence[Detection2D]) -> TrackerInput:
    """Convert actual detector rows to the exact NativeByteTracker input."""
    return TrackerInput(detections)


def selected_track(tracks: np.ndarray, selected_id: int | None) -> np.ndarray | None:
    """Find only the locked target; never substitute another visible track."""
    if selected_id is None:
        return None
    for row in np.asarray(tracks):
        if int(row[4]) == selected_id:
            return row
    return None


def reacquisition_due(
    frame_index: int,
    fps: float,
    missing_since: int | None,
    last_attempt: int | None,
    interval_s: float = 0.5,
) -> bool:
    """Sample VLM reacquisition no more often than the configured interval."""
    if missing_since is None:
        return False
    interval_frames = max(1, int(round(fps * interval_s)))
    if frame_index - missing_since < interval_frames:
        return False
    return last_attempt is None or frame_index - last_attempt >= interval_frames


def crop_tracks(frame_bgr: np.ndarray, tracks: np.ndarray) -> list[np.ndarray]:
    """Crop active track boxes in row order for numbered VLM selection."""
    height, width = frame_bgr.shape[:2]
    crops: list[np.ndarray] = []
    for row in tracks:
        x1, y1, x2, y2 = np.rint(row[:4]).astype(int)
        x1, x2 = np.clip([x1, x2], 0, width)
        y1, y2 = np.clip([y1, y2], 0, height)
        crops.append(frame_bgr[y1:y2, x1:x2].copy())
    return crops


async def select_person(
    description: str,
    crops_bgr: list[np.ndarray],
    chat: Chat,
    *,
    reference_crop_bgr: np.ndarray | None = None,
) -> int:
    """Use the repository VLM selector without attaching its CLIP fallback."""
    selector = PersonTracker()

    return await selector.choose_target_with_vlm(
        description,
        crops_bgr,
        chat,
        reference_crop_bgr=reference_crop_bgr,
    )


async def select_track_id(
    description: str,
    frame_bgr: np.ndarray,
    tracks: np.ndarray,
    chat: Chat,
    *,
    reference_crop_bgr: np.ndarray | None = None,
) -> tuple[int | None, int]:
    """Return only a VLM-selected active track ID and its crop index."""
    selected_index = await select_person(
        description,
        crop_tracks(frame_bgr, tracks),
        chat,
        reference_crop_bgr=reference_crop_bgr,
    )
    if selected_index < 0:
        return None, -1
    return int(tracks[selected_index, 4]), selected_index


def _track_dict(row: np.ndarray) -> dict[str, Any]:
    return {
        "track_id": int(row[4]),
        "bbox_xyxy": [round(float(value), 3) for value in row[:4]],
        "score": round(float(row[5]), 6),
        "class_id": int(row[6]),
        "detection_index": int(row[7]),
    }


def _draw_tracks(
    frame_bgr: np.ndarray,
    tracks: np.ndarray,
    selected_id: int | None,
    status: str,
) -> np.ndarray:
    import cv2

    output = frame_bgr.copy()
    for row in tracks:
        track_id = int(row[4])
        x1, y1, x2, y2 = np.rint(row[:4]).astype(int)
        color = (0, 215, 255) if track_id == selected_id else (70, 200, 70)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            output,
            f"ID {track_id} {float(row[5]):.2f}",
            (x1, max(20, y1 - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    cv2.putText(
        output,
        status,
        (16, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return output


def _numbered_source(frame_bgr: np.ndarray, tracks: np.ndarray) -> np.ndarray:
    import cv2

    output = frame_bgr.copy()
    for number, row in enumerate(tracks, start=1):
        x1, y1, x2, y2 = np.rint(row[:4]).astype(int)
        cv2.rectangle(output, (x1, y1), (x2, y2), (255, 180, 30), 3)
        cv2.putText(
            output,
            str(number),
            (x1 + 5, max(28, y1 + 26)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 180, 30),
            3,
            cv2.LINE_AA,
        )
    return output


def _contact_sheet(crops: list[np.ndarray]) -> np.ndarray | None:
    import cv2

    if not crops:
        return None
    tile_width, tile_height = 220, 300
    tiles: list[np.ndarray] = []
    for number, crop in enumerate(crops, start=1):
        if crop.size == 0:
            tile = np.zeros((tile_height, tile_width, 3), dtype=np.uint8)
        else:
            height, width = crop.shape[:2]
            scale = min(tile_width / width, (tile_height - 34) / height)
            resized = cv2.resize(crop, (max(1, int(width * scale)), max(1, int(height * scale))))
            tile = np.zeros((tile_height, tile_width, 3), dtype=np.uint8)
            y = 34 + (tile_height - 34 - resized.shape[0]) // 2
            x = (tile_width - resized.shape[1]) // 2
            tile[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
        cv2.putText(tile, f"Person {number}", (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        tiles.append(tile)
    return np.hstack(tiles)


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def _save_selection_artifacts(
    output_dir: Path,
    frame_index: int,
    stage: str,
    frame_bgr: np.ndarray,
    tracks: np.ndarray,
) -> dict[str, str | None]:
    import cv2

    selection_dir = output_dir / "selection"
    selection_dir.mkdir(exist_ok=True)
    stem = f"{frame_index:06d}-{stage}"
    source_path = selection_dir / f"{stem}-source.jpg"
    crops_path = selection_dir / f"{stem}-crops.jpg"
    cv2.imwrite(str(source_path), _numbered_source(frame_bgr, tracks))
    sheet = _contact_sheet(crop_tracks(frame_bgr, tracks))
    if sheet is not None:
        cv2.imwrite(str(crops_path), sheet)
    return {
        "source": str(source_path),
        "crops": str(crops_path) if sheet is not None else None,
    }


def _create_local_chat(model_path: Path, device: str):
    from tools.datasets.qwen_chat import QwenChat

    return QwenChat(model_path, device=device)


def _create_chat(args: argparse.Namespace) -> tuple[Any, dict[str, Any]]:
    """Create either the local Qwen runtime or a registered real API client."""
    if args.backend:
        from decision.modules.llm import LLMModule

        model = args.model
        if not model and args.backend.strip().lower() in {"kimi", "moonshot"}:
            model = "kimi-k3"
        module = LLMModule(
            backend=args.backend,
            model=model,
            api_key_env=args.api_key_env,
            timeout_sec=args.timeout_sec,
            base_url=args.base_url,
        )
        client = module._create_client()
        if not client.is_available():
            key_env = client.config.api_key_env
            raise RuntimeError(
                f"llm_api_unavailable: missing API key env var '{key_env}' "
                f"for backend '{args.backend}'"
            )
        if not client.supports_vision:
            raise RuntimeError(
                f"llm_api_has_no_verified_vision_support: backend '{args.backend}' "
                f"model '{client.config.model}'"
            )
        return client, {
            "backend": args.backend,
            "model": client.config.model,
            "base_url": client.config.base_url,
            "real_api_requested": True,
        }

    model = _create_local_chat(Path(args.model_path), args.device)
    return model, {
        "backend": "local_qwen",
        "model": str(Path(args.model_path)),
        "base_url": "",
        "real_api_requested": False,
    }


async def _close_chat(model: Any) -> None:
    close = getattr(model, "close", None)
    if close is None:
        return
    result = close()
    if inspect.isawaitable(result):
        await result


async def run(args: argparse.Namespace) -> dict[str, Any]:
    """Run the replay and close API resources on every exit path."""
    model, model_evidence = _create_chat(args)
    try:
        return await _run_with_model(args, model, model_evidence)
    finally:
        await _close_chat(model)


async def _run_with_model(
    args: argparse.Namespace,
    model: Any,
    model_evidence: dict[str, Any],
) -> dict[str, Any]:
    """Run the bounded language-to-person-track replay experiment."""
    import cv2

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    llm_calls: list[dict[str, Any]] = []

    async def recorded_chat(stage: str, messages: list[dict[str, Any]]) -> str:
        started = time.perf_counter()
        response = str(await model.chat(messages))
        llm_calls.append(
            {
                "stage": stage,
                "messages": messages,
                "raw_response": response,
                "elapsed_s": time.perf_counter() - started,
            }
        )
        _write_json(output_dir / "llm-calls.json", llm_calls)
        return response

    decomposer = TaskDecomposer()
    plan_messages = decomposer.build_decomposition_prompt(args.instruction)
    plan_response = await recorded_chat("instruction_decomposition", plan_messages)
    plan = decomposer.parse_decomposition_response(args.instruction, plan_response)
    plan_evidence = PlanEvidence(plan_messages, plan_response, plan)
    _write_json(output_dir / "plan.json", plan_evidence.to_dict())
    action, description = validate_plan(plan)

    base_summary = {
        "instruction": args.instruction,
        "action": action.value,
        "target_description": description,
        "source_video": str(Path(args.video)),
        "source_url": args.source_url,
        "real_video_replay": action is SubGoalAction.FOLLOW,
        "llm_instruction_parse": True,
        "vlm_target_selection_requested": action is SubGoalAction.FOLLOW,
        "metric_depth_available": False,
        "robot_motion": False,
        "rgbd": False,
        "native_planner": False,
        "ground_truth_accuracy_claim": False,
        "model": model_evidence["model"],
        "backend": model_evidence["backend"],
        "base_url": model_evidence["base_url"],
        "real_api_requested": model_evidence["real_api_requested"],
    }
    if action is SubGoalAction.STOP:
        summary = {**base_summary, "status": "stopped_by_instruction", "processed_frames": 0}
        _write_json(output_dir / "summary.json", summary)
        return summary

    detector = get("detector", "yolo_world").create(
        DetectorSpec(
            name="yolo_world",
            model_path=str(Path(args.yolo_weights)),
            confidence=0.1,
            device=args.device,
        )
    )
    detector.load_model()
    capture = cv2.VideoCapture(str(Path(args.video)))
    if not capture.isOpened():
        detector.shutdown()
        raise RuntimeError(f"cannot_open_video:{args.video}")

    fps = float(capture.get(cv2.CAP_PROP_FPS)) or 30.0
    tracker = NativeByteTracker(
        frame_rate=max(1, int(round(fps))),
        track_high_thresh=0.25,
        track_low_thresh=0.1,
        new_track_thresh=0.25,
        track_buffer=30,
    )
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    video_path = output_dir / "follow.mp4"
    writer = cv2.VideoWriter(
        str(video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        capture.release()
        detector.shutdown()
        raise RuntimeError("cannot_open_output_video")

    selected_id: int | None = None
    initial_selected_id: int | None = None
    original_target_crop: np.ndarray | None = None
    selection_result = "not_reached"
    selection_events: list[dict[str, Any]] = []
    missing_since: int | None = None
    last_reacquisition_attempt: int | None = None
    frame_records: list[dict[str, Any]] = []
    target_visible_frames = 0
    target_missing_frames = 0

    async def attempt_selection(
        stage: str,
        frame_index: int,
        frame_bgr: np.ndarray,
        tracks: np.ndarray,
        old_track_id: int | None,
        reference_crop_bgr: np.ndarray | None,
    ) -> tuple[int | None, int]:
        artifacts = _save_selection_artifacts(
            output_dir,
            frame_index,
            stage,
            frame_bgr,
            tracks,
        )
        call_count_before = len(llm_calls)

        async def selection_chat(messages: list[dict[str, Any]]) -> str:
            return await recorded_chat(f"person_selection_{stage}", messages)

        selection_candidate_id, selected_index = await select_track_id(
            description,
            frame_bgr,
            tracks,
            selection_chat,
            reference_crop_bgr=reference_crop_bgr,
        )
        identity_result: str | None = None
        new_track_id: int | None = None
        if selection_candidate_id is not None:
            candidate_crop = crop_tracks(frame_bgr, tracks)[selected_index]

            async def verification_chat(messages: list[dict[str, Any]]) -> str:
                return await recorded_chat(f"person_identity_verification_{stage}", messages)

            identity_result, new_track_id = await verify_candidate(
                description,
                candidate_crop,
                selection_candidate_id,
                verification_chat,
                reference_crop_bgr=reference_crop_bgr,
            )
        event = {
            "frame": frame_index,
            "time_s": round(frame_index / fps, 6),
            "stage": stage,
            "old_track_id": old_track_id,
            "selection_candidate_id": selection_candidate_id,
            "identity_verdict": identity_result,
            "new_track_id": new_track_id,
            "result": "selected" if new_track_id is not None else "no_match",
            "candidate_track_ids": [int(row[4]) for row in tracks],
            "reference_used": reference_crop_bgr is not None,
            "vlm_called": len(llm_calls) > call_count_before,
            "artifacts": artifacts,
        }
        selection_events.append(event)
        _write_json(output_dir / "selection-events.json", selection_events)
        return new_track_id, selected_index

    try:
        frame_index = 0
        while frame_index < args.max_frames:
            ok, frame = capture.read()
            if not ok:
                break
            detections = detector.detect(frame, "person")
            current_tracker_input = tracker_input(detections)
            tracks = tracker.update(current_tracker_input, frame)

            if frame_index == args.selection_frame:
                crops = crop_tracks(frame, tracks)
                selected_id, selected_index = await attempt_selection(
                    "initial",
                    frame_index,
                    frame,
                    tracks,
                    None,
                    None,
                )
                initial_selected_id = selected_id
                cv2.imwrite(str(output_dir / "selection-source.jpg"), _numbered_source(frame, tracks))
                sheet = _contact_sheet(crops)
                if sheet is not None:
                    cv2.imwrite(str(output_dir / "selection-crops.jpg"), sheet)
                if selected_id is not None:
                    selection_result = "selected"
                    original_target_crop = crops[selected_index].copy()
                    cv2.imwrite(
                        str(output_dir / "selection" / "reference-target.jpg"),
                        original_target_crop,
                    )
                else:
                    selection_result = "no_match"

            target = selected_track(tracks, selected_id)
            if selected_id is not None and target is None:
                if missing_since is None:
                    missing_since = frame_index
                if original_target_crop is not None and reacquisition_due(
                    frame_index,
                    fps,
                    missing_since,
                    last_reacquisition_attempt,
                    args.reacquire_interval_s,
                ):
                    old_track_id = selected_id
                    last_reacquisition_attempt = frame_index
                    new_track_id, _ = await attempt_selection(
                        "reacquire",
                        frame_index,
                        frame,
                        tracks,
                        old_track_id,
                        original_target_crop,
                    )
                    if new_track_id is not None:
                        selected_id = new_track_id
                        target = selected_track(tracks, selected_id)
                        missing_since = None
            elif target is not None:
                missing_since = None
                last_reacquisition_attempt = None

            if selected_id is None:
                status = "NO TARGET SELECTED"
            elif target is None:
                status = f"WAITING FOR ID {selected_id}"
                target_missing_frames += 1
            else:
                status = f"FOLLOWING ID {selected_id}"
                target_visible_frames += 1

            writer.write(_draw_tracks(frame, tracks, selected_id, status))
            frame_records.append(
                {
                    "frame": frame_index,
                    "time_s": round(frame_index / fps, 6),
                    "detections": current_tracker_input.to_records(),
                    "tracks": [_track_dict(row) for row in tracks],
                    "selected_track_id": selected_id,
                    "target": _track_dict(target) if target is not None else None,
                    "status": status.lower().replace(" ", "_"),
                }
            )
            frame_index += 1
    finally:
        writer.release()
        capture.release()
        detector.shutdown()

    _write_json(output_dir / "tracks.json", frame_records)
    summary = {
        **base_summary,
        "status": "completed",
        "processed_frames": len(frame_records),
        "selection_frame": args.selection_frame,
        "selection_result": selection_result,
        "vlm_target_selection_call": any(
            call["stage"].startswith("person_selection_") for call in llm_calls
        ),
        "reacquisition_interval_s": args.reacquire_interval_s,
        "selection_events": len(selection_events),
        "reacquisition_attempts": sum(
            event["stage"] == "reacquire" for event in selection_events
        ),
        "successful_reacquisitions": sum(
            event["stage"] == "reacquire" and event["new_track_id"] is not None
            for event in selection_events
        ),
        "initial_selected_track_id": initial_selected_id,
        "selected_track_id": selected_id,
        "target_visible_frames": target_visible_frames,
        "target_missing_frames": target_missing_frames,
        "annotated_video": str(video_path),
        "track_space": "source_video_pixels_xyxy",
        "tracker_input_detections_recorded": True,
        "tracker": "NativeByteTracker",
        "detector": "YOLO-World",
    }
    _write_json(output_dir / "summary.json", summary)
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", required=True, type=Path)
    parser.add_argument("--yolo-weights", required=True, type=Path)
    model_source = parser.add_mutually_exclusive_group(required=True)
    model_source.add_argument("--model-path", type=Path)
    model_source.add_argument(
        "--backend",
        nargs="?",
        const="kimi",
        help="Use a registered API backend; '--backend' alone selects Kimi.",
    )
    parser.add_argument("--model", default="")
    parser.add_argument("--base-url", default="")
    parser.add_argument("--api-key-env", default="")
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-frames", type=int, default=150)
    parser.add_argument("--selection-frame", type=int, default=0)
    parser.add_argument("--reacquire-interval-s", type=float, default=0.5)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--source-url", default="")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.max_frames <= 0:
        raise SystemExit("--max-frames must be positive")
    if args.selection_frame < 0 or args.selection_frame >= args.max_frames:
        raise SystemExit("--selection-frame must be within processed frames")
    if args.reacquire_interval_s <= 0:
        raise SystemExit("--reacquire-interval-s must be positive")
    if args.timeout_sec <= 0:
        raise SystemExit("--timeout-sec must be positive")
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
