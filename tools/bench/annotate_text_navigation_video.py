"""Annotate a MuJoCo motion recording with a reproducible text-task plan.

This is an evidence helper: it records the parsed task and stage transitions on
top of an existing physics video. It does not claim that text parsing itself
controls the native planner.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from decision.tasks.decomposition import TaskDecomposer  # noqa: E402


TASK = "先去会议室，然后找到桌子旁边的箱子"
FONT = "C:/Windows/Fonts/Noto Sans SC (TrueType).otf"


def build_plan() -> dict[str, object]:
    symbolic = TaskDecomposer().parse_decomposition_response(
        TASK,
        {
            "subgoals": [
                {"action": "navigate", "target": "会议室", "parameters": {}},
                {"action": "find", "target": "桌子旁边的箱子", "parameters": {}},
            ]
        },
    )
    return {
        "schema_version": "lingtu.text_navigation_demo.v1",
        "input_text": TASK,
        "parser": "TaskDecomposer.parse_decomposition_response",
        "symbolic_plan": symbolic.to_dict(),
        "steps": [
            {"index": 1, "action": "navigate", "target": "会议室", "predicate": "in"},
            {
                "index": 2,
                "action": "find",
                "target": "桌子旁边的箱子",
                "predicate": "near",
            },
        ],
        "execution_scope": "mujoco_policy_motion_with_stage_overlay",
        "semantic_binding": "demo_plan_only",
        "motion_authorization": False,
    }


def render(input_path: Path, output_path: Path, plan_path: Path) -> dict[str, object]:
    capture = cv2.VideoCapture(str(input_path))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open input video: {input_path}")
    fps = float(capture.get(cv2.CAP_PROP_FPS) or 15.0)
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
    )
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f"cannot open output video: {output_path}")
    plan = build_plan()
    plan_path.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        progress = frame_index / max(1, total - 1)
        if progress < 0.18:
            stage = "1 解析文本任务"
            detail = "go_to(meeting_room)"
        elif progress < 0.62:
            stage = "2 前往会议室"
            detail = "native MuJoCo policy motion"
        else:
            stage = "3 查找目标"
            detail = "find(box) near(table)"

        overlay = frame.copy()
        cv2.rectangle(overlay, (22, 20), (width - 22, 160), (10, 15, 23), -1)
        cv2.addWeighted(overlay, 0.84, frame, 0.16, 0.0, frame)
        white = (240, 245, 250)
        muted = (174, 190, 205)
        cyan = (110, 225, 255)
        cv2.putText(frame, "LINGTU TEXT NAVIGATION / MUJOCO EVIDENCE", (42, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.72, cyan, 2, cv2.LINE_AA)
        # OpenCV's default Hershey font has no CJK glyphs; use a system CJK
        # font for the user command and stage, while keeping the HUD crisp.
        pil_frame = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil_frame)
        cjk = ImageFont.truetype(FONT, 25)
        cjk_small = ImageFont.truetype(FONT, 23)
        draw.text((42, 60), TASK, font=cjk, fill=(240, 245, 250))
        draw.text((42, 96), stage, font=cjk_small, fill=(240, 245, 250))
        frame = cv2.cvtColor(__import__("numpy").asarray(pil_frame), cv2.COLOR_RGB2BGR)
        cv2.putText(frame, detail, (42, 144), cv2.FONT_HERSHEY_SIMPLEX, 0.55, muted, 1, cv2.LINE_AA)

        bar_x, bar_y, bar_w = 42, height - 42, width - 84
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + 9), (65, 78, 92), -1)
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + int(bar_w * progress), bar_y + 9), cyan, -1)
        cv2.putText(frame, f"frame {frame_index + 1}/{total}  |  policy motion", (42, height - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.48, muted, 1, cv2.LINE_AA)
        writer.write(frame)
        frame_index += 1

    writer.release()
    capture.release()
    return {
        "input_video": str(input_path),
        "output_video": str(output_path),
        "plan": str(plan_path),
        "frames": frame_index,
        "fps": fps,
        "duration_s": frame_index / fps if fps else 0.0,
        "text_visible": True,
        "robot_physics_motion": True,
        "motion_source": "fixed_velocity_command_to_thunderv4_policy",
        "native_planner_executed": False,
        "plan_parser": "TaskDecomposer.parse_decomposition_response",
        "semantic_target_binding": "demo_plan_only",
        "motion_authorization": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    report = render(args.input, args.output, args.plan)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
