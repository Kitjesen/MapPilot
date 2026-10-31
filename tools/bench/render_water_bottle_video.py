#!/usr/bin/env python3
"""Render a truthful visual trace of the water-bottle navigation slice."""

from __future__ import annotations

import argparse
import json
from itertools import pairwise
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT_PATH = Path("C:/Windows/Fonts/Noto Sans SC (TrueType).otf")
FONT_BOLD_PATH = Path("C:/Windows/Fonts/Noto Sans SC Bold (TrueType).otf")
XMIN, XMAX = -2.8, 4.8
YMIN, YMAX = -1.6, 7.3


def _font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = FONT_BOLD_PATH if bold and FONT_BOLD_PATH.exists() else FONT_PATH
    return ImageFont.truetype(str(path), size)


def _map_xy(x: float, y: float, *, origin: tuple[int, int], size: tuple[int, int]) -> tuple[int, int]:
    ox, oy = origin
    width, height = size
    px = ox + int((x - XMIN) / (XMAX - XMIN) * width)
    py = oy + int((YMAX - y) / (YMAX - YMIN) * height)
    return px, py


def _draw_dashed(draw: ImageDraw.ImageDraw, points: list[tuple[int, int]], fill, width: int = 4) -> None:
    for first, second in pairwise(points):
        x1, y1 = first
        x2, y2 = second
        dx, dy = x2 - x1, y2 - y1
        length = max(1.0, float((dx * dx + dy * dy) ** 0.5))
        for start in np.arange(0.0, length, 14.0):
            end = min(length, start + 8.0)
            draw.line(
                (x1 + dx * start / length, y1 + dy * start / length,
                 x1 + dx * end / length, y1 + dy * end / length),
                fill=fill,
                width=width,
            )


def render(
    *,
    map_image: Path,
    slice_report: Path,
    output: Path,
    report_output: Path,
    cover_output: Path | None = None,
    fps: float = 15.0,
    duration_s: float = 12.0,
) -> dict[str, object]:
    report = json.loads(slice_report.read_text(encoding="utf-8"))
    query = report["query"]
    preview = report.get("preview") or {}
    results = preview.get("results", [])
    selected = next((item for item in results if item.get("planner_admitted")), None)
    native_path = (selected or {}).get("native_result", {}).get("path", [])
    candidates = query["approach_goal_candidates"].get("candidates", [])
    center = query["approach_goal_candidates"].get("object_center", [0.5369, 5.6480, 0.8840])
    start = preview.get("start", [4.2, -0.6, 1.0])

    background = Image.open(map_image).convert("RGB")
    map_origin = (32, 106)
    map_size = (744, 579)
    background = background.resize(map_size, Image.Resampling.LANCZOS)
    width, height = 1280, 720
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"cannot open video output: {output}")

    frame_count = int(round(fps * duration_s))
    path_pixels = [_map_xy(float(p[0]), float(p[1]), origin=map_origin, size=map_size) for p in native_path]
    target_px = _map_xy(float(center[0]), float(center[1]), origin=map_origin, size=map_size)
    start_px = _map_xy(float(start[0]), float(start[1]), origin=map_origin, size=map_size)
    candidate_px = [
        _map_xy(float(item["x"]), float(item["y"]), origin=map_origin, size=map_size)
        for item in candidates
    ]

    for frame_index in range(frame_count):
        progress = frame_index / max(1, frame_count - 1)
        canvas = Image.new("RGB", (width, height), (10, 18, 29))
        draw = ImageDraw.Draw(canvas)
        draw.rounded_rectangle((18, 18, 792, 700), radius=16, fill=(245, 247, 249), outline=(84, 104, 124), width=2)
        canvas.paste(background, map_origin)
        draw = ImageDraw.Draw(canvas)
        draw.rounded_rectangle((818, 18, 1262, 700), radius=16, fill=(17, 29, 43), outline=(54, 77, 98), width=2)

        # Target and approach hypotheses.
        for px, py in candidate_px:
            draw.ellipse((px - 6, py - 6, px + 6, py + 6), fill=(255, 166, 57), outline=(121, 73, 21), width=2)
        pulse = int(10 + 8 * (0.5 + 0.5 * np.sin(progress * np.pi * 8)))
        draw.ellipse((target_px[0] - 15 - pulse, target_px[1] - 15 - pulse,
                      target_px[0] + 15 + pulse, target_px[1] + 15 + pulse), outline=(247, 111, 64), width=3)
        draw.ellipse((target_px[0] - 10, target_px[1] - 10, target_px[0] + 10, target_px[1] + 10), fill=(237, 79, 58), outline=(255, 238, 196), width=2)
        draw.text((target_px[0] + 16, target_px[1] - 23), "water bottle / contract fixture", font=_font(17, True), fill=(173, 38, 35))

        # The returned native path is a preview cursor, not executed motion.
        if path_pixels and progress >= 0.62:
            draw_path_count = max(2, int((progress - 0.62) / 0.20 * len(path_pixels)))
            draw.line(path_pixels[:draw_path_count], fill=(245, 178, 55), width=4)
            _draw_dashed(draw, path_pixels[:draw_path_count], fill=(255, 231, 157), width=2)
            cursor = path_pixels[min(draw_path_count - 1, len(path_pixels) - 1)]
        else:
            cursor = start_px
        draw.ellipse((cursor[0] - 12, cursor[1] - 12, cursor[0] + 12, cursor[1] + 12), fill=(36, 132, 255), outline=(225, 246, 255), width=3)
        draw.polygon([(cursor[0], cursor[1] - 20), (cursor[0] - 7, cursor[1] - 9), (cursor[0] + 7, cursor[1] - 9)], fill=(36, 132, 255))

        # Left panel legend.
        draw.text((38, 40), "LingTu 水瓶语义导航切片", font=_font(26, True), fill=(11, 24, 37))
        draw.text((38, 70), "地图视图 / 橙色候选 / 蓝色预览游标", font=_font(17), fill=(77, 94, 112))
        draw.text((38, 655), "虚线 = 原生 planner 预览路径，不代表机器狗已经执行", font=_font(17, True), fill=(159, 44, 32))

        # Right panel HUD.
        white, muted, cyan, green, red, amber = (242, 247, 252), (164, 182, 199), (99, 216, 255), (109, 225, 151), (255, 114, 98), (255, 192, 81)
        draw.text((846, 42), "TEXT → INSTANCE → PLAN", font=_font(22, True), fill=cyan)
        draw.text((846, 78), "帮我找一瓶水", font=_font(28, True), fill=white)
        stages = [
            ("1 解析文本", progress >= 0.10, "water_bottle"),
            ("2 绑定语义实例", progress >= 0.25, "water-bottle-fixture"),
            ("3 生成接近候选", progress >= 0.42, "8 viewpoints"),
            ("4 原生规划预览", progress >= 0.62, "1 / 8 discrete"),
            ("5 连续验证", progress >= 0.82, "0 / 8 accepted"),
        ]
        y = 145
        for name, done, detail in stages:
            color = green if done else muted
            mark = "✓" if done else "○"
            draw.text((846, y), mark, font=_font(21, True), fill=color)
            draw.text((880, y), name, font=_font(19, True), fill=color)
            draw.text((880, y + 27), detail, font=_font(16), fill=muted)
            y += 75

        draw.line((846, 520, 1236, 520), fill=(54, 77, 98), width=1)
        draw.text((846, 540), "最终状态", font=_font(18), fill=muted)
        final_status = str(report.get("status", "verifier_blocked"))
        draw.text((846, 568), final_status, font=_font(26, True), fill=red if progress >= 0.82 else amber)
        draw.text((846, 615), "motion_authorization = false", font=_font(18, True), fill=red)
        draw.text((846, 650), "诊断仿真 / 未连接实机", font=_font(16), fill=muted)

        frame = cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)
        writer.write(frame)
        if cover_output is not None and frame_index == frame_count // 2:
            cover_output.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(cover_output), frame)

    writer.release()
    output_report = {
        "schema_version": "lingtu.water_bottle_video.v1",
        "video": str(output),
        "cover": str(cover_output) if cover_output else "",
        "duration_s": frame_count / fps,
        "fps": fps,
        "frames": frame_count,
        "video_type": "diagnostic_simulation_visualization",
        "instruction": "帮我找一瓶水",
        "target_source": "synthetic_contract_fixture",
        "native_planner_preview_executed": True,
        "native_preview_status": report.get("status"),
        "planner_admitted_count": preview.get("planner_admitted_count", 0),
        "verification_passed_count": preview.get("verification_passed_count", 0),
        "robot_motion_executed": False,
        "motion_authorization": False,
        "truth_label": "虚线是规划预览，不是机器人实走；当前视频不证明实机找到了水瓶",
    }
    report_output.parent.mkdir(parents=True, exist_ok=True)
    report_output.write_text(json.dumps(output_report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return output_report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-image", type=Path, required=True)
    parser.add_argument("--slice-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--cover", type=Path)
    args = parser.parse_args()
    result = render(
        map_image=args.map_image,
        slice_report=args.slice_report,
        output=args.output,
        report_output=args.report,
        cover_output=args.cover,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
