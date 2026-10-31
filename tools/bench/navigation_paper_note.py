"""Append measured navigation and height-diagnosis evidence to the dated paper.

Uses existing reports only. No replay, map modification, robot, or model call.
Run with the installed Codex Python that provides reportlab/pypdf/pypdfium2.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from datetime import datetime
from itertools import pairwise
from pathlib import Path
from statistics import mean
from xml.sax.saxutils import escape

from pypdf import PdfReader, PdfWriter
from reportlab.graphics import renderPDF
from reportlab.graphics.shapes import Circle, Drawing, Line, PolyLine, String
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    PageBreak,
    PageTemplate,
    Paragraph,
    Spacer,
    Table,
    TableStyle,
)

ROOT = Path(__file__).resolve().parents[2]
EXPERIMENT = ROOT / "build/nav-dataset-execution-20261021"
WORK = EXPERIMENT / "paper-note"
BASE_PDF = ROOT / "output/pdf/lingtu-mapping-paper-20261021-expanded.pdf"
OUT = ROOT / "output/pdf/lingtu-mapping-paper-20261021-navigation.pdf"
PAGE_W, PAGE_H = A4
MARGIN = 48
WIDTH = PAGE_W - 2 * MARGIN
BLUE = colors.HexColor("#23679b")
ORANGE = colors.HexColor("#b75b35")
SOURCES: dict = {}
TABLES: dict = {}
TEXT: list[str] = []


def source(relative: str) -> str:
    path = ROOT / relative
    text = path.read_text(encoding="utf-8-sig")
    destination = WORK / "sources" / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text, encoding="utf-8")
    SOURCES[relative] = {
        "path": str(path),
        "bytes": path.stat().st_size,
        "mtime": datetime.fromtimestamp(path.stat().st_mtime).isoformat(),
    }
    return text


def load(relative: str):
    return json.loads(source(relative))


def rows(relative: str):
    return list(csv.DictReader(source(relative).splitlines()))


def fmt(value, precision=3):
    return "未测" if value is None else f"{value:.{precision}f}"


def route_metrics(route):
    xyz = [[float(row[k]) for k in ("x_m", "y_m", "z_m")] for row in route]
    distances = [math.dist(a, b) for a, b in pairwise(xyz)]
    return {
        "points": len(xyz),
        "length_m": sum(distances),
        "spacing_min_m": min(distances),
        "spacing_mean_m": mean(distances),
        "spacing_max_m": max(distances),
        "xyz": xyz,
        "segments_m": distances,
    }


def run_data(label):
    prefix = f"build/nav-dataset-execution-20261021/runs/{label}"
    summary = load(f"{prefix}/summary.json")
    command = load(f"{prefix}/command.json")
    fixture = load(f"{prefix}/collision-fixture.json")
    resolved = load(f"{prefix}/resolved-config.json")
    feedback = rows(f"{prefix}/feedback.csv")
    route = route_metrics(rows(f"{prefix}/global-route.csv"))
    active = [r for r in feedback if r["tracking_active"] == "1"]
    errors = [float(r["position_error_m"]) for r in active]
    goal = load(f"build/nav-dataset-execution-20261021/input/{command['case']}.{command['variant']}-request.json")[
        "goal"
    ]
    final = summary["final_position"]
    return {
        "label": label,
        "summary": summary,
        "command": command,
        "fixture": fixture,
        "resolved": resolved,
        "feedback": feedback,
        "route": route,
        "active_samples": len(active),
        "tracking_rmse_m": math.sqrt(mean(e * e for e in errors)) if errors else None,
        "tracking_max_m": max(errors) if errors else None,
        "requested_goal_xy_error_m": math.hypot(final[0] - goal[0], final[1] - goal[1]),
        "requested_goal_z_error_m": abs(final[2] - goal[2]),
    }


def figure(run):
    """Draw measured route/feedback and point spacing at native Arial 10 pt."""
    drawing = Drawing(WIDTH, 208)
    route = run["route"]
    points = route["xyz"]
    feedback = run["feedback"]
    tracked = [[float(r["x_m"]), float(r["y_m"])] for r in feedback]
    tracked = tracked[:: max(1, len(tracked) // 200)] + tracked[-1:]

    def text(x, y, value, anchor="start", color=colors.black):
        drawing.add(String(x, y, value, fontName="Arial", fontSize=10, textAnchor=anchor, fillColor=color))

    def axes(x0, y0, width, height, xlim, ylim, xticks, yticks, xlabel, ylabel):
        def xy(x, y):
            return x0 + (x - xlim[0]) / (xlim[1] - xlim[0]) * width, y0 + (y - ylim[0]) / (ylim[1] - ylim[0]) * height

        drawing.add(Line(x0, y0, x0 + width, y0, strokeWidth=0.6))
        drawing.add(Line(x0, y0, x0, y0 + height, strokeWidth=0.6))
        for tick in xticks:
            x, y = xy(tick, ylim[0])
            drawing.add(Line(x, y, x, y - 3, strokeWidth=0.5))
            text(x, y - 15, f"{tick:g}", "middle")
        for tick in yticks:
            x, y = xy(xlim[0], tick)
            drawing.add(Line(x, y, x - 3, y, strokeWidth=0.5))
            text(x - 6, y - 3, f"{tick:g}", "end")
        text(x0 + width / 2, y0 - 33, xlabel, "middle")
        text(x0, y0 + height + 2, ylabel)
        return xy

    x0, y0, pw, ph = 40, 40, 190, 118
    xs = [p[0] for p in points] + [p[0] for p in tracked]
    ys = [p[1] for p in points] + [p[1] for p in tracked]
    xmid, ymid = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    xspan = max(max(xs) - min(xs), (max(ys) - min(ys)) * pw / ph) * 1.15
    yspan = xspan * ph / pw
    xlim, ylim = (xmid - xspan / 2, xmid + xspan / 2), (ymid - yspan / 2, ymid + yspan / 2)
    xticks = [math.ceil(xlim[0]), math.floor(xlim[1])]
    yticks = [round(ylim[0] + yspan * 0.25, 1), round(ylim[0] + yspan * 0.75, 1)]
    xy = axes(x0, y0, pw, ph, xlim, ylim, xticks, yticks, "X (m)", "Y (m)")
    drawing.add(PolyLine([v for p in points for v in xy(p[0], p[1])], strokeColor=BLUE, strokeWidth=1.0))
    for p in points:
        x, y = xy(p[0], p[1])
        drawing.add(Circle(x, y, 1.4, strokeColor=BLUE, fillColor=colors.white, strokeWidth=0.5))
    drawing.add(PolyLine([v for p in tracked for v in xy(*p)], strokeColor=ORANGE, strokeWidth=1.1))
    text(40, 194, "(a) Global route and XY feedback")
    drawing.add(Line(41, 177, 55, 177, strokeColor=BLUE, strokeWidth=1.0))
    text(59, 174, "Route")
    drawing.add(Line(113, 177, 127, 177, strokeColor=ORANGE, strokeWidth=1.0))
    text(131, 174, "XY feedback")

    xmax = len(route["segments_m"])
    xy2 = axes(
        294,
        40,
        WIDTH - 307,
        118,
        (1, xmax),
        (0.08, 0.17),
        [1, 5, 10, 15, 20, xmax],
        [0.10, 0.15],
        "Segment index",
        "3D spacing (m)",
    )
    drawing.add(
        PolyLine(
            [v for i, value in enumerate(route["segments_m"], 1) for v in xy2(i, value)],
            strokeColor=BLUE,
            strokeWidth=1.0,
        )
    )
    text(294, 194, "(b) Returned route spacing")
    directory = WORK / "figures"
    directory.mkdir(exist_ok=True)
    renderPDF.drawToFile(drawing, str(directory / "route-feedback-spacing.pdf"))
    return drawing


def build(labels):
    WORK.mkdir(parents=True, exist_ok=True)
    for name, filename in (("Song", "simsun.ttc"), ("Hei", "msyhbd.ttc"), ("Arial", "arial.ttf")):
        pdfmetrics.registerFont(TTFont(name, str(Path("C:/Windows/Fonts") / filename), subfontIndex=0))
    pdfmetrics.registerFontFamily("Song", normal="Song", bold="Hei")
    runs = [run_data(label) for label in labels]
    standard = next(r for r in runs if r["label"] == "geode-product-standard")
    maps = {
        name: load(f"build/nav-dataset-execution-20261021/input/{name}.metadata.json") for name in ("geode", "garage")
    }
    manifest = load("build/nav-dataset-execution-20261021/input/manifest.json")
    aggregate = load("build/nav-dataset-execution-20261021/results.json")
    load("build/nav-dataset-execution-20261021/garage-start-probe/result.json")
    source("build/nav-dataset-execution-20261021/garage-start-probe/witness.csv")
    height = load("build/corridor-height-20261021/trajectory/analysis.json")
    source("build/corridor-height-20261021/trajectory/README.md")
    trace = source("build/corridor-height-20261021/trace-analysis/README.md")
    source("build/corridor-height-20261021/trace/README.md")
    genz = load("build/genz-fusion-20261021/v2/comparison.json")
    for literal in ("-0.1794771601", "+0.0039450024", "-0.1755321577"):
        if literal not in trace:
            raise ValueError(f"Trace accounting source changed: {literal}")
    for rel in (
        "src/nav/cpp/planning/global/contract.hpp",
        "src/nav/cpp/planning/global/octoplanner/octoplanner3d_core.hpp",
        "tests/nav/cpp/benchmark_saved_route.cpp",
        "tools/bench/saved_route.py",
        "tools/bench/navigation_paper_note.py",
        "build/nav-dataset-execution-20261021/source-record.json",
        "build/nav-dataset-execution-20261021/build-record.json",
    ):
        source(rel)
    body = ParagraphStyle("Body", fontName="Song", fontSize=10.5, leading=16, wordWrap="CJK", spaceAfter=7)
    small = ParagraphStyle("Small", parent=body, fontSize=9.5, leading=13, spaceAfter=5)
    heading = ParagraphStyle("Heading", fontName="Hei", fontSize=13, leading=19, spaceAfter=10, keepWithNext=True)
    caption = ParagraphStyle(
        "Caption", parent=body, fontSize=10, leading=14, spaceAfter=7, alignment=1, keepWithNext=True
    )
    cell = ParagraphStyle("Cell", fontName="Arial", fontSize=10, leading=13, wordWrap="CJK")
    story = []

    def p(text, compact=False):
        rendered = text.replace("−", '<font name="Arial">−</font>')
        story.append(Paragraph(rendered, small if compact else body))
        TEXT.append(text)

    def h(text):
        story.append(Paragraph(text, heading))
        TEXT.append("## " + text)

    def cell_text(value):
        value = escape(str(value)).replace("\n", "<br/>")
        return re.sub(r"([\u2e80-\u9fff\uff00-\uffef]+)", r'<font name="Song">\1</font>', value)

    def table(number, title, columns, data, widths, note=""):
        TABLES[number] = {"title": title, "columns": columns, "rows": data, "note": note}
        story.append(Paragraph(f"表 {number}　{title}", caption))
        rendered = [[Paragraph(cell_text(value), cell) for value in row] for row in [columns, *data]]
        item = Table(rendered, colWidths=widths, repeatRows=1, hAlign="CENTER")
        item.setStyle(
            TableStyle(
                [
                    ("LINEABOVE", (0, 0), (-1, 0), 0.8, colors.black),
                    ("LINEBELOW", (0, 0), (-1, 0), 0.5, colors.black),
                    ("LINEBELOW", (0, -1), (-1, -1), 0.8, colors.black),
                    ("VALIGN", (0, 0), (-1, -1), "TOP"),
                    ("LEFTPADDING", (0, 0), (-1, -1), 4),
                    ("RIGHTPADDING", (0, 0), (-1, -1), 4),
                    ("TOPPADDING", (0, 0), (-1, -1), 5),
                    ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
                ]
            )
        )
        story.extend([item, Spacer(1, 7)])
        TEXT.extend([f"表 {number} {title}", " | ".join(columns), *[" | ".join(map(str, row)) for row in data]])
        if note:
            p(note, True)

    h("附录 D　导航组件复测与高度误差追查")
    p(
        "原稿第1–27页保留2026年10月21日19:15的证据截点。本附录补入其后已完成的原生导航组件实验、"
        "Corridor高度诊断和GenZ v2否决结果。原稿中“SCAN尚未测试”“Corridor尚无独立诊断”及“v2尚无完整结论”"
        "须结合本附录阅读；这些更新不代表完整Product、物理仿真或实机导航已通过。"
    )
    p("D.1　正式核心的输入与地图转换", True)
    p(
        "正式服务和本实验共用C++ PlannerSession / OctoPlanner3D。调用输入由三部分构成：三维OctoMap文件路径；"
        "MapIdentity及代次；GlobalPlanRequest（起终点XYZ、机体净空、支撑高度、坡度/台阶、搜索和终点容差、可选禁行体）。"
        "输出为XYZ参考折线、成功状态、终点误差及失败原因，随后仍需局部轨迹和执行反馈。"
    )
    data = []
    for name, title in (("geode", "GEODE"), ("garage", "旧Garage部分图")):
        octo = maps[name]["artifacts"]["octomap"]
        stats = octo["stats"]
        data.append(
            [
                title,
                fmt(octo["resolution"], 2),
                f"{stats['valid_endpoints']:,}",
                f"{stats['free_updates']:,}",
                f"{stats['hit_updates']:,}",
            ]
        )
    table(
        "D1",
        "已有保存地图的实际转换记录",
        ["地图", "分辨率/m", "有效射线端点", "free更新", "hit更新"],
        data,
        [105, 74, 110, 110, WIDTH - 399],
        "两图均为saved_rays、native_octomap、full_log_odds编码；更新次数不是独立体素数。"
        "本轮冻结复制.ot，不重新建图。新全程Garage的6982位姿/7GCP结果不能代替此处旧部分地图。",
    )
    p(
        "保存扫描patches、poses与scan_origin先恢复每帧点和物理原点的地图坐标，再以真实射线累积自由与占据证据。"
        "未观测空间保持未知。因此地图已完成转换；maps_rebuilt=false仅表示本轮未重复转换。"
        "这是三维占据与自由证据，不是二维代价图，也没有通过填补未知区域制造通路。"
    )
    p(
        "局部SCAN组件需要碰撞位图。本实验从同一.ot逐0.05 m查询occupied/free/unknown，保留粗叶完整体积，"
        "标准层按生产离散膨胀offset构造；源占据阈值与射线历史仍不同于实时mapd。"
        "初始朝向为路线首末连线的固定合成值，非实测朝向。原请求到吸附起点的接入段未执行。"
    )
    table(
        "D2",
        "本轮证据层级",
        ["层级", "此次覆盖", "仍不能据此认定"],
        [
            ["原生组件", "真实C++全局、SCAN、Follower/Executor", "完整Product/API/最终仲裁通过"],
            ["反馈模型", "vx/vy/wz理想积分；Z固定", "足端、接触、打滑和上下楼可执行"],
            ["取消", "clear后下一tick命令为零", "物理机器人已经停稳"],
        ],
        [88, 203, WIDTH - 291],
    )

    story.append(PageBreak())
    h("D.2　固定请求的全局—局部—反馈结果")
    execution = []
    performance = []
    endpoints = []
    for index, run in enumerate(runs, 1):
        s = run["summary"]
        r = run["route"]
        name = f"R{index} " + run["label"].replace("geode", "G").replace("garage", "C").replace("product", "P").replace(
            "diagnostic", "D"
        ).replace("standard", "std")
        execution.append(
            [
                name,
                r["points"],
                "是" if s["local_ready"] else "否",
                run["active_samples"],
                fmt(s["feedback_planar_distance_m"]),
                "是" if s["feedback_goal_reached"] else "否",
                s["collision_state_count"],
            ]
        )
        performance.append(
            [
                f"R{index}",
                fmt(s["cold_request_ms"] / 1000),
                fmt(s["cached_request_ms"]),
                fmt(s["first_local_plan_ms"]),
                fmt(run["tracking_rmse_m"], 4),
            ]
        )
        measured = aggregate["runs"][run["label"]]
        if measured["feedback"]["tracking_active_samples"] != run["active_samples"]:
            raise ValueError(f"Active-sample summary changed for {run['label']}")
        if run["active_samples"] and not math.isclose(
            measured["feedback"]["active_xy_rmse_m"], run["tracking_rmse_m"], abs_tol=1e-12
        ):
            raise ValueError(f"Tracking metric definition changed for {run['label']}")
        endpoints.append(
            [
                f"R{index}",
                fmt(measured["requested_goal_error"]["xy_m"], 6),
                f"{measured['requested_goal_error']['z_signed_m']:+.6f}",
                fmt(measured["requested_goal_error"]["xyz_m"], 6),
                "<0.000001"
                if measured["route"]["request_to_route_start_m"] < 1e-6
                else fmt(measured["route"]["request_to_route_start_m"], 6),
                "是" if s["component_cancel_zero"] else "否",
            ]
        )
    table(
        "D3",
        "执行结果（G=GEODE，C=Garage，P=正式参数，D=旧诊断参数）",
        ["固定实验", "全局点", "SCAN\nready", "有效跟踪\n样本", "XY距离/m", "理想反馈\n到达", "碰撞tick"],
        execution,
        [127, 44, 48, 61, 76, 48, WIDTH - 404],
    )
    table(
        "D4",
        "组件耗时与有效跟踪误差",
        ["实验", "冷调用/s", "缓存调用/ms", "局部尝试/ms", "跟踪RMSE/m"],
        performance,
        [45, 95, 113, 125, WIDTH - 378],
        "冷/缓存耗时均为一次调用观测，非分布统计；缓存调用仍含请求准备，不能称纯搜索。"
        "局部ready时为首条轨迹等待耗时，失败时为500 tick累计尝试耗时。"
        "跟踪误差只统计tracking_active=1，且仅为XY到局部时间轨迹的距离，不是目标到达误差。"
        "开发机WSL单次计时受其他任务争用，未形成实时性能资格。",
    )
    table(
        "D5",
        "终点反馈与取消命令",
        ["实验", "目标XY差/m", "有符号Z差/m", "目标XYZ差/m", "起点吸附/m", "取消零输出"],
        endpoints,
        [35, 88, 103, 94, 106, WIDTH - 426],
        "取消后下一tick零命令属于组件证据，物理停稳未测。GEODE到达沿用实际容差，仍有约0.10 m高度差。",
    )
    p(
        "旧R1保守fixture额外加入径向及垂直padding；该失败仅作输入构造对照。标准fixture移除额外padding、"
        "沿用产品尺寸和同图同请求后，才能用于当前静态组件结论。此修正没有修改生产规划或mapd算法。",
        True,
    )
    p(
        "Garage正式参数返回35点，但SCAN报rebound_restart_limit，反馈无运动；起点真实占据叶命中局部后圆柱，详见D.2.1。"
        "旧诊断参数返回34点并可跟踪，20 s后XY距请求终点0.046946 m、Z差+0.396655 m，未到达。"
        "该诊断的route.front已比原始请求抬高0.10 m，测试从此处开始；原始起点接入段未执行，且Z始终固定。"
        "GEODE正式参数约5.66 s结束，Garage两次均记录完整20 s反馈窗口。",
        True,
    )
    story.append(PageBreak())
    h("D.2.1　全局点、局部轨迹和理想反馈")
    story.append(figure(standard))
    p(
        "图 D1　GEODE正式参数标准fixture：返回全局点与实际理想XY反馈、每段三维点距。"
        "反馈Z固定；图中不表示三维物理运动。字体Arial 10 pt，按原尺寸嵌入。",
        True,
    )
    p(
        "全局0.10 m的XY格在当前坡度限制下采用0.05 m的Z步长，可产生0.10、0.1118、0.1414或0.15 m等不同段长。"
        "这些点是A*返回的相邻三维格点；控制器的执行周期则是另一维度，不能用航路点间距推导速度控制频率。"
        "图中GEODE路径三维长2.768910 m，但理想XY实际里程约2.36 m，两者统计对象不同。"
        "执行从首个返回点开始，并在到达容差内结束；固定Z模型也没有走完参考折线的升降量。"
    )
    p(
        "全局使用身体中心单圆柱，而局部SCAN按朝向检查前后双圆柱。正式参数中的全局半径0.155 m、"
        "局部半径0.25 m与前后偏移0.19 m属于不同几何契约。Garage只读见证中，8个0.05 m子格均来自同一0.10 m原叶，"
        "中心(3.45,−0.15,−1.25) m，占据概率0.509091，高于实际0.5阈值。该叶Z与机体相交，"
        "但body中心至其XY边界最近距0.175446 m，大于全局半径；同姿态的SCAN后圆心落在该叶XY内。"
    )
    p(
        "全树直接查询的全局机体命中数为0，连续后圆柱命中2个占据叶，因此本静态输入的起点拒绝可以由外形契约差异解释，"
        "并非仅额外padding或离散假象。它仍是保存.ot的只读见证，不能冒充实时mapd实测。"
        "后续应按实际执行包络核对全局路径资格，保留支撑与未知空间约束；本附录未通过放宽半径取得通过。"
    )
    r = standard["route"]
    p(
        f"该次全局{r['points']}点、三维长{r['length_m']:.6f} m，点距min/mean/max为"
        f"{r['spacing_min_m']:.6f}/{r['spacing_mean_m']:.6f}/{r['spacing_max_m']:.6f} m。"
        "它不是固定0.20 m输出；0.20 m是SCAN样条控制点名义间隔，参考点另有0.50 m筛选。"
        "Follower按执行时间连续评估样条并输出速度，机器人不逐点停靠。",
        True,
    )

    story.append(PageBreak())
    h("D.3　Corridor高度退化及已完成的诊断")
    scores = height["alignment"]["scores"]
    a, b = scores["prior"], scores["fresh"]
    data = []
    for name, prior, fresh in [
        ("三维RMSE", a["rmse_m"], b["rmse_m"]),
        ("X RMSE", a["axis_rmse_m"][0], b["axis_rmse_m"][0]),
        ("Y RMSE", a["axis_rmse_m"][1], b["axis_rmse_m"][1]),
        ("Z RMSE", a["height_rmse_m"], b["height_rmse_m"]),
        ("Z最大绝对误差", a["height_max_abs_m"], b["height_max_abs_m"]),
    ]:
        data.append([name, fmt(prior, 6), fmt(fresh, 6), f"{(fresh / prior - 1) * 100:+.2f}%"])
    table(
        "D6",
        "相同8038个时刻的条件参考轨迹误差（m）",
        ["指标", "时间修复基线", "第一阶段概率平面", "变化"],
        data,
        [125, 124, 149, WIDTH - 398],
    )
    p(
        "每次运行对同一组时间样本各做一次整段SE(3)刚体拟合，scale=1。定义为e=R·p_est+t−p_ref，"
        "Z-RMSE=√mean(e_z²)。SuperLoc参考体点和独立精度尚未充分认证，但同样本前后退化成立。"
        "该值是轨迹高度误差，不是地面表面、点云逐点误差或机器人的跟踪误差。"
        "旧筛选只检查总RMSE和覆盖，XY改善掩盖了Z退化；不能据旧通过结论认定多层导航高度精度达标。"
    )
    table(
        "D7",
        "130–240 s原始起点前缀的更新分解（第一阶段减基线）",
        ["项", "同一固定高度轴增量差/m"],
        [
            ["预测/传播之间", "−0.1794771601"],
            ["LiDAR后验减先验位置校正", "+0.0039450024"],
            ["累计净变化", "−0.1755321577"],
        ],
        [282, WIDTH - 282],
        "各2390位姿的诊断前缀与其历史全程前缀逐字节相同；窗口含1091次更新。这是状态记账分解，不是相互独立的因果消融。",
    )
    p(
        "原始起终点高度增量为0.422753→0.946818 m；在同一个已有旋转下，末端增量差仍为+0.502651 m，"
        "因此不能全部解释为评估坐标变化。首次持续偏离集中在130–240 s，660–720 s继续放大。"
        "全段拟合旋转相差0.547°，它会向早期静止段分配残差；初始约31 cm残差差不能说成开机即漂移31 cm。"
    )
    p(
        "传播继承先前LiDAR更新留下的姿态、速度、bias及协方差，地图又继承历史估计。进入窗口时姿态已差0.165°，"
        "至窗口末约0.705°；记录的重力状态相同。真实平面关联几何有差异，但固定先验的垂直首步影响符号混合，"
        "尚未证明它持续向一个方向拉偏。两层平面合并的小样本复现也尚未证明是该录制根因。"
    )
    p(
        "当前决定：高度退化属实，唯一根因未隔离。下一诊断追查130 s入口之前的姿态/速度/bias如何经逐帧LiDAR反馈和传播形成；"
        "本附录没有据此推广降权LiDAR、恢复5NN、修改噪声或高度限幅。"
    )

    story.append(PageBreak())
    h("D.4　新增候选结果与本文覆盖边界")
    data = []
    for candidate, evaluation in genz["evaluations"].items():
        c = evaluation["cases"]["geode"]
        for hypothesis, comparison in c["scores"].items():
            before = comparison["common"]["prior"]["rmse_m"]
            after = comparison["common"]["fresh"]["rmse_m"]
            data.append(
                [
                    candidate,
                    "IMU" if hypothesis.startswith("imu") else "LiDAR",
                    c["common_reference_samples"],
                    fmt(before, 6),
                    fmt(after, 6),
                    f"{(after / before - 1) * 100:+.2f}%",
                ]
            )
    table(
        "D8",
        "GenZ v2完整已测GEODE结果：同样本位置RMSE（m）",
        ["候选", "参考假设", "N", "第一阶段", "v2候选", "变化"],
        data,
        [68, 77, 49, 105, 105, WIDTH - 404],
        "19:40完成，晚于原稿截点。hybrid与all两个候选在两种参考体假设下均回归，未替换默认实现。"
        "PALoc条件参考非独立动捕；两个假设均报告，不能择优拼接。",
    )
    table(
        "D9",
        "当前结果覆盖及仍缺证据",
        ["材料组", "本次放入位置", "资格/缺口"],
        [
            ["完整建图/误差/规划历史", "原稿1–27页", "保留原19:15结果和历史失败"],
            ["保存地图及输入", "D1–D2；sources元数据", "本轮地图不重建；未知不填free"],
            ["全局→SCAN→Executor", "D3–D5、图D1", "静态地图+理想XY；无三维接触物理"],
            ["Corridor高度退化", "D6–D7、诊断原记录", "已定位时间窗口；唯一根因未定"],
            ["GenZ v2", "D8", "两候选均否决；非正式改进成绩"],
            ["新全程Garage导航地图", "未纳入", "等待独立地图与固定路线交付"],
            ["完整Product/仿真/实机", "未验收", "sensor、最终仲裁、driver、足端尚缺"],
        ],
        [124, 163, WIDTH - 287],
    )
    p(
        "本次可认定：共享原生三维全局规划核心能在部分固定真实地图请求上返回参考路径，标准静态碰撞输入下可继续检验局部轨迹与理想平面反馈。"
        "不能由单条短路线认定所有场景导航可行，尤其楼梯执行、长路线连通性、地图切换和真实停止仍需各自证据。"
    )
    p(
        "配套目录保留原始JSON/CSV副本、全部表格、图D1的PDF/PNG、附录正文和生成器。具体运行输入、固定源码、二进制与构建/失败日志"
        "仍由nav-dataset-execution-20261021实验目录维护。未把日志中的默认零误差、进程退出0或目录名pass当成导航成功。"
    )
    p(
        "核心索引：tools/bench/saved_route.py；tests/nav/cpp/benchmark_saved_route.cpp；"
        "build/corridor-height-20261021/{trajectory,trace,trace-analysis}；build/genz-fusion-20261021/v2/comparison.json。",
        True,
    )

    supplement = WORK / "appendix-D.pdf"

    def header(canvas, doc):
        canvas.saveState()
        canvas.setFont("Song", 8.5)
        canvas.drawString(MARGIN, PAGE_H - 27, "LingTu：追加证据（原稿证据截点19:15）")
        canvas.setFont("Arial", 9)
        canvas.drawCentredString(PAGE_W / 2, 26, f"D{doc.page}")
        canvas.restoreState()

    doc = BaseDocTemplate(
        str(supplement),
        pagesize=A4,
        leftMargin=MARGIN,
        rightMargin=MARGIN,
        topMargin=43,
        bottomMargin=43,
        title="LingTu：导航复测与高度诊断补充",
        author="LingTu project",
    )
    doc.addPageTemplates(
        PageTemplate(
            id="appendix",
            frames=[
                Frame(MARGIN, 43, WIDTH, PAGE_H - 86, leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)
            ],
            onPage=header,
        )
    )
    doc.build(story)
    writer = PdfWriter()
    for filename in (BASE_PDF, supplement):
        for page in PdfReader(filename).pages:
            writer.add_page(page)
    writer.add_metadata({"/Title": "LingTu原生三维建图与规划：扩充稿及导航组件复测附录", "/Author": "LingTu project"})
    OUT.parent.mkdir(exist_ok=True, parents=True)
    with OUT.open("wb") as stream:
        writer.write(stream)
    (WORK / "appendix-D.md").write_text("\n\n".join(TEXT), encoding="utf-8")
    (WORK / "tables.json").write_text(json.dumps(TABLES, ensure_ascii=False, indent=2), encoding="utf-8")
    for name, value in TABLES.items():
        with (WORK / f"table-{name}.csv").open("w", encoding="utf-8-sig", newline="") as stream:
            table_writer = csv.writer(stream)
            table_writer.writerow(value["columns"])
            table_writer.writerows(value["rows"])
    stats = [
        {key: value for key, value in r.items() if key not in ("feedback", "route")} | {"route": r["route"]}
        for r in runs
    ]
    (WORK / "run-metrics.json").write_text(json.dumps(stats, ensure_ascii=False, indent=2), encoding="utf-8")
    metadata = {
        "generated_at": datetime.now().isoformat(),
        "base": str(BASE_PDF),
        "base_pages": len(PdfReader(BASE_PDF).pages),
        "base_cutoff": "2026-10-21T19:15:00+08:00",
        "appendix_pages": len(PdfReader(supplement).pages),
        "total_pages": len(writer.pages),
        "labels": labels,
        "sources": SOURCES,
        "font": "Figure: Arial 10 pt. Tables: Arial 10 pt with Song fallback for CJK.",
        "runs_physical_simulation": False,
        "new_full_garage_map_included": False,
        "maps_rebuilt": manifest["maps_rebuilt"],
        "output": str(OUT),
    }
    (WORK / "manifest.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    (WORK / "README.md").write_text(
        "# 导航复测与高度诊断附录\n\n原27页保留19:15截点；新PDF追加附录D，以D1等独立页码记录后续结果。\n\n"
        f"PDF：`{OUT.as_posix()}`\n\n运行：`{', '.join(labels)}`\n\n"
        "`appendix-D.pdf/md`、`tables.json`和`table-*.csv`为附录及表格；`figures/`为Arial 10pt原尺寸图；"
        "`run-metrics.json`保留计算数值；`sources/`为本次精确输入副本。原实验大地图/二进制不在此重复。\n\n"
        "本附录不把理想XY反馈称为物理仿真，不包含新全程Garage地图，也不证明实机、足端或完整Product。\n",
        encoding="utf-8",
    )
    print(json.dumps(metadata, ensure_ascii=False))


def render():
    import pypdfium2 as pdfium
    from PIL import Image, ImageDraw

    pdf = pdfium.PdfDocument(str(WORK / "appendix-D.pdf"))
    images = []
    for index in range(len(pdf)):
        page = pdf[index]
        image = page.render(scale=1.6).to_pil().convert("RGB")
        image.save(WORK / f"appendix-D{index + 1}.png")
        images.append(image)
    sheet = Image.new("RGB", (len(images) * 480, 700), "#e6e8e9")
    draw = ImageDraw.Draw(sheet)
    for index, image in enumerate(images):
        thumb = image.copy()
        thumb.thumbnail((465, 660))
        sheet.paste(thumb, (index * 480 + 7, 25))
        draw.text((index * 480 + 7, 7), f"D{index + 1}", fill="black")
    sheet.save(WORK / "appendix-preview.png")
    figure_pdf = pdfium.PdfDocument(str(WORK / "figures/route-feedback-spacing.pdf"))
    figure_pdf[0].render(scale=300 / 72).to_pil().save(WORK / "figures/route-feedback-spacing.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", nargs="+", required=True)
    args = parser.parse_args()
    build(args.runs)
    render()
