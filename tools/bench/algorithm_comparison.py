"""Create a four-page comparison from frozen results; no sensor replay.

Run with the installed Codex Python (ReportLab, pypdf, pypdfium2, Pillow).
"""

from __future__ import annotations

import csv
import json
import re
from datetime import datetime
from pathlib import Path
from xml.sax.saxutils import escape

from pypdf import PdfReader
from reportlab.graphics import renderPDF
from reportlab.graphics.shapes import Drawing, Line, Rect, String
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
WORK = ROOT / "build/algorithm-comparison-20261021"
OUT = ROOT / "output/pdf/lingtu-algorithm-comparison-20261021.pdf"
WIDTH, HEIGHT = A4
MARGIN = 46
CW = WIDTH - 2 * MARGIN
SOURCES = {}
TABLES = {}
TEXT = []
FIGURE_TEXT = []


def load(relative):
    path = ROOT / relative
    raw = path.read_text(encoding="utf-8-sig")
    target = WORK / "sources" / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(raw, encoding="utf-8")
    SOURCES[relative] = {"path": str(path), "bytes": path.stat().st_size}
    return json.loads(raw)


def collect():
    official = load("build/genz-assessment-20261021/evaluation.json")
    slip = load("build/genz-assessment-20261021/slippage-evaluation.json")
    probability = load("build/prob-plane-20261021/isotropic/evaluation.json")
    nav = load("build/nav-dataset-execution-20261021/results.json")
    witness = load("build/nav-dataset-execution-20261021/garage-start-probe/result.json")
    old = load("build/go2-map-save-v6-20260930/multipoint26/report.json")
    upstream = load("build/go2-map-save-v6-20260930/upstream26/report.json")
    pct = load("build/go2-map-save-v6-20260930/pct/experiment.json")
    pct_search = load("build/go2-map-save-v6-20260930/pct/search-report.json")
    methods = [
        ("A", "固定原版 Elevator-LIO", "official_elevator_lio", "external"),
        ("B", "LingTu 时间一致性修复", "ltlio_time_fix", "internal_baseline"),
        ("C", "LingTu 概率平面阶段", None, "current_frozen_candidate"),
        ("D", "官方 GenZ-LIO 完整配置", "genz_author", "external"),
        ("E", "官方 GenZ 关闭混合模块", "geode-plane-only", "external_ablation"),
        ("F", "官方 GenZ 固定0.25 m采样", "geode-fixed-sampling", "external_ablation"),
    ]
    geode = []
    for ident, name, key, category in methods:
        scores = []
        for hypothesis, pkey in (
            ("imu_reference", "imu_frame_assumption"),
            ("lidar_reference", "lidar_frame_assumption"),
        ):
            value = (
                probability["cases"]["geode"]["scores"][pkey]["common"]["fresh"]
                if key is None
                else official["scores"][hypothesis][key]["full_run_shared"]
            )
            assert value["samples"] == 3429 and value["alignment_scale"] == 1
            scores.append(value["rmse_m"])
        geode.append(
            {
                "id": ident,
                "method": name,
                "category": category,
                "samples": 3429,
                "imu_rmse_m": scores[0],
                "lidar_rmse_m": scores[1],
            }
        )
    ps = probability["cases"]["slippage"]["scores"]["imu_frame"]["common"]
    slippage = [
        ("旧 LingTu（时间修复前）", slip["scores"]["ltlio"]),
        ("LingTu 时间一致性修复", ps["prior"]),
        ("LingTu 概率平面阶段", ps["fresh"]),
        ("官方 GenZ，GEODE参数控制", slip["scores"]["genz"]),
    ]
    assert all(score["samples"] == 905 for _, score in slippage)
    corridor = probability["cases"]["corridor"]["scores"]["imu_frame"]["common"]
    assert all(score["samples"] == 8038 for score in corridor.values())
    frozen_cost = probability["cases"]["corridor"]["runs"]["fresh"]["summary"]
    assert len(old["cases"]) == 30
    raw = [case for case in upstream["cases"] if case["profile"] == "raw"]
    assert len(raw) == 30
    old_count = sum(bool(case["result"].get("path")) for case in old["cases"])
    old_valid = sum(bool(case.get("verification", {}).get("valid")) for case in old["cases"])
    pct_cases = pct_search["cases"]
    if isinstance(pct_cases, dict):
        pct_cases = list(pct_cases.values())
    historical = {
        "requests": 30,
        "lingtu_routes": old_count,
        "lingtu_valid": old_valid,
        "upstream_routes": sum(bool(case["ok"]) for case in raw),
        "pct_routes": sum(bool(case["ok"]) for case in pct_cases),
        "pct_commit": pct["commit"],
        "upstream_commit": upstream["upstream_sha"],
    }
    return geode, slippage, corridor, frozen_cost, nav, witness, historical


def figure(geode):
    chart = Drawing(CW, 246)
    blue, grey = colors.HexColor("#316b91"), colors.HexColor("#a0aeb9")
    left, plot_width, top = 133, CW - 180, 208

    def text(x, y, value, anchor="start"):
        FIGURE_TEXT.append({"text": value, "font": "Arial", "size_pt": 10})
        chart.add(String(x, y, value, fontName="Arial", fontSize=10, textAnchor=anchor))

    chart.add(Rect(134, 230, 13, 6, fillColor=blue, strokeColor=None))
    text(152, 228, "IMU reference assumption")
    chart.add(Rect(322, 230, 13, 6, fillColor=grey, strokeColor=None))
    text(340, 228, "LiDAR assumption")
    maximum = 1.2
    for tick in range(7):
        value = tick * 0.2
        x = left + plot_width * value / maximum
        chart.add(Line(x, 37, x, 220, strokeColor=colors.HexColor("#e2e5e7"), strokeWidth=0.5))
        text(x, 22, f"{value:.1f}", "middle")
    text(left + plot_width / 2, 4, "Position RMSE (m)", "middle")
    labels = [
        "A  Elevator-LIO",
        "B  LingTu time fix",
        "C  LingTu plane",
        "D  GenZ full",
        "E  GenZ plane only",
        "F  GenZ fixed 0.25",
    ]
    for index, (row, label) in enumerate(zip(geode, labels, strict=True)):
        y = top - index * 29
        text(0, y - 1, label)
        for value, offset, color in ((row["imu_rmse_m"], 5, blue), (row["lidar_rmse_m"], -7, grey)):
            length = plot_width * value / maximum
            chart.add(Rect(left, y + offset, length, 6, fillColor=color, strokeColor=None))
            text(left + length + 4, y + offset - 1, f"{value:.4f}")
    directory = WORK / "figures"
    directory.mkdir(exist_ok=True)
    renderPDF.drawToFile(chart, str(directory / "geode-algorithms.pdf"))
    return chart


def build():
    WORK.mkdir(parents=True, exist_ok=True)
    for name, filename in (("Song", "simsun.ttc"), ("Hei", "msyhbd.ttc"), ("Arial", "arial.ttf")):
        pdfmetrics.registerFont(TTFont(name, str(Path("C:/Windows/Fonts") / filename), subfontIndex=0))
    pdfmetrics.registerFontFamily("Song", normal="Song", bold="Hei")
    geode, slippage, corridor, cost, nav, witness, historical = collect()
    body = ParagraphStyle("Body", fontName="Song", fontSize=10.5, leading=16, wordWrap="CJK", spaceAfter=7)
    small = ParagraphStyle("Small", parent=body, fontSize=9.5, leading=13, spaceAfter=6)
    heading = ParagraphStyle("Heading", fontName="Hei", fontSize=13, leading=19, spaceAfter=10)
    cell = ParagraphStyle("Cell", fontName="Arial", fontSize=10, leading=13, wordWrap="CJK")
    caption = ParagraphStyle("Caption", parent=body, alignment=1, fontSize=10, leading=14, keepWithNext=True)
    story = []

    def p(value, compact=False):
        TEXT.append(value)
        story.append(Paragraph(value.replace("−", '<font name="Arial">−</font>'), small if compact else body))

    def h(value):
        TEXT.append("## " + value)
        story.append(Paragraph(value, heading))

    def table(number, title, columns, rows, widths, note=""):
        TABLES[number] = {"title": title, "columns": columns, "rows": rows, "note": note}
        story.append(Paragraph(f"表 {number}　{title}", caption))

        def item(value):
            value = escape(str(value)).replace("\n", "<br/>")
            value = re.sub(r"([\u2e80-\u9fff\uff00-\uffef]+)", r'<font name="Song">\1</font>', value)
            return Paragraph(value, cell)

        grid = Table([[item(v) for v in row] for row in [columns, *rows]], colWidths=widths, repeatRows=1)
        grid.setStyle(
            TableStyle(
                [
                    ("LINEABOVE", (0, 0), (-1, 0), 0.8, colors.black),
                    ("LINEBELOW", (0, 0), (-1, 0), 0.5, colors.black),
                    ("LINEBELOW", (0, -1), (-1, -1), 0.8, colors.black),
                    ("VALIGN", (0, 0), (-1, -1), "TOP"),
                    ("LEFTPADDING", (0, 0), (-1, -1), 4),
                    ("RIGHTPADDING", (0, 0), (-1, -1), 4),
                    ("TOPPADDING", (0, 0), (-1, -1), 4),
                    ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
                ]
            )
        )
        story.extend([grid, Spacer(1, 6)])
        TEXT.extend([f"表 {number} {title}", " | ".join(columns), *[" | ".join(map(str, row)) for row in rows]])
        if note:
            p(note, True)

    h("LingTu：算法对照与当前瓶颈")
    p(
        "2026年10月21日。基于已冻结实跑结果整理，同步展示外部算法、内部版本和失败证据。"
        "当前短路线已有全局到局部组件反馈；地图高度、长路线连通性与执行合同仍有缺口，尚未形成完整三维物理导航验收。"
    )
    table(
        "1",
        "GEODE Stairs Alpha：相同3429个参考时刻",
        ["方法", "IMU假设\nRMSE/m", "LiDAR假设\nRMSE/m"],
        [[r["id"] + "　" + r["method"], f"{r['imu_rmse_m']:.6f}", f"{r['lidar_rmse_m']:.6f}"] for r in geode],
        [CW - 180, 90, 90],
    )
    story.append(figure(geode))
    p("图1　完整覆盖共同样本上的位置RMSE（m，越低越好）。图中文字Arial 10 pt，原尺寸矢量嵌入。", True)
    current, genz = geode[2], geode[3]
    p(
        f"当前冻结概率平面阶段比官方GenZ完整配置高{(current['imu_rmse_m'] / genz['imu_rmse_m'] - 1) * 100:.2f}% / "
        f"{(current['lidar_rmse_m'] / genz['lidar_rmse_m'] - 1) * 100:.2f}%（IMU/LiDAR假设），差距仍存在；关闭自适应采样的单序列消融反而更低，不能认定每个模块都必然有益。"
    )
    p(
        "六方法使用同一GEODE载体、逐位相同参考时间mask、各自SE(3)刚体拟合、scale=1、无时间偏移拟合。"
        "PALoc来自先验扫描地图，非独立动捕；参考体点约定未认证，两假设均保留。GenZ保留作者完整GEODE配置，"
        "与LingTu的滤点、最小距离、噪声和初始化并非完全相同，因此是整套实现/配置对照。E还改变地图表示，非仅关闭残差权重。",
        True,
    )

    story.append(PageBreak())
    h("2　普通场景改善，高度与运行成本仍有回归")
    table(
        "2",
        "Slippage：相同905个光学真值时刻",
        ["方法", "位置RMSE/m", "高度RMSE/m"],
        [[name, f"{score['rmse_m']:.6f}", f"{score['height_rmse_m']:.6f}"] for name, score in slippage],
        [CW - 182, 91, 91],
        "第一行是旧LingTu，不能误写为官方Elevator-LIO。GenZ控制沿用GEODE估计/噪声，仅修正已知Livox输入、外参和最小距离；不是作者Slippage预设或独立噪声标定。",
    )
    p(
        "此控制下当前LingTu误差低于GenZ，说明楼梯上的领先不能推成所有场景领先；替换整套系统仍需代表场景与匹配标定。"
        "Slippage近直线运动对绕行进方向的旋转对齐较弱，保留原评价限制。"
    )
    before, after = corridor["prior"], corridor["fresh"]
    table(
        "3",
        "Corridor02：同8038时刻的分轴退化",
        ["指标", "时间修复基线", "概率平面阶段", "相对变化"],
        [
            [label, f"{before[key]:.6f}", f"{after[key]:.6f}", f"{(after[key] / before[key] - 1) * 100:+.2f}%"]
            for label, key in (
                ("位置RMSE/m", "rmse_m"),
                ("高度RMSE/m", "height_rmse_m"),
                ("最大高度误差/m", "height_max_abs_m"),
            )
        ],
        [CW - 279, 93, 93, 93],
    )
    p(
        "XY改善推动总RMSE降低，却掩盖高度恶化。旧筛选只要求总误差和覆盖不退化，未单独约束高度。"
        "同参考轴下末端高度增量差仍约0.503 m，排除了全部仅由对齐造成的解释；130–240 s已出现持续分化，唯一根因仍未隔离。"
    )
    p(
        f"同一冻结概率平面回放处理{cost['sensor_duration_s']:.2f} s输入，开发机共享主机耗时{cost['elapsed_s']:.2f} s。"
        "这是一次离线程序成本，尚未证明目标板实时；官方GenZ在Corridor的同机同条件完整回放未测，不能引用论文数值填成本机对照。"
    )
    table(
        "4",
        "目前外部基线覆盖",
        ["对象", "本机实跑", "缺少的证据"],
        [
            ["原版Elevator-LIO / 官方GenZ", "GEODE完整共同样本", "当前完整Garage同版地图与导航链"],
            ["官方GenZ控制", "Slippage同905时刻", "该传感器独立噪声标定与作者预设"],
            ["官方GenZ / FAST-LIO2 / Point-LIO", "Corridor同条件：未测", "本机完整轨迹、覆盖、Z与运行成本"],
        ],
        [166, 142, CW - 308],
    )

    story.append(PageBreak())
    h("3　规划结果：外部历史对照与当前资格")
    table(
        "5",
        "903历史30个固定请求（2026-09-30）",
        ["实现", "返回路线", "独立完整扫掠/运动验收", "输入/条件"],
        [
            [
                "LingTu OctoPlanner3D",
                f"{historical['lingtu_routes']}/30",
                f"{historical['lingtu_valid']}/30",
                "5 cm派生OctoMap",
            ],
            ["固定原版OctoPlanner3D", f"{historical['upstream_routes']}/30", "未测", "同派生图，原版约束"],
            ["PCT官方A* + GPMP", f"{historical['pct_routes']}/30", "未测", "CPU tomography适配"],
        ],
        [134, 69, 151, CW - 354],
        "历史输入是903 PCD及建模自由空间，非本轮GEODE/Garage保存射线地图。机器人包络、支撑、未知空间、分层与搜索预算不同；返回路线数量不能作为同几何条件的算法胜负。",
    )
    p(
        "原版OctoPlanner采用半径0.20 m、严格直接支撑、snap8/250k；LingTu采用0.155 m、邻域支撑与显式机体净空、snap24/500k。"
        "PCT为0.10 m网格/0.50 m切片等配置，CPU转换为适配实现；两条优化轨迹只有采样代价检查，未验证完整机体扫掠，起点回溯还省略约15 cm连接。",
        True,
    )
    selected = [
        ("GEODE正式", "geode-product-standard"),
        ("GEODE旧诊断", "geode-diagnostic-standard"),
        ("Garage正式", "garage-product-standard"),
        ("Garage旧诊断", "garage-diagnostic-standard"),
    ]
    table(
        "6",
        "当前保存地图上的原生组件实验",
        ["固定案例", "全局点", "SCAN", "理想XY/m", "组件到达", "目标Z差/m"],
        [
            [
                label,
                str(nav["runs"][key]["summary"]["global_point_count"]),
                "ready" if nav["runs"][key]["summary"]["local_ready"] else "blocked",
                f"{nav['runs'][key]['summary']['feedback_planar_distance_m']:.3f}",
                "是" if nav["runs"][key]["summary"]["feedback_goal_reached"] else "否",
                f"{nav['runs'][key]['requested_goal_error']['z_signed_m']:+.3f}",
            ]
            for label, key in selected
        ],
        [117, 52, 69, 84, 77, CW - 399],
    )
    p(
        "反馈只积分vx/vy/wz，高度固定。Garage旧诊断直接从抬高0.10 m的吸附起点初始化，未执行原起点接入；"
        "GEODE的组件到达也保留约0.10 m高度差，不能当成三维物理或楼梯执行通过。各次取消后组件命令为零，物理停稳未测。"
    )
    p(
        f"Garage正式起点查询显示：{witness['input_source_subcells']}个0.05 m命中子格来自{witness['unique_source_leaves']}个0.10 m原叶。"
        "该叶在全局中心单圆柱外、在SCAN后圆柱内；实际树阈值0.5，原叶概率0.509091。全局/局部包络不一致能解释本固定输入下的起点拒绝，不能删格或改阈值求通过。"
    )
    p(
        "当前GEODE/Garage缺少独立外部规划器在同一保存地图、同一起终点和同一机体/支撑/未知策略下的结果。"
        "历史903对照展示已有工作，但不能填补这一空缺。官方PCD是数据来源，本身也不代表运行了另一个算法。"
    )

    story.append(PageBreak())
    h("4　当前卡点与下一次结果应如何判断")
    table(
        "7",
        "已经定位的瓶颈与仍未隔离的原因",
        ["环节", "实测事实", "下一项具体动作"],
        [
            [
                "轨迹高度与成本",
                "Corridor高度RMSE +41.30%；传播/历史状态分化已定位窗口，唯一根因未定；离线成本较高。",
                "追查130 s之前姿态/速度/bias与LiDAR反馈，单项修改后固定样本复测Z、覆盖、失锁与成本。",
            ],
            [
                "保存地图连通性",
                "三条原长路线仍empty_path。射线修正恢复地面226格；更密GEODE占据150→356，仍未连通。",
                "按固定路线找第一处支撑/自由空间断口，修保存观测或查询，不填假地面。",
            ],
            [
                "全局→局部执行",
                "全局单圆柱与SCAN双圆柱不同；Garage正式起点已见冲突；吸附点连接未执行。",
                "统一可执行机体包络与原始到吸附起点合同，再以同图同请求复测。",
            ],
            [
                "版本与完整闭环",
                "导航用旧Garage部分no_loop图；新约700 s/6982位姿/7 GCP前端成绩尚无同版完整导航地图与执行证据。",
                "同版前端→保存地图→规划→反馈连续留证，再用同源三维物理反馈验证高度、支撑、到达和停止。",
            ],
        ],
        [91, 205, CW - 296],
    )
    p(
        "当前效果不稳来自这些问题叠加。一次短路线可运行说明组件可接通；它没有同时证明地图精度、长路连通性和完整三维运动。"
        "各模块的较好单项成绩也不能跨版本拼成一次已完成的端到端结果。"
    )
    table(
        "8",
        "以后每批结果的固定展示内容",
        ["必须同时展示", "本轮落实方式"],
        [
            ["LingTu与外部算法", "相同参考时刻并排；无对应实跑写未测；历史/作者论文值单列。"],
            ["当前、基线和失败", "冻结版本命名；拒绝候选不冒充作者完整算法；覆盖/失锁一并报告。"],
            ["几何、轨迹和执行", "地图Z/位置误差、路线/扫掠、局部ready、真实反馈到达分别判断。"],
            ["输入与资源边界", "记录数据、标定、机体、未知策略、平台、成本测量范围，避免不等条件速度排名。"],
        ],
        [142, CW - 142],
    )
    p(
        "本报告仅整理、计算和绘制已有冻结证据，没有新传感回放或算法修正。全部表格、数值JSON、原尺寸Arial 10 pt图和精确来源副本随报告提供。"
        "完整程序与旧日志仍归原实验目录维护；此材料不自动授权部署、实机运动或上传。",
        True,
    )
    p(
        "主要来源：genz-assessment-20261021、prob-plane-20261021/isotropic、nav-dataset-execution-20261021、"
        "go2-map-save-v6-20260930/{multipoint26,upstream26,pct}。各条精确相对路径见manifest.json及sources/。",
        True,
    )

    def header(canvas, doc):
        canvas.saveState()
        canvas.setFont("Song", 8.5)
        if doc.page > 1:
            canvas.drawString(MARGIN, HEIGHT - 25, "LingTu：算法对照与瓶颈（冻结实测证据）")
        canvas.setFont("Arial", 9)
        canvas.drawCentredString(WIDTH / 2, 25, str(doc.page))
        canvas.restoreState()

    OUT.parent.mkdir(exist_ok=True, parents=True)
    document = BaseDocTemplate(str(OUT), pagesize=A4, title="LingTu算法对照与当前瓶颈", author="LingTu project")
    document.addPageTemplates(
        PageTemplate(
            id="comparison",
            frames=[Frame(MARGIN, 43, CW, HEIGHT - 84, leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)],
            onPage=header,
        )
    )
    document.build(story)
    pages = len(PdfReader(OUT).pages)
    if pages != 4:
        raise ValueError(f"Expected four readable pages, got {pages}")
    metrics = {
        "geode": geode,
        "slippage": [{"method": name, **score} for name, score in slippage],
        "corridor": corridor,
        "historical_planning": historical,
        "scope": "same-sample algorithm/configuration comparison; not a universal ranking",
    }
    (WORK / "results.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    (WORK / "tables.json").write_text(json.dumps(TABLES, ensure_ascii=False, indent=2), encoding="utf-8")
    for name, value in TABLES.items():
        with (WORK / f"table-{name}.csv").open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(value["columns"])
            writer.writerows(value["rows"])
    (WORK / "report.md").write_text("\n\n".join(TEXT), encoding="utf-8")
    manifest = {
        "generated_at": datetime.now().isoformat(),
        "pages": pages,
        "sources": SOURCES,
        "output": str(OUT),
        "figure_text": FIGURE_TEXT,
        "sensor_replay": False,
        "production_algorithm_changed": False,
        "reference_check": "Read-only independent trajectory/scorer audit confirmed identical GEODE3429 and Slippage905 masks; result fields checked during generation.",
    }
    (WORK / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    (WORK / "README.md").write_text(
        "# 算法对照与当前瓶颈\n\n2026-10-21。4页独立报告，保留原32页论文。\n\n"
        f"- PDF：`{OUT.as_posix()}`\n- 正文：`report.md`；数值：`results.json`。\n"
        "- `table-*.csv`/`tables.json`：8张表，保留失败和未测。\n"
        "- `figures/geode-algorithms.pdf/png`：正式矢量/300dpi图，Arial 10 pt。\n"
        "- `sources/`、`manifest.json`：精确输入副本及条件；`verification.json`为文档检查记录。\n\n"
        "本轮仅整理/重算既有冻结证据；没有新的传感回放或算法修正。GEODE六方法同3429参考mask，"
        "Slippage四版本同905光学参考mask。GenZ与LingTu各用声明的配置，不能把整实现对照归给单个模块。"
        "903历史规划约束不同，不能与当前保存射线地图混排名。新完整Garage前端成绩未与旧部分地图导航拼接。\n\n"
        "生成程序：`D:/inovxio/brain/lingtu/tools/bench/algorithm_comparison.py`，使用已安装Codex Python的ReportLab/pypdf/pypdfium2/Pillow。"
        "源码中的输入路径指向当前工作区，归档不是可重定位软件发行包。\n",
        encoding="utf-8",
    )
    return manifest


def render_and_check(manifest):
    import pypdfium2 as pdfium
    from PIL import Image, ImageDraw

    pdf = pdfium.PdfDocument(str(OUT))
    canvas = Image.new("RGB", (1920, 700), "#e5e7e9")
    draw = ImageDraw.Draw(canvas)
    for index in range(len(pdf)):
        image = pdf[index].render(scale=1.6).to_pil().convert("RGB")
        image.save(WORK / f"page-{index + 1}.png")
        image.thumbnail((465, 660))
        canvas.paste(image, (index * 480 + 7, 25))
        draw.text((index * 480 + 7, 7), str(index + 1), fill="black")
    canvas.save(WORK / "preview.png")
    graph = pdfium.PdfDocument(str(WORK / "figures/geode-algorithms.pdf"))
    graph[0].render(scale=300 / 72).to_pil().save(WORK / "figures/geode-algorithms.png")
    extracted = "\n".join(page.extract_text() for page in PdfReader(OUT).pages)
    assert "\x00" not in extracted
    assert all(item["font"] == "Arial" and item["size_pt"] == 10 for item in FIGURE_TEXT)
    (WORK / "body-text.txt").write_text(extracted, encoding="utf-8")
    print(
        json.dumps(
            {"pdf": str(OUT), "pages": manifest["pages"], "figure_text_items": len(FIGURE_TEXT)}, ensure_ascii=False
        )
    )


if __name__ == "__main__":
    render_and_check(build())
