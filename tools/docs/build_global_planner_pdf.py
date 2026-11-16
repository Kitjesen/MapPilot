from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Flowable, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs/assets/planning/global-planner-mls-roadmap-20261115.pdf"
pdfmetrics.registerFont(TTFont("SimHei", "C:/Windows/Fonts/simhei.ttf"))
INK = colors.HexColor("#17212B")
MUTED = colors.HexColor("#53606D")
BLUE = colors.HexColor("#185ADB")
PALE_BLUE = colors.HexColor("#EAF1FF")
PALE_GREEN = colors.HexColor("#EAF7F0")
LINE = colors.HexColor("#D8E0E8")


styles = getSampleStyleSheet()
styles.add(ParagraphStyle(name="title_cn", fontName="SimHei", fontSize=24, leading=34, textColor=INK, alignment=TA_CENTER, spaceAfter=8 * mm))
styles.add(ParagraphStyle(name="subtitle_cn", fontName="SimHei", fontSize=12, leading=20, textColor=MUTED, alignment=TA_CENTER, spaceAfter=10 * mm))
styles.add(ParagraphStyle(name="h1_cn", fontName="SimHei", fontSize=16, leading=23, textColor=INK, spaceBefore=4 * mm, spaceAfter=3 * mm))
styles.add(ParagraphStyle(name="h2_cn", fontName="SimHei", fontSize=11.5, leading=18, textColor=BLUE, spaceBefore=3 * mm, spaceAfter=2 * mm))
styles.add(ParagraphStyle(name="body_cn", fontName="SimHei", fontSize=9.5, leading=16, textColor=INK, spaceAfter=2.6 * mm))
styles.add(ParagraphStyle(name="small_cn", fontName="SimHei", fontSize=8, leading=13, textColor=MUTED, spaceAfter=1.8 * mm))
styles.add(ParagraphStyle(name="quote_cn", fontName="SimHei", fontSize=10, leading=17, textColor=INK, leftIndent=6 * mm, rightIndent=4 * mm, borderColor=BLUE, borderWidth=1, borderPadding=4 * mm, backColor=PALE_BLUE, spaceBefore=2 * mm, spaceAfter=4 * mm))
styles.add(ParagraphStyle(name="code_cn", fontName="SimHei", fontSize=8.2, leading=13, textColor=INK, leftIndent=5 * mm, backColor=colors.HexColor("#F4F6F8"), borderColor=LINE, borderWidth=0.5, borderPadding=3 * mm, spaceBefore=2 * mm, spaceAfter=3 * mm))


def para(text, name="body_cn"):
    return Paragraph(text, styles[name])


class FlowDiagram(Flowable):
    def __init__(self, labels, width=170 * mm, height=45 * mm):
        super().__init__()
        self.labels, self.width, self.height = labels, width, height

    def draw(self):
        box_gap = 5 * mm
        box_w = (self.width - box_gap * (len(self.labels) - 1)) / len(self.labels)
        box_h = 15 * mm
        y = (self.height - box_h) / 2
        for i, label in enumerate(self.labels):
            x = i * (box_w + box_gap)
            fill = PALE_BLUE if i < 4 else PALE_GREEN
            stroke = BLUE if i < 4 else colors.HexColor("#2A8C59")
            self.canv.setFillColor(fill)
            self.canv.setStrokeColor(stroke)
            self.canv.roundRect(x, y, box_w, box_h, 2.5 * mm, fill=1, stroke=1)
            self.canv.setFillColor(INK)
            self.canv.setFont("SimHei", 8)
            lines = label.split("\n")
            start_y = y + box_h / 2 + (len(lines) - 1) * 3
            for line in lines:
                self.canv.drawCentredString(x + box_w / 2, start_y, line)
                start_y -= 6
            if i < len(self.labels) - 1:
                ax = x + box_w + 1 * mm
                ay = y + box_h / 2
                self.canv.setStrokeColor(MUTED)
                self.canv.line(ax, ay, ax + box_gap - 2 * mm, ay)
                self.canv.line(ax + box_gap - 2 * mm, ay, ax + box_gap - 4 * mm, ay + 1.2 * mm)
                self.canv.line(ax + box_gap - 2 * mm, ay, ax + box_gap - 4 * mm, ay - 1.2 * mm)


def footer(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(LINE)
    canvas.line(18 * mm, 14 * mm, A4[0] - 18 * mm, 14 * mm)
    canvas.setFillColor(MUTED)
    canvas.setFont("SimHei", 7.5)
    canvas.drawString(18 * mm, 8 * mm, "LingTu · 三维全局规划路线 · 2026-11-15")
    canvas.drawRightString(A4[0] - 18 * mm, 8 * mm, f"第 {doc.page} 页")
    canvas.restoreState()


def grid(rows, widths):
    cells = [[para(str(c), "small_cn") for c in row] for row in rows]
    table = Table(cells, colWidths=widths, repeatRows=1, hAlign="LEFT")
    commands = [("GRID", (0, 0), (-1, -1), 0.35, LINE), ("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4), ("TOPPADDING", (0, 0), (-1, -1), 4), ("BOTTOMPADDING", (0, 0), (-1, -1), 4), ("BACKGROUND", (0, 0), (-1, 0), BLUE), ("TEXTCOLOR", (0, 0), (-1, 0), colors.white)]
    for row in range(2, len(rows), 2):
        commands.append(("BACKGROUND", (0, row), (-1, row), colors.HexColor("#F8FAFC")))
    table.setStyle(TableStyle(commands))
    return table


story = []
story += [Spacer(1, 25 * mm), para("LingTu 三维全局规划路线", "title_cn"), para("PCT 的多层表达 + DimOS MLS 的图生命周期 + LingTu 的执行几何", "subtitle_cn")]
story += [FlowDiagram(["OctoMap\nmapd", "Surface\nProvider", "Nodes", "Edges /\nCorridor", "Planner\nArtifact", "GlobalRoute\n→ SCAN"]), Spacer(1, 12 * mm), para("技术路线说明书", "h1_cn"), para("版本 1.0 · 2026-11-15"), para("这份文档用于确定全局规划的实现边界、外部算法的复用方式和后续验收口径。离线全局路线、局部动态避障、速度跟踪和底层执行分层验收。", "quote_cn"), PageBreak()]

story += [para("1. 结论和决策", "h1_cn"), para("PCT 最适合作为多层空间表达和论文方法参考。DimOS MLS 最适合作为表面、稀疏节点、corridor 边和区域更新的实现参考。LingTu 继续负责机器人机身、地面支撑、步高、坡度、连续扫掠和现场执行。")]
story.append(grid([["维度", "PCT Planner", "DimOS MLS", "LingTu"], ["空间表达", "tomography，多层结构先于搜索", "每个 Z 层的 standable surface", "OctoMap + SurfaceIndex"], ["稀疏图", "tomogram 上的全局搜索结构", "Nodes + Voronoi corridor edges", "SurfaceGraph 节点和 corridor chain"], ["更新方式", "偏场景级重建", "update_region 和区域修复", "当前整张 PlannerArtifact 替换"], ["执行约束", "需要另外的执行验证", "依赖自身配置", "三维机身、支撑、步高、坡度、扫掠"], ["许可证", "GPLv2", "Apache-2.0", "现有产品与上游许可边界"]], [26 * mm, 48 * mm, 48 * mm, 48 * mm]))
story += [para("PCT Planner 的公开仓库标明 GPLv2，不能直接复制到产品。DimOS MLS 文件带 Apache-2.0 许可，可以在保留版权和许可证声明的条件下适配。DimOS 的完整 crate 还带自己的 runtime 和消息依赖，因此只迁移算法核心。"), para("最终采用的组合是 PCT 的问题定义、DimOS 的图组织与更新方式、LingTu 的三维执行合同。", "quote_cn"), PageBreak()]

story += [para("2. 当前代码和目标架构", "h1_cn"), para("LingTu 已有 SurfaceIndex、SurfaceGraph、PlannerSession 和准备文件缓存。第一步已经把表面和图收进 PlannerArtifact，让地图派生状态共同保存、共同恢复、共同失效。"), para("目标架构", "h2_cn"), FlowDiagram(["OctoMap\nsnapshot", "Surface\nProvider", "Node\nProvider", "Edge\nProvider", "Planner\nArtifact", "Global\nRoute"]), Spacer(1, 4 * mm), para("PlannerArtifact 是静态地图派生资产边界。地图内容、机身模型、支撑高度、步高、坡度和表面闭合参数改变时重建。目标、容差、吸附半径、搜索预算和同层偏好只更新查询。"), para("执行链路", "h2_cn"), FlowDiagram(["GlobalRoute", "SCAN\nlocal trajectory", "Follower\ndesired velocity", "Driver\nplatform command"]), para("SCAN 处理当前路线段和实时局部障碍。Follower 跟踪局部轨迹并输出期望速度。Driver 负责平台控制和命令执行。"), PageBreak()]

story += [para("3. Rust 与 C++ 的边界", "h1_cn"), para("Rust 负责纯算法数据结构和图计算。C++ 负责地图事实、机器人几何、DDS 和既有运行接口。"), para("Rust 侧", "h2_cn"), para("Voxel columns → multi-level surface extraction → bounded per-Z closing → sparse node placement → corridor edge construction → serializable planner artifact", "code_cn"), para("C++ 侧", "h2_cn"), para("• OctoMap 读取、地图身份、generation、snapshot 生命周期。\n• 机器人机身模型和标定。\n• body occupancy、ground support、step、slope、swept route。\n• no-go、临时 overlay、取消、错误传播和 GlobalPlanRequest/Result。\n• PlannerSession、准备文件、DDS 和 SCAN 接入。"), para("Rust 只需要输出 SurfaceCell、Node、Edge 和 PlannerArtifact 数据。DimOS module runtime、generated messages、Tokio worker loop 和 DimOS visualization message 不进入 navd。", "quote_cn"), PageBreak()]

story += [para("4. 地图生命周期", "h1_cn"), para("第一阶段采用整图资产", "h2_cn"), para("地图保存或加载 → OctoMap snapshot → SurfaceProvider → NodeProvider → EdgeProvider → C++ 三维几何复核 → PlannerArtifact", "code_cn"), para("mapd 已有 generation、sequence、occupied/free 证据和完整快照。它还没有向全局规划器提供区域体素增量或区域快照，所以第一阶段保持整张资产替换。"), para("第二阶段采用区域增量", "h2_cn"), para("mapd region snapshot / cell delta → 局部表面重建 → 局部节点迁移 → 局部 corridor 边修复 → 与旧资产合并 → 发布新的不可变 PlannerArtifact", "code_cn"), para("区域结果必须和同一输入下的整图重建逐项对照。表面集合、节点连通性、边 corridor、路线和几何复核都要一致。"), PageBreak()]

story += [para("5. 分阶段实施", "h1_cn"), grid([["阶段", "实现内容", "通过条件"], ["Phase 0", "冻结 903、公开建筑、楼梯、平台、过顶结构、窄门和动态障碍基线", "表面、图、路线、性能和执行指标都有固定记录"], ["Phase 1", "统一 PlannerArtifact 生命周期", "查询参数不重建静态图，缓存恢复一致"], ["Phase 2", "导入 Apache-2.0 DimOS MLS 算法片段，做 Rust/C++ A/B", "相同地图和参数下比较表面、节点、边、路线和几何复核"], ["Phase 3", "C++ 统一执行几何复核", "Rust 路线通过机身、支撑、步高、坡度和扫掠检查"], ["Phase 4", "mapd 区域快照或体素增量，接入局部修复", "区域修复和整图重建结果逐项一致"], ["Phase 5", "通过门槛后切换生产默认 backend", "最新对照、Rust A/B、SCAN、MuJoCo 和实机证据齐全"]], [24 * mm, 78 * mm, 78 * mm]), para("PCT 保留为离线研究对照，不进入产品运行时。", "small_cn"), PageBreak()]

story += [para("6. 验收指标", "h1_cn"), para("离线全局规划、路线几何、运行性能和实际执行分层记录。"), grid([["层级", "指标"], ["Surface / Graph", "多高度表面、楼梯平台连接、节点/边/组件、起终点吸附、重复构建和缓存恢复"], ["Route geometry", "机身占据、连续扫掠、地面支撑、步高、坡度、未知空间证据、终点 XYZ/XY/Z 误差"], ["Runtime performance", "冷准备、缓存恢复、热查询、表面提取、节点边构建、端点接入、搜索、平滑分别计时"], ["Execution", "SCAN 接入、动态障碍、Follower 速度、Driver 命令、到达、停车和碰撞"]], [40 * mm, 140 * mm]), para("现有 matched replay 使用较早的 DimOS 26144f 构建。LingTu 在建筑跨区、建筑楼梯和广场三例中返回路线，DimOS matched 在建筑两例中出现入口组件断开，在广场返回的路线未通过 LingTu 支撑复核。LingTu 广场热查询中位数为 74.3 ms，DimOS matched 为 16.4 ms。最新 DimOS main 为 5e0ebfd422，尚未重新跑 matched benchmark。"), para("这些结果属于离线全局规划和路线几何证据，不能代表 SCAN 动力学、MuJoCo 完整导航或 Go2/Thunder 实机通过。", "quote_cn"), PageBreak()]

story += [para("7. 当前证据和待验证项", "h1_cn"), grid([["场景", "LingTu 冷准备", "LingTu 热查询", "DimOS 构图", "DimOS 热查询"], ["建筑跨区", "32.9 s", "318.7 ms", "4.02 s，入口组件断开", "14.10 ms"], ["建筑楼梯", "21.9 s", "97.1 ms", "共享建筑图，入口组件断开", "9.45 ms"], ["广场穿越", "98.2 s", "74.3 ms", "19.84 s，路线返回", "16.4 ms"]], [34 * mm, 30 * mm, 30 * mm, 54 * mm, 30 * mm]), para("尚未验证的项目包括最新 DimOS main 的公平重建、Rust 适配后的 Surface/Graph A/B、区域增量输入、区域修复一致性、SCAN 接入、MuJoCo 完整运动、Thunder 上下楼和 Go2 实机导航。"), para("这些未验证项必须在生产切换前完成。", "quote_cn"), PageBreak()]

story += [para("8. 最终决策", "h1_cn"), para("采用 PCT 的多层空间问题定义，采用 DimOS MLS 的表面、节点、边和增量生命周期，保留 LingTu 的原生三维几何、OctoMap、mapd、SCAN、Follower、Driver 和 DDS。"), para("Rust 进入纯算法核心，C++ 保留地图身份、机器人几何、取消、数据接口和执行边界。Rust 后端先以 A/B 方式接入，只有在固定场景下通过连通性、几何复核、性能和执行验收后，才切换为现场默认。", "quote_cn"), para("参考资料", "h2_cn"), para("DimOS MLS  https://github.com/dimensionalOS/dimos", "small_cn"), para("PCT Planner  https://github.com/byangw/PCT_planner", "small_cn"), para("PCT paper  https://ieeexplore.ieee.org/document/10531813", "small_cn"), para("LingTu matched comparison  docs/research/planner-public-dataset-comparison-20261114.md", "small_cn"), para("LingTu surface graph evidence  docs/research/surface-graph-20261031.md", "small_cn")]

doc = SimpleDocTemplate(str(OUT), pagesize=A4, rightMargin=18 * mm, leftMargin=18 * mm, topMargin=16 * mm, bottomMargin=20 * mm, title="LingTu 三维全局规划路线")
doc.build(story, onFirstPage=footer, onLaterPages=footer)
print(OUT)
