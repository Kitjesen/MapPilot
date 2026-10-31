"""Build the dated paper report from retained native experiment evidence.

Uses the existing bundled ReportLab/pypdf runtime. Original vector plots are
placed at scale 1 so their Arial 10 pt text is preserved.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from xml.sax.saxutils import escape

from pypdf import PdfReader, PdfWriter
from pypdf.generic import ArrayObject, DecodedStreamObject, DictionaryObject, FloatObject, NameObject
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen.canvas import Canvas
from reportlab.platypus import Paragraph, Table, TableStyle

ROOT = Path(__file__).resolve().parents[2]
BUNDLE = ROOT.parent / "lingtu-paper/20261021/runs/20261021T072510163854Z"
SOURCE = BUNDLE / "workspace"
OUT = ROOT / "output/pdf"
TMP = ROOT / "tmp/pdfs/mapping-report-20261021"
PDF = OUT / "lingtu-mapping-planning-results-20261021.pdf"
BASE = "build/dataset-mapping-20261021"
FIX = "build/dataset-planning-fix-20261021"
REPAIR = "build/map-repair-20261021"
WIDTH, HEIGHT = landscape(A4)
INK = colors.HexColor("#192630")
MUTED = colors.HexColor("#53636d")
LINE = colors.HexColor("#d9e0e3")
TEAL = colors.HexColor("#176d68")
PALE = colors.HexColor("#f1f5f5")


def read(relative):
    return json.loads((SOURCE / relative).read_text(encoding="utf-8"))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    TMP.mkdir(parents=True, exist_ok=True)
    for name, filename in (("CJK", "msyh.ttc"), ("CJKB", "msyhbd.ttc"),
                           ("Arial", "arial.ttf"), ("ArialB", "arialbd.ttf")):
        pdfmetrics.registerFont(TTFont(name, str(Path("C:/Windows/Fonts") / filename), subfontIndex=0))
    pdfmetrics.registerFontFamily("CJK", normal="CJK", bold="CJKB")
    pdfmetrics.registerFontFamily("Arial", normal="Arial", bold="ArialB")
    base_pdf = TMP / "report-text.pdf"
    canvas = Canvas(str(base_pdf), pagesize=(WIDTH, HEIGHT), pageCompression=1)
    canvas.setTitle("LingTu 原生三维建图与路径规划：实验结果与适用边界")
    canvas.setAuthor("LingTu project")
    canvas.setSubject("2026-10-21 evidence report; offline native components")
    placements = []
    pages = []
    sources_used = set()

    def para(text, x, y, width, size=10, leading=15, color=INK, bold=False):
        style = ParagraphStyle("body", fontName="CJKB" if bold else "CJK", fontSize=size,
                               leading=leading, textColor=color, wordWrap="CJK", alignment=TA_LEFT)
        item = Paragraph(text, style)
        _, height = item.wrap(width, 1000)
        if y - height < 43:
            raise ValueError(f"Text exceeds page {len(pages)}: {text[:50]}")
        item.drawOn(canvas, x, y-height)
        return y-height

    def page(title, subtitle, section):
        if pages:
            canvas.showPage()
        pages.append(title)
        canvas.setFillColor(INK)
        canvas.setFont("ArialB", 9)
        canvas.drawString(34, HEIGHT-28, "LINGTU  /  RESEARCH EVIDENCE")
        canvas.setFont("Arial", 9)
        canvas.setFillColor(MUTED)
        canvas.drawRightString(WIDTH-34, HEIGHT-28, f"2026.10.21   |   {section}")
        para(title, 34, HEIGHT-51, WIDTH-68, 22, 28, bold=True)
        para(subtitle, 35, HEIGHT-87, WIDTH-70, 9.5, 14, MUTED)
        canvas.setStrokeColor(LINE)
        canvas.line(34, 39, WIDTH-34, 39)
        canvas.setFont("Arial", 8)
        canvas.setFillColor(MUTED)
        canvas.drawString(34, 24, "Native offline evidence | No robot execution qualification")
        canvas.drawRightString(WIDTH-34, 24, f"{len(pages):02d}")

    def table(headers, rows, widths, y, x=34, size=10):
        style = ParagraphStyle("table", fontName="CJK", fontSize=size, leading=14,
                               textColor=INK, wordWrap="CJK")
        data = [[Paragraph(escape(str(v)), style) for v in row] for row in [headers, *rows]]
        item = Table(data, colWidths=widths, hAlign="LEFT")
        item.setStyle(TableStyle([
            ("BACKGROUND", (0,0), (-1,0), PALE), ("VALIGN", (0,0), (-1,-1), "TOP"),
            ("LINEBELOW", (0,0), (-1,0), 1, LINE),
            ("LINEBELOW", (0,1), (-1,-1), .4, LINE),
            ("LEFTPADDING", (0,0), (-1,-1), 10), ("RIGHTPADDING", (0,0), (-1,-1), 10),
            ("TOPPADDING", (0,0), (-1,-1), 9), ("BOTTOMPADDING", (0,0), (-1,-1), 9)]))
        _, h = item.wrap(WIDTH-68, HEIGHT)
        if y-h < 48:
            raise ValueError(f"Table exceeds page {len(pages)}: {h}")
        item.drawOn(canvas, x, y-h)
        return y-h

    def plot(relative, x, y):
        file = SOURCE / relative
        pg = PdfReader(file).pages[0]
        w, h = float(pg.mediabox.width), float(pg.mediabox.height)
        if x+w > WIDTH-25 or y+h > HEIGHT-107 or y < 43:
            raise ValueError(f"Figure exceeds page: {relative} {x,y,w,h}")
        placements.append({"page": len(pages)-1, "file": str(file), "x": x, "y": y,
                           "width": w, "height": h, "scale": 1.0})
        sources_used.add(relative)
        return w, h

    def note(title, text, y, x=566, width=238):
        y = para(title, x, y, width, 11, 16, TEAL, True)
        return para(text, x, y-7, width, 10, 15)-20

    def figure_page(title, subtitle, relative, notes, ref):
        page(title, subtitle, "MAPS")
        pg = PdfReader(SOURCE/relative).pages[0]
        w, h = float(pg.mediabox.width), float(pg.mediabox.height)
        plot(relative, 34+(500-w)/2, 83+(382-h)/2)
        y = 474
        for heading, body in notes:
            y = note(heading, body, y)
        para(ref, 38, 70, 760, 8.5, 12, MUTED)

    corridor = read(f"{BASE}/runs/corridor/replay.summary.json")
    geode = read(f"{BASE}/runs/geode/replay.summary.json")
    acc_c = read(f"{BASE}/evaluation/corridor/result.json")
    acc_g = read(f"{BASE}/evaluation/geode/result.json")
    repair = read(f"{REPAIR}/results.json")
    short = read(f"{FIX}/planning/geode/local_geometry_control/summary.json")
    points = short["result"]["path"]
    length = sum(math.dist(a,b) for a,b in zip(points,points[1:]))
    slope = max(abs(b[2]-a[2])/math.hypot(b[0]-a[0],b[1]-a[1])
                for a,b in zip(points,points[1:]))
    used = [f"{BASE}/README.md", f"{BASE}/manifest.json", f"{BASE}/runs/corridor/replay.summary.json",
            f"{BASE}/runs/geode/replay.summary.json", f"{BASE}/evaluation/corridor/result.json",
            f"{BASE}/evaluation/geode/result.json", f"{FIX}/README.md", f"{FIX}/results.json",
            f"{FIX}/planning/geode/local_geometry_control/summary.json", f"{REPAIR}/README.md",
            f"{REPAIR}/results.json", f"{REPAIR}/loops/geometry-diagnostic/late-rays-summary.json",
            f"{REPAIR}/source-provenance/audit.json"]
    sources_used.update(used)

    page("原生三维建图与路径规划", "实验结果与适用边界 | 本报告中的“跟踪”指 SLAM 位姿跟踪，不指人物跟踪。", "RESULTS")
    para("目前不能认定所有场景均可稳定跟踪并完成导航。",34,463,774,17,23,INK,True)
    para("系统已经能对生成的三维地图调用原生全局规划器，并在部分固定请求上返回满足几何约束的路径；"
         "但跟踪覆盖、地图证据与路径连通性仍有未通过项。",34,426,774,11,17)
    y=table(["验收层","现有证据","当前结论"],[
        ["SLAM 位姿跟踪","Corridor02、GEODE 本轮 Windows 完整回放均持续输出 TRACKING；其他退化场景存在 LOST。","部分数据集通过完整回放；不证明全场景精度。"],
        ["保存与导航地图","两组基线完成 PCD、真实扫描/原点与 OctoMap；密集 Corridor 两次完整保存失败。","基线可用于离线规划；密集保存未完成资格。"],
        ["原生全局规划","当前两图四条固定请求：一条短路线通过，三条长路线 empty_path。","已有有效短路径，长路线连通性仍失败。"],
        ["机器人执行","本轮未运行 Product 导航仿真，也未进行实机运动。","没有端到端导航执行通过结论。"]],[132,390,252],380)
    para("统计口径：四条请求是固定诊断案例，不是随机采样的场景成功率。其他车库/退化回放单列于第 13 页。",34,y-17,774,9.5,14,MUTED)

    page("数据、处理链与固定规划请求", "用本机原生 LTLIO 生成地图；没有用数据集发布的 GT 点云替代生成地图。", "METHOD")
    labels=["原始 LiDAR / IMU","原生 LTLIO","PCD + patches / origins","saved-ray OctoMap","OctoPlanner3D"]
    widths=[135,115,192,159,125]
    x=34
    for i,(label,w) in enumerate(zip(labels,widths)):
        canvas.setFillColor(PALE); canvas.rect(x,431,w,39,fill=1,stroke=0)
        para(label,x+8,457,w-16,10,14)
        if i<4:
            canvas.setStrokeColor(MUTED);canvas.line(x+w+3,450,x+w+9,450)
        x+=w+12
    y=table(["输入/基线输出","Corridor02","GEODE Stairs Alpha"],[
        ["全部点云 / IMU",f"{corridor['clouds']:,} / {corridor['imus']:,}",f"{geode['clouds']:,} / {geode['imus']:,}"],
        ["输出位姿 / 状态",f"{corridor['poses']:,} / TRACKING",f"{geode['poses']:,} / TRACKING"],
        ["保存地图 / 关键帧","10,644,022 点 / 495 帧","6,243,593 点 / 303 帧"],
        ["条件位置比较 RMSE",f"{acc_c['rmse_m']:.3f} m ({acc_c['matched_poses']:,} 匹配位姿)",f"{acc_g['rmse_m']:.3f} m ({acc_g['matched_poses']:,} 匹配位姿)"]],[220,277,277],409)
    y=para("位置比较使用一次 scale=1 的 SE(3) 对齐，不拟合时间。参考坐标系/杆臂未完整认证；GEODE 参考来自 PALoc 地图配准，"
         "不是独立动作捕捉。RMSE 是条件比较，不能作为无条件定位精度或论文复现分数。 [S01-S09]",34,y-12,774,9.5,14,MUTED)
    y=table(["固定请求（轨迹时间取样）","原起终点修正后结果","执行资格"],[
        ["Corridor 140→180 s / 180→300 s","均 empty_path，搜索候选耗尽","未执行"],
        ["GEODE 15→45 s","empty_path；更密地图复测仍失败","未执行"],
        ["GEODE 20→22 s",f"26 点，{length:.3f} m，最大段坡度 {slope:.1f}","几何检查通过；未执行"]],[310,330,134],y-10,size=9.5)

    common_c=[("数据与地图","完整 Corridor02 原始录制经 LTLIO 生成。展示使用保留的采样点；完整地图约 1064 万点。"),
              ("图中轨迹","是 SLAM 估计的 IMU 轨迹，不是独立真值，也不是机器人执行的全局路径。"),
              ("规划状态","原两条长请求及射线修正后的同请求仍无路径。几何看起来连续，不代表支撑/自由空间证据连续。")]
    for title,key,extra in [("Corridor02 / 三维结构","map3d",common_c),
        ("Corridor02 / 俯视覆盖","topdown",[("查看重点","平面覆盖、往返轨迹与通道关系。俯视投影只用于解释，不能替代三维碰撞、地面和高度查询。"),*common_c[1:]]),
        ("Corridor02 / 侧视高度","side",[("查看重点","观察估计轨迹与地图在高度上的一致性。持续 TRACKING 并不自动排除高度漂移或重访不一致。"),*common_c[1:]])]:
        figure_page(title,"原始基线生成地图 | 轨迹范围取景，未夸大 Z；图表按原尺寸保留 Arial 10 pt。",
                    f"{BASE}/figures/corridor/{key}_focus.pdf",extra,"来源 [S01-S05]；这组三视图保持历史基线，后续修复结果单列。")
    common_g=[("数据与地图","GEODE Stairs Alpha 完整录制生成。基线约 624 万点、303 个保存关键帧。多层楼梯在三维/侧视中可辨。"),
              ("本图边界","原始基线未做闭环校正；直接 replay-save 未经过 Product 保存清理阶段，不能据此声称动态残影已清理。"),
              ("规划状态","20→22 s 短路线经后续算法修正通过；15→45 s 长路线在基线和更密地图上均失败。")]
    for title,key in [("GEODE Stairs Alpha / 三维结构","map3d"),("GEODE Stairs Alpha / 俯视覆盖","topdown"),("GEODE Stairs Alpha / 侧视楼层","side")]:
        figure_page(title,"原始基线生成地图 | 这是楼梯数据集，不是 Corridor02，也不是 903 实机地图。",
                    f"{BASE}/figures/geode/{key}_focus.pdf",common_g,"来源 [S01-S04, S06-S08]；显示点云完整性与导航证据完整性分别评估。")

    figure_page("已通过的固定短路线", "GEODE 20→22 s | 当前修正后的实际输出，沿用原地图、原起终点和机器人约束。",
                f"{FIX}/figures/geode/local_route/map3d_focus.pdf",[
                    ("真实结果",f"26 个路径点、25 段；长度 {length:.6f} m；最大离散坡度 {slope:.1f}。route、连续机体 sweep 与 motion diagnostic 均通过。"),
                    ("修复内容","原等距搜索格会生成超过坡度限制的升降边。现使用独立细 Z 搜索，并在入队时检查米制坡度、步高和扫掠。"),
                    ("不能扩大结论","没有放宽原坡度或缩小机体。该短路线通过不代表三条长路线通过，更不代表 SCAN/步态控制已执行成功。")],
                "来源 [S07-S09]。诊断规划器记录 elapsed_ms=23890.7；不是实时控制频率或算法性能基准。")

    page("地面可见，为什么规划仍会断？", "地图展示、测量证据与可执行支撑不是同一件事。", "DIAGNOSIS")
    plot(f"{FIX}/cell-probe/measured-floor-voxel-rays.pdf",32,134)
    para("图中为实际测量点和射线。已证实部分 miss 只穿过地面体素上方的空隙，旧更新却清除了格内地面。修正后原 Corridor ROI 有 226 格 free→occupied，原两条路线仍失败。",34,116,774,10,15)
    para("另一个缺口：GEODE 指定 ROI 的 687 个 PCD 表面体素中，537 个没有原保存扫描命中；不能把这些显示点直接补成导航地面。以上均为局部诊断，不是整图覆盖率。 [S07-S08]",34,76,774,8.5,12,MUTED)

    page("重访对齐：同一组测量的局部对照", "原 495 帧、原标定与原始扫描；恢复默认闭环设置，使用相同早期测量基准比较。", "LOOP ALIGNMENT")
    plot(f"{REPAIR}/loops/geometry-diagnostic/paper/loop-floor-origins.pdf",40,276)
    plot(f"{REPAIR}/loops/geometry-diagnostic/paper/loop-floor-rays.pdf",34,44)
    y=note("观察到的变化","两个固定邻域的晚期穿面射线从 12/9 条变为 0/0 条；A 新增两帧共 14 个真实 hit，B 仍无 hit。",472)
    y=note("严格保持的基准","输出锚在最新 odom。统一用早期 scan3 作坐标基变换比较；不是 GT 对齐，也没有把首帧坐标变化当成漂移量。",y)
    y=note("当前结论","局部重访冲突改善。65 次闭环/优化和这些图仍不能代替完整校正地图重建、长路线连通性或执行验收。",y)
    note("来源","[S10-S12]。曲线零线是早期实测点拟合面；三维图只显示实际射线和有限拟合面。",y)

    page("保存与内存：已修复项和剩余失败", "不删测量点、不吞掉失败、不以加载成功代替完整保存成功。", "SAVE / RELOAD")
    y=table(["实验","结果","可证明的范围"],[
        ["密集 Corridor 首次保存","完整 8859 云 / 8843 位姿；写出 2816 patches、37196794 点后 bad allocation。","磁盘中间测量文件可诊断；整次 save 失败。"],
        ["ICP 地图 target 修复","修正 PCL 线性整数索引溢出与整图有限点副本；普通质心和原分辨率保留。","原生局部回归通过；定位目标构建机制已修。"],
        ["同一 3719 万点 PCD 原生加载","31.406 s；峰值 2.39 GB；有初值扫描匹配 success。","扫描本身在图中；非独立定位精度，非完整保存。"],
        ["修复后完整密集保存复测","LIO 轨迹逐值一致；在聚合大数组、尚未写 PCD 时再次 bad allocation。","完整保存仍失败；不能把较小峰值当成成功降内存。"],
        ["最终默认策略","生产默认恢复 1 s；0 s 空间选帧保留显式实验选项。","不推广未通过完整保存验收的候选。"]],[181,333,260],476)
    para("源代码版本控制：旧基线与新密集回放之间还存在 IMU 时间查询/deskew 算法变动，最大轨迹差 2.064 m，不能作为纯采样密度消融。"
         "两次新密集回放的轨迹逐值相同。来源与前后可执行文件均保留。 [S10-S13]",34,y-17,774,10,15)

    page("适用范围与下一步验收", "其他场景必须单列，不能把两组完整回放外推到所有环境。", "QUALIFICATION")
    extra_file = OUT / "mapping-report-extra-evidence.json"
    extra = json.loads(extra_file.read_text(encoding="utf-8")) if extra_file.exists() else []
    if not extra:
        raise RuntimeError("Expected separately audited cross-scene evidence before authoring report")
    y=table(["其他已测场景","已有记录","判断"],[[d["scene"],d["evidence"],d["judgment"]] for d in extra],[177,353,244],476,size=9.5)
    para("后续关闭顺序",34,y-18,774,13,19,TEAL,True)
    y=para("1. 分块写出完整 PCD、直接构建最终合同云，避免中间整图数组；原始点、字段、坐标和保存/重定位行为保持。<br/>"
           "2. 用同一完整录制验证密集保存，再检查静止观测覆盖与闭环校正后的真实地面/自由空间。<br/>"
           "3. 重建完整 OctoMap，按原起终点、机体尺寸与坡度复测三条失败路线。<br/>"
           "4. 原生仿真执行通过后，再验证目标板上的保存重载、定位、导航与停止。",34,y-45,774,10,16)
    para("成功条件必须同时覆盖跟踪、地图、路径和执行。人物跟随、VLM 目标识别的历史视频不是本报告 SLAM 结论的依据。",34,y-13,774,9.5,14,MUTED)

    refs=[
        ("S01","两组原始建图记录",f"{BASE}/README.md"),
        ("S02","数据、固定请求及机器人参数",f"{BASE}/manifest.json"),
        ("S03","Corridor 完整原生回放",f"{BASE}/runs/corridor/replay.summary.json"),
        ("S04","GEODE 完整原生回放",f"{BASE}/runs/geode/replay.summary.json"),
        ("S05","Corridor 条件位置比较",f"{BASE}/evaluation/corridor/result.json"),
        ("S06","GEODE 条件位置比较",f"{BASE}/evaluation/geode/result.json"),
        ("S07","地图与规划修正说明",f"{FIX}/README.md"),
        ("S08","原图、射线修正与更密地图规划汇总",f"{FIX}/results.json"),
        ("S09","修正后固定短路线与验证",f"{FIX}/planning/geode/local_geometry_control/summary.json"),
        ("S10","本轮地图修复总记录",f"{REPAIR}/README.md"),
        ("S11","保存、加载和默认策略最终结果",f"{REPAIR}/results.json"),
        ("S12","同测量、同早期基准射线对照",f"{REPAIR}/loops/geometry-diagnostic/late-rays-summary.json"),
        ("S13","IMU/deskew 源码混杂变量审计",f"{REPAIR}/source-provenance/audit.json"),
    ]
    for d in extra:
        for rel in d["sources"]:
            if rel not in [r[2] for r in refs]:
                refs.append((f"S{len(refs)+1:02d}",d["scene"]+"记录",rel))
                sources_used.add(rel)
    for start in range(0,len(refs),9):
        page("证据与复现入口"+(" / 续" if start else ""), "路径相对于冻结快照的 workspace/；程序、配置与失败记录跟随各自实验目录保留。", "SOURCES")
        y=478
        for code,desc,relative in refs[start:start+9]:
            if not (SOURCE/relative).exists():
                raise FileNotFoundError(SOURCE/relative)
            y=para(f"<b>[{code}] {escape(desc)}</b>",34,y,774,10,14)
            y=para(escape(relative),52,y-3,756,8.5,12,MUTED)-13
        if start==0:
            para("数据集主页和原始包来源见各实验 README/manifest。本报告汇总本地实测结果，不宣称复现论文排行榜分数。",34,76,774,9.5,14,MUTED)
        else:
            y=para("冻结材料根目录",34,y-3,774,11,16,TEAL,True)
            y=para(escape(str(BUNDLE).replace("\\","/")),34,y-7,774,9,14)-10
            y=para("生成程序：tools/bench/mapping_report.py；原图直接嵌入矢量 PDF，未缩放，Arial 10 pt 保留。中文正文使用嵌入微软雅黑。"
                   "图表呈现基线/后续修复时均注明版本，没有把历史图片改写成最新结果。",34,y,774,9.5,14)
    canvas.save()

    writer=PdfWriter()
    for pg in PdfReader(base_pdf).pages:
        writer.add_page(pg)
    for index,item in enumerate(placements):
        pg=PdfReader(item["file"]).pages[0]
        # Preserve dense scatter plots as vector forms without parsing every
        # drawing operator to merge page-level graphics resources.
        form=DecodedStreamObject()
        form.set_data(pg.get_contents().get_data())
        form.update({NameObject("/Type"):NameObject("/XObject"),
                     NameObject("/Subtype"):NameObject("/Form"),
                     NameObject("/BBox"):ArrayObject([FloatObject(v) for v in pg.mediabox]),
                     NameObject("/Resources"):pg["/Resources"].clone(writer)})
        target=writer.pages[item["page"]]
        resources=target["/Resources"]
        if "/XObject" not in resources:
            resources[NameObject("/XObject")]=DictionaryObject()
        name=NameObject(f"/EvidencePlot{index}")
        resources["/XObject"][name]=writer._add_object(form.flate_encode())
        content=DecodedStreamObject()
        content.set_data(target.get_contents().get_data()+
                         f"\nq 1 0 0 1 {item['x']} {item['y']} cm {name} Do Q\n".encode("ascii"))
        target[NameObject("/Contents")]=writer._add_object(content.flate_encode())
    for index,title in enumerate(pages):
        writer.add_outline_item(title,index)
    writer.add_metadata({"/Title":"LingTu 原生三维建图与路径规划：实验结果与适用边界", "/Author":"LingTu project", "/Subject":"2026-10-21 native offline evidence report"})
    with PDF.open("wb") as stream:
        writer.write(stream)
    manifest={"pdf":str(PDF),"pages":pages,"source_bundle":str(BUNDLE),
              "figures":placements,"sources":sorted(sources_used),
              "scope":"Native offline evidence; no all-scene or execution pass claim",
              "short_route":{"points":len(points),"length_m":length,"max_slope":slope},
              "fonts":{"body":"embedded Microsoft YaHei","plots":"original Arial 10 pt, scale=1"}}
    (OUT/"mapping-report-manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({"pdf":str(PDF),"pages":len(pages),"figures":len(placements),"bytes":PDF.stat().st_size},ensure_ascii=False))


if __name__=="__main__":
    main()
