"""Generate the dated Chinese manuscript from recorded results, without replay.

Run with the installed Codex bundled Python (ReportLab and pypdf). Scientific
figures are generated separately by mapping_paper_figures.py with project Python.
"""
from __future__ import annotations

import csv
import json
import math
from datetime import datetime
from pathlib import Path
from statistics import median
from xml.sax.saxutils import escape

from pypdf import PdfReader, PdfWriter
from pypdf.generic import ArrayObject, DecodedStreamObject, DictionaryObject, FloatObject, NameObject
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen.canvas import Canvas
from reportlab.platypus import (
    BaseDocTemplate, Flowable, Frame, KeepTogether, PageBreak, PageTemplate,
    Paragraph, Spacer, Table, TableStyle,
)
from mapping_paper_supplement import append_supplement

ROOT = Path(__file__).resolve().parents[2]
WORK = ROOT / "build/paper-manuscript-20261021-rev2"
OUT = ROOT / "output/pdf/lingtu-mapping-paper-20261021-expanded.pdf"
FIG = WORK / "figures"
BASE = "build/dataset-mapping-20261021"
FIX = "build/dataset-planning-fix-20261021"
REPAIR = "build/map-repair-20261021"
PROB = "build/prob-plane-20261021"
W, H = A4
MARGIN = 48
CW = W-2*MARGIN
SOURCES = {}
TABLES = {}
PLACEMENTS = []
MARKDOWN = []


def load(relative):
    p = ROOT / relative
    raw = p.read_text(encoding="utf-8-sig")
    SOURCES[relative] = {"path": str(p), "bytes": p.stat().st_size,
                         "modified": datetime.fromtimestamp(p.stat().st_mtime).isoformat()}
    target = WORK / "sources" / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(raw, encoding="utf-8")
    return json.loads(raw)


def fmt(v, digits=4):
    return "—" if v is None else f"{v:.{digits}f}"


def route_stats(result):
    pts = result.get("path", [])
    if not pts:
        return {"points": 0, "length": None, "spacing_min": None,
                "spacing_median": None, "spacing_max": None, "max_slope": None,
                "goal_error": None}
    dist = [math.dist(a,b) for a,b in zip(pts,pts[1:])]
    slopes = [abs(b[2]-a[2])/math.hypot(b[0]-a[0],b[1]-a[1])
              for a,b in zip(pts,pts[1:]) if math.hypot(b[0]-a[0],b[1]-a[1])>0]
    return {"points": len(pts), "length": sum(dist), "spacing_min": min(dist),
            "spacing_median": median(dist), "spacing_max": max(dist),
            "max_slope": max(slopes), "goal_error": result.get("diagnostics",{}).get("goal_error_m")}


def score_rows(evaluation, batch):
    rows=[]
    for case,c in evaluation["cases"].items():
        for hypothesis,s in c["scores"].items():
            for version,score in s["common"].items():
                coverage = c["coverage"][version]
                run=c["runs"][version]
                rows.append({"batch":batch,"case":case,"hypothesis":hypothesis,"version":version,
                             "N":score["samples"],"rmse":score["rmse_m"],
                             "median":score.get("median_m"),"p95":score.get("p95_m"),
                             "max":score.get("max_m"),"z_rmse":score.get("height_rmse_m"),
                             "reference_sample_coverage":coverage["reference_sample_fraction"],
                             "covered_duration_s":coverage["covered_duration_s"],
                             "state":run["summary"]["final_state"],
                             "poses":run["summary"]["poses"]})
    return rows


class NativePDF(Flowable):
    def __init__(self,path):
        super().__init__()
        self.path=Path(path)
        page=PdfReader(self.path).pages[0]
        self.width=float(page.mediabox.width)
        self.height=float(page.mediabox.height)
        if self.width>CW+.5 or self.height>H-125:
            raise ValueError(f"Native figure does not fit; regenerate at Arial10, do not scale: {path}, {self.width}x{self.height}")
        self.hAlign="CENTER"
    def draw(self):
        x,y=self.canv.absolutePosition(0,0)
        PLACEMENTS.append({"page":self.canv.getPageNumber()-1,"path":str(self.path),
                           "x":x,"y":y,"scale":1.,"width":self.width,"height":self.height})


class Pipeline(Flowable):
    def __init__(self):
        super().__init__();self.width=CW;self.height=111
    def draw(self):
        c=self.canv;c.setFont("Arial",10);c.setLineWidth(.55)
        nodes=[("Saved rays",0,68,84),("3D OctoMap",103,68,89),("A* route vertices",211,68,126),
               ("SCAN reference",354,68,139),("B-spline trajectory",112,7,133),
               ("Controller + checks",263,7,142),("Driver",423,7,70)]
        for text,x,y,w in nodes:
            c.rect(x,y,w,27,stroke=1,fill=0);c.drawCentredString(x+w/2,y+9,text)
        def arrow(x1,y1,x2,y2):
            c.line(x1,y1,x2,y2);a=math.atan2(y2-y1,x2-x1)
            for da in (-.45,.45):c.line(x2,y2,x2-5*math.cos(a+da),y2-5*math.sin(a+da))
        for a,b in [(84,103),(192,211),(337,354)]:arrow(a+2,81,b-2,81)
        c.line(424,68,424,49);c.line(424,49,178,49);arrow(178,49,178,35)
        arrow(247,20,261,20);arrow(407,20,421,20)
        c.drawString(0,16,"Live pose / LiDAR")
        arrow(94,20,110,20)


def main():
    WORK.mkdir(parents=True,exist_ok=True);OUT.parent.mkdir(parents=True,exist_ok=True);FIG.mkdir(exist_ok=True)
    for name,file in [("Song","simsun.ttc"),("Hei","msyhbd.ttc"),("TimesNR","times.ttf"),
                      ("TimesB","timesbd.ttf"),("Arial","arial.ttf")]:
        pdfmetrics.registerFont(TTFont(name,str(Path("C:/Windows/Fonts")/file),subfontIndex=0))
    pdfmetrics.registerFontFamily("Song",normal="Song",bold="Hei")
    pipeline_canvas=Canvas(str(FIG/"planning-pipeline.pdf"),pagesize=(CW,111))
    Pipeline().drawOn(pipeline_canvas,0,0)
    pipeline_canvas.save()
    style=ParagraphStyle("Body",fontName="Song",fontSize=10.5,leading=16,wordWrap="CJK",firstLineIndent=21,spaceAfter=6)
    small=ParagraphStyle("Small",parent=style,fontSize=9,leading=13,firstLineIndent=0,spaceAfter=5)
    caption=ParagraphStyle("Caption",parent=small,fontSize=10,leading=14,alignment=TA_CENTER,spaceBefore=5,spaceAfter=9)
    headings={1:ParagraphStyle("H1",fontName="Hei",fontSize=13,leading=19,spaceBefore=13,spaceAfter=8,keepWithNext=True),
              2:ParagraphStyle("H2",fontName="Hei",fontSize=11,leading=16,spaceBefore=9,spaceAfter=5,keepWithNext=True)}
    story=[]
    def p(text,kind=None):
        story.append(Paragraph(text,small if kind=="small" else style));MARKDOWN.append(text)
    def h(text,level=1):
        story.append(Paragraph(text,headings[level]));MARKDOWN.append("#"*level+" "+text)
    def tbl(number,title,headers,rows,widths,note=""):
        TABLES[str(number)]={"title":title,"columns":headers,"rows":rows,"note":note}
        titlep=Paragraph(f"表 {number}　{title}",ParagraphStyle("TC",parent=caption,keepWithNext=True,spaceBefore=9))
        cell=ParagraphStyle("Cell",fontName="Song",fontSize=9,leading=12,wordWrap="CJK")
        data=[[Paragraph(escape(str(v)),cell) for v in row] for row in [headers,*rows]]
        t=Table(data,colWidths=widths,repeatRows=1,hAlign="CENTER")
        t.setStyle(TableStyle([("LINEABOVE",(0,0),(-1,0),.8,colors.black),
                              ("LINEBELOW",(0,0),(-1,0),.5,colors.black),
                              ("LINEBELOW",(0,-1),(-1,-1),.8,colors.black),
                              ("VALIGN",(0,0),(-1,-1),"TOP"),
                              ("LEFTPADDING",(0,0),(-1,-1),4),
                              ("RIGHTPADDING",(0,0),(-1,-1),4),
                              ("TOPPADDING",(0,0),(-1,-1),5),
                              ("BOTTOMPADDING",(0,0),(-1,-1),5)]))
        story.extend([titlep,t,Spacer(1,5)])
        lines=[" | ".join(headers)," | ".join("---" for _ in headers),
               *[" | ".join(map(str,row)) for row in rows]]
        MARKDOWN.extend([f"表 {number} {title}","\n".join(lines)])
        if note:p(note,"small")
    def fig(path,number,text):
        f=NativePDF(path);cap=Paragraph(f"图 {number}　{text}",caption)
        story.append(KeepTogether([f,cap]));MARKDOWN.append(f"![图 {number} {text}]({Path(path).as_posix()})")
    def source_text(rel):
        raw=(ROOT/rel).read_text(encoding="utf-8")
        SOURCES[rel]={"path":str(ROOT/rel),"bytes":(ROOT/rel).stat().st_size}
        dest=WORK/"sources"/rel;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(raw,encoding="utf-8")
        return raw

    baseline=[load(f"{BASE}/evaluation/{case}/result.json") for case in ("corridor","geode")]
    primary=[load(f"{BASE}/runs/{case}/replay.summary.json") for case in ("corridor","geode")]
    prob=load(f"{PROB}/isotropic/evaluation.json")
    prob_rows=score_rows(prob,"probability_plane")
    time_eval=load("build/geode-consistency-20261021/time-candidate/evaluation.json")
    other_eval=load("build/geode-consistency-20261021/other-scenes/evaluation.json")
    time_rows=score_rows(time_eval,"time_query")+score_rows(other_eval,"time_query")
    aist=load("build/degeneracy-update-20261021/projected-aist-comparison.json")
    all_plans=load(f"{FIX}/results.json")
    garage=load("build/garage-validation-20261021/nav/comparison.json")
    repair=load(f"{REPAIR}/results.json")
    source_text(f"{PROB}/README.md")
    source_text("src/nav/cpp/planning/global/octoplanner/README.md")
    source_text("src/nav/cpp/planning/local/scan/README.md")
    source_text("config/system/products/nav.yaml")
    source_text("src/nav/cpp/planning/local/scan/upstream/plan_manage/planner_fsm.cpp")
    source_text("src/nav/cpp/planning/local/scan/backend.cpp")
    short=load(f"{FIX}/planning/geode/local_geometry_control/summary.json")
    dense=load(f"{FIX}/dense-retry/planning/geode/local_geometry_control/summary.json")
    st,dt=route_stats(short["result"]),route_stats(dense["result"])

    story.append(Paragraph("面向四足机器人的原生三维建图与路径规划：<br/>实现、误差评测与地图一致性分析",
                           ParagraphStyle("Title",fontName="Hei",fontSize=17,leading=25,alignment=TA_CENTER,spaceAfter=12)))
    story.append(Paragraph("LingTu 项目技术研究稿　|　扩充版：2026 年 10 月 21 日 19:15 证据截点",ParagraphStyle("By",parent=small,alignment=TA_CENTER,spaceAfter=13)))
    MARKDOWN.append("# 面向四足机器人的原生三维建图与路径规划：实现、误差评测与地图一致性分析")
    p("<b>摘要：</b>本文整理 LingTu 原生导航系统从 LiDAR–IMU 观测、保存射线地图到三维全局路径的实现与公开数据实测。"
      "系统以原生 LTLIO 提供位姿，以保存扫描及物理原点构建 OctoMap，使用带支撑、坡度与扫掠约束的三维 A* 生成参考折线，"
      "再由 SCAN 局部轨迹与速度控制衔接机器人。Corridor02 和 GEODE 的 Windows 基线分别完成 8,843 与 3,430 个位姿及整图保存，"
      "但四条固定规划请求仅一条短路线通过全部几何检查。该路线含 26 个顶点、长 2.630 m；这些点不等于底层速度指令。"
      "后续 Linux 配对实验中，已接受概率平面模型的 GEODE 条件位置 RMSE 从 0.5178 降至 0.2923 m，Slippage 从 0.02314 降至 0.02043 m；"
      "Corridor 高度RMSE却增加41.30%。最新车库全程回放输出6982位姿，覆盖7个测绘点，稀疏同点拟合RMSE为0.4232 m；"
      "这一新增结果尚未形成新地图规划证据。本文保留隧道失锁、平墙部分覆盖、大图保存失败及被否决候选，"
      "将定位误差、规划终点几何误差和机器人实际跟踪误差分开，"
      "不把局部路线几何通过外推为全场景导航成功。")
    p("<b>关键词：</b>四足机器人；LiDAR–惯性里程计；三维占据地图；全局路径规划；误差评测；原生导航", "small")

    h("1　引言")
    p("点云在视觉上形成完整走廊或多层楼梯，并不直接证明机器人可以沿其导航。导航还需要区分占据、观测自由与未知空间，"
      "确认机体下方的真实支撑，并检查相邻姿态之间的连续碰撞和运动约束。本文研究的是这一观测到路径的工程链路；"
      "不将展示点云、SLAM 状态或单独的全局路径作为整机执行成功的替代指标。")
    p("LingTu 的现场热路径使用原生 C++，进程间通过类型化 DDS 连接，Python Host 承担语义和 API 层。"
      "LTLIO 的来源是固定版本 Elevator-LIO；占据地图采用 OctoMap [1]，全局规划由固定版本 OctoPlanner3D 集成，"
      "局部规划基于 SCAN-Planner [2–4]。本文报告本项目的集成、约束修正及实测边界，不声称这些上游算法均由本项目原创。")
    p("本稿汇总截至声明截点已完成的记录，包括逐批次精度、全部固定请求、八段 AIST、概率平面与后续GenZ候选，"
      "并补充地图转换凭证、实验与正式配置差异、计算耗时和各组现有图件。"
      "范围为建图、定位与几何导航实验；人物跟随、语义识别和 UI 素材继续保留在总资料包，不混入本稿的精度或成功率统计。")

    h("2　系统方法")
    h("2.1　观测、位姿与持久地图",2)
    p("原始 LiDAR 与 IMU 经保留扫描结束时刻的载体进入 LTLIO。地图保存包含融合点云、保存关键帧、位姿、时间戳和真实扫描原点。"
      "导航占据地图通过保存射线构建：测量端点提供占据证据，原点到端点之间的射线提供自由空间证据，未观测区域保持未知。"
      "融合 PCD 中存在某个点，并不保证保存的关键帧中存在对应射线；因而不能直接把所有显示点填成可通行地面。")
    p("当前概率平面第一阶段在已有地图所有者中维护局部平面支持与不确定性，保留既有传感方差 1/lidar_cov_inv。"
      "关联使用查询位姿不确定性，测量噪声中不重复累计查询状态协方差。模型没有完整表达共享位姿点与共享平面残差的互相关，"
      "仍有 CPU 和内存成本；其四组前端配对成绩与旧地图规划必须作为不同实验批次报告。")
    h("2.2　三维全局搜索及运动约束",2)
    p("全局输入为激活的三维 OctoMap、起终点和机器人几何参数。OctoPlanner3D 在三维 26 邻接上执行 A*，"
      "候选顶点经过占据与地面支撑查询；候选边在入队前检查米制步高、坡度和连续机体扫掠。代价结合距离、楼层变化和障碍间距。"
      "未知空间不由二维投影补成自由空间，同一 XY 下的不同高度表面保留。")
    p("外层离线JSON提供map_path、start、goal、options；核心的GlobalPlanRequest保存起终点、约束、地图身份/代次等，"
      "地图路径则单独传给PlannerSession。实际实验读取已构建的octomap.ot。pcd_conversion=false表示规划时没有再次转换PCD，"
      "并不否认先前的地图构建。实验与正式Product共用C++核心，但测试采用0.44 m保守圆包络；当前Go2正式全局圆半径为0.155 m，"
      "完整朝向相关包络由SCAN负责。配置差异与转换凭证见附录B，不能将离线测试等同于正式Product端到端验收。")
    p("本组诊断地图的 XY 步长和 OctoMap 分辨率为 r=0.10 m。支撑机体高度模式使用细分 Z 搜索步长，以避免等距邻接格最小上升坡度"
      "大于允许坡度的问题。此处 s_max=0.57735，得到 Δz=0.05 m：")
    p("<font name='TimesNR'>Δz = r / ceil(1 / s_max);　s = |z₂ − z₁| / √[(x₂ − x₁)² + (y₂ − y₁)²] ≤ s_max.</font>","small")
    p("找到目标后回溯相邻格点序列，点数由路线长度、转弯、升降和障碍决定，不固定为某个数量。返回点仍须经过独立验证器检查。"
      "原请求到吸附起点的接入段不属于返回折线的验证范围；规划终点也使用声明到达容差，不能由 reached_goal 推断零误差。")
    h("2.3　路径点、参考航路点与速度命令",2)
    story.append(KeepTogether([Pipeline(),Paragraph("图 1　原生规划和控制的数据关系。图为源码结构示意，不表示本次已执行完整运动链。",caption)]))
    MARKDOWN.append(f"![图 1 原生规划和控制的数据关系]({(FIG/'planning-pipeline.pdf').as_posix()})")
    p("全局结果进入任务和 Executor 时保留完整路径点。SCAN 收到参考后，按距最近保留点至少 0.5 m 的欧氏距离选取参考点并保留末点。"
      "对离线 GEODE 的 26 点直接应用该规则，选中索引 [0,5,10,15,20,25]，得到 6 个参考点。"
      "这只是对保存折线的确定性计算；正式运行还包含实测起点、投影接入、高度处理和重规划，因此不是已测下发计数。")
    p("SCAN 根据参考和实时三维碰撞信息生成有时间参数的 B 样条，控制点数随优化改变。跟踪器利用当前位置/速度和轨迹产生 vx、vy、wz，"
      "经最终限速、碰撞与输入就绪检查后，通过 DDS 交给唯一运动 driver，再调用所选平台的运动 API。"
      "正式 nav 配置的 100 Hz 是目标调度频率；本轮 headless 几何实验未测该链路的实时频率或机器人跟踪误差。")
    tbl(1,"本次结果中的四种“点”及其含义",["层级","当前可报告的量","是否为机器人执行证据"],[
        ["全局折线顶点", "GEODE 原图 26 点；车库 34/145 点", "否；原生搜索输出"],
        ["SCAN 参考航路点", "0.5 m 抽稀并保留末点；离线26点示例为6点", "未记录正式运行数量"],
        ["B 样条控制点/结点", "由每次局部优化生成，数量不固定", "本轮未运行局部轨迹链"],
        ["速度控制命令", "vx、vy、wz；nav 目标调度100 Hz", "本轮未驱动机器人"]],[111,242,CW-353])

    h("3　实验设置与指标")
    p("实验分为独立批次：W 为 Windows 完整基线建图，T 为 Linux 时间查询一致性修复，P 为 Linux 概率平面配对实验，"
      "G 为保存地图上的原生规划。不同平台、代码和地图制品不合并为单变量消融。初始设置、输入载体、"
      "条件参考点、源码及二进制快照保留在附录索引。共享主机上的单次耗时不作为公平实时性能排名。")
    tbl(2,"主地图实验输入及输出（W批次）",["数据集","LiDAR/IMU条数","位姿","点云/关键帧","终态"],[
        ["Corridor02","8859 / 178499","8843","10,644,022 / 495","TRACKING"],
        ["GEODE Stairs Alpha","3456 / 34556","3430","6,243,593 / 303","TRACKING"]],[113,105,48,161,CW-427],
        "两图无闭环图优化，直接 replay-save 未经过正式保存服务的动态清理；不能据此声称无漂移或无残影。")
    p("位置精度采用固定时间关联与无尺度 SE(3) 对齐。令估计位置为 p、参考为 p*，"
      "R 和 t 分别为旋转和平移，N 为匹配样本数：")
    p("<font name='TimesNR'>e<sub>i</sub> = Rp<sub>i</sub> + t - p<sub>i</sub>*;　"
      "RMSE = √[(1/N) Σ ||e<sub>i</sub>||<super>2</super>].</font>","small")
    p("配对比较使用完全相同的参考时刻，并同时报告覆盖。"
      "GEODE 的 PALoc/Leica 地图配准派生参考不是独立动作捕捉，IMU 与 LiDAR 两种参考点假设均报告，不能挑选较小值。"
      "Corridor 参考体点及独立精度尚未完全认证。")
    p("规划指标为顶点数 N、折线长度 L（三维相邻点距离之和）、三维点距、边坡度、末点到请求目标的几何误差，"
      "并分别记录 route、sweep 和 motion 检查。机器人路径跟踪误差、实际终端到达误差和速度误差没有本轮测量，统一标为未测。"
      "表中“—”表示未提供或不适用，不等于零；失败路线与缺覆盖未测请求分别记录。")

    h("4　结果")
    h("4.1　主地图与基线位置误差",2)
    tbl(3,"W批次条件位置误差（单位：m）",["数据集","匹配N","RMSE","中位","P95","最大"],
        [[name,str(d["matched_poses"]),fmt(d["rmse_m"]),fmt(d["median_m"]),fmt(d["p95_m"]),fmt(d["max_m"])] for name,d in zip(["Corridor02","GEODE"],baseline)],
        [119,58,80,80,80,CW-417],"表3只对应生成下列地图的W批次，不用后续P批次更好轨迹分数替换原图精度。")
    p("图2和图3呈现两组真实保存点云及估计轨迹。为了印刷可读性，仅显示原估计轨迹XYZ范围外扩5 m的点云采样，"
      "不修改源地图、不将裁剪后的点数作为地图总点数。三维视图保持物理XYZ比例；二维投影只用于观察，不能授权运动。")
    fig(FIG/"map-corridor.pdf",2,"Corridor02 基线地图的三维、俯视与侧视。点按地图Z着色，橙线为估计IMU轨迹。完整源图约1064万点；侧视采用独立坐标轴范围。")
    fig(FIG/"map-geode.pdf",3,"GEODE Stairs Alpha 基线地图。多层高度结构可见，完整源图约624万点；可视结构不等于可执行楼梯路径，侧视采用独立坐标轴范围。")
    h("4.2　配对时间修复与当前概率平面精度",2)
    tr=[]
    for r in time_rows:
        if r["version"]!="prior":continue
        f=next(x for x in time_rows if x["case"]==r["case"] and x["hypothesis"]==r["hypothesis"] and x["version"]=="fresh")
        hyp="IMU→棱镜" if r["hypothesis"]=="internal_imu_to_prism" else "IMU" if "imu" in r["hypothesis"] else "LiDAR" if "lidar" in r["hypothesis"] else "参考"
        tr.append([r["case"]+" / "+hyp,str(r["N"]),fmt(r["rmse"]),fmt(f["rmse"]),f"{r['poses']}→{f['poses']}",f["state"]])
    tbl(4,"T批次：相同参考时刻上的时间查询修复（RMSE单位：m）",["序列/参考点","共同N","前RMSE","后RMSE","位姿数前→后","后终态"],tr,
        [115,48,75,75,106,CW-419],"GEODE共同749样本仅覆盖早期；候选全覆盖3429样本的IMU/LiDAR RMSE为0.517826/0.535975 m。Tunnel由IMU位姿旋转杆臂至棱镜位置，与棱镜参考比较。共同区间不会惩罚失锁尾段。")
    p("BikeTunnel2 的共同区间误差从6.4042 m升至6.4955 m，两次均失锁；扩展后自身覆盖2369个参考样本的误差达到27.2196 m。"
      "因此更长的输出覆盖不是精度通过，不能只展示共同早期区间或更晚的失锁时刻。")
    pr=[]
    for r in prob_rows:
        label={"geode":"GEODE","slippage":"Slippage","corridor":"Corridor"}.get(r["case"],r["case"])
        label += "/IMU" if "imu" in r["hypothesis"] else "/LiDAR" if "lidar" in r["hypothesis"] else ""
        pr.append([label,"基线" if r["version"]=="prior" else "当前P",str(r["N"]),fmt(r["rmse"]),fmt(r["median"]),fmt(r["max"]),fmt(r["z_rmse"])])
    tbl(5,"P批次：完整共同样本精度（单位：m）",["序列/假设","版本","N","RMSE","中位","最大","Z-RMSE"],pr,
        [105,49,47,74,74,75,CW-424],"每行来自对应evaluation.json。该批次未存P95，不推算P95或RPE；当前P即已接受isotropic第一阶段。")
    p("P批次在四组普通参考条件下的位置RMSE均降低：GEODE的IMU/LiDAR假设、Slippage及Corridor分开报告。"
      "但改善并非所有分量一致：Corridor的Z-RMSE由0.5234 m升至0.7395 m，说明总体RMSE降低不能推出高度估计也改善。"
      "Office1最大相邻地图高度修正由0.287527 m降至0.238465 m，且保留4651位姿与两次进出；无独立高度真值，这仅为连续性指标。")
    fig(FIG/"precision-comparison.pdf",4,"时间一致性基线与已接受概率平面模型的条件RMSE。各子图坐标范围独立，参考样本固定；单次配对无显著性或置信区间结论。")

    h("4.3　全局路径点数与规划几何精度",2)
    plan_rows=[]
    route_records=[]
    for item in all_plans:
        path=Path(item["source"])
        rel=path.relative_to(ROOT).as_posix()
        d=load(rel);result=d.get("result",{})
        m=route_stats(result)
        record={**item,**m};route_records.append(record)
        plan_rows.append([item["dataset"]+" / "+item["case"].replace("local_geometry_control","20→22").replace("partial_stairs_diagnostic","15→45").replace("straight_corridor","140→180").replace("around_corner","180→300"),
                          {"planning":"原图修正","guarded/planning":"射线修正","dense-retry/planning":"更密保存"}[item["variant"]],
                          str(m["points"]),fmt(m["length"],3),fmt(m["goal_error"],5),"通过" if item["valid"] else "无路径"])
    tbl(6,"主地图所有修正后固定请求",["数据集/时间端点s","地图版本","点数","长度m","末点误差m","几何结果"],plan_rows,
        [152,73,42,70,84,CW-421],"误差为规划末点与请求目标的距离，非机器人实际到达误差。原始旧GEODE短路径另有26点、2.707107 m、3段坡度1.0失败；未作为成功隐藏。")
    p("修正后原GEODE短路线有26点、25边，长度2.630056 m；三维点距最小/中位/最大为0.100000/0.100000/0.141421 m。"
      "其中19条边长0.10 m、4条边长0.111803 m、2条边长0.141421 m。四条升降边水平0.10 m、垂直0.05 m，最大坡度0.5。"
      "更密地图虽仍有26点，长度为2.626832 m、最大点距0.15 m，不是同一组坐标。")
    fig(FIG/"geode-waypoints.pdf",5,"原GEODE地图上修正后的26个全局路径顶点。展示实际XY和高度随累计弧长变化；未平滑，也不是机器人执行轨迹。")
    gr=[]
    for batch in garage:
        for c in batch["cases"]:
            gr.append([batch["map"],c["id"].replace("local_","").replace("longer_","").replace("segment_","").replace("cross_level_","").replace("_","→"),
                       str(c["path_points"]) if c["verification_valid"] is not None else "未测",
                       fmt(c["path_length_m"],3) if c["verification_valid"] is not None else "—",
                       fmt(c["goal_error_m"],5),"通过" if c["verification_valid"] else "缺覆盖未测"])
    tbl(7,"Hilti车库两组部分地图的全部固定请求",["地图批次","时间端点s","点数","长度m","末点误差m","结果"],gr,
        [91,92,44,75,84,CW-386],"两组均1343位姿后LOST，只覆盖前2/7 GCP。34点路线接入吸附段约0.10 m尚未验证；145点路线亦无实际执行资格。末点误差使用原请求精确记录。")
    p("因此用户任务目标、全局路径顶点、SCAN参考航路点和样条控制点必须分开记录。当前可报告全局顶点数和几何末点误差；"
      "这些数据集路线没有启动SCAN执行、物理仿真或实机跟随，机器人横向跟踪RMSE、实际到达误差、速度误差及实际参考点数量均为未测。")

    h("4.4　地图缺陷与受控修复",2)
    p("在Corridor指定2849格ROI中，真实地面体素可能被穿过格内空隙而未穿过实测表面的射线误清除。"
      "局部平面/实际体素交段修正恢复226个free体素为occupied，代表体素占据概率从0.279升至0.566，"
      "但原两条长路线仍失败。新图起点完整可达分量3754个状态与A* closed一致，前向9个坡度允许邻居因ground_support拒绝，"
      "body-only均通过；此断口不是迭代次数不足或机体碰撞。")
    p("同一原495帧恢复默认闭环设置后，统一在早期scan3测量基准下检查晚期全部射线，两个邻域的穿面射线由12/9条变为0/0条，"
      "A新增两帧14个真实命中，B仍没有命中。坐标基变换不是GT对齐；局部改善尚未完成全图重建与固定长路线复测。")
    loop_figure=ROOT/f"{REPAIR}/loops/geometry-diagnostic/paper/loop-floor-origins.pdf"
    (FIG/loop_figure.name).write_bytes(loop_figure.read_bytes())
    fig(FIG/loop_figure.name,6,"同一早期拟合面下晚期传感器的有符号距离。闭环校正消除该局部重访冲突；零线为实测拟合面，不是外部真值。")
    p("覆盖缺口仍独立存在：GEODE指定687格表面ROI中，原303个关键帧有537格无保存命中。更密1301帧地图使occupied从150增至356，"
      "但关键帧只与原集合重合131帧，并非严格超集，且构建器同时改变，不能将每格变化单独归因于密度。长路线仍无路径。")
    tbl(8,"保存与重载的实际结果及限制",["实验","实测结果","状态"],[
        ["密集Corridor候选","8859云/8843位姿；写出2816帧/37196794点后bad allocation","整次保存失败"],
        ["ICP目标修复后的同大图加载","31.406 s，采样峰值2.390 GB，同图扫描有初值匹配成功","组件通过；非独立精度"],
        ["完整密集保存再测","轨迹逐值一致；整图数组聚合时bad allocation，尚未写出PCD","整次保存仍失败"],
        ["最终默认","保持1 s保存限频，0 s空间选帧仅显式实验","未推广未通过候选"]],[115,282,CW-397],
        "保存函数先聚合64B/点PCL云，再转最终32B/点输出，形成内存峰值。后续分块/紧凑保存须保留全部观测、字段与发布合同，不删点换通过。")

    h("5　讨论与局限")
    p("首先，TRACKING只表示估计器仍输出状态。旧车库批次约134.3 s后因速度越界失锁，完整读取6996帧不等于完整跟踪；"
      "AIST第06段虽有约0.390 m的局部RMSE，但GT时间覆盖仅7.97%。其次，精度对比需要同时间样本和参考点假设，"
      "不同平台与代码批次的数字不能拼接成一个改进比例。旧W基线与新的密集回放之间存在IMU查询/deskew变化，已单列来源。")
    p("新增第一阶段车库完整回放已保留6982位姿及7/7测绘点，更新了旧批次的覆盖结论；它没有重建旧表7中的导航地图，也未重跑对应路线。"
      "Corridor高度误差从0.523353升至0.739513 m，是41.30%的明确退化。原工程准则只约束总RMSE和覆盖，缺少分轴验收。"
      "因此第一阶段通过既定门限，不等于三维定位、特别是多层导航所需的高度精度已通过。")
    p("第三，地图可见结构与规划支撑证据不同；把未知补成自由或把没有射线的显示点填成地面会改变问题。"
      "第四，当前评价依赖单次配对，未提供多次独立重复、置信区间或公平跨平台实时基准。概率平面使总体误差降低，"
      "但Corridor高度误差变差、隧道失锁和CPU成本仍需处理。最后，全局几何通过不等于足端落点、SCAN轨迹执行或整机安全停止通过。")
    p("后续顺序为：先以保留全部真实点的分块写出关闭大图保存失败，再验证静止覆盖和完整闭环地图；随后保持原目标及机器人限制复测长路线；"
      "最后通过原生仿真和具名目标平台的保存重载、定位、导航与停止验收。本文没有通过移动端点、缩小机体或放宽坡度获得成功。")
    h("6　结论")
    p("LingTu已具备原生观测建图、saved-ray占据地图、三维全局搜索和局部轨迹控制的代码链路，并取得部分离线路径和配对定位改善。"
      "当前26/34/145等计数是全局折线顶点数；机器人实际执行依赖后续参考处理、样条和速度控制。"
      "已有证据不足以宣称所有场景稳定跟踪或端到端导航成功。正式结果必须同时报告精度、覆盖、失败及执行层级。")

    h("参考文献与软件来源")
    references=[
        "[1] Hornung A, Wurm K M, Bennewitz M, et al. OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees. Autonomous Robots, 2013. DOI: 10.1007/s10514-012-9321-0. https://octomap.github.io/",
        "[2] Zheng H, Chen Z, Fu Y, Yang M, Qin T. SCAN-Planner: Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigation. arXiv:2606.19555, 2026. 软件固定版本348e8a590a50a5a6bbab8d8c6dcfd171f009be26. https://github.com/wuyi2121/SCAN-Planner",
        "[3] JackJu-HIT. OctoPlanner3D [软件]. 固定版本9a9cc431ea905a5878975cc6fbbce6c9618b31a4. https://github.com/JackJu-HIT/OctoPlanner3D",
        "[4] Zhang Y, Huang Y, Zhang Y, et al. Elevator-LIO [软件]. 本项目来源固定版本1d79af77f3d9747ea57ef52a9b01d326a8ec561a. https://github.com/xiaofan4122/Elevator-LIO",
        "[5] SuperLoc 数据集. 官方序列、标定与参考轨迹入口. https://superodometry.com/superloc.html",
        "[6] GEODE Dataset. 官方多场景数据与参考说明. https://thisparticle.github.io/geode/ ; https://github.com/PengYu-Team/GEODE_dataset",
        "[7] Hilti SLAM Challenge 2023. 官方数据与Robot平台说明. https://hilti-challenge.com/dataset-2023.html",
    ]
    for ref in references:p(escape(ref),"small")
    p("[1]–[4]书目信息/软件来源已核对官方网站与仓库；本文结果来自本地记录，不引用上游展示视频作为LingTu执行证据。", "small")

    story.append(PageBreak());h("附录 A　失败、候选与补充精度全表")
    ar=[]
    for c in aist["cases"]:
        b,f=c["baseline"],c["candidate"]
        ar.append([c["sequence"].replace("flatwall_",""),str(c["common_gt_samples"]),f"{100*c['common_gt_duration_fraction']:.2f}",
                   fmt(b["common_score"]["position_rmse_m"]),fmt(f["common_score"]["position_rmse_m"]),
                   fmt(f["common_score"]["position_max_m"]),f["summary"]["final_state"]])
    tbl("A1","AIST八段全部结果：退化更新v2候选",["序列","N","GT覆盖%","基线RMSE","v2 RMSE","v2最大","终态"],ar,
        [37,35,66,90,90,82,CW-400],"单位m。v2未被采用，不是已接受概率平面模型；仅共同时间区间，无P95/中位记录。03/06失锁保留，06短覆盖不构成全段精度。")
    labels=[("plane-v1","否决"),("voxel-v2","未过全场景"),("final-v4","Corridor回归"),("cap100","Office回归"),
            ("corridor-noise","未通过"),("gate-control","诊断"),("weight-control","诊断"),("isotropic","接受第一阶段")]
    cr=[]
    for label,decision in labels:
        d=load(f"{PROB}/{label}/evaluation.json")
        vals=[]
        for case,hyp in [("geode","imu_frame_assumption"),("geode","lidar_frame_assumption"),("slippage",None),("corridor",None)]:
            c=d.get("cases",{}).get(case)
            if c:
                s=c["scores"][hyp] if hyp else next(iter(c["scores"].values()))
                vals.append(s["common"]["fresh"]["rmse_m"])
            else:vals.append(None)
        office=d.get("office1",{}).get("fresh",{}).get("max_adjacent_height_step_m")
        cr.append([label,*[fmt(v) for v in vals],fmt(office),decision])
    cr.append(["balanced support","—","—","—","—","1.1327","Office回归"])
    source_text(f"{PROB}/diagnostics/balanced-support/REPORT.md")
    load(f"{PROB}/diagnostics/balanced-support/comparison.json")
    tbl("A2","概率平面候选与诊断对照（完整已测项）",["候选","GEODE-I","GEODE-L","Slippage","Corridor","Office阶跃","决定"],cr,
        [90,69,69,66,66,74,CW-434],"前四列为位置RMSE/m；Office列为相邻地图高度修正/m，不是精度。—为未测。未将前一候选更好的单项成绩拼接为最终版本。")
    h("A.1　官方PCD几何试验与上游数值基线",2)
    official=load("artifacts/segment-motion-20261016/comparison.json")
    # These counts are retained in the named audit; the source JSON is shipped
    # with the manuscript so they remain separate from measured-ray cases.
    tbl("A3","官方PCD的历史固定路线（2026-10-16）",["案例","旧点数","修正点数","修正扫掠失败","坡度失败段","聚合结论"],[
        ["Building跨楼","322","329","0","134","失败"],["Building楼梯","138","141","0","67","失败"],
        ["Plaza","100","100","0","2","失败"]],[110,61,70,91,80,CW-412],
        "该批次使用PCD表面和建模自由空间，不是saved-ray建图。终态为segment_motion_failed；几何起点负例start_snap_exhausted，占据/无支撑目标负例goal_snap_exhausted。")
    source_text("artifacts/segment-motion-20261016/README.md")
    source_text("build/elevator-official-20261016/README.md")
    load("build/elevator-official-20261016/comparison.json")
    tbl("A4","固定上游Elevator-LIO数值基线",["序列","位姿","入/出","相邻IMU高度变化m","说明"],[
        ["Office1","4651","2 / 2","-0.337676","无完整轨迹真值"],
        ["Dormitory1","4889","3 / 3","-0.068721","无完整轨迹真值"],
        ["Campus3","6838","2 / 1","+0.292839","退出后重新入梯；末帧缺后续IMU"]],[90,58,58,141,CW-347],
        "这是上游数值baseline，不是当前LingTu全场景成绩；高度变化带符号，不是ATE。Office/Dormitory/Campus均进程退出0，不等于行为与精度通过。")
    source_text("build/ltlio-official-comparison-20261017/report.md")
    load("build/ltlio-official-comparison-20261017/comparison.json")
    tbl("A5","Native与固定上游的轨迹实现差（2026-10-17）",["序列","N","实现差RMS/m","P95/m","最大/m","严格parity"],[
        ["Office1","4651","0.002950","0.007413","0.031869","false"],
        ["Dormitory1","4889","0.040815","0.095730","0.141395","false"],
        ["Campus3","6838","0.056548","0.144272","0.180357","false"]],[94,50,110,80,80,CW-414],
        "实现对照不是真值比较，不能作为定位ATE，也不是当前概率平面模型成绩。")
    load("build/garage-validation-20261021/gcp-baseline-comparison.json")
    load("build/garage-validation-20261021/diagnosis/linux-gcp-comparison.json")
    tbl("A6","车库稀疏GCP距离对照",["批次/实现","覆盖GCP","真值间距m","估计间距m","差值m"],[
        ["Windows 当前","2 / 7","28.284083","27.546558","-0.737525"],
        ["Linux 当前","2 / 7","28.284083","27.948118","-0.335965"],
        ["Linux 原上游","2 / 7","28.284083","45.509751","+17.225668"]],[147,69,94,94,CW-404],
        "只覆盖前两点，不能拟合并评价完整轨迹；不同平台不挑选较小误差混合排名。")
    p("2026-10-17 native/official逐消息配对与耗时记录另列来源清单；这些是来源等价/性能诊断，不是新增独立真值精度。"
      "合成扰动、缓存前缀等价、编译与单元测试按类型列入资料包，没有换算成真实场景导航成功率。", "small")

    h("A.2　有效短路线的完整坐标",2)
    pts=short["result"]["path"]
    coordinates=[]
    for i in range(13):
        coordinates.append([str(i),*[fmt(v,6) for v in pts[i]],str(i+13),*[fmt(v,6) for v in pts[i+13]]])
    tbl("A7","GEODE原图修正后的26个全局顶点",["i","X/m","Y/m","Z/m","i","X/m","Y/m","Z/m"],coordinates,
        [25,78,72,74,25,78,72,CW-424],
        "离线地图坐标；索引从0开始。该表是原生全局路径输出，不是直接下发机器人的运动指令。配套CSV还记录累计弧长和段长。")

    story.append(PageBreak())
    append_supplement(ROOT,WORK,CW,p,h,tbl,fig,load,source_text,lambda:story.append(PageBreak()))
    story.append(PageBreak());h("附录 C　证据覆盖与复现")
    coverage=[
        ["W原始完整建图","两数据集、误差分位、三视图、旧短路失败","表2/3，图2/3，sources数据"],
        ["G规划","原图/射线修正/更密图8项；车库2版本12请求","表6/7，route_metrics.json"],
        ["T时间修复","GEODE双假设、Slippage、Corridor、隧道与覆盖","表4，accuracy-full.csv"],
        ["P概率平面","接受版本全部误差，拒绝/诊断候选与连续性","表5/A2，图4"],
        ["AIST退化","八段结果、失锁与GT时间覆盖","表A1"],
        ["地图缺陷","射线误清除、覆盖、闭环局部对照、保存失败","4.4，图6，原始JSON"],
        ["官方PCD与上游","历史3路线/负例；3上游序列；来源配对","表A3/A4，sources"],
        ["新增/展开证据","转换与配置差异、分轴退化、GenZ候选、全程Garage、图件","附录B"],
        ["未形成数字的资格","实际跟踪误差、足端、仿真/实机执行、实时资格","明确未测，不填0"]]
    tbl("C1","本稿纳入的结果范围",["结果组","覆盖内容","位置"],coverage,[114,252,CW-366])
    p("本文并非把总资料包中全部截图和日志逐页粘贴：可比较的结果进入表格，失败和未测请求不省略。"
      "本稿sources目录保存用于正文与表格的输入副本；完整回放日志、地图和程序仍在总资料包及各实验目录，"
      "不等于已全部复制进本稿。附录B补入旧隔离hybrid、独立GenZ v1已完成结果及最新全程车库；"
      "截至19:15核对时，v2未形成完整精度结论，不作通过陈述。本文不声称覆盖该截点以后新完成的运行。")
    p("配套材料包含正文Markdown、表格CSV/JSON、全部正式图的PNG/PDF、精确来源副本和生成程序。"
      "图中文字均为Arial 10 pt，点云层300 dpi栅格化，坐标/图例/曲线保留矢量；PDF按原物理尺寸嵌入图件。"
      "前版论文、旧报告和原冻结资料快照均保留。")
    p("关键源码：src/nav/cpp/planning/global/octoplanner/；src/nav/cpp/navigation/；"
      "src/nav/cpp/planning/local/scan/；src/nav/cpp/tracking/follower.cpp；"
      "src/nav/cpp/endpoint/nav/control/final.cpp；src/drivers/real/motion/。", "small")
    h("C.1　完整来源路径",2)
    for i,rel in enumerate(sorted(SOURCES),1):p(f"E{i:02d}　{escape(rel)}","small")

    base=WORK/"manuscript-text.pdf"
    def header(c,doc):
        c.saveState();c.setFont("Song",8.5);c.setFillColor(colors.HexColor("#555555"))
        if doc.page>1:c.drawString(MARGIN,H-27,"LingTu：原生三维建图与路径规划实验")
        c.setFont("TimesNR",9);c.drawCentredString(W/2,26,str(doc.page));c.restoreState()
    doc=BaseDocTemplate(str(base),pagesize=A4,leftMargin=MARGIN,rightMargin=MARGIN,topMargin=43,bottomMargin=43,
                        title="面向四足机器人的原生三维建图与路径规划",author="LingTu project")
    doc.addPageTemplates(PageTemplate(id="paper",frames=[Frame(MARGIN,43,CW,H-86,leftPadding=0,rightPadding=0,topPadding=0,bottomPadding=0)],onPage=header))
    doc.build(story)
    writer=PdfWriter()
    for pg in PdfReader(base).pages:writer.add_page(pg)
    for index,item in enumerate(PLACEMENTS):
        pg=PdfReader(item["path"]).pages[0];form=DecodedStreamObject();form.set_data(pg.get_contents().get_data())
        form.update({NameObject("/Type"):NameObject("/XObject"),NameObject("/Subtype"):NameObject("/Form"),
                     NameObject("/BBox"):ArrayObject([FloatObject(v) for v in pg.mediabox]),
                     NameObject("/Resources"):pg["/Resources"].clone(writer)})
        page=writer.pages[item["page"]];resources=page["/Resources"]
        if "/XObject" not in resources:resources[NameObject("/XObject")]=DictionaryObject()
        name=NameObject(f"/PaperFigure{index}");resources["/XObject"][name]=writer._add_object(form.flate_encode())
        stream=DecodedStreamObject();stream.set_data(page.get_contents().get_data()+
            f"\nq 1 0 0 1 {item['x']} {item['y']} cm {name} Do Q\n".encode("ascii"))
        page[NameObject("/Contents")]=writer._add_object(stream.flate_encode())
    writer.add_metadata({"/Title":"面向四足机器人的原生三维建图与路径规划：实现、误差评测与地图一致性分析","/Author":"LingTu project"})
    with OUT.open("wb") as f:writer.write(f)
    (WORK/"manuscript.md").write_text("\n\n".join(MARKDOWN),encoding="utf-8")
    (WORK/"tables.json").write_text(json.dumps(TABLES,ensure_ascii=False,indent=2),encoding="utf-8")
    (WORK/"route_metrics.json").write_text(json.dumps(route_records,ensure_ascii=False,indent=2),encoding="utf-8")
    with (WORK/"accuracy-full.csv").open("w",newline="",encoding="utf-8-sig") as f:
        rows=time_rows+prob_rows;wr=csv.DictWriter(f,fieldnames=list(rows[0]));wr.writeheader();wr.writerows(rows)
    for key,value in TABLES.items():
        with (WORK/f"table-{key}.csv").open("w",newline="",encoding="utf-8-sig") as f:
            wr=csv.writer(f);wr.writerow(value["columns"]);wr.writerows(value["rows"])
    manifest={"output":str(OUT),"pages":len(writer.pages),"generated_at":datetime.now().isoformat(),
              "sources":SOURCES,"figures":PLACEMENTS,"tables":list(TABLES),
              "scope":"mapping/localization/native global planning; no robot tracking qualification",
              "evidence_cutoff":"2026-10-21T19:15:00+08:00",
              "standalone_pipeline_figure":str(FIG/"planning-pipeline.pdf"),
              "source_policy":"Exact report inputs copied into sources; no sensor replay or algorithm modification",
              "literature_verified_urls":["https://octomap.github.io/","https://github.com/wuyi2121/SCAN-Planner","https://github.com/JackJu-HIT/OctoPlanner3D","https://github.com/xiaofan4122/Elevator-LIO"]}
    (WORK/"manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({"pdf":str(OUT),"pages":len(writer.pages),"bytes":OUT.stat().st_size,"tables":len(TABLES),"figures":len(PLACEMENTS)},ensure_ascii=False))


if __name__=="__main__":
    main()
