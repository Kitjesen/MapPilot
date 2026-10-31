"""Append dated map-input, regression and additional-result evidence to the paper."""
from __future__ import annotations

import json
import math
from pathlib import Path
from statistics import median


def append_supplement(root, work, width, p, h, table, figure, load, source_text, new_page):
    figures = work / "figures"
    h("附录 B　输入凭证、分轴退化与新增实验")
    p("本附录对应2026-10-21 19:15的结果核对截点。它补充上一版缺少的地图转换、配置差异、计算成本、"
      "失败图件及新完成结果；不重新运行任何算法，也不将新定位成绩追溯标注到旧地图。")
    h("B.1　实际转换和规划输入",2)
    maps = [
        ("Corridor W", "build/dataset-mapping-20261021/maps/corridor"),
        ("GEODE W", "build/dataset-mapping-20261021/maps/geode"),
        ("Garage/no_loop", "build/garage-validation-20261021/maps/no_loop"),
        ("Garage/default", "build/garage-validation-20261021/maps/default_loop"),
        ("Corridor guarded", "build/dataset-planning-fix-20261021/guarded/maps/corridor"),
        ("GEODE dense", "build/dataset-planning-fix-20261021/dense-retry/maps/geode"),
    ]
    rows, stats = [], []
    for label, folder in maps:
        data = load(f"{folder}/metadata.json")
        tree = data["artifacts"]["octomap"]
        points = data["artifacts"]["map_pcd"]["point_count"]
        rows.append([label, f"{points:,}", f"{(root/folder/'octomap.ot').stat().st_size/1048576:.2f}",
                     tree["evidence_source"], tree["builder"]["version"]])
        s = tree["stats"]
        stats.append([label, f"{s['retained_endpoints']:,}", str(s['dropped_endpoints']),
                      f"{s['hit_updates']:,}", f"{s['free_updates']:,}"])
    table("B1","已生成且当前存在的导航地图",["地图/批次","PCD点数",".ot MiB","证据","Builder"],rows,
          [124,105,75,107,width-411],
          "全部采用0.10 m OctoMap、full_log_odds、odom坐标。三种支撑/自由空间膨胀均为0；这些是实际保存射线构图。MiB=2^20字节。")
    table("B2","地图构建中的保存射线统计",["地图/批次","保留端点","丢弃","hit更新","free更新"],stats,
          [124,111,45,105,width-385],
          "更新计数不是不同体素数量，也不是定位精度。PCD融合点数与用于射线构图的保存端点数并不相同。")
    commands = [
        "build/dataset-mapping-20261021/runs/corridor/build_resume.command.json",
        "build/dataset-mapping-20261021/runs/geode/build.command.json",
        "build/garage-validation-20261021/nav/no_loop/map_build.command.json",
        "build/garage-validation-20261021/nav/default_loop/map_build.command.json",
        "build/dataset-planning-fix-20261021/guarded/runs/corridor/build_guarded.command.json",
        "build/dataset-planning-fix-20261021/dense-retry/runs/geode/build_guarded.command.json",
    ]
    for path in commands:
        load(path)
    source_text("src/maps/cpp/build/pipeline.cpp")
    source_text("src/nav/cpp/planning/global/contract.hpp")
    source_text("src/nav/cpp/planning/global/octoplanner/octoplanner3d_core.hpp")
    source_text("src/nav/cpp/planning/global/octoplanner/octoplanner3d_core.cpp")
    source_text("src/lingtu/assembly/config/native_navigation.py")
    source_text("config/robots/unitree/go2/robot.yaml")
    request = load("build/dataset-mapping-20261021/planning/geode/local_geometry_control_native/request.json")
    p("地图由lingtu-mapctl build从保存关键帧、位姿和scan_origin构建。MapsPipelineCore的saved_rays分支累积真实命中及自由空间。"
      "原始图为builder 0.3.1，修正图为0.3.2。历史Building/Plaza的PCD支撑与自由空间推断属于另一个对照批次，不能与此混用。")
    p("离线JSON外层输入为map_path、start、goal和options。GEODE固定请求中的start="
      f"{request['start']}，goal={request['goal']}；地图路径指向保存的octomap.ot。核心会依据观测支撑吸附端点，"
      "因此请求坐标与返回折线第一点未必相同。GlobalPlanRequest携带坐标、参数和地图身份/代次；"
      "路径通过PlannerSession::run的独立参数传入。核心可接取消检查；现场全局规划不接管实时局部避障。", "small")
    table("B3","本轮离线诊断与当前Go2正式Product配置",["参数","离线诊断","正式Go2 nav"],[
        ["全局圆包络半径/m","0.44","0.155（半机宽）"],
        ["机体下/上净空/m","0.10 / 0.24","0.10 / 0.24"],
        ["支撑高度及容差/m","0.35 ± 0.05","0.35 ± 0.05"],
        ["最大坡度 / 台阶m","0.57735 / 0.45","tan(30°) / 0.45"],
        ["端点吸附搜索/cells","2","24"],
        ["终点总/XY/Z容差/m","0.20","0.15"],
        ["跨层惩罚","4","6"],
        ["障碍净空半径/权重","4 cells / 2.0","2 cells / 1.5"],
    ],[219,132,width-351],
          "共用核心不等于同一配置。0.44 m是离线保守外接圆；正式完整朝向相关双圆柱由SCAN检查。表列Go2仅用于解释具名案例，不收窄产品机型。")
    p("metadata中的navigation_ready=true仅表明地图制品满足该构建/加载合同。它不能替代路线连通性、SCAN执行、定位闭环或机器人验收。")

    h("B.2　Corridor高度退化与原门限缺口",2)
    prob = load("build/prob-plane-20261021/isotropic/evaluation.json")
    scores = prob["cases"]["corridor"]["scores"]["imu_frame"]["common"]
    b, f = scores["prior"], scores["fresh"]
    quantities = [("三维RMSE",b["rmse_m"],f["rmse_m"])]
    quantities += [(f"{axis} RMSE",b["axis_rmse_m"][i],f["axis_rmse_m"][i]) for i,axis in enumerate("XYZ")]
    quantities += [("XY合成RMSE",math.hypot(*b["axis_rmse_m"][:2]),math.hypot(*f["axis_rmse_m"][:2])),
                   ("Z最大绝对误差",b["height_max_abs_m"],f["height_max_abs_m"])]
    table("B4","相同8038参考样本的分轴误差（m）",["指标","基线","第一阶段","变化%"],
          [[name,f"{old:.6f}",f"{new:.6f}",f"{100*(new/old-1):+.2f}"] for name,old,new in quantities],
          [185,105,105,width-395],
          "两次各自在相同样本上做全序列SE(3)、scale=1拟合，覆盖891.660667 s。XY合成是已有X/Y均方根的平方和开根号，不是独立新测量。")
    p("Z-RMSE增加0.216160 m（41.30%），Z最大绝对误差由1.192168增至1.933381 m；X/Y改善使总RMSE仍下降3.37%。"
      "原冻结工程准则只要求Corridor总RMSE不回归并保留覆盖，没有Z门限。第一阶段通过该准则的事实保留，"
      "但不能写成各维精度均改善，亦不能作为多层导航精度通过。")
    p("没有针对这次isotropic Z退化的独立根因诊断。已有final-v4或Office姿态分析属于其他候选/场景。"
      "几何弱约束、平面权重、姿态/重力/偏置耦合与全程对齐分配目前只是待验证假设。后续应先定位同时间段的Z与姿态偏差，"
      "再做固定输入消融；不能用高度限幅或事后移动参考轴宣称修复。")
    figure(figures/"corridor-rmse-components.pdf","S1","Corridor同样本总位置与分轴RMSE。Z退化明确保留；总指标降低不代表高度问题解决。")

    h("B.3　新增候选与最新全程Garage结果",2)
    hybrid = load("build/prob-plane-20261021/hybrid/evaluation.json")
    source_text("build/prob-plane-20261021/hybrid/REPORT.md")
    rows=[]
    for case,c in hybrid["cases"].items():
        for hyp,s in c["scores"].items():
            old,new=s["common"]["prior"],s["common"]["fresh"]
            label=case+("/LiDAR" if "lidar" in hyp else "/IMU")
            rows.append([label,str(old["samples"]),f"{old['rmse_m']:.6f}",f"{new['rmse_m']:.6f}",f"{100*(new['rmse_m']/old['rmse_m']-1):+.3f}"])
    table("B5","已结束的隔离hybrid第二阶段（RMSE/m）",["序列/参考","N","第一阶段","hybrid","变化%"],rows,
          [145,52,104,104,width-405],
          "GEODE主指标仅改善0.335%，未达到预先5%增量目标；Slippage小幅退化。两段完整保留覆盖，但组合门限失败，按预先顺序不继续Corridor/Office，未提升为默认算法。")
    fusion = load("build/genz-fusion-20261021/v1/comparison.json")
    source_text("build/genz-fusion-20261021/README.md")
    source_text("src/localization/ltlio/frontend.hpp")
    rows=[]
    for variant,evaluation in fusion["evaluations"].items():
        for case,c in evaluation["cases"].items():
            state=c["runs"]["fresh"]["summary"]["final_state"]
            for hyp,s in c["scores"].items():
                old,new=s["common"]["prior"],s["common"]["fresh"]
                rows.append([variant,case+("/L" if "lidar" in hyp else "/I"),str(old["samples"]),
                             f"{old['rmse_m']:.6f}",f"{new['rmse_m']:.6f}",state])
    table("B6","独立GenZ三机制v1批次已完成比较（RMSE/m）",["机制","序列","共同N","第一阶段","v1候选","终态"],rows,
          [70,109,52,99,99,width-429],
          "I/L分别为IMU/LiDAR假设。Corridor此行仅6344共同样本，不能与完整8038样本的1.551559直接比较；其较小数值没有惩罚失锁尾部。")
    office=fusion["evaluations"]["all"]["office1"]
    p("完整组合GEODE误差明显增加，hybrid-only和sampling-only也未达到要求。Corridor完整组合仅保留6577/8843位姿、"
      "6344/8038参考样本并失锁；Office保留3825/4651位姿且失锁，其最大相邻地图高度修正"
      f"{office['prior']['max_adjacent_height_step_m']:.6f}→{office['fresh']['max_adjacent_height_step_m']:.6f} m。"
      "因此v1拒绝默认启用。当前共享源码包含可选机制，但三个顶层开关默认false；这与旧隔离hybrid是不同批次。v2在截点未形成完整精度结论。")
    garage=load("build/genz-fusion-20261021/garage/comparison-first-stage-v1-all-first-stage-full.json")
    rows=[]
    for name,run in garage["runs"].items():
        summary=run["summary"]
        gcp=garage["own_sparse_gcps"][name]["runs"][name]
        rows.append([name,f"{summary['sensor_duration_s']:.2f}",str(summary["poses"]),
                     f"{gcp['covered_gcps']}/7",f"{gcp['sparse_gcp_rmse_m']:.6f}" if gcp.get("alignment_available") else "—",
                     summary["final_state"]])
    table("B7","新Garage回放覆盖与稀疏测绘误差",["批次","输入秒","位姿","GCP","拟合RMSE/m","终态"],rows,
          [121,65,56,47,114,width-403],
          "full为原始699.750037s录制：6996云/139949IMU、6982位姿、零错误/拒绝/丢帧。稀疏7点同时用于SE(3)拟合和评价，不能称密集ATE或留出精度。")
    gcp=garage["own_sparse_gcps"]["first-stage-full"]["runs"]["first-stage-full"]
    p(f"全程7点RMSE为{gcp['sparse_gcp_rmse_m']:.6f} m，最大误差{gcp['sparse_gcp_max_error_m']:.6f} m；"
      f"21对点间距离RMSE为{gcp['pairwise_distance_rmse_m']:.6f} m。145s第一阶段与v1-all仅覆盖2点，"
      "间距有符号误差分别+0.041741和+0.069095 m，不计算不足约束的完整轨迹拟合精度。"
      "全程前端回放已补足旧车库覆盖缺口，但尚无该新轨迹生成的完整OctoMap及固定路线规划结果。")
    figure(figures/"garage-full-gcp-errors.pdf","S2","最新全程车库的7个实存测绘点残差。同点SE(3)拟合；不是密集轨迹真值，也不是路径跟踪误差。")

    h("B.4　耗时与路径采样的完整记录",2)
    table("B8","主要回放的观测成本（独立批次）",["运行","输入时长s","回放观测s","更新tick P95/ms"],
          [[name,f"{run['summary']['sensor_duration_s']:.3f}",f"{run['summary']['elapsed_s']:.3f}",
            f"{run['summary']['lidar_attempt_tick_p95_ms']:.3f}"] for name,run in garage["runs"].items()]
          +[[case+"/第一阶段",f"{c['runs']['fresh']['summary']['sensor_duration_s']:.3f}",
             f"{c['runs']['fresh']['summary']['elapsed_s']:.3f}",
             f"{c['runs']['fresh']['summary']['lidar_attempt_tick_p95_ms']:.3f}"] for case,c in prob["cases"].items()],
          [174,103,107,width-384],
          "共享主机、独立时间运行，不能计算公平速度排行榜。全程Garage回放1463s大于约700s传感时长，尚无NX实时资格。tick P95不是发布频率。")
    plan_records=load("build/dataset-planning-fix-20261021/results.json")
    rows=[]
    for item in plan_records:
        data=load(Path(item["source"]).relative_to(root).as_posix())["result"]
        path=data.get("path",[])
        distances=[math.dist(a,b) for a,b in zip(path,path[1:])]
        rows.append([item["dataset"]+"/"+item["case"].replace("local_geometry_control","20-22").replace("partial_stairs_diagnostic","15-45").replace("straight_corridor","140-180").replace("around_corner","180-300"),
                     item["variant"].replace("/planning","").replace("planning","original"),str(len(path)),
                     f"{min(distances):.3f}/{median(distances):.3f}/{max(distances):.3f}" if distances else "—",
                     f"{data['diagnostics']['elapsed_ms']/1000:.3f}"])
    for variant in ("no_loop","default_loop"):
        for case in ("local_20_25_native","longer_40_70_native"):
            result=load(f"build/garage-validation-20261021/planning/{variant}/{case}/result.json")
            distances=[math.dist(a,b) for a,b in zip(result["path"],result["path"][1:])]
            rows.append(["Garage/"+("20-25" if "local" in case else "40-70"),variant,str(len(result["path"])),
                         f"{min(distances):.3f}/{median(distances):.3f}/{max(distances):.3f}",f"{result['diagnostics']['elapsed_ms']/1000:.3f}"])
    table("B9","所有已调用固定路线的点距与原生耗时",["固定请求","地图版本","点数","点距最小/中位/最大m","耗时s"],rows,
          [113,99,41,170,width-423],
          "elapsed_ms来自原生runPlan，包含冷加载与规划设置/搜索，不是缓存热路径延迟。缺覆盖而未调用的Garage请求见表7，未填0秒。")
    p("全局折线在本批多数约0.10m一个点，斜走或同时升降可达约0.15m；不是沿弧长严格等间距重采样。"
      "SCAN约0.5m抽取一次参考点（欧氏距离规则），并保留终点。B样条由时间连续定义；100Hz是正式nav目标调度频率，"
      "与空间点距没有固定换算关系，也不是本次实测控制频率。")

    new_page();h("B.5　配套地图、误差和历史失败图件",2)
    figure(figures/"garage-partial-routes.pdf","S3","旧Garage no_loop部分地图及34/145点路线。只覆盖约134s的旧地图；不能作为最新约700s完整回放的地图。")
    p("以下图件来自保留的原实验，统一按原物理尺寸嵌入。图中SLAM估计轨迹、参考轨迹和全局规划路线按图例区分；"
      "失败案例不补画成功路径。AIST/Tunnel只有已有轨迹或精度证据，未生成的完整点云地图不补造。")
    supplementary = []
    for case in ("corridor","geode"):
        for kind,title in (("trajectory","估计与参考轨迹"),("height","估计与参考高度"),("error","条件位置误差随时间")):
            supplementary.append((f"build/dataset-mapping-20261021/evaluation/{case}/{kind}.pdf",
                                  f"{case.upper()} W基线：{title}。与正文表3同批次，不是最新概率平面结果。"))
    supplementary.append(("build/dataset-planning-fix-20261021/dense-retry/figures/geode/map3d_focus.pdf",
                          "GEODE更密保存地图。1301关键帧，687格ROI的occupied由150增至356，长路线仍无路径；关键帧不是严格超集，构建器亦有变化。"))
    for label,stem in (("Building跨层","A_building_cross"),("Building楼梯","B_building_stairs"),("Plaza","C_plaza")):
        supplementary.append((f"artifacts/paper-figures-20261016/{stem}_3d.pdf",
                              f"历史官方PCD {label}几何路线。只为modeled-free对照；表A3仍报告运动坡度失败，不代表执行通过。"))
    figure_index=[]
    for index,(source,caption) in enumerate(supplementary,4):
        target=figures/f"supplement-{index:02}.pdf"
        target.write_bytes((root/source).read_bytes())
        png=root/source.replace(".pdf",".png")
        if png.is_file():target.with_suffix(".png").write_bytes(png.read_bytes())
        figure(target,f"S{index}",caption)
        figure_index.append({"source":source,"file":target.name,"caption":caption,"scale":1})
    (work/"supplement-figures.json").write_text(json.dumps(figure_index,ensure_ascii=False,indent=2),encoding="utf-8")
