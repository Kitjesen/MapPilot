"""Compare frozen binaries' fixed-query outcomes and paired planning runs."""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path


def median(rows: list[dict], key: str) -> float:
    return statistics.median(row[key] for row in rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--candidate", default="updated")
    parser.add_argument("--fixed-before", default="fixed-before")
    parser.add_argument("--fixed-after", default="fixed-after")
    parser.add_argument("--report", default="README")
    args = parser.parse_args()
    fixed = []
    count = 0
    for source in sorted((args.root / args.fixed_before).glob("*/*_queries.json")):
        relative = source.relative_to(args.root / args.fixed_before)
        before = json.loads(source.read_text())
        after = json.loads((args.root / args.fixed_after / relative).read_text())
        if len(before) != 5 or len(after) != 5:
            raise ValueError(f"Missing fixed-query repetitions: {relative}")
        for first, second in zip(before, after):
            for key in ("poses", "edges", "pose_queries"):
                if first[key] != second[key]:
                    raise ValueError(f"Changed fixed-query outcome: {relative}/{key}")
            count += len(first["poses"]) + len(first["edges"])
        fixed.append({"scene": source.parent.name, "model": source.stem.split("_")[0],
                      **{f"{phase}_{key}": median(rows,key)
                         for phase, rows in (("before", before), ("after", after))
                         for key in ("pose_ms", "edge_ms")}})
    if len(fixed) != 14:
        raise ValueError("Expected seven scenes and two bodies")
    planning = []
    for scene in sorted({row["scene"] for row in fixed}):
        for model in ("obb", "parts"):
            groups = {}
            for stage in ("baseline", "updated"):
                directory = "baseline" if stage=="baseline" else args.candidate
                paths = sorted((args.root / directory / scene).glob(f"{model}_[0-9].json"))
                groups[stage] = [json.loads(path.read_text()) for path in paths]
                if len(groups[stage]) != 3:
                    raise ValueError(f"Missing planning repetitions: {scene}/{model}/{stage}")
            for before, after in zip(groups["baseline"], groups["updated"]):
                for key in ("ok", "reason", "path", "body_sweep_valid"):
                    if before[key] != after[key]:
                        raise ValueError(f"Changed planning result: {scene}/{model}/{key}")
            planning.append({"scene": scene, "model": model,
                             "before_ms": median(groups["baseline"],"elapsed_ms"),
                             "after_ms": median(groups["updated"],"elapsed_ms"),
                             "ok": groups["updated"][0]["ok"],
                             "reason": groups["updated"][0]["reason"]})
    result = {"candidate": args.candidate, "fixed_results_compared": count, "fixed": fixed, "planning": planning}
    (args.root / f"{args.report}.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    lines = ["# 部件变换与扫掠半径复用 · 2026-10-25", "",
             f"本报告比较 baseline 与 {args.candidate}。updated 是未保留的两项组合实验，radius 是半径复用实验。",
             f"固定查询逐项比较 {count} 个结果，返回值、失败原因、命中部件和内部姿态查询次数一致。",
             "七场景、两种模型、每组 64 个姿态与 64 条边，重复五次。每轮独立 BodyQuery，",
             "先姿态后边；计时不包含建图、构造 BodyQuery 和写文件。后者包含一次半径准备，",
             "其成本仍计入完整规划测试。时间单位 ms，下面是每批 64 次查询的中位数。",
             "", "| 场景 | 模型 | 姿态：前→后 | 边：前→后 |",
             "|---|---|---:|---:|"]
    for row in fixed:
        lines.append(f'| {row["scene"]} | {row["model"]} | '
                     f'{row["before_pose_ms"]:.3f} → {row["after_pose_ms"]:.3f} | '
                     f'{row["before_edge_ms"]:.3f} → {row["after_edge_ms"]:.3f} |')
    lines += ["", "## 整次规划", "",
              "前后均含上一轮直接姿态候选，本轮仅比较几何准备复用。独立 session 先准备地图，",
              "两模型交替顺序、各三轮，每轮 10 秒合作截止。84 次运行的成功状态、失败原因、",
              "返回路径和扫掠复查结果一致。截止不是无路证明，也不是严格的硬实时期限。",
              "", "| 场景 | 模型 | 前 ms | 后 ms | 结果 |", "|---|---|---:|---:|---|"]
    for row in planning:
        lines.append(f'| {row["scene"]} | {row["model"]} | {row["before_ms"]:.2f} | '
                     f'{row["after_ms"]:.2f} | {"通过" if row["ok"] else row["reason"]} |')
    lines += ["", "## 边界与复现", "",
              "Windows Release 本机诊断，不是目标板或实机运动结果。测量时未并行编译，",
              "但未隔离系统负载；短查询时间容易受噪声影响。固定站姿、机体尺寸、余量、",
              "支撑规则、未知判断和扫掠采样均未改变。本轮没有将 prepared API 接入 SCAN",
              "体素循环；旧接口继续工作。支撑接触内部的几何计算仍沿用原实现。",
              "", "```powershell",
              "# baseline/updated 内保存冻结可执行文件与四个修改文件的源码快照。",
              "& <stage>/demo.exe config/robots/unitree/go2/body/go2_mid360_flatpack.body2 <output> --matched",
              "& <stage>/demo.exe config/robots/unitree/go2/body/go2_mid360_flatpack.body2 <fixed-output> --queries",
              "uv run python tools/bench/prepared_query_report.py build/body-query-prepared-20261025",
              "```", ""]
    (args.root / f"{args.report}.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({"fixed_outcomes_equal": count, "planning_groups_equal": len(planning)}))


if __name__ == "__main__":
    main()
