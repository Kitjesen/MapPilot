"""Replay unchanged physical poses before/after a native support predicate edit."""
from __future__ import annotations

import argparse
import csv
import html
import json
import shutil
import subprocess
from collections import Counter
from pathlib import Path


def run(source: Path, output: Path, native: Path, label: str) -> None:
    target = output / label
    target.mkdir(parents=True, exist_ok=False)
    summary = []
    for case in sorted(source.iterdir()):
        if not (case / "probe.txt").exists():
            continue
        folder = target / case.name
        folder.mkdir()
        for name in ("scene.txt", "nominal.body2", "probe.txt"):
            shutil.copy2(case / name, folder / name)
        shutil.copytree(case / "snapshots", folder / "snapshots")
        with (folder / "native.log").open("w", encoding="utf-8") as stream:
            subprocess.run([str(native.resolve()), str(folder.resolve())], check=True,
                           stdout=stream, stderr=subprocess.STDOUT)
        with (folder / "native.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        result = {"case": case.name, "poses": len(rows)}
        for kind in ("nominal", "observed"):
            result[kind + "_accepted"] = sum(int(r[kind + "_free"]) for r in rows)
            result[kind + "_reasons"] = dict(Counter(r[kind + "_reason"] for r in rows if r[kind + "_reason"]))
        summary.append(result)
        print(json.dumps(result), flush=True)
    (target / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (target / "run.json").write_text(json.dumps({"native": str(native.resolve()),
        "source": str(source.resolve()), "scope": "same recorded states; no new physics execution"}, indent=2), encoding="utf-8")
    if label != "after":
        return
    changes = []
    for case in summary:
        name = case["case"]
        def read(label: str) -> list[dict]:
            with (output / label / name / "native.csv").open(newline="") as stream:
                return list(csv.DictReader(stream))
        before, after = read("before"), read("after")
        if len(before) != len(after) or any(a["t"] != b["t"] for a,b in zip(before,after)):
            raise ValueError(f"pose sequence changed: {name}")
        for old, new in zip(before, after):
            for kind in ("nominal", "observed"):
                keys = [kind + "_free", kind + "_reason", kind + "_link"]
                if any(old[k] != new[k] for k in keys):
                    changes.append({"case": name, "t": old["t"], "kind": kind,
                                    "before": {k: old[k] for k in keys},
                                    "after": {k: new[k] for k in keys}})
    (output / "changes.json").write_text(json.dumps(changes, indent=2), encoding="utf-8")
    print(f"Changed predicate results: {len(changes)}", flush=True)
    baseline = json.loads((output / "before" / "summary.json").read_text())
    table = []
    for old, new in zip(baseline, summary):
        if old["case"] != new["case"]:
            raise ValueError("case order changed")
        name = new["case"]
        table.append(f'<tr><td>{html.escape(name)}</td><td>{new["poses"]}</td>'
            f'<td>{old["observed_accepted"]} → {new["observed_accepted"]}</td>'
            f'<td>{html.escape(str(old["observed_reasons"]))}</td>'
            f'<td>{html.escape(str(new["observed_reasons"]))}</td>'
            f'<td><a href="before/{name}/native.csv">之前</a> · <a href="after/{name}/native.csv">之后</a></td></tr>')
    page = '''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>支撑与碰撞余量 · 同状态对照</title>
<style>body{max-width:1200px;margin:40px auto;padding:0 24px;background:#f5f6f7;color:#24313a;font:16px/1.7 Arial,"Microsoft YaHei",sans-serif}table{border-collapse:collapse;width:100%;background:white}th,td{text-align:left;border-bottom:1px solid #dce2e6;padding:12px}h1{font-size:28px}a{color:#236e91}</style>
<h1>支撑范围与碰撞余量：同一组记录姿态的修改前后对照</h1>
<p>只改变原生支撑谓词；地图、物理记录、机体、碰撞余量和查询时刻保持相同。本轮没有重新执行物理或导航闭环。</p>
<p>支撑使用未膨胀的保守部件投影；碰撞仍使用完整余量。墙或未知空间仍可能使最终结果拒绝。50 cm槽记录没有有效站立，不能参与成功率解释。</p>
<table><thead><tr><th>场景</th><th>姿态数</th><th>实测构型接受数</th><th>原拒绝原因</th><th>新拒绝原因</th><th>CSV</th></tr></thead><tbody>'''+"\n".join(table)+'''</tbody></table>
<p><a href="changes.json">逐条变化</a> · <a href="red-test.log">旧实现反例日志</a> · <a href="green-test.log">修改后测试日志</a> · <a href="../../docs/research/support-margin-fix-20261025.md">方法、验收与剩余问题</a> · <a href="../support-gait-20261025-r1/index.html">原物理实验与视频</a></p></html>'''
    (output / "index.html").write_text(page, encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--label", choices=("before", "after"), required=True)
    args = parser.parse_args()
    run(args.source, args.out, args.native, args.label)
