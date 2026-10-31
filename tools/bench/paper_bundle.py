"""Collect a dated, local paper evidence bundle without changing active files.

Uses only the standard library. Each invocation retains a new source/evidence
snapshot, including uncommitted files. Raw inputs, models, dependencies and
assets remain at their recorded source paths. No scientific pass is inferred.
"""
from __future__ import annotations

import argparse
import csv
import html
import json
import os
import shutil
import subprocess
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

SOURCE_ROOTS = ("src", "tools", "tests", "config", "scripts", "cmake", "docker",
                "web", "sim", "docs", ".github")
RESULT_ROOTS = ("artifacts", "build", "research")
TEXT_SUFFIXES = set(".py .pyi .cpp .cc .c .h .hh .hpp .cxx .hxx .cu .cuh .cs .rs .go .java .kt .kts .gradle .swift .m .mm .js .jsx .ts .tsx .mjs .cjs .vue .svelte .css .scss .html .htm .md .rst .txt .json .jsonl .yaml .yml .toml .ini .cfg .conf .xml .idl .msg .srv .proto .pbtxt .sql .sh .bash .ps1 .bat .cmd .cmake .make .mk .in .lock .csv .tsv .tum .log .out .err .tex .bib .ipynb .patch .diff .urdf .xacro .sdf .world .material .usda .uplugin .uproject .props .targets .usf .ush .glsl .vert .frag .hlsl .wgsl .qml .qrc .rc .def .natvis".split())
TEXT_SUFFIXES.update({".manifest", ".launch", ".service", ".socket", ".timer", ".target", ".rules", ".desktop", ".rviz", ".params", ".runtime-xml"})
RESULT_SUFFIXES = TEXT_SUFFIXES | set(".png .jpg .jpeg .webp .gif .pdf .svg .mp4 .webm .avi .mov .mkv .h264 .pcd .ply .ot .bt .las .laz .npz .npy".split())
RAW_SUFFIXES = set(".bag .db3 .mcap .ltu .npy .safetensors .onnx .pt .pth .ckpt .tflite .engine .gguf .bin .tar .gz .tgz .zip .7z .rar".split())
ASSET_SUFFIXES = {".uasset", ".umap", ".uexp", ".ubulk", ".locres", ".sog", ".glb", ".gltf", ".fbx", ".stl", ".dae", ".blend", ".usd", ".usdz", ".exr", ".hdr", ".tga", ".dds"}
COMPILED_SUFFIXES = set(".o .obj .a .lib .so .dll .exe .pdb .ilk .pch .pyc .whl .dylib .exp .res .pak .ucas .utoc".split())
CACHES = {"__pycache__", ".pytest_cache", ".ruff_cache", ".mypy_cache", "CMakeFiles", "CMakeScripts", "CMakeScratch", "CMakeTmp", ".git", ".cache", "DerivedDataCache", "Intermediate"}
DEPENDENCIES = {"node_modules", "third_party", "third-party", "thirdparty", "_deps", "site-packages", ".venv", "venv", ".imports", "relocated-prefix", "prefix", "sysroot", "engine"}
STAGING = {".builds", ".feeder", ".transactions", ".promotion-staging"}
FROZEN_BINARIES = ("build/dataset-mapping-20261021/bin/",
                   "build/dataset-planning-fix-20261021/source-before/",
                   "build/map-repair-20261021/before/",
                   "build/map-repair-20261021/loop-replay-build/",
                   "build/map-repair-20261021/save-memory/before/",
                   "build/map-repair-20261021/save-memory/after/",
                   "build/map-repair-20261021/save-memory/tested/",
                   "build/map-repair-20261021/save-memory/default-restored/",
                   "build/map-repair-20261021/tested/")
EXTERNAL_PREFIXES = ("datasets/", "sim/evaluation/data/", "sim/library/sources/",
                     "build/deps/", "build/toolchain/", "build/uv-cache/", "build/uv/",
                     "build/unreal-ddc/", "build/ue-ddc/", "build/unreal-assets/",
                     "build/official-global-planner-data/")
FIELDS = ("source_path", "repository_path", "bundle_path", "bytes", "disposition",
          "reason", "evidence_review", "mtime_ns")

# Review notes describe the evidence boundary, never a computed pass/fail.
CURATED = [
    ("artifacts/paper-figures-20261016", "论文图 / Official PCD figures", "Arial 10 pt six views; manifest and export/check scripts retained. execution_validated=false. This export set and render_mapping exports record Arial 10 metadata; other historical figures have not been checked."),
    ("artifacts/official-global-planner-20260929", "Official PCD global planning", "Offline native algorithm evidence on modeled free space; original PCDs contain surfaces, not saved laser rays. Does not qualify field mapping or motion."),
    ("artifacts/segment-motion-20261016", "Segment motion diagnostics", "2026-10-16 motion diagnostics fail; retain failures alongside rendered routes. A route image is not successful execution."),
    ("artifacts/route-audit-20261016", "Route audit", "Geometry/path audit scope only; inspect original report before citing."),
    ("build/dataset-mapping-20261021", "Dataset mapping", "Offline native mapping outputs; original inputs remain external. Native component replay is neither simulation nor field acceptance."),
    ("build/dataset-planning-fix-20261021", "Dataset route planning", "Corridor/GEODE three long routes fail while short route passes in the reviewed record. Read actual reports; do not generalize from filenames."),
    ("build/map-repair-20261021", "Map repair / save-memory evidence", "Earlier spatial replay reached 8,859 clouds / 8,843 poses in TRACKING, then save failed with bad_alloc; its partial PCD is not a successfully saved map. A separate native CLI load/match on 37,196,794 points passed in 31.406 s with about 2.39 GB peak memory; this is not a full save transaction. The full retry failed while allocating the aggregate array under system committed-memory pressure before PCD save. Two explicit 0-second candidate runs retained identical trajectories; the default is restored to 1 second. The 65-loop diagnostic is a pose candidate, not a repaired map. Mock-flow and ICP regression passes remain local/component evidence; see final README/results and frozen before/after/default-restored executables."),
    ("build/garage-validation-20261021", "Garage validation", "Offline native replay and algorithm evidence; inspect report for scenario and failure scope."),
    ("build/geode-consistency-20261021", "GEODE consistency", "Native replay/consistency record. Does not establish Product acceptance or robot motion."),
    ("build/degeneracy-update-20261021", "Degeneracy update", "Native algorithm experiment; inspect retained report and configuration."),
    ("build/ltlio-official-comparison-20261017", "LTLIO official comparison", "Offline component comparison; retain dataset, configuration, trajectories and timing limitations."),
    ("build/elevator-official-20261016", "Elevator replay", "Offline component replay; no lift-riding or physical robot acceptance claim."),
    ("build/vlnav", "VL-Nav research", "Partial reproduction and reference material; remote server/WSL references were not checked by this collector."),
    ("build/person-follow", "Person selection / real video", "Real-video language selection/tracking is separate from the controlled native MuJoCo follow test. Their combined end-to-end loop remains unverified."),
    ("build/person-follow-native-20261009-tolerance-fix", "Controlled native MuJoCo follow", "Controlled RGB-D/native MuJoCo component evidence; do not combine with separate real-video language selection as a single end-to-end run."),
    ("build/dino-follow", "DINO tracking", "Video tracking experiment; missing/wrong-person frames and reacquisition failures remain part of the result."),
    ("build/dino-return", "Returning target / DINO", "same_frames_and_detector_input_for_ab=false: not a strict controlled A/B comparison. Preserve raw metrics and failures."),
    ("build/dino-camera", "Camera / tracking diagnostics", "camera-disabled placeholders are not camera evidence and are excluded from evidence selection; retained files keep original context."),
    ("build/semantic-acceptance-20261017", "Semantic acceptance / superseded flow", "mujoco-task-flow.json is superseded by build/semantic-field-20261018/task-flow.json. Other records retain their original scope."),
    ("build/semantic-field-20261018", "Semantic task flow / current record", "task-flow.json uses fixture grounding, pending state and no motion. It is not a complete navigation pass."),
    ("build/go2-map-save-v6-20260930", "Go2 map save and planning", "Historical field mapping/save evidence; dated scripts retain original NX paths/options. Field planning replay needs the actual saved .ot ray map."),
    ("build/go2-acceptance-20260920", "Go2 historical acceptance", "Historical named-platform evidence; inspect actual no-motion versus supervised-motion scope in reports."),
]


def classify(relative: str, source: Path, *, historical: bool = False) -> tuple[str, str]:
    """Return retention disposition and reason, without judging experiment success."""
    parts = Path(relative).parts
    suffix = source.suffix.lower()
    environment_example = (source.name == ".env" or source.name.startswith(".env.")) and any(token in source.name.lower().split(".") for token in ("example", "sample", "template"))
    if (source.name == ".env" or source.name.startswith(".env.")) and not environment_example:
        return "external", "runtime credential/environment file; metadata only, contents not read"
    if any(p in STAGING or p.startswith((".feeder-", ".formal-feeder-artifacts-")) for p in parts):
        return "excluded", "duplicate transaction/build staging or feeder assets"
    if any(p in CACHES for p in parts):
        return "excluded", "generated cache or compiled output; source and reports retained"
    if relative.startswith("build/") and source.name in {"CMakeCache.txt", "cmake_install.cmake", "Makefile", "build.ninja", ".ninja_log", ".ninja_deps"}:
        return "excluded", "generated CMake/Ninja build control; experiment source and logs retained"
    if any(relative.startswith(p) for p in FROZEN_BINARIES) and (suffix in {".exe", ".elf", ".a", ".so", ".dll", ".lib"} or not suffix):
        return "copied", "frozen executable/library explicitly retained with named experiment for reproduction"
    if suffix in COMPILED_SUFFIXES:
        return "excluded", "generated cache or compiled output; source and reports retained"
    if any(p.lower() in DEPENDENCIES for p in parts):
        return "external", "dependency checkout/runtime; reinstall from retained manifests"
    if not historical and any(relative.startswith(p) for p in EXTERNAL_PREFIXES):
        return "external", "raw dataset, external asset, dependency cache or upstream input"
    if suffix in RAW_SUFFIXES:
        return "external", "raw sensor/array input, model weight or packed distribution"
    if suffix in ASSET_SUFFIXES:
        return "external", "simulation/visual model, texture or engine asset; source programs and metadata retained"
    if suffix in {".png", ".jpg", ".jpeg", ".exr", ".tiff"} and any(p in {"rgb", "depth", "video-frames"} for p in parts):
        return "external", "raw RGB-D or rendered video-frame sequence; encoded evaluation video and figures retained"
    if environment_example:
        return "copied", "explicit environment example/template, not runtime credentials"
    if parts and parts[0] in SOURCE_ROOTS:
        if suffix in TEXT_SUFFIXES or source.name in {"Dockerfile", "Makefile", "LICENSE", "NOTICE"} or not suffix:
            return "copied", "working-tree source/configuration/documentation"
        if parts[0] == "docs" and suffix in RESULT_SUFFIXES:
            return "copied", "documentation figure or reference document"
        return "external", "source-tree asset; metadata and program source retained"
    if suffix in RESULT_SUFFIXES or source.name in {"Dockerfile", "Makefile", "LICENSE", "NOTICE"}:
        return "copied", "experimental record, script, configuration, evaluation media or output map"
    if len(parts) == 1 and (source.name.startswith(".") or not suffix):
        return "copied", "root build/format/dependency metadata"
    return "unreviewed", "unrecognized artifact format; retained at exact source path for review"


def filesystem_path(path: Path) -> Path:
    """Allow the extra bundle prefix on existing long Windows source paths."""
    if os.name == "nt" and not str(path).startswith("\\\\?\\"):
        return Path("\\\\?\\" + str(path.absolute()))
    return path


def iter_files(root: Path, on_error=None, skip_directory=None):
    """Enumerate scientific/source files and optionally prune non-evidence trees."""
    if root.is_file():
        yield root
        return
    if skip_directory is not None and skip_directory(root):
        return
    def walk_error(error):
        if on_error is None:
            raise error
        on_error(error)
    for directory, dirs, names in os.walk(filesystem_path(root), onerror=walk_error):
        if skip_directory is not None:
            dirs[:] = [name for name in dirs if not skip_directory(Path(str(Path(directory) / name).removeprefix("\\\\?\\")))]
        priority = {Path(relative).name for relative, _, _ in CURATED}
        dirs.sort(key=lambda name: (name not in priority, name))
        for name in sorted(names):
            path = Path(str(Path(directory) / name).removeprefix("\\\\?\\"))
            yield path


def git_metadata(repo: Path) -> dict:
    result = {}
    for name, args in (("revision", ["rev-parse", "HEAD"]), ("branch", ["branch", "--show-current"]),
                       ("status_porcelain", ["status", "--porcelain=v1", "--untracked-files=normal"])):
        proc = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True, encoding="utf-8", errors="replace", check=False)
        result[name] = proc.stdout.rstrip() if proc.returncode == 0 else {"error": proc.stderr.strip()}
    result["snapshot_basis"] = "actual working-tree files, including uncommitted and untracked source; not git HEAD export"
    return result


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def collect(repo: Path, output: Path, archive: Path | None, *, budget_bytes: int = 100 * 1024**3, run_id: str | None = None) -> dict:
    repo, output = repo.resolve(), output.resolve()
    if output == repo or repo in output.parents:
        raise ValueError("Output must be outside the active repository")
    marker = output / ".lingtu-paper-bundle"
    if output.exists() and any(output.iterdir()) and not marker.exists():
        raise ValueError(f"Refusing to replace unrelated existing output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    marker.write_text("Owned by tools/bench/paper_bundle.py; retained run snapshots.\n", encoding="utf-8")
    run_id = run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    run = output / "runs" / run_id
    run.mkdir(parents=True, exist_ok=False)
    (run / "workspace").mkdir()
    (run / "README.md").write_text("# Collection in progress\n\nThis snapshot is incomplete until summary.json and index.html are written.\n", encoding="utf-8")
    started = datetime.now(timezone.utc).isoformat()
    write_json(run / "git.json", git_metadata(repo))
    roots = [(repo / name, False) for name in (*SOURCE_ROOTS, *RESULT_ROOTS, "datasets", "third_party", "dist") if (repo / name).exists()]
    roots += [(p, False) for p in sorted(repo.iterdir()) if p.is_file()]
    if archive and archive.exists():
        roots.extend((p, True) for p in (archive / "artifacts", archive / "ARTIFACTS.md", archive / "README.md") if p.exists())
    counts, sizes, groups = Counter(), Counter(), {}
    copied_bytes = 0
    errors = []
    changed_during_copy = []
    not_traversed = []
    free = shutil.disk_usage(output).free
    budget_bytes = min(budget_bytes, max(0, free - 5 * 1024**3))
    with (run / "inventory.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        for root, historical in roots:
            print(f"Inventory/copy: {root}", flush=True)
            base = archive if historical else repo
            assert base is not None
            def skip_directory(path, base=base):
                relative = path.relative_to(base).as_posix()
                name = path.name
                disposition, reason = "", ""
                if name in CACHES:
                    disposition, reason = "excluded", "generated cache/build metadata"
                elif name in STAGING or name.startswith((".feeder-", ".formal-feeder-artifacts-")):
                    disposition, reason = "excluded", "duplicate transaction/build staging or feeder assets"
                elif name.lower() in DEPENDENCIES:
                    disposition, reason = "external", "dependency checkout/runtime or external import"
                elif Path(relative).parts[0] in (*RESULT_ROOTS, "web") and (name.startswith(("profile-", "edge-profile", "chrome-profile", "playwright-profile")) or name == ".profile"):
                    disposition, reason = "excluded", "browser runtime profile; private browsing state is not research evidence"
                elif relative in {"build/deps", "build/toolchain", "build/uv-cache", "build/uv", "build/unreal-ddc", "build/ue-ddc", "build/unreal-assets", "sim/library/sources"}:
                    disposition, reason = "external", "dependency cache or external asset library"
                if not disposition:
                    return False
                not_traversed.append({"source_path": str(path), "repository_path": relative,
                                      "disposition": disposition, "reason": reason, "bytes": None,
                                      "inventory_scope": "directory only; contents and file count not inventoried"})
                return True
            for source in iter_files(root, on_error=lambda exc: errors.append({"source_path": exc.filename, "error": str(exc)}), skip_directory=skip_directory):
                relative = source.relative_to(base).as_posix()
                target_relative = ("historical/lingtu-cleanup-20260929/" if historical else "workspace/") + relative
                target = run / target_relative
                try:
                    stat = filesystem_path(source).lstat()
                    disposition, reason = classify(relative, source, historical=historical)
                    review = "reviewed_scope_only" if not historical and any(relative == p or relative.startswith(p + "/") for p, _, _ in CURATED) else "unreviewed"
                    if "camera-disabled" in relative.lower() or "camera_disabled" in relative.lower():
                        review = "placeholder_not_evidence"
                    if disposition == "copied" and copied_bytes + stat.st_size > budget_bytes:
                        disposition, reason = "external", "copy budget reached; source retained, increase --budget-gib for next snapshot"
                    if disposition == "copied":
                        filesystem_path(target.parent).mkdir(parents=True, exist_ok=True)
                        shutil.copy2(filesystem_path(source), filesystem_path(target))
                        after = filesystem_path(source).stat()
                        if (after.st_size, after.st_mtime_ns) != (stat.st_size, stat.st_mtime_ns):
                            changed_during_copy.append(str(source))
                        stat = filesystem_path(target).stat()
                        copied_bytes += stat.st_size
                    writer.writerow(dict(source_path=str(source), repository_path=relative,
                                         bundle_path=target_relative if disposition == "copied" else "",
                                         bytes=stat.st_size, disposition=disposition, reason=reason,
                                         evidence_review=review, mtime_ns=stat.st_mtime_ns))
                    counts[disposition] += 1
                    sizes[disposition] += stat.st_size
                    group = ("historical/" if historical else "") + "/".join(Path(relative).parts[:2])
                    entry = groups.setdefault(group, {"files": 0, "bytes": 0, "dispositions": {}, "source_path": str(base / group.removeprefix("historical/"))})
                    entry["files"] += 1
                    entry["bytes"] += stat.st_size
                    entry["dispositions"][disposition] = entry["dispositions"].get(disposition, 0) + 1
                except OSError as exc:
                    errors.append({"source_path": str(source), "error": str(exc)})
                    counts["unreviewed"] += 1
                    writer.writerow(dict(source_path=str(source), repository_path=relative, bundle_path="", bytes="", disposition="unreviewed", reason=f"I/O error: {exc}", evidence_review="unreviewed", mtime_ns=""))
    omitted = [{"source_path": str(p), "disposition": "excluded", "bytes": None, "reason": "outside program/evidence scope; local tooling, environment or temporary state", "inventory_scope": "directory only; contents and file count not inventoried"} for p in sorted(repo.iterdir()) if p.is_dir() and p.name not in (*SOURCE_ROOTS, *RESULT_ROOTS, "datasets", "third_party", "dist")]
    catalog = []
    for relative, title, note in CURATED:
        folder = run / "workspace" / relative
        links = []
        if folder.exists():
            files = list(iter_files(folder))
            preferred = sorted(files, key=lambda p: (p.name.lower() not in {"readme.md", "summary.json", "report.json", "report.md", "task-flow.json", "manifest.json"}, len(p.parts), p.name))
            links = [p.relative_to(run).as_posix() for p in preferred[:12]]
        catalog.append({"title": title, "source_path": str(repo / relative), "bundle_directory": "workspace/" + relative,
                        "available": folder.exists(), "qualification": note, "links": links})
    summary = {"run_id": run_id, "started_utc": started, "finished_utc": datetime.now(timezone.utc).isoformat(),
               "repository": str(repo), "run_directory": str(run), "copy_budget_bytes": budget_bytes,
               "counts": dict(counts), "bytes": dict(sizes), "groups": groups, "errors": errors,
               "changed_during_copy": changed_during_copy, "not_traversed": not_traversed + omitted,
               "external_references": "Remote server and WSL paths in documents were not accessed or verified.",
               "snapshot_consistency": "Files copied sequentially from working tree; not an atomic filesystem snapshot. Refresh once active experiments finish."}
    write_json(run / "summary.json", summary)
    write_json(run / "catalog.json", catalog)
    write_landing(run, summary, catalog)
    write_json(output / "latest.json", {"run_id": run_id, "path": "runs/" + run_id, "counts": dict(counts), "bytes": dict(sizes)})
    (output / "README.md").write_text(f"# LingTu 论文材料总入口\n\n最新快照：[论文目录与证据说明](runs/{run_id}/README.md) · [可搜索图表和运行目录](runs/{run_id}/index.html)\n\n打开 [index.html](index.html) 进入最新目录。每次收集保留独立 `runs/` 快照；源码来自实际工作树，包含未提交修改。\n\n机器清单、原路径、字节数和排除原因见最新快照 `inventory.csv` 与 `summary.json`。未复制不等于丢失：数据集、模型、依赖与资产仍在记录的原位置，本目录不是完全离线的运行环境。\n\n重建命令：\n```powershell\n& '{sys.executable}' '{repo.as_posix()}/tools/bench/paper_bundle.py' --repo '{repo.as_posix()}' --output '{output.as_posix()}'\n```\n", encoding="utf-8")
    (output / "index.html").write_text(f'<!doctype html><meta charset="utf-8"><title>LingTu paper bundle</title><p><a href="runs/{run_id}/index.html">打开最新论文目录 / Open latest paper catalog</a></p><script>location.replace("runs/{run_id}/index.html")</script>', encoding="utf-8")
    if any(p.is_dir() and not (p / "summary.json").exists() for p in (output / "runs").iterdir()):
        with (output / "README.md").open("a", encoding="utf-8") as handle:
            handle.write("\n交付/分享请只使用上面链接的最新完整运行目录。本日期目录还保留未完成的诊断快照；这些初版可能含无关浏览器运行状态，不应分享整个日期目录或 runs 集合。\n")
    return summary


def write_landing(run: Path, summary: dict, catalog: list) -> None:
    rows = "\n".join(f"| {key} | {value:,} | {summary['bytes'][key]:,} |" for key, value in summary["counts"].items())
    sections = "\n\n".join(f"### {item['title']}\n\n{item['qualification']}\n\n" + (" · ".join(f"[{Path(p).name}]({p})" for p in item["links"]) or "当前未找到已复制文件；查清单原路径。") for item in catalog)
    readme = f"""# LingTu 论文材料与实验记录

快照：`{summary['run_id']}`，UTC {summary['finished_utc']}。源码与结果保留原仓库相对路径，入口为 [workspace](workspace/)。历史归档在 [historical](historical/)。

[可搜索图表/实验目录](index.html) · [文件级机器清单](inventory.csv) · [分类与体积/收集问题](summary.json) · [工作树版本与修改状态](git.json) · [结构化实验目录](catalog.json)

| 处置 | 文件数 | 字节数 |
| --- | ---: | ---: |
{rows}

`copied` 为已复制；`external` 为外部保留的数据/模型/依赖/资产；`excluded` 为缓存、编译产物、重复 staging/.feeder；`unreviewed` 为未识别格式或读取失败。所有已遍历文件保留绝对原路径、复制路径、字节数、理由。依赖、缓存、浏览器运行 profile、staging 及本机工具目录会剪枝，其绝对目录、处置与理由单列于 summary.json 的 not_traversed，bytes=null，明确不申报内部文件数/体积。上表只统计已遍历文件，不是整个磁盘或全部外部材料总量。datasets 与 sim/evaluation/data 继续逐文件外部盘点。已复制文件的 evidence_review=unreviewed 只表示尚未审阅科学结论。

这里收集的是材料与原始结论，不是新的验收。文件名含 pass/acceptance 不构成通过证据。必须区分本地契约、原生仿真、实机无运动、受监督实机运动；组件运行不能升级为完整 Product 成功。参见 [验证规则](workspace/docs/testing.md)、[研究证据索引](workspace/docs/research/README.md)。远程服务器/WSL 引用未经本次访问核验。UI mockup 仅为展示；10 月 16 日论文图与 render_mapping 生成图有 Arial 10 pt 元数据声明，其他历史截图未校核。

原始 bag/LTU/NPY/RGB-D、模型及依赖保持外部；包括 datasets 与 sim/evaluation/data。输出 PCD/OT 地图按预算复制。原路径仍被旧脚本引用，归档不会自动改写运行配置。工作树逐文件复制并非原子快照，运行中的实验结束后需要重新收集；每次运行保留新快照，不覆盖之前的源码和结果。若 summary.json 中 errors 或 changed_during_copy 非空，先处理/刷新对应条目再引用。

## 已审阅范围的实验入口

{sections}

## 其他实验与历史结果

其余 build/artifacts/research 运行只做材料保留，科学结论标记 unreviewed。可在网页的完整运行目录按名称检索，再打开原始 report/summary/metrics。历史目录保留 [ARTIFACTS.md](historical/lingtu-cleanup-20260929/ARTIFACTS.md) 的迁移上下文。下载论文属于研究参考，不是 LingTu 实现证据。
"""
    (run / "README.md").write_text(readme, encoding="utf-8")
    cards = []
    for item in catalog:
        links = " ".join(f'<a href="{html.escape(p, quote=True)}">{html.escape(Path(p).name)}</a>' for p in item["links"])
        cards.append(f'<article data-search="{html.escape(item["title"] + " " + item["source_path"], quote=True)}"><h2>{html.escape(item["title"])}</h2><p>{html.escape(item["qualification"])}</p><small>{html.escape(item["source_path"])}</small><nav>{links or "未复制 / missing"}</nav></article>')
    figures_dir = run / "workspace/artifacts/paper-figures-20261016"
    figures = "".join(f'<a href="{p.relative_to(run).as_posix()}"><img loading="lazy" src="{p.relative_to(run).as_posix()}" alt="{html.escape(p.stem)}"><span>{html.escape(p.stem)} · PNG</span></a>' for p in sorted(figures_dir.glob("*.png")))
    map_figures = []
    for relative in ("build/dataset-mapping-20261021", "build/dataset-planning-fix-20261021", "build/map-repair-20261021"):
        folder = run / "workspace" / relative
        for p in sorted(p for p in folder.rglob("*.png") if p.name in {"map3d_focus.png", "topdown_focus.png", "side_focus.png"}):
            link = p.relative_to(run).as_posix()
            label = p.relative_to(run / "workspace").as_posix()
            formats = " ".join(f'<a href="{p.with_suffix(suffix).relative_to(run).as_posix()}">{suffix[1:].upper()}</a>' for suffix in (".pdf", ".svg") if p.with_suffix(suffix).exists())
            map_figures.append(f'<div data-search="{html.escape(label, quote=True)}"><a href="{link}"><img loading="lazy" src="{link}" alt="{html.escape(label)}"><span>{html.escape(label)}</span></a>{formats}</div>')
    map_gallery = '<h2 style="margin-top:24px">地图与路线导出</h2><p>render_mapping 导出图的字体声明见各自元数据；结果与失败边界见原报告。</p><div class="figures">' + "".join(map_figures) + "</div>" if map_figures else ""
    group_rows = []
    for name, entry in sorted(summary["groups"].items()):
        if not name.startswith(("build/", "artifacts/", "research/", "historical/")):
            continue
        relative = ("historical/lingtu-cleanup-20260929/" + name.removeprefix("historical/")) if name.startswith("historical/") else "workspace/" + name
        link = f'<a href="{html.escape(relative, quote=True)}">files</a>' if entry["dispositions"].get("copied") else "original path in CSV"
        group_rows.append(f'<tr data-search="{html.escape(name, quote=True)}"><td>{html.escape(name)}</td><td>{entry["files"]:,}</td><td>{entry["bytes"] / 1024**2:,.1f}</td><td>{html.escape(str(entry["dispositions"]))}</td><td>{link}</td></tr>')
    document = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>LingTu 论文材料目录</title><style>
body{font-family:Arial,"Microsoft YaHei",sans-serif;background:#f5f5f2;color:#222;max-width:1240px;margin:32px auto;padding:0 22px;line-height:1.55}h1{font-size:30px}h2{font-size:18px;margin:0}a{color:#095e8b}header,p{max-width:1000px}small{display:block;overflow-wrap:anywhere;color:#666}input{position:sticky;top:12px;width:min(720px,90%);padding:14px;border:1px solid #999;background:white;font-size:16px;z-index:1}.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:14px;margin:22px 0}article{background:white;padding:18px;border:1px solid #ddd}nav a{display:inline-block;margin:8px 12px 0 0;font-size:13px}.figures{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px}.figures img{width:100%;background:white}.figures span{display:block}table{width:100%;border-collapse:collapse;font-size:13px}td,th{text-align:left;border-bottom:1px solid #ddd;padding:9px;overflow-wrap:anywhere}td:first-child{max-width:420px}[hidden]{display:none!important}.notice{border-left:4px solid #b87a20;padding:8px 15px;background:#fff6e7}</style>
<h1>LingTu 论文材料目录</h1><p>RUN_ID · 当前工作树源码、实验记录、图表、失败与边界说明。所有快照保留独立 runs 目录。</p><nav><a href="README.md">材料说明</a><a href="inventory.csv">文件清单 CSV</a><a href="summary.json">体积 / 外部与排除</a><a href="git.json">Git 与未提交状态</a><a href="workspace/docs/testing.md">证据等级</a><a href="workspace/docs/research/README.md">研究索引</a></nav>
<p class="notice">收集不等于实验通过。原生组件、仿真、实机无运动与实机运动须分别引用。未审阅目录保留原始结论；未连接远端/WSL。数据集、模型和资产仍需按 inventory.csv 的原路径访问。</p>
<input id="search" placeholder="搜索实验、图表或目录 / filter runs" aria-label="搜索实验目录"><h2 style="margin-top:24px">论文导出图 · Arial 10 pt · 6 views</h2><p>PNG / PDF / SVG 及 export.py、manifest.json、验证记录位于同一目录。execution_validated=false；展示路径不能作为运动成功证据。</p><div class="figures">FIGURES</div><div class="cards">CARDS</div><h2>全部运行材料 · 其余结论 unreviewed</h2><p>files 链接打开已复制目录；external/excluded 文件的准确原位置见 CSV。目录名含 pass 不代表通过。</p><table><thead><tr><th>原相对目录</th><th>文件</th><th>MiB</th><th>处置</th><th>材料</th></tr></thead><tbody>GROUPS</tbody></table><script>document.getElementById('search').addEventListener('input',e=>{let q=e.target.value.toLowerCase();document.querySelectorAll('[data-search]').forEach(x=>x.hidden=!x.dataset.search.toLowerCase().includes(q))})</script></html>'''
    document = document.replace('<div class="cards">', map_gallery + '<div class="cards">', 1)
    document = document.replace("RUN_ID", html.escape(summary["run_id"])).replace("FIGURES", figures).replace("CARDS", "".join(cards)).replace("GROUPS", "".join(group_rows))
    (run / "index.html").write_text(document, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path, default=Path("D:/inovxio/brain/lingtu-paper/20261021"))
    parser.add_argument("--archive", type=Path, default=Path("D:/inovxio/brain/lingtu-cleanup-20260929"))
    parser.add_argument("--budget-gib", type=float, default=100)
    args = parser.parse_args()
    summary = collect(args.repo, args.output, args.archive, budget_bytes=int(args.budget_gib * 1024**3))
    print(json.dumps({key: summary[key] for key in ("run_directory", "counts", "bytes", "errors", "changed_during_copy")}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
