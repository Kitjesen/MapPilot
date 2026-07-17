#!/usr/bin/env python3
"""Export a standalone 3D Canvas viewer for MuJoCo OctoPlanner3D artifacts."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


def _resolve_path(report_path: Path, value: Any) -> Path:
    raw = str(value or "")
    if not raw:
        return Path("")
    path = Path(raw)
    if path.is_absolute():
        return path
    for candidate in (report_path.parent / path, Path.cwd() / path):
        if candidate.exists():
            return candidate
    return Path.cwd() / path


def _load_xyz(path: Path, limit: int) -> list[list[float]]:
    points: list[list[float]] = []
    if not path.is_file():
        return points
    for line in path.read_text(encoding="ascii", errors="ignore").splitlines():
        parts = line.strip().split()
        if len(parts) < 3:
            continue
        try:
            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
        except ValueError:
            continue
        if 0 < limit <= len(points):
            break
    return points


def _load_pcd_xyz(path: Path, limit: int) -> list[list[float]]:
    points: list[list[float]] = []
    if not path.is_file():
        return points
    data = False
    for line in path.read_text(encoding="ascii", errors="ignore").splitlines():
        stripped = line.strip()
        if not data:
            if stripped.upper().startswith("DATA"):
                data = True
            continue
        parts = stripped.split()
        if len(parts) < 3:
            continue
        try:
            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
        except ValueError:
            continue
        if 0 < limit <= len(points):
            break
    return points


def _load_trajectory(path: Path) -> list[list[float]]:
    if not path.is_file():
        return []
    with path.open("r", encoding="utf-8", newline="") as fh:
        rows = csv.DictReader(fh)
        out: list[list[float]] = []
        for row in rows:
            if not row.get("x") or not row.get("y"):
                continue
            out.append([float(row["x"]), float(row["y"]), float(row.get("z") or 0.0)])
        return out


def _load_local_records(path: Path, limit: int) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    records: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        records.append(
            {
                "t": record.get("t"),
                "robot": [record.get("x"), record.get("y"), record.get("z")],
                "target": record.get("target"),
                "local_path": record.get("local_path_map") or [],
                "person": record.get("person"),
            }
        )
        if 0 < limit <= len(records):
            break
    return records


def _artifact_path(report_path: Path, report: dict[str, Any], key: str) -> Path:
    artifacts = report.get("artifacts") if isinstance(report.get("artifacts"), dict) else {}
    return _resolve_path(report_path, artifacts.get(key))


def _support_projection_path(
    path: list[Any], occupied: list[list[float]], *, radius_m: float = 0.28
) -> list[list[float]]:
    projected: list[list[float]] = []
    if not path or not occupied:
        return projected
    radius2 = radius_m * radius_m
    for raw in path:
        if not raw or len(raw) < 3:
            continue
        try:
            x = float(raw[0])
            y = float(raw[1])
            z = float(raw[2])
        except (TypeError, ValueError):
            continue
        best_z: float | None = None
        for point in occupied:
            dx = float(point[0]) - x
            dy = float(point[1]) - y
            pz = float(point[2])
            if dx * dx + dy * dy > radius2 or pz > z:
                continue
            if best_z is None or pz > best_z:
                best_z = pz
        if best_z is not None:
            projected.append([x, y, best_z])
    return projected


def _extract_payload(report_path: Path, report: dict[str, Any], point_limit: int, frame_limit: int) -> dict[str, Any]:
    plan = report.get("plan") if isinstance(report.get("plan"), dict) else {}
    plan_report = report.get("plan_report") if isinstance(report.get("plan_report"), dict) else {}
    raw_plan = plan_report.get("plan") if isinstance(plan_report.get("plan"), dict) else {}
    global_path = raw_plan.get("path") or plan.get("path") or []

    occupied_path = _artifact_path(report_path, report, "path_debug_map_points")
    occupied = _load_xyz(occupied_path, point_limit)
    if not occupied:
        occupied = _load_pcd_xyz(_resolve_path(report_path, report.get("map_pcd")), point_limit)

    local_records = _load_local_records(_artifact_path(report_path, report, "local_path_jsonl"), frame_limit)
    trajectory = _load_trajectory(_artifact_path(report_path, report, "trajectory_csv"))
    person_trace = [record["person"] for record in local_records if record.get("person")]
    acceptance = report.get("acceptance") if isinstance(report.get("acceptance"), dict) else {}
    scene_acceptance = report.get("scene_acceptance") if isinstance(report.get("scene_acceptance"), dict) else {}
    diagnostics = plan.get("diagnostics") if isinstance(plan.get("diagnostics"), dict) else {}
    constraints = diagnostics.get("constraints") if isinstance(diagnostics.get("constraints"), dict) else {}
    return {
        "meta": {
            "scene_preset": report.get("scene_preset"),
            "planner": diagnostics.get("planner") or "octoplanner3d",
            "search_algorithm": diagnostics.get("search_algorithm"),
            "octomap": diagnostics.get("runtime_map_path") or str(report.get("map_dir") or ""),
            "occupied_points": len(occupied),
            "global_path_points": len(global_path),
            "local_frames": len(local_records),
            "trajectory_points": len(trajectory),
            "max_step_height": constraints.get("max_step_height"),
            "robot_radius": constraints.get("robot_radius"),
            "product_ready": acceptance.get("product_ready"),
            "scene_acceptance_ok": scene_acceptance.get("ok"),
            "blockers": acceptance.get("product_blockers") or scene_acceptance.get("blockers") or [],
        },
        "occupied": occupied,
        "global_path": global_path,
        "global_support_path": _support_projection_path(global_path, occupied),
        "trajectory": trajectory,
        "local_records": local_records,
        "person_trace": person_trace,
        "start": plan_report.get("start") or report.get("start"),
        "goal": plan_report.get("goal") or report.get("goal"),
    }


def _html(payload: dict[str, Any]) -> str:
    data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>LingTu OctoPlanner3D 3D Voxel Viewer</title>
<style>
html,body{{margin:0;height:100%;overflow:hidden;background:#070b12;color:#e5e7eb;font-family:Inter,Segoe UI,Arial,sans-serif}}
#app{{display:grid;grid-template-columns:minmax(760px,1fr)380px;height:100vh}}
#stage{{position:relative;background:#080d18}}canvas{{display:block;width:100%;height:100%}}
.hud{{position:absolute;left:16px;top:14px;padding:12px 14px;background:rgba(7,11,20,.80);border:1px solid #334155;border-radius:8px;max-width:880px}}
.hud h1{{font-size:18px;margin:0 0 6px}}.hud p{{margin:0;font-size:13px;color:#cbd5e1;line-height:1.45}}
aside{{overflow:auto;background:#111827;border-left:1px solid #263244;padding:16px}}
.card{{background:#0f172a;border:1px solid #293548;border-radius:8px;padding:12px;margin-bottom:12px}}
.bad{{background:#7f1d1d;color:#fecaca;border-color:#ef4444}}.ok{{background:#052e1b;color:#bbf7d0;border-color:#16a34a}}
.kv{{display:grid;grid-template-columns:145px 1fr;gap:6px 10px;font-size:12px}}.kv div:nth-child(odd){{color:#94a3b8}}
input[type=range]{{width:100%}}button{{background:#1e293b;color:#e5e7eb;border:1px solid #475569;border-radius:6px;padding:7px 10px;margin-right:6px;cursor:pointer}}
label{{display:block;margin:8px 0;font-size:13px}}.sw{{display:inline-block;width:20px;height:5px;border-radius:99px;margin-right:8px;vertical-align:middle}}.dot{{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:8px}}
</style>
</head>
<body>
<div id="app">
<div id="stage"><canvas id="view"></canvas><div class="hud"><h1>LingTu OctoPlanner3D - 真实三维体素图</h1><p>灰色是规划地图点/occupied leaf centers；橙色是三维 global path；绿色是当前 local path；红色是执行轨迹；紫色是动态行人轨迹。拖拽旋转，滚轮缩放，Shift+拖拽平移。</p></div></div>
<aside>
<div id="status" class="card"></div>
<div class="card">
<b>Frame</b> <span id="frameText"></span><input id="frame" type="range" min="0" max="0" value="0"/>
<b>Z clip</b> <span id="zText"></span><input id="zClip" type="range" min="0" max="100" value="100"/>
<b>Z exaggeration</b> <span id="zScaleText"></span><input id="zScale" type="range" min="80" max="320" value="160"/>
<div style="margin-top:10px"><button id="play">Play</button><button id="reset">Reset</button><button id="top">Top</button></div>
<label><input id="occ" type="checkbox" checked/> OctoMap/map points</label>
<label><input id="global" type="checkbox" checked/> OctoPlanner3D global path</label>
<label><input id="local" type="checkbox" checked/> 当前 local path</label>
<label><input id="trail" type="checkbox" checked/> robot trail</label>
<label><input id="person" type="checkbox" checked/> walking person trail</label>
<label><input id="grid" type="checkbox" checked/> XYZ axes/grid</label>
</div>
<div class="card"><b>Legend</b><p><span class="dot" style="background:#cbd5e1"></span>map voxels/points</p><p><span class="sw" style="background:#f97316"></span>global path</p><p><span class="sw" style="background:#22c55e"></span>local path</p><p><span class="sw" style="background:#ef4444"></span>executed trail</p><p><span class="sw" style="background:#a855f7"></span>walking person</p><p><span class="dot" style="background:#10b981"></span>start</p><p><span class="dot" style="background:#f43f5e"></span>goal</p></div>
<div class="card"><b>Metadata</b><div class="kv" id="meta"></div></div>
<div class="card"><b>Current Frame</b><div class="kv" id="cur"></div></div>
</aside>
</div>
<script id="payload" type="application/json">{data}</script>
<script>
const data=JSON.parse(document.getElementById('payload').textContent);
const canvas=document.getElementById('view'),ctx=canvas.getContext('2d');
const ids=['frame','zClip','zScale','play','reset','top','occ','global','local','trail','person','grid'];
const els=Object.fromEntries(ids.map(id=>[id,document.getElementById(id)]));
let w=0,h=0,frame=0,playing=false,cam={{yaw:-0.72,pitch:-0.55,zoom:58,panX:0,panY:0}};
els.frame.max=Math.max(0,data.local_records.length-1);
function resize(){{w=canvas.clientWidth;h=canvas.clientHeight;canvas.width=w*devicePixelRatio;canvas.height=h*devicePixelRatio;ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);draw();}}
window.addEventListener('resize',resize);
function bounds(points){{let xs=[],ys=[],zs=[];for(const p of points){{if(!p)continue;xs.push(p[0]);ys.push(p[1]);zs.push(p[2]||0);}}if(xs.length===0)return {{cx:0,cy:0,cz:0,zmax:1}};return {{cx:(Math.min(...xs)+Math.max(...xs))/2,cy:(Math.min(...ys)+Math.max(...ys))/2,cz:(Math.min(...zs)+Math.max(...zs))/2,zmax:Math.max(...zs)}}}}
const B=bounds([...data.occupied,...data.global_path,...data.trajectory]);
function project(p){{const zScale=Number(els.zScale.value)/100;let x=p[0]-B.cx,y=p[1]-B.cy,z=((p[2]||0)-B.cz)*zScale;let cy=Math.cos(cam.yaw),sy=Math.sin(cam.yaw),cp=Math.cos(cam.pitch),sp=Math.sin(cam.pitch);let x1=x*cy-y*sy,y1=x*sy+y*cy,z1=z;let y2=y1*cp-z1*sp,z2=y1*sp+z1*cp;let s=cam.zoom/(1+z2*0.015);return [w/2+cam.panX+x1*s,h/2+cam.panY-y2*s,z2];}}
function line(points,color,width=3){{if(!points||points.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();let a=project(points[0]);ctx.moveTo(a[0],a[1]);for(let i=1;i<points.length;i++){{let p=project(points[i]);ctx.lineTo(p[0],p[1]);}}ctx.stroke();}}
function dot(p,r,color){{if(!p)return;let q=project(p);ctx.fillStyle=color;ctx.beginPath();ctx.arc(q[0],q[1],r,0,Math.PI*2);ctx.fill();}}
function drawGrid(){{for(let x=-2;x<=14;x++)line([[x,-5,0],[x,5,0]],'rgba(80,100,130,.24)',1);for(let y=-5;y<=5;y++)line([[-2,y,0],[14,y,0]],'rgba(80,100,130,.24)',1);line([[B.cx,B.cy,0],[B.cx+1.2,B.cy,0]],'#ef4444',3);line([[B.cx,B.cy,0],[B.cx,B.cy+1.2,0]],'#22c55e',3);line([[B.cx,B.cy,0],[B.cx,B.cy,1.2]],'#60a5fa',3);}}
function draw(){{ctx.clearRect(0,0,w,h);ctx.fillStyle='#070b12';ctx.fillRect(0,0,w,h);const zClip=(Number(els.zClip.value)/100)*(B.zmax+0.5);if(els.grid.checked)drawGrid();if(els.occ.checked){{let pts=data.occupied.filter(p=>(p[2]||0)<=zClip).map(p=>[p,project(p)[2]]).sort((a,b)=>a[1]-b[1]);for(const [p,z] of pts){{let q=project(p);let shade=Math.max(90,Math.min(230,158+z*9));ctx.fillStyle=`rgb(${{shade}},${{shade}},${{shade}})`;ctx.fillRect(q[0]-1,q[1]-1,2.4,2.4);}}}}if(els.global.checked)line(data.global_path,'#f97316',4);let rec=data.local_records[frame]||{{}};if(els.local.checked)line(rec.local_path||[],'#22c55e',4);if(els.trail.checked)line(data.trajectory.slice(0,Math.min(data.trajectory.length,frame*5+1)),'#ef4444',3);if(els.person.checked)line(data.person_trace.slice(0,Math.min(data.person_trace.length,frame*5+1)),'#a855f7',3);dot(data.start,6,'#10b981');dot(data.goal,8,'#f43f5e');dot(rec.robot,7,'#3b82f6');dot(rec.person,7,'#a855f7');document.getElementById('frameText').textContent=`${{frame}}/${{Math.max(0,data.local_records.length-1)}}`;document.getElementById('zText').textContent=`<=${{zClip.toFixed(2)}}m`;document.getElementById('zScaleText').textContent=`x${{(Number(els.zScale.value)/100).toFixed(2)}}`;renderKv('cur',{{t:rec.t,robot:rec.robot,target:rec.target,local_points:(rec.local_path||[]).length,person:rec.person}});}}
function renderKv(id,obj){{const el=document.getElementById(id);el.innerHTML='';for(const [k,v] of Object.entries(obj||{{}})){{let a=document.createElement('div'),b=document.createElement('div');a.textContent=k;b.textContent=Array.isArray(v)?v.map(x=>Number.isFinite(Number(x))?Number(x).toFixed(2):x).join(', '):String(v);el.append(a,b);}}}}
renderKv('meta',data.meta);
const blockers=data.meta.blockers||[];
const ready=data.meta.product_ready===true||data.meta.scene_acceptance_ok===true;
document.getElementById('status').className='card '+(ready?'ok':'bad');
document.getElementById('status').textContent=ready?'Viewer data loaded':'Not accepted / blockers: '+blockers.join('; ');
for(const id of ['frame','zClip','zScale','occ','global','local','trail','person','grid'])els[id].addEventListener('input',()=>{{frame=Number(els.frame.value);draw();}});
els.play.onclick=()=>{{playing=!playing;els.play.textContent=playing?'Pause':'Play';}};
els.reset.onclick=()=>{{cam={{yaw:-0.72,pitch:-0.55,zoom:58,panX:0,panY:0}};draw();}};
els.top.onclick=()=>{{cam.pitch=-Math.PI/2;cam.yaw=0;draw();}};
let drag=null;canvas.onmousedown=e=>{{drag={{x:e.clientX,y:e.clientY,shift:e.shiftKey}}}};canvas.onmouseup=()=>drag=null;canvas.onmousemove=e=>{{if(!drag)return;let dx=e.clientX-drag.x,dy=e.clientY-drag.y;drag.x=e.clientX;drag.y=e.clientY;if(drag.shift){{cam.panX+=dx;cam.panY+=dy;}}else{{cam.yaw+=dx*0.008;cam.pitch=Math.max(-1.45,Math.min(-0.05,cam.pitch+dy*0.008));}}draw();}};
canvas.onwheel=e=>{{e.preventDefault();cam.zoom*=Math.exp(-e.deltaY*0.001);draw();}};
setInterval(()=>{{if(!playing)return;frame=(frame+1)%Math.max(1,data.local_records.length);els.frame.value=frame;draw();}},50);
resize();
</script>
</body>
</html>
"""


def export_viewer(report_path: Path, output: Path, *, point_limit: int, frame_limit: int) -> None:
    report = json.loads(report_path.read_text(encoding="utf-8"))
    payload = _extract_payload(report_path, report, point_limit, frame_limit)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(_html(payload), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--point-limit", type=int, default=30000)
    parser.add_argument("--frame-limit", type=int, default=2400)
    args = parser.parse_args()
    output = args.output or args.report.with_name("octoplanner3d_map_viewer.html")
    export_viewer(args.report, output, point_limit=args.point_limit, frame_limit=args.frame_limit)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
