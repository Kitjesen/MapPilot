#!/usr/bin/env python3
"""Observe, choose, move and re-observe in Habitat with model-generated objects.

Habitat's official NavMesh authorizes simulator actions. This experiment does
not invoke LingTu's native 3D planner or a quadruped locomotion controller.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from tools.datasets.habitat_identity import association_report, evaluate_masks
from tools.datasets.habitat_objects import camera_transform
from tools.datasets.habitat_scene import build_scene, query_scene
from tools.datasets.habitat_vision import DEFAULT_OBJECT_LABELS, Vision

from decision.frontiers.scorer import FrontierScorer
from decision.frontiers.types import FREE_CELL, OCCUPIED_CELL, UNKNOWN_CELL, angle_diff
from decision.frontiers.vlnav import Candidate, cue_from_box, rank_candidates, unknown_ratio, vl_score
from perception.tracking.instance_tracker import InstanceTracker
from perception.tracking.projection import project_to_3d, transform_point
from runtime.msgs.sensor import CameraIntrinsics


class World:
    def __init__(self, command: list[str], output: Path):
        self.log = (output / "habitat.log").open("w")
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.log, text=True, bufsize=1)

    def call(self, command: str, **values) -> dict:
        self.process.stdin.write(json.dumps({"command": command, **values}) + "\n")
        self.process.stdin.flush()
        while True:
            line = self.process.stdout.readline()
            if not line:
                raise RuntimeError(f"Habitat process ended: {self.process.poll()}; see habitat.log")
            if line.startswith("{"):
                return json.loads(line)

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.write('{"command":"close"}\n')
            self.process.stdin.flush()
            self.process.wait(timeout=20)
        self.log.close()


class ObservedMap:
    """A depth-derived projection for exploration proposals and inspection only."""

    def __init__(self, resolution=0.15, size=300, origin=(-15.0, -15.0)):
        self.resolution, self.origin = resolution, np.asarray(origin)
        self.grid = np.full((size, size), UNKNOWN_CELL, dtype=np.int16)
        self.clouds = []

    def cell(self, xy):
        return np.rint((np.asarray(xy)[:2] - self.origin) / self.resolution).astype(int)[::-1]

    def update(self, depth, intrinsics, transform, floor_z):
        vs, us = np.mgrid[0:depth.shape[0]:12, 0:depth.shape[1]:12]
        d = depth[vs, us]
        valid = np.isfinite(d) & (d > 0.3) & (d < 10)
        us, vs, d = us[valid], vs[valid], d[valid]
        points = np.column_stack(((us-intrinsics.cx)*d/intrinsics.fx,
                                  (vs-intrinsics.cy)*d/intrinsics.fy, d))
        points = points @ transform[:3, :3].T + transform[:3, 3]
        self.clouds.append(points)
        endpoints = np.rint((points[:, :2]-self.origin)/self.resolution).astype(int)
        start = self.cell(transform[:3, 3])[::-1]
        free = np.zeros(self.grid.shape, dtype=np.uint8)
        for end in endpoints:
            cv2.line(free, tuple(start), tuple(end), 1, 1)
        self.grid[(free != 0) & (self.grid != OCCUPIED_CELL)] = FREE_CELL
        obstacles = (points[:, 2] > floor_z+0.15) & (points[:, 2] < floor_z+1.8)
        endpoints = endpoints[obstacles]
        inside = np.all((endpoints >= 0) & (endpoints < len(self.grid)), axis=1)
        endpoints = endpoints[inside]
        self.grid[endpoints[:, 1], endpoints[:, 0]] = OCCUPIED_CELL

    def frontiers(self, position, yaw, cues):
        scorer = FrontierScorer(min_frontier_size=4, max_frontiers=30, cluster_radius_cells=2,
                                tsp_reorder=False)
        scorer.update_costmap(self.grid, self.resolution, *self.origin)
        candidates = []
        for frontier in scorer.extract_frontiers(position):
            xy = frontier.center_world
            row, col = self.cell(xy)
            offset = angle_diff(math.atan2(*(xy-position[:2])[::-1]), yaw)
            candidates.append(Candidate(
                id=f"frontier:{row}:{col}", kind="frontier", position=(*map(float, xy), float(position[2])),
                vl=vl_score(offset, cues, math.pi/2),
                unknown=unknown_ratio(self.grid, (int(row), int(col)), 10),
            ))
        return candidates


def action_to_point(position, yaw, goal):
    delta = np.asarray(goal)[:2] - np.asarray(position)[:2]
    offset = angle_diff(math.atan2(delta[1], delta[0]), yaw)
    return ("left" if offset > 0 else "right") if abs(offset) > math.radians(12) else "forward"


def approach_points(object_position, floor_z):
    return [(float(object_position[0]+radius*math.cos(angle)),
             float(object_position[1]+radius*math.sin(angle)), float(floor_z))
            for radius in (0.9, 1.2) for angle in np.linspace(-math.pi, math.pi, 12, endpoint=False)]


def check_renderer_projection(samples, transform, intrinsics):
    """Compare LingTu projection with the renderer's inverse graphics matrices."""
    legacy = CameraIntrinsics(width=intrinsics.width, height=intrinsics.height,
                              fx=intrinsics.fx, fy=intrinsics.fy,
                              cx=intrinsics.cx+0.5, cy=intrinsics.cy+0.5)
    rows = []
    for sample in samples:
        if not 0.3 < sample["depth"] < 15:
            continue
        reference = np.asarray(sample["renderer_world_point"])[[0, 2, 1]] * [1, -1, 1]
        current = transform_point(project_to_3d(sample["u"], sample["v"], sample["depth"], intrinsics), transform)
        old = transform_point(project_to_3d(sample["u"], sample["v"], sample["depth"], legacy), transform)
        rows.append({"u": sample["u"], "v": sample["v"], "depth": sample["depth"],
                     "error_m": float(np.linalg.norm(current-reference)),
                     "legacy_error_m": float(np.linalg.norm(old-reference))})
    return rows


def has_approached_target(selection, travelled, distance):
    """Require translation and distance reduction after selecting this target."""
    return (selection is not None and travelled-selection["travelled_m"] >= 0.15
            and selection["distance_m"]-distance >= 0.1)


def is_target(label, prompt):
    """Select search candidates without discarding other observed categories."""
    label = label.lower().replace("_", " ").strip()
    prompt = prompt.lower().replace("_", " ").strip()
    bottles = {"bottle", "water bottle"}
    return label == prompt or (label in bottles and prompt in bottles)


def record_object_view(views, observation, obj, current, step):
    """Keep up to three real, spatially or angularly distinct object views."""
    stored = views.setdefault(obj.object_id, [])
    position = np.asarray(current["camera_position"])
    rotation = np.asarray(current["camera_rotation_xyzw"])
    for old in stored:
        distance = np.linalg.norm(position-np.asarray(old["camera_position"]))
        cosine = min(1.0, abs(float(rotation @ np.asarray(old["camera_rotation_xyzw"]))))
        angle = 2*math.acos(cosine)
        if distance < 0.3 and angle < math.radians(20):
            return
    if len(stored) < 3:
        stored.append({"step": step, "rgb": current["rgb"], "label": observation.label,
                       "bbox_xyxy": observation.bbox_xyxy.tolist(),
                       "camera_position": current["camera_position"],
                       "camera_rotation_xyzw": current["camera_rotation_xyzw"],
                       "camera_pose_frame": "habitat_y_up", "confidence": observation.confidence})


def record_reload_observation(event, step, camera_position, detections, matches):
    """Keep looking after an empty frame for the first real checkpoint match."""
    if "next_frame_step" not in event:
        event.update(next_frame_step=step, next_camera_position=camera_position,
                     next_frame_detection_count=len(detections))
    event["nonempty_observation_seen"] |= bool(detections)
    if "matched_frame_step" in event:
        return
    existing = [obj for obj in matches if str(obj.object_id) in event["counts"]]
    if existing:
        event.update(matched_frame_step=step, matched_camera_position=camera_position,
                     matched_existing_ids=[obj.object_id for obj in existing],
                     counts_incremented=all(obj.detection_count == event["counts"][str(obj.object_id)]+1
                                            for obj in existing))


def map_panel(grid, trajectory, objects, goal=None):
    """Render the observed part of the map with world +Y pointing up."""
    observed = np.argwhere(grid.grid != UNKNOWN_CELL)
    points = [observed] if len(observed) else []
    for position in [*trajectory, *(obj["position"] for obj in objects), *([goal] if goal is not None else [])]:
        points.append(grid.cell(position)[None, :])
    bounds = np.concatenate(points)
    low = bounds.min(axis=0)
    high = bounds.max(axis=0)
    span = int(np.max(high-low))+1
    side = min(min(grid.grid.shape), max(24, span+2*max(5, math.ceil(span*0.12))))
    center = (low+high)/2
    start = np.rint(center-side/2).astype(int)
    start = np.maximum(0, np.minimum(start, np.asarray(grid.grid.shape)-side))
    row0, col0 = map(int, start)
    row1, col1 = row0+side, col0+side

    view = np.full((side, side, 3), 30, dtype=np.uint8)
    crop = grid.grid[row0:row1, col0:col1]
    view[crop == FREE_CELL] = [210, 220, 225]
    view[crop == OCCUPIED_CELL] = [80, 85, 95]
    image = Image.fromarray(np.flipud(view)).resize((480, 480), Image.Resampling.NEAREST)
    draw = ImageDraw.Draw(image)
    scale = 480/side

    def pixel(position):
        row, col = grid.cell(position)
        return ((col-col0+0.5)*scale, (row1-row-0.5)*scale)

    path = [pixel(position) for position in trajectory]
    if len(path) > 1:
        draw.line(path, fill=(80, 220, 140), width=4, joint="curve")
    if goal is not None:
        gx, gy = pixel(goal)
        draw.line((gx-7, gy, gx+7, gy), fill=(255, 210, 70), width=3)
        draw.line((gx, gy-7, gx, gy+7), fill=(255, 210, 70), width=3)
    object_font = ImageFont.load_default(size=14)
    label_boxes = [(6, 6, 256, 30)]
    label_boxes.extend((x-7, y-7, x+7, y+7) for x, y in
                       (pixel(obj["position"]) for obj in objects))
    for obj in objects:
        oid = int(obj["id"])
        x, y = pixel(obj["position"])
        color = ((67+oid*89) % 230, (179+oid*43) % 230, 220)
        draw.ellipse((x-6, y-6, x+6, y+6), fill=color, outline="white", width=2)
        label = f'{oid}x{obj["count"]}'
        offsets = ((10, -16), (10, 3), (-48, -16), (-48, 3), (10, -34), (-48, -34))
        for dx, dy in offsets:
            origin = (x+dx, y+dy)
            box = draw.textbbox(origin, label, font=object_font, stroke_width=1)
            inside = box[0] >= 0 and box[1] >= 0 and box[2] < 480 and box[3] < 480
            overlaps = any(not (box[2] < old[0] or box[0] > old[2]
                                or box[3] < old[1] or box[1] > old[3]) for old in label_boxes)
            if inside and not overlaps:
                break
        draw.rectangle((box[0]-2, box[1]-1, box[2]+2, box[3]+1), fill="#15202b")
        draw.text(origin, label, fill=color, font=object_font, stroke_width=1, stroke_fill="white")
        label_boxes.append(box)
    x, y = path[-1]
    draw.ellipse((x-7, y-7, x+7, y+7), fill=(255, 80, 80), outline="white", width=2)
    draw.rectangle((6, 6, 256, 30), fill="#15202b")
    legend = [((80, 220, 140), "path"), ((255, 80, 80), "agent")]
    if goal is not None:
        legend.append(((255, 210, 70), "goal"))
    legend.append(((150, 190, 220), "object id x views"))
    left = 12
    for color, label in legend:
        draw.ellipse((left, 14, left+7, 21), fill=color)
        draw.text((left+10, 10), label, fill="white")
        left += 16+6*len(label)
    draw.text((438, 8), "Y +", fill="white", stroke_width=1, stroke_fill="#15202b")
    draw.text((444, 458), "X +", fill="white", stroke_width=1, stroke_fill="#15202b")
    return image


def panel(rgb, observations, matches, grid, trajectory, objects, goal, status):
    rgb = rgb.copy()
    paired = list(zip([o for o in observations if o.detection is not None], matches))
    for observation, obj in paired:
        color = np.array([(67+obj.object_id*89) % 230, (179+obj.object_id*43) % 230, 220])
        rgb[observation.mask] = (0.65*rgb[observation.mask]+0.35*color).astype(np.uint8)
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image)
    for observation, obj in paired:
        color = ((67+obj.object_id*89) % 230, (179+obj.object_id*43) % 230, 220)
        draw.rectangle(tuple(map(float, observation.bbox_xyxy)), outline=color, width=3)
        draw.text((observation.bbox_xyxy[0], max(0, observation.bbox_xyxy[1]-13)),
                  f"{observation.label} id={obj.object_id} {observation.confidence:.2f}", fill="white",
                  stroke_width=1, stroke_fill="black")
    canvas = Image.new("RGB", (1120, 530), "#15202b")
    canvas.paste(image, (0, 0))
    canvas.paste(map_panel(grid, trajectory, objects, goal), (640, 0))
    ImageDraw.Draw(canvas).text((12, 492), status, fill="white")
    return canvas


def run(args):
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "panels").mkdir(exist_ok=True)
    (args.output / "views").mkdir(exist_ok=True)
    np.random.seed(args.seed)
    intrinsics = CameraIntrinsics(width=640, height=480, fx=320, fy=320, cx=319.5, cy=239.5)
    world = World([str(args.habitat_python), str(Path(__file__).with_name("habitat_world.py")),
                   "--dataset", str(args.dataset), "--navmesh", str(args.navmesh), "--scene", args.scene,
                   "--output", str(args.output), "--seed", str(args.seed)], args.output)
    tracker = InstanceTracker(merge_distance=0.35, stale_timeout=3600,
                              use_hungarian_matching=False, enable_dedup_merge=False)
    observed = ObservedMap()
    records, trajectory, best_views, geometry_checks = [], [], {}, []
    object_views = {}
    identity_records = []
    excluded = set()
    goal, target_id = None, None
    target_selection, reload_event = None, None
    travelled, collisions, actions = 0.0, 0, []
    outcome = "budget_exhausted"
    video_frames = []
    current = world.call("observe")
    object_map = json.loads((args.output/"identity/object-map.json").read_text())
    try:
        with Vision(args.prompt, args.yolo_weights, args.fastsam_weights, device="cpu",
                    clip_weights=args.clip_weights, observation_labels=args.observe_labels) as vision:
            for step in range(args.max_steps):
                rgb = np.asarray(Image.open(args.output/current["rgb"]).convert("RGB"))
                depth = np.load(args.output/current["depth"])
                transform = camera_transform(current["camera_position"], current["camera_rotation_xyzw"])
                position = np.asarray(current["agent_position"])[[0, 2, 1]] * [1, -1, 1]
                yaw = math.atan2(transform[1, 2], transform[0, 2])
                trajectory.append(position.tolist())
                result = vision.infer(rgb, depth, intrinsics, transform)
                previous_features = {k: obj.features.copy() for k, obj in tracker.objects.items()}
                matches = tracker.update(result.detections, camera_pos=transform[:3, 3],
                                         camera_forward=transform[:3, 2], intrinsics_fx=320)
                if reload_event is not None:
                    record_reload_observation(reload_event, step, current["camera_position"],
                                              result.detections, matches)
                projected = [o for o in result.observations if o.detection is not None]
                identity = world.call("identity")
                identity_evidence = evaluate_masks(projected, matches,
                    np.load(args.output/identity["identity"]), valid_ids=[int(k) for k in object_map])
                identity_records.extend({"step": step, **row} for row in identity_evidence)
                target_pairs = [(o, obj) for o, obj in zip(projected, matches)
                                if is_target(o.label, args.prompt)]
                cues = [cue_from_box(tuple(o.bbox_xyxy), o.confidence, fx=intrinsics.fx, cx=intrinsics.cx)
                        for o, _ in target_pairs]
                tracker.record_view(transform[:3, 3], [obj.object_id for obj in matches])
                for observation, obj in zip(projected, matches):
                    record_object_view(object_views, observation, obj, current, step)
                    old = best_views.get(obj.object_id)
                    if old is None or observation.confidence > old["confidence"]:
                        name = f"views/object_{obj.object_id}.png"
                        Image.fromarray(rgb).save(args.output/name)
                        best_views[obj.object_id] = {"rgb": name, "confidence": observation.confidence,
                                                    "camera_position": current["camera_position"],
                                                    "camera_rotation_xyzw": current["camera_rotation_xyzw"],
                                                    "agent_position": position.tolist(), "step": step}
                observed.update(depth, intrinsics, transform, position[2])
                visible = {obj.object_id for _, obj in target_pairs}
                target = tracker.objects.get(target_id)
                if target is not None and target_id in visible:
                    distance = float(np.linalg.norm(target.position[:2]-position[:2]))
                    fixed_distance = float(np.linalg.norm(
                        np.asarray(target_selection["target_position"])[:2]-position[:2]))
                    offset = angle_diff(math.atan2(*(target.position[:2]-position[:2])[::-1]), yaw)
                    if (distance <= 1.35 and abs(offset) < math.radians(20)
                            and has_approached_target(target_selection, travelled, fixed_distance)):
                        outcome = "candidate_reobserved_at_approach"
                        goal = None
                if outcome == "budget_exhausted":
                    instances = [Candidate(id=f"object:{obj.object_id}", kind="instance",
                                            position=tuple(obj.position), confidence=obj.best_score,
                                            vl=vl_score(cue.bearing, cues, math.pi/2))
                                 for (_, obj), cue in zip(target_pairs, cues) if obj.detection_count >= 2]
                    candidates = instances + observed.frontiers(position, yaw, cues)
                    ranked = rank_candidates(candidates, tuple(position), detection_threshold=0.35,
                                             reached_distance=0.4, distance_weight=0.2, vl_weight=1,
                                             unknown_scale=2, excluded_ids=frozenset(excluded))
                    if target_id is None or target is None:
                        if goal is not None and np.linalg.norm(np.asarray(goal)[:2]-position[:2]) < 0.35:
                            goal = None
                        for rank in ranked:
                            proposal = rank.candidate
                            if proposal.kind == "frontier" and goal is not None:
                                break
                            goals = (approach_points(proposal.position, position[2])
                                     if proposal.kind == "instance" else [proposal.position])
                            plans = world.call("paths", goals=goals)["paths"]
                            paths = [p for p in plans if p is not None and p["length"] > 0.25]
                            if proposal.kind == "instance":
                                paths = [p for p in paths if np.linalg.norm(
                                    np.asarray(p["goal"])[:2]-np.asarray(proposal.position)[:2]) <= 1.25]
                            if not paths:
                                excluded.add(proposal.id)
                                continue
                            chosen = min(paths, key=lambda p: p["length"])
                            goal = chosen["goal"]
                            if proposal.kind == "instance":
                                target_id = int(proposal.id.split(":")[1])
                                target_selection = {"id": target_id, "step": step, "travelled_m": travelled,
                                                    "distance_m": rank.distance,
                                                    "target_position": list(proposal.position)}
                            break
                    if goal is not None:
                        path = world.call("paths", goals=[goal])["paths"][0]
                        if path is None:
                            goal, target_id, target_selection = None, None, None
                        else:
                            waypoint = next((p for p in path["points"][1:]
                                             if np.linalg.norm(np.asarray(p)[:2]-position[:2]) > 0.15), goal)
                            if target_id is not None and np.linalg.norm(np.asarray(goal)[:2]-position[:2]) < 0.25:
                                action = action_to_point(position, yaw, tracker.objects[target_id].position)
                                if action == "forward":
                                    excluded.add(f"object:{target_id}")
                                    target_id, goal, target_selection = None, None, None
                                    action = "left"
                            else:
                                action = action_to_point(position, yaw, waypoint)
                    else:
                        action = "left"
                else:
                    action = "stop"
                if step == args.max_steps-1:
                    action, goal = "stop", None
                if step % 10 == 0 or action == "stop":
                    samples = world.call("geometry")["samples"]
                    geometry_checks.append({"step": step,
                                            "samples": check_renderer_projection(samples, transform, intrinsics)})
                record = {"step": step, **current, "position_z_up": position.tolist(), "yaw": yaw,
                          "action": action, "mode": (("verify" if outcome == "candidate_reobserved_at_approach"
                                                       else "budget_end") if action == "stop" else
                                                     "approach" if target_id is not None else
                                                     "explore" if goal is not None else "scan"), "goal": goal,
                          "target_id": target_id, "travelled_m": travelled,
                          "target_selection": target_selection,
                          "identity_evidence": identity_evidence,
                          "objects": [{"id": obj.object_id, "label": obj.label, "position": obj.position.tolist(),
                                       "count": obj.detection_count, "feature_dim": int(obj.features.size),
                                       "feature_norm": float(np.linalg.norm(obj.features))}
                                      for obj in tracker.objects.values()],
                          "detections": [{"id": obj.object_id, "label": o.label,
                                          "confidence": o.confidence, "position": o.detection.position.tolist(),
                                          "bbox_xyxy": o.bbox_xyxy.tolist(),
                                          "features": o.detection.features.tolist(),
                                          "feature_dim": int(o.detection.features.size),
                                          "feature_norm": float(np.linalg.norm(o.detection.features)),
                                          "previous_feature_cosine": (tracker._cosine_similarity(
                                              o.detection.features, previous_features[obj.object_id])
                                              if obj.object_id in previous_features else None),
                                          "quality": vars(o.quality)} for o, obj in zip(projected, matches)],
                          "perception_rejections": [{"label": o.label, "quality": vars(o.quality)}
                                                    for o in result.observations if o.detection is None]}
                records.append(record)
                annotated = panel(rgb, result.observations, matches, observed, trajectory,
                                  record["objects"], goal,
                                  f"{args.prompt} | step {step} | {record['mode']} | {action} | "
                                  f"travel {travelled:.2f} m | objects {len(tracker.objects)}")
                annotated.save(args.output/f"panels/{step:04d}.jpg", quality=92)
                (args.output/"steps.json").write_text(json.dumps(records, indent=2)+"\n")
                print(json.dumps({"step": step, "action": action, "objects": len(tracker.objects),
                                  "travelled_m": travelled, "target_id": target_id}), flush=True)
                if action == "stop":
                    break
                if reload_event is None and tracker.objects and step >= 5:
                    checkpoint = args.output/"checkpoint.json"
                    tracker.save_to_file(str(checkpoint))
                    reloaded = InstanceTracker(merge_distance=0.35, stale_timeout=3600,
                                               use_hungarian_matching=False, enable_dedup_merge=False)
                    if not reloaded.load_from_file(str(checkpoint)):
                        raise RuntimeError("mid-episode checkpoint could not be reloaded")
                    reload_event = {"checkpoint_step": step, "camera_position": current["camera_position"],
                                    "counts": {str(k): obj.detection_count for k, obj in tracker.objects.items()},
                                    "nonempty_observation_seen": False, "matched_existing_ids": None,
                                    "counts_incremented": None}
                    tracker = reloaded
                current = world.call("step", action=action)
                travelled += current["translation_m"]
                collisions += current["collisions"]
                video_frames.extend(f["rgb"] for f in current["video_frames"])
                actions.append(action)
    finally:
        world.close()
    tracker.save_to_file(str(args.output/"tracker.json"))
    restored = InstanceTracker(merge_distance=0.35, stale_timeout=3600,
                               use_hungarian_matching=False, enable_dedup_merge=False)
    loaded = restored.load_from_file(str(args.output/"tracker.json"))
    same = loaded and set(restored.objects) == set(tracker.objects) and all(
        np.allclose(restored.objects[k].points, v.points) and np.allclose(restored.objects[k].position, v.position)
        for k, v in tracker.objects.items())
    same_features = loaded and set(restored.objects) == set(tracker.objects) and all(
        restored.objects[k].features.shape == obj.features.shape
        and np.allclose(restored.objects[k].features, obj.features)
        for k, obj in tracker.objects.items())
    clouds = {f"object_{k}": obj.points for k, obj in tracker.objects.items()}
    clouds["background"] = np.concatenate(observed.clouds)
    np.savez_compressed(args.output/"clouds.npz", **clouds)
    scene = build_scene(tracker, object_views)
    identity_result = association_report(identity_records)
    (args.output/"association.json").write_text(json.dumps(identity_result, indent=2)+"\n")
    (args.output/"scene.json").write_text(json.dumps(scene, indent=2)+"\n")
    (args.output/"object-views.json").write_text(json.dumps(object_views, indent=2)+"\n")
    queries = {text: query_scene(scene, text) for text in args.query}
    (args.output/"queries.json").write_text(json.dumps(queries, ensure_ascii=False, indent=2)+"\n")
    summary = {"scene": args.scene, "seed": args.seed, "prompt": args.prompt, "outcome": outcome,
               "perception_steps": len(records), "actions": len(actions), "travelled_m": travelled,
               "collision_microsteps": collisions, "objects": len(tracker.objects),
               "target_id": target_id, "target_selection": target_selection, "best_views": best_views,
               "reload_object_count": len(tracker.objects),
               "reload_geometry_ok": bool(same) if tracker.objects else None,
               "reload_features_ok": bool(same_features) if tracker.objects else None,
               "image_feature_encoder": "clip", "clip_weights": str(args.clip_weights) if args.clip_weights else "openai",
               "encoded_observations": sum(len(record["detections"]) for record in records),
               "object_feature_dimensions": sorted({int(obj.features.size) for obj in tracker.objects.values()}),
               "observed_categories": sorted({obj.label for obj in tracker.objects.values()}),
               "relation_count": len(scene["relations"]),
               "objects_with_multiple_views": sum(len(views) > 1 for views in object_views.values()),
               "observation_prompt": vision.prompt,
               "identity_evaluation": identity_result,
               "instance_oracle_used_for_evaluation": True,
               "instance_oracle_used_for_tracking": False,
               "object_feature_norms": {str(k): float(np.linalg.norm(obj.features))
                                        for k, obj in tracker.objects.items()},
               "reload_during_episode": reload_event,
               "resumed_ids_ok": (bool(reload_event["matched_existing_ids"])
                                  if reload_event and reload_event["nonempty_observation_seen"] else None),
               "resumed_counts_ok": reload_event.get("counts_incremented") if reload_event else None,
               "dataset_semantic_labels_used": False, "habitat_navmesh_used": True,
               "native_planner_used": False, "quadruped_dynamics_used": False,
               "simulator_pose_used": True, "llm_executed": False, "video_frames": video_frames,
               "frame_id": "habitat_z_up", "camera_intrinsics": vars(intrinsics),
               "projection_reference": "Habitat inverse graphics matrices at raster pixel centers",
               "projection_max_error_m": max((s["error_m"] for c in geometry_checks for s in c["samples"]), default=None),
               "legacy_projection_max_error_m": max((s["legacy_error_m"] for c in geometry_checks
                                                     for s in c["samples"]), default=None)}
    if target_id is not None:
        target = tracker.objects[target_id]
        summary["final_target_horizontal_distance_m"] = float(np.linalg.norm(target.position[:2]-position[:2]))
        summary["target_observations"] = target.detection_count
        summary["target_approach_travel_m"] = travelled-target_selection["travelled_m"]
        summary["final_fixed_target_distance_m"] = float(np.linalg.norm(
            np.asarray(target_selection["target_position"])[:2]-position[:2]))
        summary["target_distance_reduction_m"] = target_selection["distance_m"]-summary["final_fixed_target_distance_m"]
    (args.output/"geometry-checks.json").write_text(json.dumps(geometry_checks, indent=2)+"\n")
    (args.output/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    video(args.output, video_frames, "first_person.mp4", (640, 480), fps=12)
    video(args.output, [f"panels/{i:04d}.jpg" for i in range(len(records))],
          "search.mp4", (1120, 530), fps=3)
    print(json.dumps({k: v for k, v in summary.items() if k not in {"video_frames", "best_views"}}), flush=True)
    return summary


def video(output, frames, name, size, fps):
    writer = cv2.VideoWriter(str(output/name), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    if not writer.isOpened():
        raise RuntimeError("video writer could not open")
    try:
        for name in frames:
            writer.write(cv2.cvtColor(np.asarray(Image.open(output/name).convert("RGB")), cv2.COLOR_RGB2BGR))
    finally:
        writer.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("dataset", "navmesh", "habitat-python", "yolo-weights", "fastsam-weights", "output"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--scene", default="apt_0")
    parser.add_argument("--prompt", default="chair")
    parser.add_argument("--observe-labels", nargs="*", default=DEFAULT_OBJECT_LABELS,
                        help="Context object labels; pass without values for a target-only comparison")
    parser.add_argument("--clip-weights", type=Path, help="Local OpenCLIP ViT-B/32 state-dict checkpoint")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-steps", type=int, default=120)
    parser.add_argument("--query", action="append", default=[], help="Ground a relation query in the observed scene")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
