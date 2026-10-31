#!/usr/bin/env python3
"""Segment recorded detections, then project and fuse them using LingTu tracking."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image


def camera_transform(position, rotation_xyzw) -> np.ndarray:
    """OpenCV optical camera -> right-handed Habitat Z-up experiment frame.

    Habitat world [x,y,z] maps to [x,-z,y]. This is NOT the older dataset
    adapter's reflected [x,z,y] frame, nor an activated native navigation map.
    """
    q = np.asarray(rotation_xyzw, dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    world_axes = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    optical_to_gl = np.diag([1, -1, -1])
    transform = np.eye(4)
    transform[:3, :3] = world_axes @ rotation @ optical_to_gl
    transform[:3, 3] = world_axes @ np.asarray(position)
    return transform


def segment(recording: Path, cues: Path, output: Path, weights: Path) -> None:
    import torch
    from ultralytics import FastSAM

    torch.set_num_threads(4)
    model = FastSAM(str(weights))
    report = json.loads(cues.read_text())
    metadata = json.loads((recording / "report.json").read_text())
    sources = {f["index"]: f for f in metadata["frames"]}
    output.mkdir(parents=True, exist_ok=True)
    frames = []
    for frame in report["frames"]:
        rows, masks = [], {}
        if frame["detections"]:
            image = Image.open(recording / sources[frame["index"]]["rgb"]).convert("RGB")
            results = model.predict(image, device="cpu", retina_masks=True, imgsz=640,
                                    conf=0.25, iou=0.9, verbose=False)
            all_boxes = results[0].boxes.xyxy.cpu().numpy()
            for i, detection in enumerate(frame["detections"]):
                selected = model.predictor.prompt(results, bboxes=[detection["bbox_xyxy"]])[0]
                if selected.masks is None or not len(selected):
                    rows.append({**detection, "status": "no_mask"})
                    continue
                mask = selected.masks.data[0].cpu().numpy().astype(bool)
                source_box = selected.boxes.xyxy[0].cpu().numpy()
                mask_id = int(np.flatnonzero(np.all(all_boxes == source_box, axis=1))[0])
                key = f"d{i}"
                masks[key] = mask
                box = np.asarray(detection["bbox_xyxy"], dtype=int)
                x1, y1 = np.maximum(box[:2], 0)
                x2, y2 = np.minimum(box[2:], [mask.shape[1], mask.shape[0]])
                intersection = int(mask[y1:y2, x1:x2].sum())
                union = int(mask.sum()) + (x2-x1)*(y2-y1) - intersection
                rows.append({**detection, "status": "segmented", "mask_key": key,
                             "source_mask_id": mask_id, "mask_pixels": int(mask.sum()),
                             "mask_box_iou": intersection / max(1, union)})
        name = f'{frame["index"]:02d}.npz'
        np.savez_compressed(output / name, **masks)
        frames.append({"index": frame["index"], "mask_file": name, "detections": rows})
        print(json.dumps({"frame": frame["index"], "masks": len(masks)}), flush=True)
    (output / "masks.json").write_text(json.dumps({
        "scene": metadata["scene"], "prompt": report["prompt"], "weights": str(weights),
        "model": "FastSAM-s", "frames": frames, "dataset_labels_used": False,
    }, indent=2) + "\n")


def build(recording: Path, masks_path: Path, output: Path, merge_distance: float = 0.35) -> dict:
    from perception.tracking.instance_tracker import InstanceTracker
    from perception.tracking.projection import Detection3D, mask_to_pointcloud
    from runtime.msgs.sensor import CameraIntrinsics

    np.random.seed(7)
    metadata = json.loads((recording / "report.json").read_text())
    segmented = json.loads(masks_path.read_text())
    if metadata["scene"] != segmented["scene"]:
        raise ValueError("recording and masks must belong to the same scene")
    camera = metadata["camera"]
    intrinsics = CameraIntrinsics(**{k: camera[k] for k in ("fx", "fy", "cx", "cy", "width", "height")})
    tracker = InstanceTracker(merge_distance=merge_distance, stale_timeout=3600,
                              use_hungarian_matching=False, enable_dedup_merge=False)
    output.mkdir(parents=True, exist_ok=True)
    sources = {f["index"]: f for f in metadata["frames"]}
    observations, evidence, clouds = [], {}, {}
    background = []
    for frame in segmented["frames"]:
        source = sources[frame["index"]]
        depth = np.load(recording / source["depth"])
        transform = camera_transform(source["camera_position"], source["camera_rotation_xyzw"])
        context = mask_to_pointcloud(np.ones(depth.shape, dtype=bool), depth, intrinsics, transform,
                                    depth_scale=1.0, max_depth=15, max_points=3000)
        if context is not None:
            background.append(context)
        detections, records = [], []
        # A segmentation region may be selected by multiple overlapping YOLO boxes.
        seen_masks = set()
        with np.load(masks_path.parent / frame["mask_file"]) as masks:
            for detection in sorted(frame["detections"], key=lambda d: -d["confidence"]):
                row = {**detection, "frame": frame["index"]}
                observations.append(row)
                if detection["status"] != "segmented":
                    continue
                if detection["source_mask_id"] in seen_masks:
                    row["status"] = "duplicate_mask"
                    continue
                seen_masks.add(detection["source_mask_id"])
                mask = masks[detection["mask_key"]]
                points = mask_to_pointcloud(mask, depth, intrinsics, transform, depth_scale=1.0,
                                            max_depth=15, max_points=512)
                if points is None:
                    row["status"] = "insufficient_depth"
                    continue
                position = points.mean(axis=0)
                valid = mask & np.isfinite(depth) & (depth > 0.3) & (depth < 15)
                detections.append(Detection3D(
                    position=position, label=detection["label"], score=detection["confidence"],
                    bbox_2d=np.asarray(detection["bbox_xyxy"]), depth=float(np.median(depth[valid])),
                    features=np.array([]), points=points,
                ))
                row.update(status="projected", position=position.tolist(), point_count=len(points))
                records.append(row)
        matched = tracker.update(detections, camera_pos=transform[:3, 3],
                                 camera_forward=transform[:3, 2], intrinsics_fx=intrinsics.fx)
        rgb = np.asarray(Image.open(recording / source["rgb"]).convert("RGB")).copy()
        from PIL import ImageDraw

        for row, obj in zip(records, matched):
            row["object_id"] = obj.object_id
            color = np.array([(67 + obj.object_id*89) % 230, (179 + obj.object_id*43) % 230, 220])
            with np.load(masks_path.parent / frame["mask_file"]) as masks:
                mask = masks[row["mask_key"]]
            rgb[mask] = (0.55*rgb[mask] + 0.45*color).astype(np.uint8)
            best = evidence.get(obj.object_id)
            if best is None or row["confidence"] > best["confidence"]:
                evidence[obj.object_id] = {"frame": frame["index"], "rgb": source["rgb"],
                    "bbox_xyxy": row["bbox_xyxy"], "confidence": row["confidence"],
                    "camera_world_frame": "habitat_y_up",
                    "camera_position": source["camera_position"],
                    "camera_rotation_xyzw": source["camera_rotation_xyzw"],
                    "optical_to_habitat_z_up": transform.tolist()}
        panel = Image.fromarray(rgb)
        draw = ImageDraw.Draw(panel)
        for row in records:
            box = row["bbox_xyxy"]
            draw.rectangle(box, outline="white", width=2)
            draw.text((box[0], max(0, box[1]-13)), f'object {row["object_id"]}',
                      fill="white", stroke_width=1, stroke_fill="black")
        panel.save(output / f'{frame["index"]:02d}.jpg')
    objects = []
    (output / "views").mkdir(exist_ok=True)
    for oid, obj in tracker.objects.items():
        clouds[f"object_{oid}"] = obj.points
        best = evidence[oid]
        source_rgb = recording / best["rgb"]
        best["source_rgb"] = str(source_rgb.resolve())
        best["rgb"] = f"views/object_{oid}.png"
        Image.open(source_rgb).convert("RGB").save(output / best["rgb"])
        seen = [row for row in observations if row.get("object_id") == oid]
        positions = np.asarray([row["position"] for row in seen])
        spread = float(np.max(np.linalg.norm(positions[:, None] - positions[None, :], axis=-1)))
        objects.append({"id": oid, "label": obj.label, "position": obj.position.tolist(),
                        "observations": obj.detection_count, "best_view": evidence[oid],
                        "observation_frames": [row["frame"] for row in seen],
                        "max_observed_centroid_separation_m": spread,
                        "point_count": len(obj.points)})
    clouds["background"] = np.concatenate(background) if background else np.empty((0, 3))
    np.savez_compressed(output / "clouds.npz", **clouds)
    tracker.save_to_file(str(output / "tracker.json"))
    restored = InstanceTracker()
    loaded = restored.load_from_file(str(output / "tracker.json"))
    reload_ok = loaded and set(restored.objects) == set(tracker.objects) and all(
        np.allclose(restored.objects[k].position, v.position) for k, v in tracker.objects.items())
    reload_clouds_ok = reload_ok and all(
        restored.objects[k].points.shape == v.points.shape
        and np.allclose(restored.objects[k].points, v.points) for k, v in tracker.objects.items())
    report = {"scene": metadata["scene"], "frame_id": "habitat_z_up",
              "world_axes": "Habitat [x,y,z] -> [x,-z,y]", "prompt": segmented["prompt"],
              "merge_distance_m": merge_distance, "objects": objects, "observations": observations,
              "reload_ids_positions_ok": bool(reload_ok), "source": "YOLO-World + FastSAM + recorded RGB-D",
              "reload_clouds_ok": bool(reload_clouds_ok),
              "geometry_file": "clouds.npz", "tracking_state_file": "tracker.json",
              "limitations": ["single-position yaw views; no translation or SLAM error evaluated",
                              "predicted masks may include background; no semantic ground-truth evaluation",
                              "tracker reload preserves IDs, positions and clouds; resumed tracking is not evaluated"],
              "dataset_labels_used": False, "robot_motion_executed": False,
              "native_map_binding": False, "llm_executed": False}
    (output / "objects.json").write_text(json.dumps(report, indent=2) + "\n")
    plot_clouds(clouds, objects, output / "map.png")
    return report


def plot_clouds(clouds: dict, objects: list[dict], output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 8))
    background = clouds["background"]
    ax.scatter(background[:, 0], background[:, 1], s=0.4, c="#cccccc")
    for obj in objects:
        points = clouds[f'object_{obj["id"]}']
        color = np.array([(67 + obj["id"]*89) % 230, (179 + obj["id"]*43) % 230, 220]) / 255
        ax.scatter(points[:, 0], points[:, 1], s=3, color=color,
                   label=f'object {obj["id"]}: {obj["observations"]} observations')
        ax.annotate(str(obj["id"]), obj["position"][:2])
    ax.set(xlabel="Habitat Z-up X (m)", ylabel="Habitat Z-up Y (m)", aspect="equal",
           title="Observed object candidates: YOLO-World + FastSAM + RGB-D\nNo dataset labels; no robot motion")
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection="3d")
    ax.scatter(*background[::3].T, s=0.3, c="#aaaaaa", alpha=0.25)
    for obj in objects:
        points = clouds[f'object_{obj["id"]}']
        color = np.array([(67 + obj["id"]*89) % 230, (179 + obj["id"]*43) % 230, 220]) / 255
        ax.scatter(*points.T, s=2, color=color)
        ax.text(*obj["position"], str(obj["id"]))
    ax.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)",
           title=f"RGB-D geometry and {len(objects)} tracked candidates\nHabitat Z-up experiment frame; no motion")
    ax.set_box_aspect(np.ptp(background, axis=0))
    fig.tight_layout()
    fig.savefig(output.with_name("map3d.png"), dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    masks = commands.add_parser("segment")
    masks.add_argument("recording", type=Path)
    masks.add_argument("--cues", type=Path, required=True)
    masks.add_argument("--output", type=Path, required=True)
    masks.add_argument("--weights", type=Path, required=True)
    fusion = commands.add_parser("build")
    fusion.add_argument("recording", type=Path)
    fusion.add_argument("--masks", type=Path, required=True)
    fusion.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "segment":
        segment(args.recording, args.cues, args.output, args.weights)
    else:
        result = build(args.recording, args.masks, args.output)
        print(json.dumps({"objects": len(result["objects"]), "reload_ok": result["reload_ids_positions_ok"]}))


if __name__ == "__main__":
    main()
