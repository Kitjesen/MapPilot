"""Coordinate and depth contracts for the recorded Habitat object experiment."""

import json
import math

import numpy as np
from PIL import Image
from tools.datasets.habitat_objects import camera_transform


def test_fusion_keeps_best_image_and_saved_geometry(tmp_path, monkeypatch):
    from tools.datasets import habitat_objects

    recording, masks, output = (tmp_path / name for name in ("recording", "masks", "output"))
    recording.mkdir()
    masks.mkdir()
    frames, detections = [], []
    mask = np.zeros((10, 10), dtype=bool)
    mask[3:7, 3:7] = True
    for i, confidence in enumerate([0.6, 0.8]):
        Image.new("RGB", (10, 10), (20 + i, 40, 60)).save(recording / f"{i}.png")
        np.save(recording / f"{i}.npy", np.full((10, 10), 2.0))
        np.savez(masks / f"{i}.npz", d0=mask)
        frames.append({"index": i, "rgb": f"{i}.png", "depth": f"{i}.npy",
                       "camera_position": [0, 1, 0], "camera_rotation_xyzw": [0, 0, 0, 1]})
        detections.append({"index": i, "mask_file": f"{i}.npz", "detections": [{
            "label": "chair", "confidence": confidence, "bbox_xyxy": [3, 3, 7, 7],
            "status": "segmented", "source_mask_id": 0, "mask_key": "d0"}]})
    (recording / "report.json").write_text(json.dumps({"scene": "fixture", "frames": frames,
        "camera": {"fx": 100, "fy": 100, "cx": 5, "cy": 5, "width": 10, "height": 10}}))
    (masks / "masks.json").write_text(json.dumps({"scene": "fixture", "prompt": "chair", "frames": detections}))
    monkeypatch.setattr(habitat_objects, "plot_clouds", lambda *args: None)
    result = habitat_objects.build(recording, masks / "masks.json", output)
    assert len(result["objects"]) == 1
    obj = result["objects"][0]
    assert obj["observations"] == 2
    assert obj["best_view"]["frame"] == 1
    assert obj["best_view"]["confidence"] == 0.8
    assert Image.open(output / obj["best_view"]["rgb"]).getpixel((0, 0)) == (21, 40, 60)
    with np.load(output / "clouds.npz") as clouds:
        assert len(clouds["object_0"]) > 0
    assert result["reload_ids_positions_ok"]
    assert result["reload_clouds_ok"]
    assert result["native_map_binding"] is False

from perception.tracking.projection import mask_to_pointcloud, project_to_3d, transform_point
from runtime.msgs.sensor import CameraIntrinsics


def test_optical_axes_and_habitat_world_conversion_preserve_handedness():
    transform = camera_transform([1, 2, 3], [0, 0, 0, 1])
    np.testing.assert_allclose(transform[:3, 3], [1, -3, 2])
    np.testing.assert_allclose(transform_point(np.array([1, 2, 4]), transform), [2, 1, 0])
    np.testing.assert_allclose(np.linalg.det(transform[:3, :3]), 1)
    turned = camera_transform([0, 0, 0], [0, math.sin(math.pi/4), 0, math.cos(math.pi/4)])
    np.testing.assert_allclose(transform_point(np.array([0, 0, 2]), turned), [-2, 0, 0], atol=1e-12)


def test_two_rotated_views_project_to_same_known_world_point():
    intrinsics = CameraIntrinsics(fx=320, fy=320, cx=320, cy=240)
    # A Habitat world point [-1, 0, -3], seen from camera [0, 1.2, 0].
    optical_points = [np.array([-1, 1.2, 3]), np.array([math.sqrt(2), 1.2, 2*math.sqrt(2)])]
    for angle, point in zip([0, math.pi/4], optical_points):
        u, v = 320 + point[0]/point[2]*320, 240 + point[1]/point[2]*320
        back_projected = project_to_3d(u, v, point[2], intrinsics)
        transform = camera_transform([0, 1.2, 0], [0, math.sin(angle/2), 0, math.cos(angle/2)])
        np.testing.assert_allclose(transform_point(back_projected, transform), [-1, 3, 0], atol=1e-10)


def test_mask_projection_uses_metres_and_excludes_background_depth():
    mask = np.zeros((10, 10), dtype=bool)
    mask[3:7, 3:7] = True
    depth = np.full((10, 10), 12.0)
    depth[mask] = 2
    points = mask_to_pointcloud(mask, depth, CameraIntrinsics(fx=100, fy=100, cx=5, cy=5),
                               camera_transform([0, 0, 0], [0, 0, 0, 1]), depth_scale=1.0)
    assert len(points) == 16
    np.testing.assert_allclose(points[:, 1], 2)
    assert np.max(np.abs(points[:, [0, 2]])) < 0.05
