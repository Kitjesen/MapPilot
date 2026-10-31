# Sensor type preview sources

Classification: **schematic**, not measurement data. All nine diagrams are
original code-authored functional illustrations; they are not photographs,
hardware/product renders, AI-generated imagery, or edits of existing images.
They do not demonstrate sensor availability, accuracy, calibration or runtime
readiness. Grids, traces, ranges, temperatures and IDs are illustrative only;
semantic IDs are not a project label taxonomy. Truth odometry depicts a simulator
reference pose, not a physical sensor or localization estimate.

## Authorship and reproduction

Source of truth: `sim/tools/assets/render_sensor_type_previews.py`. No third-party image assets or external image
sources were used. The bundled SVGs retain editable vector geometry and text.
The PNGs are drawn from the same primitives on new Pillow canvases at 3x size,
then downsampled to 480 x 240. Pillow's bundled default font is used for raster
labels; SVG text uses the viewer's sans-serif font. No CairoSVG is required.
No UE, Blender, Tripo, network access, paid services or dependency installation
is involved. Existing Pillow is required to regenerate; pytest checks the pack.

Run from the repository root using an existing Python with Pillow:

```console
python sim/tools/assets/render_sensor_type_previews.py
python sim/tools/assets/render_sensor_type_previews.py --output-dir <directory>
python -m pytest tests/sim/test_sensor_type_previews.py -q
```

The light-gray background, dark ink and blue accent follow the selector's
visual direction in `SLingTuSimRobotMapPicker.cpp`; modality-specific colors
encode RGB channels, temperature bands and semantic regions only.
`manifest.json` classifies every 480 x 240 PNG/SVG pair as schematic and
explicitly sets `measurement` to false. UI integration belongs to the caller.

## Generated pairs

| PNG | Bundled SVG source | Meaning |
| --- | --- | --- |
| `rgb.png` | `rgb.svg` | Camera lens and a visible-light scene with RGB channels. |
| `depth.png` | `depth.svg` | Near-to-far distance bands and a range ruler in metres. |
| `lidar.png` | `lidar.svg` | Radial ranging rays ending at discrete return points. |
| `imu.png` | `imu.svg` | Three body axes, acceleration vector and angular-motion arc. |
| `thermal.png` | `thermal.svg` | Illustrative temperature grid and a Kelvin scale. |
| `acoustic.png` | `acoustic.svg` | Sound propagation and a pressure waveform over time. |
| `semantic.png` | `semantic.svg` | Region masks with illustrative class and instance identifiers. |
| `gnss.png` | `gnss.svg` | Satellite signals intersecting at an illustrative position fix. |
| `truth_odom.png` | `truth_odom.svg` | Reference trajectory and pose axes, not an estimated path. |
