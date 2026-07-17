# Camera-LiDAR calibration provenance

The directories below are source snapshots kept with their upstream license
files. They are calibration tools and are not linked into the LingTu product
runtime.

| Directory | Upstream | License | Upstream HEAD checked 2026-07-17 |
| --- | --- | --- | --- |
| `livox_camera_calib/` | <https://github.com/hku-mars/livox_camera_calib> | GPL-2.0-only | `061fdaa647fc806e59d73a8505a05a10dfcfdaa1` |
| `livox_camera_lidar_calibration/` | <https://github.com/Livox-SDK/livox_camera_lidar_calibration> | MIT | `27f9e6c8a1eda3326b7673da418e6d7112559c2f` |
| `mlcc/` | <https://github.com/hku-mars/mlcc> | GPL-2.0-only | `20f3b3a42ed88cdc287fd93d24d24a2df2a81999` |

The recorded hashes identify the upstream references checked during import;
the local snapshots may contain LingTu path or build adaptations. Consult each
directory's `LICENSE` and `README.md` before redistributing modifications.

`livox_calib_standalone/` is a LingTu adaptation that includes headers from
`livox_camera_calib/`. It is therefore distributed under GPL-2.0-only; the
corresponding license text is `livox_camera_calib/LICENSE`.
