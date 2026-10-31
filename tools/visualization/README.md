# Visualization

Choose the input you want to inspect:

| Input | Entry |
| --- | --- |
| Product saved-map library | Gateway `/map/viewer?map=MAP_ID` → the product Maps page |
| Local recording exports | [`map.html`](map.html) with `?data=...` |
| Live Gateway streams | [`rerun_gateway_live.py`](rerun_gateway_live.py) |
| SLAM replay in Rerun | [`run_rerun_mapping.py`](run_rerun_mapping.py) |

## Product maps

Use `<gateway>/map/viewer?map=MAP_ID` to open a saved map in the existing React
Maps page. The entry redirects to `/?tab=map&map=MAP_ID`; without a map it opens
the library. Previewing does not activate a map, switch Product, or send motion.

When starting from this tool on a local HTTP server, use
`map.html?service=http://GATEWAY_HOST:5050&map=MAP_ID`. It joins that same product
page. Service mode does not load Three.js from a CDN, `view.json`, or local
binary exports. The product page reads the native PCD through Gateway and
checks its map ID, content epoch, and frame headers. It follows the product's
installed UI and font assets.

## Local recording exports

Serve the repository root with Python's existing HTTP server:

```bash
python -m http.server 4186 --bind 127.0.0.1
```

Open `/tools/visualization/map.html?data=../../build/MAP/view.json` on that
server. Replace `MAP` with the recording directory. Direct `file://` opening
does not support the module and data fetches used here.

The maintained viewer has three short files: `map.html` for structure,
`map.css` for presentation, and `map.js` for loading and interaction. Each
recording keeps its own `view.json`, data, and historical planning results.
Resource paths in the descriptor are relative to the descriptor itself;
moving the viewer does not require copying the point cloud.

The 903 recording remains at `build/go2-map-save-v6-20260930/`. Its existing
`full-view.html` redirects to the maintained viewer. The descriptor preserves
the recording's labels, statistics, view presets, and historical route labels.
It is display configuration, not saved-map activation metadata.

Keep the descriptor limited to the viewer's inputs:

| Field | Content |
| --- | --- |
| `title`, `subtitle`, `documentTitle`, `ariaLabel`, `mapName`, `summary` | Recording identity and display text |
| `files` | `cloud`, `occupancy`, and optional `poses` / `plans` resource paths |
| `stats` | Displayed `points`, `occupied`, `resolution`, and `frames` from the recording |
| `source` | `description` and `limits` explaining provenance and claim boundaries |
| `ceiling` | Display-only `min`, `max`, `step`, and initial `value` |
| `presets` | `initial`, `full`, and `focus`: XYZ `centre` and `distance` in meters, `elevation` in radians; `initial` also requires `azimuth` in radians (optional on the other presets); `focus.label` names the close-up control |
| `routeLabels` | Optional display names keyed by the retained route labels |

Supported display exports are:

- `cloud.xyz`: little-endian float32 binary XYZ triples in meters. Despite the
  extension, this is **not** an ASCII XYZ file or a PCD file.
- `occupied.bin`: little-endian float32 records of `x, y, z, size, occupancy`.
  The viewer draws the occupied cubes; this export does not retain free or
  unknown-space evidence from the original OctoMap.
- `poses.json`: an array of XYZ positions for the recorded collection path.
- `plans.json`: retained offline requests and results with `cases`, including
  each case's `label`, `start`, `goal`, and `result.path`.

Trajectory and route overlays start hidden. The height slider only clips the
display; it does not remove points or fill ground. A visible route is a stored
planning result, not evidence that a robot executed it or that the current map
is ready for navigation. The viewer does not send commands to a robot.

Fonts reuse the local assets in `web/public/assets/fonts/` and follow
[`web/DESIGN.md`](../../web/DESIGN.md#typography). Three.js retains the existing
`0.170.0` CDN import, which requires network access; viewing saved data does not
mean the page is available without a network connection. No new package is
required by this organization.

One-off browser checks and screenshots remain beside their dated evidence.
For the 903 recording, those are `verify-view.mjs`, `verify-type.mjs`, and their
JSON reports. They contain local browser/runtime paths and are not portable
test entrypoints. The native planning regression remains documented under
[`tools/bench/`](../bench/README.md).
