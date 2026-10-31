# Robot selection categories

Categories are authored presentation metadata, not locomotion capabilities or
runtime qualification. A wheeled-legged robot does not become driveable merely
because it belongs to that category. Existing exact SessionBundle entries still
own launch eligibility.

## Current 29-model library

| Category | Value | Count | Models |
| --- | --- | ---: | --- |
| 四足 | `quadruped` | 9 | ANYmal D, Unitree Go2/B2/A1, DeepRobotics Lite3, Zsibot ZSL1, MagicLab MagicDog, Agibot D1, Boston Dynamics Spot |
| 轮足 | `wheeled_legged` | 7 | Thunder V4, Unitree Go2W/B2W, DeepRobotics M20, DDTRobot Tita, Zsibot ZSL1W, MagicLab MagicDog-W |
| 人形 | `humanoid` | 10 | Unitree G1/H1, FFTAI GR1T1/GR1T2, Booster T1, RobotEra Xbot, OpenLoong Loong, RoboParty ATOM01, MagicLab MagicBot-Gen1/MagicBot-Z1 |
| 轮式 | `wheeled` | 1 | OmniCart |
| 机械臂 | `manipulator` | 2 | Franka Panda, Universal Robots UR5e |

`other` is available for uncategorized imported data. Old catalogs that omit
`category` still load, with those models in 其他; no name-based guessing is used.
The 其他 tab is absent when its count is zero. The 全部 tab is a view filter,
not a model category.

Authored sources:

- `sim/sessions/robot-library.v1.json`: imported model summaries.
- `sim/sessions/game-selection.v1.yaml`: existing session robot summaries.
- `sim/tools/game_selection_catalog.py`: validates and emits metadata without
  changing physics plans or creating runnable entries.

## Selection interaction

- Type tabs have stable total counts; the visible list count also reflects
  search. Search matches model labels and IDs without case sensitivity.
- Filtering selects the first matching robot only when the current selection
  is outside the filtered list. A still-visible selection is retained.
- No results means no stale model preview and no Next action. Back clears an
  active search before leaving the robot page.
- Search text owns letter and editing keys. Enter in the search box transfers
  focus to the current result; it does not launch a session.
- Robot rows have real transparent thumbnails where available, a category
  subtitle and selected-row highlighting. A missing preview never borrows
  another model's image.
- Keyboard focus scrolls into view. Category and search state survive visiting
  the map step and returning. Expanded preview remains modal.
- A runtime's read-only library cannot filter into a different robot or mutate
  its selection; editable filtering belongs to the preparation front end.

The preview remains a static render of real model geometry, not a live 3D orbit
viewer. The DAE conversion and per-robot control integration limits recorded in
`robot-library-import.md` are unchanged by this UI work.

## Verification — 2026-10-16

- Catalog and launcher Python tests: 66 passed; Ruff and py_compile clean.
- Controlled UE Editor build: succeeded in 59.24 seconds,
  `build/unreal-tests/ubt-robot-categories-20261016.log`.
- UI Automation: 90 successes, zero warnings/failures/not-run,
  `build/unreal-tests/robot-categories-20261016/report/index.json`.
- The CategorySearch test exercises actual tab activation, type/search
  intersection, case-insensitive matching, empty results, selection restoration,
  map/back persistence, text focus and shortcut suppression.
- Real UE 1920x1080 capture exited 0, with no selection intent and no physics:
  `build/unreal-tests/robot-map-20261016-210923/result.json`.
- Images:
  `sim/runtime/visual/RobotSimUE/Saved/Automation/LingTuSimUI/FrontEnd/robot-map-20261016-210923/asset-library.png`
  and `category-humanoid.png` in the same directory.

The current catalog is generated into
`build/robot-category-selection-render-20261016/` using the existing compiler
with `--robot-library sim/sessions/robot-library.v1.json`.
