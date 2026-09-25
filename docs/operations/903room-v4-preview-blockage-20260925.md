# 903room_v4 click navigation blockage — 2026-09-25

Field release .75, session product-024aabc206e140839e77c09034e25fcc.
User sees goal marker but only preview/re-preview. Native navigation input gate ready,
localization TRACKING, admission ACCEPTING, no active path or motion command submitted.

A no-motion POST /api/v1/navigation/goal_candidate at (0.6,-0.25,0.03)
returned preview_infeasible / start_body_occupied in 0.05152 ms. Start was
(0.0888717,-0.253331,0.0259962). Frontend correctly withheld execution for this
failed plan; repeated previews cannot repair the start geometry.

The saved OctoMap has 0.20 m resolution. A native OctoMap leaf-volume inspection
at (0.089,-0.253,0.026), radius 0.43 m, vertical clearance +/-0.10 m found seven
occupied leaves overlapping the body. Their centers are (x,y,z):
(0.3,-0.7,0.1), (0.5,-0.5,0.1), (0.5,-0.3,0.1), (0.5,-0.1,0.1),
(-0.3,0.1,0.1), (0.3,0.1,0.1), (0.5,0.1,0.1), all size 0.20 m.
The 1,005,587-point binary XYZ map.pcd contains no returns within XY radius
0.48 m and vertical +/-0.15 m at this start. Returns in the seven intersecting
cells have nearest XY distances 0.495–0.551 m. This isolates coarse voxel
expansion as the static start rejection, rather than proving a real body collision.
It does not prove the complete route or local support is executable.

Field evidence and read-only C++ probe:
/home/unitree/teleop-reconnect-20260925/v4-body-probe.cpp
/home/unitree/teleop-reconnect-20260925/v4-start-overlap.json

Unfinished: build a finer-resolution candidate without replacing the active map,
verify start geometry, support and path against native planner before activation.
Do not shrink the physical envelope or bypass the collision gate. Current save
resolution originates in src/maps/cpp/mapd/main.cpp via
LINGTU_MAP_SAVE_OCTOMAP_RESOLUTION default 0.20; changing it alone does not rebuild
already saved maps. No production map, configuration or service changed in this audit.

## Fine-resolution candidate follow-up

Built from all saved scans/rays plus retained map.pcd on NX using the installed
converter, without replacing the active map or restarting services:

| Resolution | Start overlapping occupied leaves | Result |
| --- | ---: | --- |
| 0.20 m | 7 | start_body_occupied |
| 0.10 m | 4 | start_body_occupied |
| 0.05 m | 0 | start_ground_support_unconfirmed |

5 cm candidate: 23,116,541 bytes (22.05 MiB), build 66.81 s, peak child RSS
381,452 KiB; 331,926 occupied voxels. 10 cm build 15.79 s, peak 138,744 KiB.
The candidate lives at /home/unitree/teleop-reconnect-20260925/v4-fine-candidate/octomap.ot.
Its copied metadata still describes the original map; this directory is an offline
experiment, not an importable/activated production map bundle.

Native installed planner source was compiled into a read-only probe. Three short
candidate plans failed StartGroundSupportMissing at the same start; body-only
query passed. Three of the five footprint support samples failed
robot_ground_support_unobserved. At those columns the expected floor cells are
ray-observed free, with occupied floor neighbors. Nearby retained floor points
are consistent with the configured 0.35 m body height. Do not equate a nearby
plane with direct supported ground or erase observed free evidence. Inspect
saved-ray integration and retained-point sampling before changing the shared
surface query; filling cells or disabling support has not been authorized as a fix.

Code candidate: OctomapBuildOptions defaults to 0.05 m / 180 s; mapd save and
service build default resolution/timeout derive from these existing options.
Explicit overrides remain effective; persisted old jobs retain their recorded
settings. These source changes are NOT deployed. New Go2 voxel-clearance
regression passed with the full ARM grid_smoke suite: 20/10 cm conservative
expansion remains blocked, 5 cm is clear, true in-envelope obstacles stay blocked.
Evidence: v4-fine-build.log, v4-fine-plan.log, v4-10cm-plan.log,
v4-support-probe.cpp and v4-grid-query.log in the same remote directory.

ARM syntax checks for modified mapd main.cpp/service_dispatch.cpp against the
candidate options header passed. No service installation or map activation was
performed. The new default therefore applies only after this source candidate
is packaged and deployed; the running .75 map remains unchanged.

## Navigation service restored after reboot

At the user's request, ProductControl successfully restored installed release
.75 nav/camera with the original 903room_v4 map. Session:
product-5b7a19b85e6341dfb029826c43811e46. Web HTTP 200, localization TRACKING,
input_gate ready, driver ready, no resume requirement, no active path, final
velocity zero. No movement goals submitted. Candidate 5 cm map and source
changes remain undeployed because support/path validation has not passed.
Startup report: /home/unitree/teleop-reconnect-20260925/nav-v4-reboot-second.json.

## Latest reboot recovery

ProductControl restored .75 nav/camera + 903room_v4 after another user-reported
power-on. Session product-b699bbaa8f3e4eb380186679561aea97. HTTP 200,
TRACKING, input/driver ready, no active path and zero final velocity.
No-motion preview to (0.6,-0.25,0.03) still failed start_body_occupied;
current start (-0.316045,-0.234191,0.020220) differs from the prior fine-map
candidate evaluation. Do not apply the earlier start-clear result to this pose.
No motion goal sent. Startup report nav-v4-reboot-third.json in remote evidence
directory. Fine-map candidate and new defaults remain undeployed.

## Root cause and support-query repair (candidate .76)

The retained map, map.clean.pcd and map.pcd.preclean all have zero floor returns
in the three failing 5 cm columns. Prune did not delete these column returns;
they are point-sampling gaps. Adjacent returns are at the calibrated floor height.
At the old start, the missing columns have ray-free cells at z indices -5/-6/-7,
and unknown cells at -8 through -11. The old query equated a ray crossing the
surface-containing cell with an observed drop and required four fixed cardinal
neighbors to interpolate. OctoMap ray traversal labels intersected cells; this
does not establish that a grazing ray crossed below the inferred surface.
Reference: https://octomap.github.io/octomap/doc/OccupancyOcTreeBase_8hxx_source.html
The interpolation policy below is LingTu's extension, not an upstream OctoMap or
SCAN guarantee.

Shared 3D query candidate: for a missing column, fit measured first-surface
neighbors within two cells and 10 cm per axis; require returns in all four
quadrants, observed overhead clearance, calibrated height, bounded slope and
one-voxel height residual. Reject observed free cells below the fitted surface
cell, measured lower floors, nearer slabs, one-sided/degenerate evidence and
large gaps. No occupied/free voxel is changed and inferred samples are never
recursively reused. Direct measured-surface behavior remains unchanged.

At old start (0.089,-0.253,0.026), both body-only and support queries pass after
repair. Native planning found a short 4-waypoint route for goal
(0.089,-0.353,0.026); another direction remained infeasible. This is offline
ARM replay, not field-motion acceptance. The current start near
(-0.316,-0.234,0.020) differs: the retained map contains a point
(-0.6543476,-0.4924177,0.07005435) inside its 0.43 m global cylinder (about
0.426 m radial distance). The user cannot currently confirm the physical object.
Do not erase it or shrink the global envelope to make this pose pass.

The map pipeline built an independent 903room_v4_5cm map transaction successfully,
with recorded saved-ray provenance, explicit 0.05 m resolution, no support/free
dilation, and a 180 s conversion timeout. Original 903room_v4 is retained.
The candidate's copied experimental metadata is not used for this production
map: the production map was rebuilt through lingtu-mapctl's transaction.
