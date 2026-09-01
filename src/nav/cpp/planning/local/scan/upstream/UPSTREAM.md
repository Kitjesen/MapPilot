# SCAN-Planner upstream

- Repository: https://github.com/wuyi2121/SCAN-Planner
- Commit: `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`
- License: Apache-2.0
- License copy: [`../vendor/LICENSE`](../vendor/LICENSE)

Files below this directory preserve the corresponding SCAN-Planner algorithm
bodies. LingTu changes are limited to removing ROS process APIs, placing the
code in the LingTu namespace, and adapting map/time inputs at the module seam.

`plan_env/grid_map.*` is that explicit seam: Mapd owns the upstream occupancy,
raycast, log-odds, and incremental-inflation behavior, while this class exposes
the resulting read-only bitmap through the upstream occupancy-query API.
