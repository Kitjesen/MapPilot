# Optional integrations

Integrations are retained capabilities that sit outside the default LingTu
Product runtime. They must not silently become Products, Profiles, env values,
or services installed by the canonical field deployment.

| Integration | Status | Runtime relationship |
| --- | --- | --- |
| [Open-RMF](open_rmf/README.md) | optional sidecar | Communicates through LingTu's published interfaces. |
| [Super-LIO](super_lio/README.md) | experimental | Lab-only external ROS 2 SLAM adapter retained for evaluation; not packaged in native releases and not installed or selected by default. |

Promotion into the field runtime requires a typed adapter, ProductControl
ownership, and the relevant simulation and S100P acceptance gates.


