# Downloaded papers

These PDFs were downloaded or retained through 2026-11-16 for local engineering reference.
The links in the table point to the paper's public source. The local copy is
only a reference artifact; the repository does not treat a paper prototype as
an implemented LingTu capability.

| Local file | Paper | Why it matters to LingTu | Source |
| --- | --- | --- | --- |
| [op-follower.pdf](op-follower.pdf) | OP-Follower (JFR 2026), author-hosted PDF, 15 pages | Onboard Go2 person following, asynchronous perception, delay prediction and 50 Hz control | [Author PDF](https://op-follower.github.io/static/pdfs/OP_Follower-1.pdf), [publication](https://doi.org/10.1002/rob.70363) |
| [tpt-bench-v2.pdf](tpt-bench-v2.pdf) | TPT-Bench, author-linked arXiv v2 dated 2025-07-09, 16 pages; journal publication in IJRR 2026 | Long-term robot-view person tracking, target absence and re-identification evaluation | [Downloaded preprint](https://arxiv.org/pdf/2505.07446v2), [journal record](https://doi.org/10.1177/02783649261447308) |
| [hov-sg.pdf](hov-sg.pdf) | HOV-SG: Hierarchical Open-Vocabulary 3D Scene Graphs | Room/floor/object hierarchy and language queries | [arXiv](https://arxiv.org/abs/2403.17846) |
| [orionnav.pdf](orionnav.pdf) | OrionNav | Onboard semantic mapping and hierarchical scene-graph navigation on a quadruped | [arXiv](https://arxiv.org/abs/2410.06239) |
| [clio.pdf](clio.pdf) | Clio | Task-driven semantic compression instead of saving every feature | [arXiv](https://arxiv.org/abs/2404.13696) |
| [findanything.pdf](findanything.pdf) | FindAnything | Object-centric volumetric submaps and queryable geometry | [arXiv](https://arxiv.org/abs/2504.08603) |
| [dualmap.pdf](dualmap.pdf) | DualMap | Separate global abstract candidate selection from local precise navigation | [arXiv](https://arxiv.org/abs/2506.01950) |
| [supermap.pdf](supermap.pdf) | SuperMap | Persistent object association, confidence, stale pruning, and asynchronous geometry/semantics | [arXiv](https://arxiv.org/abs/2608.22896) |
| [open3dsg.pdf](open3dsg.pdf) | Open3DSG | Point-cloud-based open-vocabulary object and relation graphs | [arXiv](https://arxiv.org/abs/2312.02343) |
| [tag-map.pdf](tag-map.pdf) | Tag Map | Compact text representation for exposing map context to an LLM | [PMLR](https://proceedings.mlr.press/v270/zhang25e.html) |
| [open-vocabulary-octree-graph.pdf](open-vocabulary-octree-graph.pdf) | Open-Vocabulary Octree-Graph | Octree geometry plus semantic graph structure | [arXiv](https://arxiv.org/abs/2411.16253) |
| [ovi-map.pdf](ovi-map.pdf) | OVI-MAP: Open-Vocabulary Instance-Semantic Mapping (CVPR 2026 Highlight) | Separate incremental instance geometry from asynchronous VLM semantic evidence | [CVF Open Access](https://openaccess.thecvf.com/content/CVPR2026/html/Deng_OVI-MAP_Open-Vocabulary_Instance-Semantic_Mapping_CVPR_2026_paper.html) |
| [sysnav.pdf](sysnav.pdf) | SysNav: Multi-Level Systematic Cooperation Enables Real-World, Cross-Embodiment Object Navigation (2026) | Building-scale room-level VLM decisions with classical in-room exploration; Habitat Challenge benchmarks and Go2/G1/ wheeled robots | [arXiv](https://arxiv.org/abs/2603.06914), [project page](https://cmu-vln.github.io/), [code](https://github.com/zwandering/SysNav) |
| [vl-nav.pdf](vl-nav.pdf) | VL-Nav: Neuro-Symbolic Reasoning-based Vision-Language Navigation (IROS 2026) | Symbolic 3D scene graph, image memory, dynamic occupancy/frontier exploration, and long-range real-robot demonstrations | [arXiv](https://arxiv.org/abs/2502.00931), [project page](https://sairlab.org/vlnav/) |
| [conceptgraphs.pdf](conceptgraphs.pdf) | ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning (ICRA 2024; 2023 preprint) | Masked RGB-D projection, object point-cloud cleanup, geometric and visual feature association, and persistent object maps | [arXiv](https://arxiv.org/abs/2309.16650), [official code](https://github.com/concept-graphs/concept-graphs) |
| [language-to-navigation-goals.pdf](language-to-navigation-goals.pdf) | From Language to Navigation Goals: A Vision-Language Approach for Semantic Navigation of Mobile Robots Using RGB-D Perception (2026 preprint) | Minimal ROS 2/Gazebo text-to-RGB-D-goal baseline; Gazebo TurtleBot3 full navigation and Go2 semantic perception | [arXiv](https://arxiv.org/abs/2607.13624) |

Other references that remain online-only are listed in the design document,
including ASHiTA, OpenGS-SLAM, Point2Graph, Khronos, DovSG, osmAG-LLM, and
the MCP representation-layer work. They are not copied into the repository
because the downloaded set is enough to support the current design decision.

## Person-follow source downloads, 2026-11-16

| Project | Local checkout | Revision | Contents checked |
| --- | --- | --- | --- |
| [TPT-BENCH-TOOLS](https://github.com/MedlarTea/TPT-BENCH-TOOLS) | `third_party/research/TPT-BENCH-TOOLS` | `2488f8bb50f757930c5131688fa60eb273e2611f` | Evaluation scripts, dataset loaders, panoramic projection, rosbag extraction and tracking visualization |
| [DobotTrack](https://github.com/Dobot-Team/dobot_track) | `third_party/research/dobot_track` | `0a99a9554c6564cd8d4bde0f896c3c48bf3f8cc4` | Q-identity history compression, InternVL3-1B configuration, training/evaluation and Orin deployment source |

The OP-Follower author account exposes only `OP-Follower.github.io` at this
check. Its project page and paper link to demonstration material. An official
algorithm source repository was not found. The website repository was not
cloned as algorithm code.

Both PDFs passed title, page-count and per-page text extraction checks. The
TPT download is the public preprint linked by the authors. The full TPT dataset
and DobotTrack model weights have not been downloaded, and the external
algorithms have not been installed, run or integrated into LingTu.
