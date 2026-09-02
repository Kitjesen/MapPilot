# Vendored dependency

`lbfgs.hpp` is the header-only L-BFGS implementation used by the SCAN rebound
optimizer. It was copied from the public SCAN-Planner source inspected at
commit `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`; its Apache-2.0 license is kept
in `LICENSE` beside the header.

Only this optimizer dependency is vendored. The ROS nodes, PCL/OpenCV adapters,
visualization, open-loop simulator, and Go2 gait publisher are not part of the
LingTu runtime. The Projected A*, B-spline, PlannerManager, replanning FSM, and
closed-loop controller sources live under [`../upstream/`](../upstream/UPSTREAM.md).
