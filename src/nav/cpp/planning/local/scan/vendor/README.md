# Vendored dependency

`lbfgs.hpp` is the header-only L-BFGS implementation used by the SCAN rebound
optimizer. It was copied from the public SCAN-Planner source inspected at
commit `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`; its Apache-2.0 license is kept
in `LICENSE` beside the header.

Only this optimizer dependency is vendored. LingTu does not copy the upstream
ROS nodes, PCL/OpenCV map pipeline, runtime FSM, controller, or map ownership.
The paper's one-cell virtual-boundary fallback was implemented from the paper
semantics because that behavior is not present as a complete reusable module in
the inspected public source.

`../uniform.*` is a dependency-free adaptation of the upstream
`UniformBspline` source, with upstream attribution and SPDX headers in each
file. It is ordinary SCAN backend source rather than a separately vendored
library.
