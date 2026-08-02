# TravExplorer Adoption Boundary

Status: research/adoption note, not a current product dependency
Audience: exploration research and product adoption planning
Replaced by: `FIELD_PRODUCTS.md` for field exploration runtime

TravExplorer is not a directly integrable ROS2 package today. It is useful to
LingTu as an algorithm reference. As of the current public repository state, the
TravExplorer GitHub repository contains README, license, and image assets, while
the README says the code will be released after acceptance. There is no public
`package.xml`, launch file, source tree, or runnable node graph to integrate.

Sources:

- https://github.com/wuyi2121/TravExplorer
- https://wuyi2121.github.io/TravExplorer/
- https://arxiv.org/abs/2605.19958

## Product Decision

LingTu should adopt the TravExplorer direction in phases while preserving the
Product-defined runtime boundary. TravExplorer's current public evidence supports
3D traversability mapping, traversable frontier extraction, semantic guidance,
hierarchical exploration planning, foothold-guided 3D path search, active
perception, and real-world Go2 validation. It does not provide a public ROS2
implementation that can be dropped into LingTu.

The P0 product path remains the LingTu inspection evidence chain:

- Gateway and ModulePort observability are the operator-facing dataflow surface.
- Product/env RunPlan switching proves simulation and real command
  boundaries.
- Saved map, tomogram, and occupancy artifacts must carry same-source metadata.
- Route preview and inspection acceptance must be non-motion unless the user
  explicitly starts a mission.
- Commands must flow through NavigationModule, CmdVelMux, SafetyRing, and the
  resolved Product/env command owner.

## Minimal LingTu-Native Slice

The first implementation slice is a `TraversableFrontierModule`, not an
external TravExplorer runtime. It consumes existing LingTu map products and
publishes ranked frontier candidates through Module ports.

Inputs:

- odometry from the active localization source
- voxel, occupancy, ESDF, elevation, and traversability map evidence
- optional semantic object and room evidence from the memory/semantic stack

Outputs:

- frontier cluster id
- 3D centroid and support surface height
- axis-aligned bounding box
- reachable score and ESDF clearance
- support type, such as flat, slope, stair-like, or unknown
- semantic value and nearby target labels
- state, such as active, visited, blocked, or dormant
- read-only runtime stream tokens `/nav/traversable_frontiers` and
  `/nav/frontier_candidate` for Gateway/Runtime Dataflow inspection

This module does not own command output. It publishes candidates for the
existing NavigationModule and planner services to preview and execute; it is not
wired to NavigationModule by default. Gateway may subscribe to the candidate
ports for SSE and Runtime Dataflow visibility, but that Gateway path remains
read-only and exposes no publish or command interface.

## Future Bridge Boundary

If TravExplorer later releases runnable code, wrap it behind a
`TravExplorerBridgeModule`. The bridge may translate LingTu runtime stream
tokens into the external process and translate candidate frontiers, paths, or
goal suggestions back into LingTu messages.

The bridge must not bypass LingTu command ownership:

- no direct hardware command sink
- no direct actuator publish path
- no external planner command that skips NavigationModule
- no control output that skips CmdVelMux or SafetyRing
- no field-readiness claim without LingTu runtime evidence gates

ROS-based transport is acceptable only at the adapter boundary. The product
acceptance interface remains Gateway, Module ports, runtime evidence, saved
artifact provenance, and whitelisted command APIs.

## Runtime Boundary Alignment

The relevant lesson is boundary replaceability. A Product should be expressed
once, then resolved inside `env=real` or `env=sim` into a RunPlan whose
declared adapters, native services, or simulation backend provide the source
and sink boundary. LingTu should therefore treat TravExplorer as one possible
future strategy provider or sim-backend integration while keeping the Product
graph and acceptance gates unchanged.

## Validation Gates

Before claiming TravExplorer-inspired capability, require evidence that:

- frontier candidates are derived from reachable support surfaces, not only
  free/unknown 2D grid edges
- candidates can be previewed through the non-motion route preview API
- candidate scores, support evidence, and rejection reasons are visible through
  Gateway Runtime Dataflow without RViz or ROS topic browsing
- local planner output and command arbitration remain inside LingTu
- dynamic obstacle checks still prove local path and command behavior
- field claims use real-runtime evidence, not simulation-only artifacts
