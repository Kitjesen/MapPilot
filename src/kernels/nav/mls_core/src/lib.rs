//! Dependency-free multi-level surface and sparse graph core.
//!
//! The data model follows the useful ownership split from the DimOS MLS
//! planner. LingTu keeps map I/O, body geometry, support checks, route
//! validation, and runtime ownership outside this crate.

use std::collections::{HashMap, VecDeque};

mod adjacency;
mod corridor;
mod edge_repair;
mod edges;
mod node_relocation;
mod nodes;
mod stable_graph;
mod surfaces;
mod voronoi_repair;

pub(crate) use edges::build_graph;
pub use stable_graph::{
    GraphRepairReport, IncrementalGraph, StableEdge, StableEdgeId, StableNode, StableNodeId,
};
pub use surfaces::{extract_surfaces, extract_surfaces_with_free};

/// An occupied or standable voxel expressed in integer map coordinates.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct SurfaceCell {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl SurfaceCell {
    pub const fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }
}

/// Surface extraction and bounded hole-closing parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceConfig {
    /// Empty vertical cells required above a cell before it is standable.
    pub clearance_cells: i32,
    /// Horizontal radius used by bounded closing.
    pub closing_passes: u32,
    /// Maximum number of cells in one inferred hole.
    pub max_hole_cells: usize,
    /// Maximum fitted surface slope in rise/run units.
    pub max_slope: f32,
}

impl Default for SurfaceConfig {
    fn default() -> Self {
        Self {
            clearance_cells: 1,
            closing_passes: 0,
            max_hole_cells: 0,
            max_slope: 0.70,
        }
    }
}

/// Output of multi-level surface extraction.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct SurfaceMap {
    /// Cells supported directly by occupied voxel columns.
    pub observed: Vec<SurfaceCell>,
    /// Cells inferred by bounded per-height closing.
    pub inferred: Vec<SurfaceCell>,
}

/// Sparse node placement parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GraphConfig {
    /// Minimum 3-D distance between sparse nodes, in voxel cells.
    pub node_spacing_cells: i32,
    /// Maximum vertical difference accepted by a surface edge.
    pub max_link_cells: i32,
    /// Hard distance from a surface boundary before a node may be placed.
    pub wall_clearance_cells: i32,
    /// Width of the soft wall-cost band.
    pub wall_buffer_cells: i32,
    /// Cost multiplier at the hard-clearance boundary.
    pub wall_buffer_weight: f32,
    /// Additional cost per vertical cell on an edge.
    pub step_penalty_weight: f32,
    /// Maximum rise/run ratio for a surface edge. Zero leaves it unbounded.
    pub max_slope: f32,
    /// Maximum vertical difference in cells for a surface edge. Zero leaves it unbounded.
    pub max_step_cells: i32,
    /// Maximum horizontal hop used to represent a slope-limited connection.
    pub horizontal_hop_cells: i32,
}

impl Default for GraphConfig {
    fn default() -> Self {
        Self {
            node_spacing_cells: 1,
            max_link_cells: 4,
            wall_clearance_cells: 0,
            wall_buffer_cells: 0,
            wall_buffer_weight: 0.0,
            step_penalty_weight: 0.0,
            max_slope: 0.0,
            max_step_cells: 0,
            horizontal_hop_cells: 1,
        }
    }
}

/// A sparse graph node located on a surface cell.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Node {
    pub id: u32,
    pub cell: SurfaceCell,
}

/// A graph edge with a minimal corridor representation.
#[derive(Clone, Debug, PartialEq)]
pub struct Edge {
    pub a: u32,
    pub b: u32,
    pub cost: f32,
    pub corridor: Vec<SurfaceCell>,
}

/// Prepared graph data. Collision and support admission remain external.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SurfaceGraph {
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
}

/// Surface and graph data that can be persisted or handed to an adapter.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PlannerArtifact {
    pub surface: SurfaceMap,
    pub graph: SurfaceGraph,
}

/// Inclusive integer-cell bounds for an offline or mapd region update.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RegionBounds {
    pub min_x: i32,
    pub max_x: i32,
    pub min_y: i32,
    pub max_y: i32,
    pub min_z: i32,
    pub max_z: i32,
}

impl RegionBounds {
    pub const fn new(
        min_x: i32,
        max_x: i32,
        min_y: i32,
        max_y: i32,
        min_z: i32,
        max_z: i32,
    ) -> Self {
        Self {
            min_x,
            max_x,
            min_y,
            max_y,
            min_z,
            max_z,
        }
    }

    fn contains(self, cell: SurfaceCell) -> bool {
        cell.x >= self.min_x
            && cell.x <= self.max_x
            && cell.y >= self.min_y
            && cell.y <= self.max_y
            && cell.z >= self.min_z
            && cell.z <= self.max_z
    }

    fn expand(self, xy: i32, z: i32) -> Self {
        Self {
            min_x: self.min_x.saturating_sub(xy),
            max_x: self.max_x.saturating_add(xy),
            min_y: self.min_y.saturating_sub(xy),
            max_y: self.max_y.saturating_add(xy),
            min_z: self.min_z.saturating_sub(z),
            max_z: self.max_z.saturating_add(z),
        }
    }
}

/// Evidence returned by an offline region update.
///
/// The current first implementation deliberately reports that graph assembly
/// was rebuilt globally. Surface work is bounded; regional graph repair is the
/// next step once mapd supplies a supported region delta.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RegionUpdateReport {
    pub expanded_region: RegionBounds,
    pub replaced_occupied: usize,
    pub replaced_known_free: usize,
    pub surface_cells_rebuilt: usize,
    pub affected_nodes: usize,
    pub affected_edges: usize,
    pub preserved_nodes: usize,
    pub preserved_edges: usize,
    pub reused_nodes: usize,
    pub new_nodes: usize,
    pub removed_nodes: usize,
    pub reused_edges: usize,
    pub rebuilt_edges: usize,
    pub route_replan_required: bool,
    pub graph_rebuilt_globally: bool,
}

/// Persistent Rust-side planner state for map updates.
///
/// Product and mapd adapters can own this value across queries. It keeps the
/// source evidence beside the prepared artifact so an update cannot silently
/// rebuild from a different occupied/free snapshot.
pub struct PlannerState {
    occupied: Vec<SurfaceCell>,
    known_free: Vec<SurfaceCell>,
    surface_config: SurfaceConfig,
    graph_config: GraphConfig,
    artifact: PlannerArtifact,
    incremental_graph: IncrementalGraph,
    surface_source: bool,
}

impl PlannerState {
    pub fn new(
        occupied: Vec<SurfaceCell>,
        known_free: Vec<SurfaceCell>,
        surface_config: SurfaceConfig,
        graph_config: GraphConfig,
    ) -> Self {
        let mut state = Self {
            occupied,
            known_free,
            surface_config,
            graph_config,
            artifact: PlannerArtifact::default(),
            incremental_graph: IncrementalGraph::default(),
            surface_source: false,
        };
        state.occupied.sort_unstable();
        state.occupied.dedup();
        state.known_free.sort_unstable();
        state.known_free.dedup();
        state.artifact = state.rebuild_artifact();
        state.incremental_graph =
            IncrementalGraph::from_cells(&graph_cells(&state.artifact.surface), graph_config);
        state
    }

    /// Prepare a graph from an already extracted surface map.
    ///
    /// The native adapter owns OctoMap extraction. This constructor keeps the
    /// Rust graph authoritative without re-interpreting the same voxels with a
    /// second surface extractor.
    pub fn from_surface(
        observed: Vec<SurfaceCell>,
        inferred: Vec<SurfaceCell>,
        graph_config: GraphConfig,
    ) -> Self {
        let mut surface = SurfaceMap { observed, inferred };
        surface.observed.sort_unstable();
        surface.observed.dedup();
        surface.inferred.sort_unstable();
        surface.inferred.dedup();
        let cells = graph_cells(&surface);
        let graph = edges::build_graph(&cells, graph_config);
        let incremental_graph = IncrementalGraph::from_cells(&cells, graph_config);
        Self {
            occupied: Vec::new(),
            known_free: Vec::new(),
            surface_config: SurfaceConfig::default(),
            graph_config,
            artifact: PlannerArtifact { surface, graph },
            incremental_graph,
            surface_source: true,
        }
    }

    pub fn artifact(&self) -> &PlannerArtifact {
        &self.artifact
    }

    pub fn occupied(&self) -> &[SurfaceCell] {
        &self.occupied
    }

    pub fn known_free(&self) -> &[SurfaceCell] {
        &self.known_free
    }

    pub fn update_region(
        &mut self,
        region: RegionBounds,
        replacement_occupied: &[SurfaceCell],
        replacement_known_free: &[SurfaceCell],
    ) -> RegionUpdateReport {
        if self.surface_source {
            return update_surface_region(
                &mut self.artifact,
                region,
                replacement_occupied,
                replacement_known_free,
                self.graph_config,
                &mut self.incremental_graph,
            );
        }
        update_region(
            &mut self.artifact,
            &mut self.occupied,
            &mut self.known_free,
            region,
            replacement_occupied,
            replacement_known_free,
            self.surface_config,
            self.graph_config,
            &mut self.incremental_graph,
        )
    }

    fn rebuild_artifact(&self) -> PlannerArtifact {
        build_artifact_with_free(
            &self.occupied,
            &self.known_free,
            self.surface_config,
            self.graph_config,
        )
    }
}

fn update_surface_region(
    artifact: &mut PlannerArtifact,
    region: RegionBounds,
    replacement_observed: &[SurfaceCell],
    replacement_inferred: &[SurfaceCell],
    graph_config: GraphConfig,
    incremental_graph: &mut IncrementalGraph,
) -> RegionUpdateReport {
    let old_nodes: HashMap<u32, SurfaceCell> = artifact
        .graph
        .nodes
        .iter()
        .map(|node| (node.id, node.cell))
        .collect();
    let affected_nodes = old_nodes
        .values()
        .filter(|cell| region.contains(**cell))
        .count();
    let affected_edges = artifact
        .graph
        .edges
        .iter()
        .filter(|edge| {
            old_nodes
                .get(&edge.a)
                .is_some_and(|cell| region.contains(*cell))
                || old_nodes
                    .get(&edge.b)
                    .is_some_and(|cell| region.contains(*cell))
                || edge.corridor.iter().any(|cell| region.contains(*cell))
        })
        .count();

    artifact
        .surface
        .observed
        .retain(|cell| !region.contains(*cell));
    artifact
        .surface
        .observed
        .extend_from_slice(replacement_observed);
    artifact.surface.observed.sort_unstable();
    artifact.surface.observed.dedup();
    artifact
        .surface
        .inferred
        .retain(|cell| !region.contains(*cell));
    artifact
        .surface
        .inferred
        .extend_from_slice(replacement_inferred);
    artifact.surface.inferred.sort_unstable();
    artifact.surface.inferred.dedup();

    let cells = graph_cells(&artifact.surface);
    let graph_report = incremental_graph.update(&cells, region, graph_config);
    let old_node_count = artifact.graph.nodes.len();
    artifact.graph = incremental_graph.to_surface_graph();
    RegionUpdateReport {
        expanded_region: region,
        replaced_occupied: replacement_observed.len(),
        replaced_known_free: replacement_inferred.len(),
        surface_cells_rebuilt: replacement_observed.len() + replacement_inferred.len(),
        affected_nodes,
        affected_edges,
        preserved_nodes: old_node_count.saturating_sub(affected_nodes),
        preserved_edges: artifact.graph.edges.len().saturating_sub(affected_edges),
        reused_nodes: old_node_count.saturating_sub(graph_report.removed_nodes),
        new_nodes: graph_report.added_nodes,
        removed_nodes: graph_report.removed_nodes,
        reused_edges: graph_report.reused_edges,
        rebuilt_edges: graph_report.rebuilt_edges,
        route_replan_required: affected_nodes > 0 || affected_edges > 0,
        graph_rebuilt_globally: graph_report.global_fallback,
    }
}

/// Build the complete static artifact from occupied voxels.
pub fn build_artifact(
    occupied: &[SurfaceCell],
    surface_config: SurfaceConfig,
    graph_config: GraphConfig,
) -> PlannerArtifact {
    build_artifact_with_free(occupied, &[], surface_config, graph_config)
}

fn build_artifact_with_free(
    occupied: &[SurfaceCell],
    known_free: &[SurfaceCell],
    surface_config: SurfaceConfig,
    graph_config: GraphConfig,
) -> PlannerArtifact {
    let surface = surfaces::extract_surfaces_with_free(occupied, known_free, surface_config);
    let mut graph_cells = surface.observed.clone();
    graph_cells.extend_from_slice(&surface.inferred);
    let graph = edges::build_graph(&graph_cells, graph_config);
    PlannerArtifact { surface, graph }
}

fn graph_cells(surface: &SurfaceMap) -> Vec<SurfaceCell> {
    let mut cells = surface.observed.clone();
    cells.extend_from_slice(&surface.inferred);
    cells
}

/// Apply one bounded source-map update and preserve stable node IDs where the
/// rebuilt graph still contains the same surface cell.
///
/// Surface extraction and graph repair are bounded to the changed region plus
/// the configured closing/clearance halo. The stable graph owns node and edge
/// identity across the update.
pub fn update_region(
    artifact: &mut PlannerArtifact,
    occupied: &mut Vec<SurfaceCell>,
    known_free: &mut Vec<SurfaceCell>,
    region: RegionBounds,
    replacement_occupied: &[SurfaceCell],
    replacement_known_free: &[SurfaceCell],
    surface_config: SurfaceConfig,
    graph_config: GraphConfig,
    incremental_graph: &mut IncrementalGraph,
) -> RegionUpdateReport {
    debug_assert!(replacement_occupied
        .iter()
        .all(|cell| region.contains(*cell)));
    debug_assert!(replacement_known_free
        .iter()
        .all(|cell| region.contains(*cell)));

    occupied.retain(|cell| !region.contains(*cell));
    occupied.extend_from_slice(replacement_occupied);
    occupied.sort_unstable();
    occupied.dedup();
    known_free.retain(|cell| !region.contains(*cell));
    known_free.extend_from_slice(replacement_known_free);
    known_free.sort_unstable();
    known_free.dedup();

    let expanded = region.expand(
        surface_config.closing_passes as i32 + 1,
        surface_config.clearance_cells.max(1) + 1,
    );
    let local_occupied: Vec<_> = occupied
        .iter()
        .copied()
        .filter(|cell| expanded.contains(*cell))
        .collect();
    let local_free: Vec<_> = known_free
        .iter()
        .copied()
        .filter(|cell| expanded.contains(*cell))
        .collect();
    let local_surface =
        surfaces::extract_surfaces_with_free(&local_occupied, &local_free, surface_config);
    let local_surface_cells = local_surface.observed.len() + local_surface.inferred.len();
    let old_node_cells: HashMap<u32, SurfaceCell> = artifact
        .graph
        .nodes
        .iter()
        .map(|node| (node.id, node.cell))
        .collect();
    let affected_nodes = old_node_cells
        .values()
        .filter(|cell| expanded.contains(**cell))
        .count();
    let affected_edges = artifact
        .graph
        .edges
        .iter()
        .filter(|edge| {
            old_node_cells
                .get(&edge.a)
                .is_some_and(|cell| expanded.contains(*cell))
                || old_node_cells
                    .get(&edge.b)
                    .is_some_and(|cell| expanded.contains(*cell))
                || edge.corridor.iter().any(|cell| expanded.contains(*cell))
        })
        .count();
    let preserved_nodes = artifact.graph.nodes.len().saturating_sub(affected_nodes);
    let preserved_edges = artifact.graph.edges.len().saturating_sub(affected_edges);
    let mut observed: Vec<_> = artifact
        .surface
        .observed
        .iter()
        .copied()
        .filter(|cell| !expanded.contains(*cell))
        .collect();
    observed.extend(local_surface.observed);
    observed.sort_unstable();
    observed.dedup();
    let mut inferred: Vec<_> = artifact
        .surface
        .inferred
        .iter()
        .copied()
        .filter(|cell| !expanded.contains(*cell))
        .collect();
    inferred.extend(local_surface.inferred);
    inferred.sort_unstable();
    inferred.dedup();
    artifact.surface = SurfaceMap { observed, inferred };

    let old_node_count = artifact.graph.nodes.len();
    let graph_cells = graph_cells(&artifact.surface);
    let graph_report = incremental_graph.update(&graph_cells, expanded, graph_config);
    artifact.graph = incremental_graph.to_surface_graph();
    RegionUpdateReport {
        expanded_region: expanded,
        replaced_occupied: replacement_occupied.len(),
        replaced_known_free: replacement_known_free.len(),
        surface_cells_rebuilt: local_surface_cells,
        affected_nodes,
        affected_edges,
        preserved_nodes,
        preserved_edges,
        reused_nodes: old_node_count.saturating_sub(graph_report.removed_nodes),
        new_nodes: graph_report.added_nodes,
        removed_nodes: graph_report.removed_nodes,
        reused_edges: graph_report.reused_edges,
        rebuilt_edges: graph_report.rebuilt_edges,
        route_replan_required: affected_nodes > 0 || affected_edges > 0,
        graph_rebuilt_globally: graph_report.global_fallback,
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LtMlsCell {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LtMlsConfig {
    pub clearance_cells: i32,
    pub closing_passes: u32,
    pub max_hole_cells: u64,
    pub max_slope: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LtMlsGraphConfig {
    pub node_spacing_cells: i32,
    pub max_link_cells: i32,
    pub wall_clearance_cells: i32,
    pub wall_buffer_cells: i32,
    pub wall_buffer_weight: f32,
    pub step_penalty_weight: f32,
    pub max_slope: f32,
    pub max_step_cells: i32,
    pub horizontal_hop_cells: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsGraphNode {
    pub id: u32,
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsGraphEdge {
    pub from: u32,
    pub to: u32,
    pub cost: f32,
    pub corridor_offset: u64,
    pub corridor_len: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsGraphOutput {
    pub nodes_ptr: *mut LtMlsGraphNode,
    pub nodes_len: u64,
    pub nodes_cap: u64,
    pub edges_ptr: *mut LtMlsGraphEdge,
    pub edges_len: u64,
    pub edges_cap: u64,
    pub corridor_ptr: *mut LtMlsCell,
    pub corridor_len: u64,
    pub corridor_cap: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsGraphSummary {
    pub node_count: u64,
    pub edge_count: u64,
    pub component_count: u64,
    pub corridor_cells: u64,
    pub max_corridor_cells: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsArtifactSummary {
    pub revision: u64,
    pub observed_surface_count: u64,
    pub inferred_surface_count: u64,
    pub node_count: u64,
    pub edge_count: u64,
    pub component_count: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsRegion {
    pub min_x: i32,
    pub max_x: i32,
    pub min_y: i32,
    pub max_y: i32,
    pub min_z: i32,
    pub max_z: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsUpdateReport {
    pub revision: u64,
    pub surface_cells_rebuilt: u64,
    pub affected_nodes: u64,
    pub affected_edges: u64,
    pub preserved_nodes: u64,
    pub preserved_edges: u64,
    pub reused_nodes: u64,
    pub new_nodes: u64,
    pub removed_nodes: u64,
    pub reused_edges: u64,
    pub rebuilt_edges: u64,
    pub route_replan_required: u8,
    pub graph_rebuilt_globally: u8,
}

#[allow(dead_code)]
pub struct LtMlsArtifactHandle {
    state: PlannerState,
    revision: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsBuffer {
    pub ptr: *mut LtMlsCell,
    pub len: u64,
    pub cap: u64,
}

pub const LT_MLS_ABI_VERSION: u32 = 5;

#[no_mangle]
pub extern "C" fn lt_mls_abi_version() -> u32 {
    LT_MLS_ABI_VERSION
}

fn put_buffer(cells: Vec<SurfaceCell>, output: &mut LtMlsBuffer) -> Result<(), ()> {
    let converted: Vec<LtMlsCell> = cells
        .into_iter()
        .map(|cell| LtMlsCell {
            x: cell.x,
            y: cell.y,
            z: cell.z,
        })
        .collect();
    let len = u64::try_from(converted.len()).map_err(|_| ())?;
    let cap = u64::try_from(converted.capacity()).map_err(|_| ())?;
    output.ptr = converted.as_ptr() as *mut LtMlsCell;
    output.len = len;
    output.cap = cap;
    std::mem::forget(converted);
    Ok(())
}

fn put_raw<T>(values: Vec<T>, ptr: &mut *mut T, len: &mut u64, cap: &mut u64) -> Result<(), ()> {
    *len = u64::try_from(values.len()).map_err(|_| ())?;
    *cap = u64::try_from(values.capacity()).map_err(|_| ())?;
    *ptr = values.as_ptr() as *mut T;
    std::mem::forget(values);
    Ok(())
}

fn graph_output(graph: SurfaceGraph, output: &mut LtMlsGraphOutput) -> Result<(), ()> {
    let nodes: Vec<LtMlsGraphNode> = graph
        .nodes
        .into_iter()
        .map(|node| LtMlsGraphNode {
            id: node.id,
            x: node.cell.x,
            y: node.cell.y,
            z: node.cell.z,
        })
        .collect();
    let mut corridor = Vec::new();
    let mut edges = Vec::with_capacity(graph.edges.len());
    for edge in graph.edges {
        let offset = u64::try_from(corridor.len()).map_err(|_| ())?;
        corridor.extend(edge.corridor.into_iter().map(|cell| LtMlsCell {
            x: cell.x,
            y: cell.y,
            z: cell.z,
        }));
        let length = u64::try_from(corridor.len()).map_err(|_| ())? - offset;
        edges.push(LtMlsGraphEdge {
            from: edge.a,
            to: edge.b,
            cost: edge.cost,
            corridor_offset: offset,
            corridor_len: length,
        });
    }
    put_raw(
        nodes,
        &mut output.nodes_ptr,
        &mut output.nodes_len,
        &mut output.nodes_cap,
    )?;
    if put_raw(
        edges,
        &mut output.edges_ptr,
        &mut output.edges_len,
        &mut output.edges_cap,
    )
    .is_err()
    {
        unsafe { lt_mls_free_graph(output) };
        return Err(());
    }
    if put_raw(
        corridor,
        &mut output.corridor_ptr,
        &mut output.corridor_len,
        &mut output.corridor_cap,
    )
    .is_err()
    {
        unsafe { lt_mls_free_graph(output) };
        return Err(());
    }
    Ok(())
}

#[allow(dead_code)]
fn graph_component_count(graph: &SurfaceGraph) -> usize {
    let mut adjacency = vec![Vec::<usize>::new(); graph.nodes.len()];
    let indices: HashMap<u32, usize> = graph
        .nodes
        .iter()
        .enumerate()
        .map(|(index, node)| (node.id, index))
        .collect();
    for edge in &graph.edges {
        let Some(&a) = indices.get(&edge.a) else {
            continue;
        };
        let Some(&b) = indices.get(&edge.b) else {
            continue;
        };
        adjacency[a].push(b);
        adjacency[b].push(a);
    }
    let mut visited = vec![false; graph.nodes.len()];
    let mut components = 0;
    for seed in 0..graph.nodes.len() {
        if visited[seed] {
            continue;
        }
        components += 1;
        let mut queue = VecDeque::from([seed]);
        visited[seed] = true;
        while let Some(node) = queue.pop_front() {
            for &next in &adjacency[node] {
                if !visited[next] {
                    visited[next] = true;
                    queue.push_back(next);
                }
            }
        }
    }
    components
}

#[allow(dead_code)]
fn graph_route(
    graph: &SurfaceGraph,
    start: SurfaceCell,
    goal: SurfaceCell,
) -> Option<Vec<SurfaceCell>> {
    graph_route_limited(graph, start, goal, None, None)
        .ok()
        .flatten()
}

fn graph_route_limited(
    graph: &SurfaceGraph,
    start: SurfaceCell,
    goal: SurfaceCell,
    max_snap_cells: Option<i32>,
    max_iterations: Option<usize>,
) -> Result<Option<Vec<SurfaceCell>>, bool> {
    if graph.nodes.is_empty() {
        return Ok(None);
    }
    let nearest_with_distance = |target: SurfaceCell| {
        graph
            .nodes
            .iter()
            .enumerate()
            .map(|(index, node)| {
                let dx = i64::from(node.cell.x) - i64::from(target.x);
                let dy = i64::from(node.cell.y) - i64::from(target.y);
                let dz = i64::from(node.cell.z) - i64::from(target.z);
                (dx * dx + dy * dy + dz * dz, index)
            })
            .min_by_key(|&(distance, _)| distance)
    };
    let Some((start_distance, start_index)) = nearest_with_distance(start) else {
        return Ok(None);
    };
    let Some((goal_distance, goal_index)) = nearest_with_distance(goal) else {
        return Ok(None);
    };
    if let Some(limit) = max_snap_cells {
        if limit < 0
            || start_distance > i64::from(limit) * i64::from(limit)
            || goal_distance > i64::from(limit) * i64::from(limit)
        {
            return Ok(None);
        }
    }
    let indices: HashMap<u32, usize> = graph
        .nodes
        .iter()
        .enumerate()
        .map(|(index, node)| (node.id, index))
        .collect();
    let mut adjacency = vec![Vec::<(usize, usize)>::new(); graph.nodes.len()];
    for (edge_index, edge) in graph.edges.iter().enumerate() {
        let (Some(&a), Some(&b)) = (indices.get(&edge.a), indices.get(&edge.b)) else {
            continue;
        };
        adjacency[a].push((b, edge_index));
        adjacency[b].push((a, edge_index));
    }
    let mut distances = vec![f32::INFINITY; graph.nodes.len()];
    let mut previous = vec![(usize::MAX, usize::MAX); graph.nodes.len()];
    let mut queue =
        std::collections::BinaryHeap::<std::cmp::Reverse<(corridor::OrderedF32, usize)>>::new();
    distances[start_index] = 0.0;
    queue.push(std::cmp::Reverse((corridor::OrderedF32(0.0), start_index)));
    let mut iterations = 0usize;
    while let Some(std::cmp::Reverse((corridor::OrderedF32(cost), current))) = queue.pop() {
        iterations += 1;
        if max_iterations.is_some_and(|limit| iterations > limit) {
            return Err(true);
        }
        if cost > distances[current] + 1e-6 {
            continue;
        }
        if current == goal_index {
            break;
        }
        for &(next, edge_index) in &adjacency[current] {
            let next_cost = cost + graph.edges[edge_index].cost;
            if next_cost + 1e-6 >= distances[next] {
                continue;
            }
            distances[next] = next_cost;
            previous[next] = (current, edge_index);
            queue.push(std::cmp::Reverse((corridor::OrderedF32(next_cost), next)));
        }
    }
    if !distances[goal_index].is_finite() {
        return Ok(None);
    }
    let mut node_path = Vec::new();
    let mut current = goal_index;
    node_path.push(current);
    while current != start_index {
        let (parent, _) = previous[current];
        if parent == usize::MAX {
            return Ok(None);
        }
        current = parent;
        node_path.push(current);
    }
    node_path.reverse();

    let mut route = vec![start];
    for pair in node_path.windows(2) {
        let (from, to) = (pair[0], pair[1]);
        let (_, edge_index) = previous[to];
        let edge = &graph.edges[edge_index];
        let forward = graph.nodes[from].id == edge.a;
        let cells: Box<dyn Iterator<Item = &SurfaceCell>> = if forward {
            Box::new(edge.corridor.iter())
        } else {
            Box::new(edge.corridor.iter().rev())
        };
        for &cell in cells {
            if route.last() != Some(&cell) {
                route.push(cell);
            }
        }
    }
    if route.last() != Some(&goal) {
        route.push(goal);
    }
    Ok(Some(route))
}

unsafe fn read_cells(ptr: *const LtMlsCell, len: u64) -> Result<Vec<SurfaceCell>, i32> {
    if len > 0 && ptr.is_null() {
        return Err(-1);
    }
    let len = usize::try_from(len).map_err(|_| -3)?;
    if len == 0 {
        return Ok(Vec::new());
    }
    Ok(std::slice::from_raw_parts(ptr, len)
        .iter()
        .map(|cell| SurfaceCell::new(cell.x, cell.y, cell.z))
        .collect())
}

fn region_from_ffi(region: LtMlsRegion) -> RegionBounds {
    RegionBounds::new(
        region.min_x,
        region.max_x,
        region.min_y,
        region.max_y,
        region.min_z,
        region.max_z,
    )
}

#[no_mangle]
/// # Safety
///
/// All input buffers must point to readable cells for the duration of the call.
/// `out` must be a valid writable handle pointer and is initialized on success.
pub unsafe extern "C" fn lt_mls_artifact_create(
    occupied: *const LtMlsCell,
    occupied_len: u64,
    known_free: *const LtMlsCell,
    known_free_len: u64,
    surface_config: *const LtMlsConfig,
    graph_config: *const LtMlsGraphConfig,
    revision: u64,
    out: *mut *mut LtMlsArtifactHandle,
) -> i32 {
    if surface_config.is_null() || graph_config.is_null() || out.is_null() {
        return -1;
    }
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let occupied = read_cells(occupied, occupied_len)?;
        let known_free = read_cells(known_free, known_free_len)?;
        let surface_config = *surface_config;
        let graph_config = *graph_config;
        if surface_config.clearance_cells < 0
            || graph_config.node_spacing_cells < 1
            || graph_config.max_link_cells < 1
        {
            return Err(-2);
        }
        let state = PlannerState::new(
            occupied,
            known_free,
            SurfaceConfig {
                clearance_cells: surface_config.clearance_cells,
                closing_passes: surface_config.closing_passes,
                max_hole_cells: usize::try_from(surface_config.max_hole_cells).map_err(|_| -3)?,
                max_slope: surface_config.max_slope,
            },
            GraphConfig {
                node_spacing_cells: graph_config.node_spacing_cells,
                max_link_cells: graph_config.max_link_cells,
                wall_clearance_cells: graph_config.wall_clearance_cells,
                wall_buffer_cells: graph_config.wall_buffer_cells,
                wall_buffer_weight: graph_config.wall_buffer_weight,
                step_penalty_weight: graph_config.step_penalty_weight,
                max_slope: graph_config.max_slope,
                max_step_cells: graph_config.max_step_cells,
                horizontal_hop_cells: graph_config.horizontal_hop_cells,
            },
        );
        *out = Box::into_raw(Box::new(LtMlsArtifactHandle { state, revision }));
        Ok(0)
    }));
    match result {
        Ok(Ok(code)) => code,
        Ok(Err(code)) => code,
        Err(_) => -4,
    }
}

#[no_mangle]
/// # Safety
///
/// `observed` and `inferred` must point to readable cells for the duration of
/// the call. `graph_config` and `out` must be valid writable/readable pointers.
pub unsafe extern "C" fn lt_mls_artifact_create_from_surface(
    observed: *const LtMlsCell,
    observed_len: u64,
    inferred: *const LtMlsCell,
    inferred_len: u64,
    graph_config: *const LtMlsGraphConfig,
    revision: u64,
    out: *mut *mut LtMlsArtifactHandle,
) -> i32 {
    if graph_config.is_null() || out.is_null() {
        return -1;
    }
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let observed = read_cells(observed, observed_len)?;
        let inferred = read_cells(inferred, inferred_len)?;
        let graph_config = *graph_config;
        if graph_config.node_spacing_cells < 1 || graph_config.max_link_cells < 1 {
            return Err(-2);
        }
        let state = PlannerState::from_surface(
            observed,
            inferred,
            GraphConfig {
                node_spacing_cells: graph_config.node_spacing_cells,
                max_link_cells: graph_config.max_link_cells,
                wall_clearance_cells: graph_config.wall_clearance_cells,
                wall_buffer_cells: graph_config.wall_buffer_cells,
                wall_buffer_weight: graph_config.wall_buffer_weight,
                step_penalty_weight: graph_config.step_penalty_weight,
                max_slope: graph_config.max_slope,
                max_step_cells: graph_config.max_step_cells,
                horizontal_hop_cells: graph_config.horizontal_hop_cells,
            },
        );
        *out = Box::into_raw(Box::new(LtMlsArtifactHandle { state, revision }));
        Ok(0)
    }));
    match result {
        Ok(Ok(code)) => code,
        Ok(Err(code)) => code,
        Err(_) => -4,
    }
}

#[no_mangle]
/// # Safety
///
/// `handle` must be null or a handle returned by `lt_mls_artifact_create` that
/// has not already been freed.
pub unsafe extern "C" fn lt_mls_artifact_free(handle: *mut LtMlsArtifactHandle) {
    if !handle.is_null() {
        drop(Box::from_raw(handle));
    }
}

#[no_mangle]
/// # Safety
///
/// `handle` and `summary` must be valid pointers for the duration of the call.
pub unsafe extern "C" fn lt_mls_artifact_summary(
    handle: *const LtMlsArtifactHandle,
    summary: *mut LtMlsArtifactSummary,
) -> i32 {
    if handle.is_null() || summary.is_null() {
        return -1;
    }
    let handle = &*handle;
    let artifact = handle.state.artifact();
    *summary = LtMlsArtifactSummary {
        revision: handle.revision,
        observed_surface_count: artifact.surface.observed.len() as u64,
        inferred_surface_count: artifact.surface.inferred.len() as u64,
        node_count: artifact.graph.nodes.len() as u64,
        edge_count: artifact.graph.edges.len() as u64,
        component_count: graph_component_count(&artifact.graph) as u64,
    };
    0
}

#[no_mangle]
/// # Safety
///
/// `handle` and output buffers must be valid. Existing output buffers are freed
/// before replacement.
pub unsafe extern "C" fn lt_mls_artifact_surfaces(
    handle: *const LtMlsArtifactHandle,
    observed: *mut LtMlsBuffer,
    inferred: *mut LtMlsBuffer,
) -> i32 {
    if handle.is_null() || observed.is_null() || inferred.is_null() {
        return -1;
    }
    let handle = &*handle;
    let observed = &mut *observed;
    let inferred = &mut *inferred;
    lt_mls_free_buffer(observed);
    lt_mls_free_buffer(inferred);
    if put_buffer(handle.state.artifact().surface.observed.clone(), observed).is_err() {
        return -3;
    }
    if put_buffer(handle.state.artifact().surface.inferred.clone(), inferred).is_err() {
        lt_mls_free_buffer(observed);
        return -3;
    }
    0
}

#[no_mangle]
/// # Safety
///
/// `handle` and `result` must be valid pointers for the duration of the call.
pub unsafe extern "C" fn lt_mls_artifact_below(
    handle: *const LtMlsArtifactHandle,
    x: i32,
    y: i32,
    z: i32,
    result: *mut i32,
) -> i32 {
    if handle.is_null() || result.is_null() {
        return -1;
    }
    let artifact = &*handle;
    let floor = artifact
        .state
        .artifact()
        .surface
        .observed
        .iter()
        .chain(artifact.state.artifact().surface.inferred.iter())
        .filter(|cell| cell.x == x && cell.y == y && cell.z <= z)
        .map(|cell| cell.z)
        .max();
    match floor {
        Some(value) => {
            *result = value;
            1
        }
        None => 0,
    }
}

#[no_mangle]
/// # Safety
///
/// `handle` must be a valid artifact pointer for the duration of the call.
pub unsafe extern "C" fn lt_mls_artifact_inferred(
    handle: *const LtMlsArtifactHandle,
    x: i32,
    y: i32,
    min_z: i32,
    max_z: i32,
) -> i32 {
    if handle.is_null() {
        return -1;
    }
    let artifact = &*handle;
    if artifact
        .state
        .artifact()
        .surface
        .inferred
        .iter()
        .any(|cell| cell.x == x && cell.y == y && cell.z >= min_z && cell.z <= max_z)
    {
        1
    } else {
        0
    }
}

#[no_mangle]
/// # Safety
///
/// `handle` and `cells` must be valid pointers. The returned buffer is owned
/// by the caller and must be released with `lt_mls_free_buffer`.
pub unsafe extern "C" fn lt_mls_artifact_occupied_cells(
    handle: *const LtMlsArtifactHandle,
    cells: *mut LtMlsBuffer,
) -> i32 {
    if handle.is_null() || cells.is_null() {
        return -1;
    }
    let handle = &*handle;
    let cells = &mut *cells;
    lt_mls_free_buffer(cells);
    put_buffer(handle.state.occupied().to_vec(), cells).map_or(-2, |_| 0)
}

#[no_mangle]
/// # Safety
///
/// `handle` and `cells` must be valid pointers. The returned buffer is owned
/// by the caller and must be released with `lt_mls_free_buffer`.
pub unsafe extern "C" fn lt_mls_artifact_known_free_cells(
    handle: *const LtMlsArtifactHandle,
    cells: *mut LtMlsBuffer,
) -> i32 {
    if handle.is_null() || cells.is_null() {
        return -1;
    }
    let handle = &*handle;
    let cells = &mut *cells;
    lt_mls_free_buffer(cells);
    put_buffer(handle.state.known_free().to_vec(), cells).map_or(-2, |_| 0)
}

#[no_mangle]
/// # Safety
///
/// `handle` and `result` must be valid pointers for the duration of the call.
pub unsafe extern "C" fn lt_mls_artifact_height(
    handle: *const LtMlsArtifactHandle,
    x: i32,
    y: i32,
    expected_z: f64,
    tolerance: f64,
    resolution: f64,
    result: *mut f64,
) -> i32 {
    if handle.is_null() || result.is_null() {
        return -1;
    }
    if !expected_z.is_finite()
        || !tolerance.is_finite()
        || tolerance < 0.0
        || !resolution.is_finite()
        || resolution <= 0.0
    {
        return -2;
    }
    let artifact = &*handle;
    let mut best = None;
    for cell in artifact
        .state
        .artifact()
        .surface
        .observed
        .iter()
        .chain(artifact.state.artifact().surface.inferred.iter())
    {
        if cell.x != x || cell.y != y {
            continue;
        }
        let lower = f64::from(cell.z) * resolution;
        let upper = f64::from(cell.z + 1) * resolution;
        let value = expected_z.clamp(lower, upper);
        if (value - expected_z).abs() > tolerance + 1e-7 {
            continue;
        }
        if (value - expected_z).abs() <= 1e-7 {
            *result = value;
            return 1;
        }
        if best.map_or(true, |current: f64| {
            (value - expected_z).abs() < (current - expected_z).abs()
        }) {
            best = Some(value);
        }
    }
    match best {
        Some(value) => {
            *result = value;
            1
        }
        None => 0,
    }
}

#[no_mangle]
/// # Safety
///
/// `handle` and `output` must be valid pointers. The returned graph is owned by
/// the caller and must be released with `lt_mls_free_graph`.
pub unsafe extern "C" fn lt_mls_artifact_graph(
    handle: *const LtMlsArtifactHandle,
    output: *mut LtMlsGraphOutput,
) -> i32 {
    if handle.is_null() || output.is_null() {
        return -1;
    }
    *output = LtMlsGraphOutput::default();
    let graph = &*handle;
    match graph_output(graph.state.artifact().graph.clone(), &mut *output) {
        Ok(()) => 0,
        Err(()) => -3,
    }
}

#[no_mangle]
/// # Safety
///
/// `handle`, `route` and input cells must be valid pointers. The route is owned
/// by the caller and must be released with `lt_mls_free_buffer`.
pub unsafe extern "C" fn lt_mls_artifact_query(
    handle: *const LtMlsArtifactHandle,
    start: LtMlsCell,
    goal: LtMlsCell,
    route: *mut LtMlsBuffer,
) -> i32 {
    if handle.is_null() || route.is_null() {
        return -1;
    }
    let handle = &*handle;
    let route = &mut *route;
    lt_mls_free_buffer(route);
    let Some(path) = graph_route(
        &handle.state.artifact().graph,
        SurfaceCell::new(start.x, start.y, start.z),
        SurfaceCell::new(goal.x, goal.y, goal.z),
    ) else {
        return -2;
    };
    put_buffer(path, route).map_or(-3, |_| 0)
}

#[no_mangle]
/// # Safety
///
/// `handle`, `route` and input cells must be valid pointers. The route is owned
/// by the caller and must be released with `lt_mls_free_buffer`.
pub unsafe extern "C" fn lt_mls_artifact_query_limited(
    handle: *const LtMlsArtifactHandle,
    start: LtMlsCell,
    goal: LtMlsCell,
    max_snap_cells: i32,
    route: *mut LtMlsBuffer,
) -> i32 {
    if handle.is_null() || route.is_null() {
        return -1;
    }
    let handle = &*handle;
    let route = &mut *route;
    lt_mls_free_buffer(route);
    let Some(path) = graph_route_limited(
        &handle.state.artifact().graph,
        SurfaceCell::new(start.x, start.y, start.z),
        SurfaceCell::new(goal.x, goal.y, goal.z),
        Some(max_snap_cells),
        None,
    )
    .ok()
    .flatten() else {
        return -2;
    };
    put_buffer(path, route).map_or(-3, |_| 0)
}

#[no_mangle]
/// # Safety
///
/// `handle`, `route` and input cells must be valid pointers. The route is owned
/// by the caller and must be released with `lt_mls_free_buffer`.
pub unsafe extern "C" fn lt_mls_artifact_query_budgeted(
    handle: *const LtMlsArtifactHandle,
    start: LtMlsCell,
    goal: LtMlsCell,
    max_snap_cells: i32,
    max_iterations: i32,
    route: *mut LtMlsBuffer,
) -> i32 {
    if handle.is_null() || route.is_null() || max_iterations < 1 {
        return -1;
    }
    let handle = &*handle;
    let route = &mut *route;
    lt_mls_free_buffer(route);
    let result = graph_route_limited(
        &handle.state.artifact().graph,
        SurfaceCell::new(start.x, start.y, start.z),
        SurfaceCell::new(goal.x, goal.y, goal.z),
        Some(max_snap_cells),
        Some(max_iterations as usize),
    );
    match result {
        Err(true) => -3,
        Ok(Some(path)) => put_buffer(path, route).map_or(-4, |_| 0),
        Ok(None) => -2,
        Err(false) => -2,
    }
}

#[no_mangle]
/// # Safety
///
/// All buffers and the handle must be valid for the duration of the call.
pub unsafe extern "C" fn lt_mls_artifact_update(
    handle: *mut LtMlsArtifactHandle,
    region: LtMlsRegion,
    replacement_occupied: *const LtMlsCell,
    replacement_occupied_len: u64,
    replacement_known_free: *const LtMlsCell,
    replacement_known_free_len: u64,
    report: *mut LtMlsUpdateReport,
) -> i32 {
    if handle.is_null() || report.is_null() {
        return -1;
    }
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let handle = &mut *handle;
        let occupied = read_cells(replacement_occupied, replacement_occupied_len)?;
        let known_free = read_cells(replacement_known_free, replacement_known_free_len)?;
        let raw = handle
            .state
            .update_region(region_from_ffi(region), &occupied, &known_free);
        handle.revision = handle.revision.saturating_add(1);
        *report = LtMlsUpdateReport {
            revision: handle.revision,
            surface_cells_rebuilt: raw.surface_cells_rebuilt as u64,
            affected_nodes: raw.affected_nodes as u64,
            affected_edges: raw.affected_edges as u64,
            preserved_nodes: raw.preserved_nodes as u64,
            preserved_edges: raw.preserved_edges as u64,
            reused_nodes: raw.reused_nodes as u64,
            new_nodes: raw.new_nodes as u64,
            removed_nodes: raw.removed_nodes as u64,
            reused_edges: raw.reused_edges as u64,
            rebuilt_edges: raw.rebuilt_edges as u64,
            route_replan_required: raw.route_replan_required as u8,
            graph_rebuilt_globally: raw.graph_rebuilt_globally as u8,
        };
        Ok(0)
    }));
    match result {
        Ok(Ok(code)) => code,
        Ok(Err(code)) => code,
        Err(_) => -4,
    }
}

#[no_mangle]
/// # Safety
///
/// `occupied` must point to `occupied_len` readable cells. `config`, `observed`
/// and `inferred` must be valid writable pointers for the duration of the call.
pub unsafe extern "C" fn lt_mls_extract_surfaces(
    occupied: *const LtMlsCell,
    occupied_len: u64,
    known_free: *const LtMlsCell,
    known_free_len: u64,
    config: *const LtMlsConfig,
    observed: *mut LtMlsBuffer,
    inferred: *mut LtMlsBuffer,
) -> i32 {
    if occupied.is_null()
        || (known_free_len > 0 && known_free.is_null())
        || config.is_null()
        || observed.is_null()
        || inferred.is_null()
    {
        return -1;
    }
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let config = *config;
        if config.clearance_cells < 0 {
            return Err(-2);
        }
        let len = usize::try_from(occupied_len).map_err(|_| -3)?;
        let input = std::slice::from_raw_parts(occupied, len);
        let cells: Vec<SurfaceCell> = input
            .iter()
            .map(|cell| SurfaceCell::new(cell.x, cell.y, cell.z))
            .collect();
        let free_len = usize::try_from(known_free_len).map_err(|_| -3)?;
        let free = if free_len == 0 {
            &[]
        } else {
            std::slice::from_raw_parts(known_free, free_len)
        };
        let free_cells: Vec<SurfaceCell> = free
            .iter()
            .map(|cell| SurfaceCell::new(cell.x, cell.y, cell.z))
            .collect();
        let max_hole_cells = usize::try_from(config.max_hole_cells).map_err(|_| -3)?;
        let map = extract_surfaces_with_free(
            &cells,
            &free_cells,
            SurfaceConfig {
                clearance_cells: config.clearance_cells,
                closing_passes: config.closing_passes,
                max_hole_cells,
                max_slope: config.max_slope,
            },
        );
        let observed_output = &mut *observed;
        let inferred_output = &mut *inferred;
        *observed_output = LtMlsBuffer::default();
        *inferred_output = LtMlsBuffer::default();
        put_buffer(map.observed, observed_output).map_err(|_| -3)?;
        if put_buffer(map.inferred, inferred_output).is_err() {
            lt_mls_free_buffer(observed_output);
            return Err(-3);
        }
        Ok(0)
    }));
    match result {
        Ok(Ok(code)) => code,
        Ok(Err(code)) => code,
        Err(_) => -4,
    }
}

#[no_mangle]
/// # Safety
///
/// `surfaces` must point to `surface_len` readable cells. `config` and
/// `summary` must be valid pointers for the duration of the call.
pub unsafe extern "C" fn lt_mls_build_graph_summary(
    surfaces: *const LtMlsCell,
    surface_len: u64,
    config: *const LtMlsGraphConfig,
    summary: *mut LtMlsGraphSummary,
) -> i32 {
    if surfaces.is_null() || config.is_null() || summary.is_null() {
        return -1;
    }
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let config = *config;
        if config.node_spacing_cells < 1 || config.max_link_cells < 1 {
            return Err(-2);
        }
        let len = usize::try_from(surface_len).map_err(|_| -3)?;
        let input = std::slice::from_raw_parts(surfaces, len);
        let cells: Vec<SurfaceCell> = input
            .iter()
            .map(|cell| SurfaceCell::new(cell.x, cell.y, cell.z))
            .collect();
        let graph = build_graph(
            &cells,
            GraphConfig {
                node_spacing_cells: config.node_spacing_cells,
                max_link_cells: config.max_link_cells,
                wall_clearance_cells: config.wall_clearance_cells,
                wall_buffer_cells: config.wall_buffer_cells,
                wall_buffer_weight: config.wall_buffer_weight,
                step_penalty_weight: config.step_penalty_weight,
                max_slope: config.max_slope,
                max_step_cells: config.max_step_cells,
                horizontal_hop_cells: config.horizontal_hop_cells,
                ..GraphConfig::default()
            },
        );
        let mut adjacency = vec![Vec::<u32>::new(); graph.nodes.len()];
        let mut corridor_cells = 0_u64;
        let mut max_corridor_cells = 0_u64;
        for edge in &graph.edges {
            adjacency[edge.a as usize].push(edge.b);
            adjacency[edge.b as usize].push(edge.a);
            let length = u64::try_from(edge.corridor.len()).map_err(|_| -3)?;
            corridor_cells = corridor_cells.checked_add(length).ok_or(-3)?;
            max_corridor_cells = max_corridor_cells.max(length);
        }
        let mut visited = vec![false; graph.nodes.len()];
        let mut component_count = 0_u64;
        for seed in 0..graph.nodes.len() {
            if visited[seed] {
                continue;
            }
            component_count += 1;
            let mut queue = VecDeque::from([seed]);
            visited[seed] = true;
            while let Some(node) = queue.pop_front() {
                for &next in &adjacency[node] {
                    let next = next as usize;
                    if !visited[next] {
                        visited[next] = true;
                        queue.push_back(next);
                    }
                }
            }
        }
        *summary = LtMlsGraphSummary {
            node_count: u64::try_from(graph.nodes.len()).map_err(|_| -3)?,
            edge_count: u64::try_from(graph.edges.len()).map_err(|_| -3)?,
            component_count,
            corridor_cells,
            max_corridor_cells,
        };
        Ok(0)
    }));
    match result {
        Ok(Ok(code)) => code,
        Ok(Err(code)) => code,
        Err(_) => -4,
    }
}

#[no_mangle]
/// # Safety
///
/// `surfaces` must point to `surface_len` readable cells. `config` and
/// `output` must be valid pointers for the duration of the call.
pub unsafe extern "C" fn lt_mls_build_graph(
    surfaces: *const LtMlsCell,
    surface_len: u64,
    config: *const LtMlsGraphConfig,
    output: *mut LtMlsGraphOutput,
) -> i32 {
    if (surface_len > 0 && surfaces.is_null()) || config.is_null() || output.is_null() {
        return -1;
    }
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let config = *config;
        if config.node_spacing_cells < 1 || config.max_link_cells < 1 {
            return Err(-2);
        }
        let len = usize::try_from(surface_len).map_err(|_| -3)?;
        let input = if len == 0 {
            &[]
        } else {
            std::slice::from_raw_parts(surfaces, len)
        };
        let cells: Vec<SurfaceCell> = input
            .iter()
            .map(|cell| SurfaceCell::new(cell.x, cell.y, cell.z))
            .collect();
        let graph = build_graph(
            &cells,
            GraphConfig {
                node_spacing_cells: config.node_spacing_cells,
                max_link_cells: config.max_link_cells,
                wall_clearance_cells: config.wall_clearance_cells,
                wall_buffer_cells: config.wall_buffer_cells,
                wall_buffer_weight: config.wall_buffer_weight,
                step_penalty_weight: config.step_penalty_weight,
                max_slope: config.max_slope,
                max_step_cells: config.max_step_cells,
                horizontal_hop_cells: config.horizontal_hop_cells,
            },
        );
        let output = &mut *output;
        *output = LtMlsGraphOutput::default();
        graph_output(graph, output).map_err(|_| -3)?;
        Ok(0)
    }));
    match result {
        Ok(Ok(code)) => code,
        Ok(Err(code)) => code,
        Err(_) => -4,
    }
}

#[no_mangle]
/// # Safety
///
/// `output` must be a graph returned by `lt_mls_build_graph` and must not be
/// freed more than once.
pub unsafe extern "C" fn lt_mls_free_graph(output: *mut LtMlsGraphOutput) {
    if output.is_null() {
        return;
    }
    let output = &mut *output;
    if !output.nodes_ptr.is_null() {
        let _ = Vec::from_raw_parts(
            output.nodes_ptr,
            output.nodes_len as usize,
            output.nodes_cap as usize,
        );
    }
    if !output.edges_ptr.is_null() {
        let _ = Vec::from_raw_parts(
            output.edges_ptr,
            output.edges_len as usize,
            output.edges_cap as usize,
        );
    }
    if !output.corridor_ptr.is_null() {
        let _ = Vec::from_raw_parts(
            output.corridor_ptr,
            output.corridor_len as usize,
            output.corridor_cap as usize,
        );
    }
    *output = LtMlsGraphOutput::default();
}

#[no_mangle]
/// # Safety
///
/// `buffer` must be a buffer returned by `lt_mls_extract_surfaces` and must
/// not be freed more than once.
pub unsafe extern "C" fn lt_mls_free_buffer(buffer: *mut LtMlsBuffer) {
    if buffer.is_null() {
        return;
    }
    let buffer = &mut *buffer;
    if !buffer.ptr.is_null() {
        let _ = Vec::from_raw_parts(buffer.ptr, buffer.len as usize, buffer.cap as usize);
    }
    *buffer = LtMlsBuffer::default();
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    #[test]
    fn preserves_multiple_heights_in_one_column() {
        let occupied = [
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(0, 0, 3),
            SurfaceCell::new(1, 0, 0),
            SurfaceCell::new(1, 0, 3),
        ];
        let map = extract_surfaces(
            &occupied,
            SurfaceConfig {
                clearance_cells: 1,
                ..SurfaceConfig::default()
            },
        );
        assert!(map.observed.contains(&SurfaceCell::new(0, 0, 0)));
        assert!(map.observed.contains(&SurfaceCell::new(0, 0, 3)));
        assert!(map.observed.contains(&SurfaceCell::new(1, 0, 0)));
        assert!(map.observed.contains(&SurfaceCell::new(1, 0, 3)));
    }

    #[test]
    fn graph_route_returns_endpoint_cells() {
        let cells: Vec<_> = (0..12).map(|x| SurfaceCell::new(x, 0, 0)).collect();
        let graph = edges::build_graph(
            &cells,
            GraphConfig {
                node_spacing_cells: 3,
                max_link_cells: 1,
                ..GraphConfig::default()
            },
        );
        let route = graph_route(
            &graph,
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(11, 0, 0),
        )
        .expect("graph route");
        assert_eq!(route.first(), Some(&SurfaceCell::new(0, 0, 0)));
        assert_eq!(route.last(), Some(&SurfaceCell::new(11, 0, 0)));
    }

    #[test]
    fn graph_respects_slope_with_a_longer_hop() {
        let cells = [
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(1, 0, 1),
            SurfaceCell::new(2, 0, 1),
        ];
        let graph_config = GraphConfig {
            node_spacing_cells: 1,
            max_link_cells: 1,
            max_slope: 0.5,
            horizontal_hop_cells: 2,
            ..GraphConfig::default()
        };
        let links = adjacency::build_adjacency(&cells, graph_config);
        assert!(links[0].iter().all(|(index, _)| *index != 1));
        assert!(links[0].iter().any(|(index, _)| *index == 2));
    }

    #[test]
    fn fills_a_small_bounded_hole_per_height() {
        let mut occupied = Vec::new();
        for y in 0..3 {
            for x in 0..3 {
                if x != 1 || y != 1 {
                    occupied.push(SurfaceCell::new(x, y, 0));
                }
            }
        }
        let map = extract_surfaces(
            &occupied,
            SurfaceConfig {
                clearance_cells: 1,
                closing_passes: 1,
                max_hole_cells: 1,
                ..SurfaceConfig::default()
            },
        );
        assert_eq!(map.inferred, vec![SurfaceCell::new(1, 1, 0)]);
    }

    #[test]
    fn leaves_open_holes_unknown() {
        let occupied = [SurfaceCell::new(0, 0, 0), SurfaceCell::new(1, 0, 0)];
        let map = extract_surfaces(
            &occupied,
            SurfaceConfig {
                clearance_cells: 1,
                closing_passes: 1,
                max_hole_cells: 8,
                ..SurfaceConfig::default()
            },
        );
        assert!(map.inferred.is_empty());
    }

    #[test]
    fn builds_sparse_nodes_and_deduplicated_edges() {
        let cells = [
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(1, 0, 0),
            SurfaceCell::new(2, 0, 0),
            SurfaceCell::new(3, 0, 0),
        ];
        let graph = edges::build_graph(
            &cells,
            GraphConfig {
                node_spacing_cells: 2,
                max_link_cells: 4,
                ..GraphConfig::default()
            },
        );
        assert_eq!(graph.nodes.len(), 2);
        assert_eq!(graph.edges.len(), 1);
        assert!(graph.edges[0].corridor.len() >= 2);
    }

    #[test]
    fn corridors_follow_surface_predecessors() {
        let cells: Vec<_> = (0..12).map(|x| SurfaceCell::new(x, 0, 0)).collect();
        let graph = edges::build_graph(
            &cells,
            GraphConfig {
                node_spacing_cells: 3,
                max_link_cells: 1,
                ..GraphConfig::default()
            },
        );
        assert!(graph.nodes.len() >= 3);
        assert!(!graph.edges.is_empty());
        assert!(graph.edges.iter().any(|edge| edge.corridor.len() > 2));
    }

    #[test]
    fn graph_does_not_bridge_separate_surface_components() {
        let mut cells = Vec::new();
        for x in 0..3 {
            for y in 0..3 {
                cells.push(SurfaceCell::new(x, y, 0));
                cells.push(SurfaceCell::new(x + 10, y, 0));
            }
        }
        let graph = edges::build_graph(
            &cells,
            GraphConfig {
                node_spacing_cells: 2,
                max_link_cells: 1,
                ..GraphConfig::default()
            },
        );
        assert!(graph.edges.iter().all(|edge| {
            let a = graph.nodes[edge.a as usize].cell;
            let b = graph.nodes[edge.b as usize].cell;
            (a.x - b.x).abs() < 5
        }));
    }

    #[test]
    fn incremental_graph_repairs_edges_inside_window() {
        let cells = (0..12)
            .flat_map(|x| (0..3).map(move |y| SurfaceCell::new(x, y, 0)))
            .collect::<Vec<_>>();
        let config = GraphConfig {
            node_spacing_cells: 2,
            max_link_cells: 1,
            ..GraphConfig::default()
        };
        let mut graph = IncrementalGraph::from_cells(&cells, config);
        let original_edge_ids = graph.edges.keys().copied().collect::<HashSet<_>>();
        let mut updated = cells.clone();
        updated.retain(|cell| cell.x != 5 || cell.y != 1);
        let report = graph.update(&updated, RegionBounds::new(5, 5, 1, 1, 0, 0), config);
        assert!(report.local_repair);
        assert!(report.changed_cells > 0);
        assert!(report.invalidated_edges > 0);
        assert!(graph.cells.contains(&SurfaceCell::new(4, 1, 0)));
        assert!(!graph.cells.contains(&SurfaceCell::new(5, 1, 0)));
        assert!(graph.edges.keys().any(|id| original_edge_ids.contains(id)));
        assert!(graph.edges.values().all(|edge| edge.id
            == crate::stable_graph::edge_id(edge.from, edge.to)
            && !edge.corridor.is_empty()));
    }

    #[test]
    fn incremental_graph_falls_back_when_update_removes_the_local_graph() {
        let cells = (0..4)
            .map(|x| SurfaceCell::new(x, 0, 0))
            .collect::<Vec<_>>();
        let config = GraphConfig {
            node_spacing_cells: 1,
            max_link_cells: 1,
            ..GraphConfig::default()
        };
        let mut graph = IncrementalGraph::from_cells(&cells, config);
        let report = graph.update(&[], RegionBounds::new(0, 3, 0, 0, 0, 0), config);
        assert!(report.global_fallback);
        assert!(!report.local_repair);
        assert!(graph.nodes.is_empty());
        assert!(graph.edges.is_empty());
    }

    #[test]
    fn update_region_matches_full_surface_rebuild_and_reuses_nodes() {
        let mut occupied = Vec::new();
        for x in 0..10 {
            for y in 0..3 {
                occupied.push(SurfaceCell::new(x, y, 0));
            }
        }
        let surface_config = SurfaceConfig::default();
        let graph_config = GraphConfig {
            node_spacing_cells: 2,
            max_link_cells: 1,
            ..GraphConfig::default()
        };
        let mut artifact = build_artifact(&occupied, surface_config, graph_config);
        let mut known_free = Vec::new();
        let old_ids: HashMap<SurfaceCell, u32> = artifact
            .graph
            .nodes
            .iter()
            .map(|node| (node.cell, node.id))
            .collect();
        let mut incremental_graph = IncrementalGraph::from_cells(
            &artifact
                .surface
                .observed
                .iter()
                .chain(artifact.surface.inferred.iter())
                .copied()
                .collect::<Vec<_>>(),
            graph_config,
        );

        let report = update_region(
            &mut artifact,
            &mut occupied,
            &mut known_free,
            RegionBounds::new(2, 2, 1, 1, 0, 0),
            &[],
            &[],
            surface_config,
            graph_config,
            &mut incremental_graph,
        );
        let expected = build_artifact(&occupied, surface_config, graph_config);
        assert_eq!(artifact.surface, expected.surface);
        assert!(!report.graph_rebuilt_globally);
        assert!(report.affected_nodes > 0);
        assert!(report.route_replan_required);
        assert!(report.preserved_nodes > 0);
        assert!(report.reused_nodes > 0);
        for node in &artifact.graph.nodes {
            if let Some(&old_id) = old_ids.get(&node.cell) {
                assert_eq!(node.id, old_id);
            }
        }
    }

    #[test]
    fn planner_state_owns_source_and_artifact_together() {
        let occupied = (0..4)
            .flat_map(|x| (0..3).map(move |y| SurfaceCell::new(x, y, 0)))
            .collect::<Vec<_>>();
        let mut state = PlannerState::new(
            occupied,
            Vec::new(),
            SurfaceConfig::default(),
            GraphConfig {
                node_spacing_cells: 2,
                max_link_cells: 1,
                ..GraphConfig::default()
            },
        );
        let report = state.update_region(RegionBounds::new(1, 1, 1, 1, 0, 0), &[], &[]);
        assert!(!report.graph_rebuilt_globally);
        assert!(state
            .occupied()
            .iter()
            .all(|cell| *cell != SurfaceCell::new(1, 1, 0)));
        assert!(state
            .artifact()
            .surface
            .observed
            .contains(&SurfaceCell::new(0, 0, 0)));
    }

    #[test]
    fn planner_state_builds_graph_from_surface_config() {
        let occupied = (0..12)
            .map(|x| SurfaceCell::new(x, 0, 0))
            .collect::<Vec<_>>();
        let state = PlannerState::new(
            occupied,
            Vec::new(),
            SurfaceConfig {
                clearance_cells: 1,
                closing_passes: 1,
                max_hole_cells: 1,
                ..SurfaceConfig::default()
            },
            GraphConfig {
                node_spacing_cells: 3,
                max_link_cells: 1,
                ..GraphConfig::default()
            },
        );
        assert!(state.artifact().graph.nodes.len() >= 3);
        assert!(!state.artifact().graph.edges.is_empty());
    }
}
