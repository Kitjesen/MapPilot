//! Dependency-free multi-level surface and sparse graph core.
//!
//! The data model follows the useful ownership split from the DimOS MLS
//! planner. LingTu keeps map I/O and process ownership outside this crate.
//! Metric route checks consume explicit robot geometry and raw map evidence.

use std::collections::{HashMap, HashSet, VecDeque};

mod adjacency;
mod corridor;
mod edge_repair;
mod edges;
mod node_relocation;
mod nodes;
mod query;
mod route_check;
mod stable_graph;
mod surfaces;
mod voronoi_repair;

pub(crate) use edges::build_graph;
pub use route_check::{
    LtMlsCancelCheck, LtMlsMetricConfig, LtMlsMetricRegion, LtMlsPoint, LtMlsPose,
};
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
    /// Exact closing radius in map cells. Zero uses `closing_passes`.
    pub closing_radius_cells: f32,
}

impl Default for SurfaceConfig {
    fn default() -> Self {
        Self {
            clearance_cells: 1,
            closing_passes: 0,
            closing_radius_cells: 0.0,
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

    pub(crate) fn expand(self, xy: i32, z: i32) -> Self {
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

/// Evidence returned by a region update.
///
/// Surface extraction and graph repair are bounded to the changed region plus
/// the configured halo. `graph_rebuilt_globally` is true only when the local
/// graph cannot be repaired while preserving a valid route graph.
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
    incremental_graph: Option<IncrementalGraph>,
    surface_source: bool,
    query_surface: std::sync::OnceLock<query::Surface>,
    geometry_index: std::sync::OnceLock<route_check::Index>,
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
            incremental_graph: None,
            surface_source: false,
            query_surface: std::sync::OnceLock::new(),
            geometry_index: std::sync::OnceLock::new(),
        };
        state.occupied.sort_unstable();
        state.occupied.dedup();
        state.known_free.sort_unstable();
        state.known_free.dedup();
        state.artifact = state.rebuild_artifact();
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
        Self {
            occupied: Vec::new(),
            known_free: Vec::new(),
            surface_config: SurfaceConfig::default(),
            graph_config,
            artifact: PlannerArtifact { surface, graph },
            incremental_graph: None,
            surface_source: true,
            query_surface: std::sync::OnceLock::new(),
            geometry_index: std::sync::OnceLock::new(),
        }
    }

    fn from_graph(
        observed: Vec<SurfaceCell>,
        inferred: Vec<SurfaceCell>,
        graph: SurfaceGraph,
        graph_config: GraphConfig,
    ) -> Self {
        let mut surface = SurfaceMap { observed, inferred };
        surface.observed.sort_unstable();
        surface.observed.dedup();
        surface.inferred.sort_unstable();
        surface.inferred.dedup();
        Self {
            occupied: Vec::new(),
            known_free: Vec::new(),
            surface_config: SurfaceConfig::default(),
            graph_config,
            artifact: PlannerArtifact { surface, graph },
            incremental_graph: None,
            surface_source: true,
            query_surface: std::sync::OnceLock::new(),
            geometry_index: std::sync::OnceLock::new(),
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

    pub fn set_geometry(
        &mut self,
        mut occupied: Vec<SurfaceCell>,
        mut known_free: Vec<SurfaceCell>,
        surface_config: SurfaceConfig,
    ) {
        occupied.sort_unstable();
        occupied.dedup();
        known_free.sort_unstable();
        known_free.dedup();
        self.occupied = occupied;
        self.known_free = known_free;
        self.surface_config = surface_config;
        self.surface_source = false;
        self.query_surface.take();
        self.geometry_index.take();
    }

    /// Replace occupied/free source evidence. Extracted-surface artifacts reject source updates.
    pub fn update_region(
        &mut self,
        region: RegionBounds,
        replacement_occupied: &[SurfaceCell],
        replacement_known_free: &[SurfaceCell],
    ) -> Result<RegionUpdateReport, &'static str> {
        if self.surface_source {
            return Err("source updates require an occupied/free artifact");
        }
        self.query_surface.take();
        self.geometry_index.take();
        let incremental_graph = self.incremental_graph.get_or_insert_with(|| {
            IncrementalGraph::from_graph(&graph_cells(&self.artifact.surface), &self.artifact.graph)
        });
        Ok(update_region(
            &mut self.artifact,
            &mut self.occupied,
            &mut self.known_free,
            region,
            replacement_occupied,
            replacement_known_free,
            self.surface_config,
            self.graph_config,
            incremental_graph,
        ))
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

    let closing = if surface_config.closing_radius_cells > 0.0 {
        surface_config.closing_radius_cells.ceil() as i32
    } else {
        surface_config.closing_passes as i32
    };
    // Closing reads through dilation and erosion; support reads a three-cell neighbourhood.
    let halo = if closing > 0 {
        closing.saturating_mul(2).max(3)
    } else {
        0
    };
    let mut expanded = region.expand(halo.saturating_add(1), 0);
    // Keep complete columns so a cropped wall cannot acquire an artificial top surface.
    for cell in occupied
        .iter()
        .chain(known_free.iter())
        .chain(&artifact.surface.observed)
        .chain(&artifact.surface.inferred)
    {
        expanded.min_z = expanded.min_z.min(cell.z);
        expanded.max_z = expanded.max_z.max(cell.z);
    }
    let context = expanded.expand(halo, 0);
    let local_occupied: Vec<_> = occupied
        .iter()
        .copied()
        .filter(|cell| context.contains(*cell))
        .collect();
    let local_free: Vec<_> = known_free
        .iter()
        .copied()
        .filter(|cell| context.contains(*cell))
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
    observed.extend(
        local_surface
            .observed
            .into_iter()
            .filter(|cell| expanded.contains(*cell)),
    );
    observed.sort_unstable();
    observed.dedup();
    let mut inferred: Vec<_> = artifact
        .surface
        .inferred
        .iter()
        .copied()
        .filter(|cell| !expanded.contains(*cell))
        .collect();
    inferred.extend(
        local_surface
            .inferred
            .into_iter()
            .filter(|cell| expanded.contains(*cell)),
    );
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
#[derive(Clone, Copy, Debug, Default)]
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
    pub closing_radius_cells: f32,
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
    /// Stable identity derived from the unordered endpoint pair.
    pub id: u64,
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

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsRouteCheckConfig {
    pub body_radius_cells: i32,
    pub body_height_cells: i32,
    pub max_step_cells: i32,
    pub max_slope: f32,
    pub has_no_go: u8,
    pub no_go: LtMlsRegion,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsRouteCheckResult {
    pub valid: u8,
    pub failure_kind: u32,
    pub failed_segment: i32,
    pub failed_cell: LtMlsCell,
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

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsPoseBuffer {
    pub ptr: *mut LtMlsPose,
    pub len: u64,
    pub cap: u64,
}

pub const LT_MLS_ABI_VERSION: u32 = 10;

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
            id: crate::stable_graph::edge_id(edge.a, edge.b),
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

#[cfg(test)]
fn graph_route(
    graph: &SurfaceGraph,
    start: SurfaceCell,
    goal: SurfaceCell,
) -> Option<Vec<SurfaceCell>> {
    graph_route_limited(graph, start, goal, None, None)
}

#[cfg(test)]
fn graph_route_limited(
    graph: &SurfaceGraph,
    start: SurfaceCell,
    goal: SurfaceCell,
    max_snap_cells: Option<i32>,
    max_iterations: Option<usize>,
) -> Option<Vec<SurfaceCell>> {
    graph_route_limited_avoiding(graph, start, goal, max_snap_cells, max_iterations, &[])
}

#[cfg(test)]
fn graph_route_limited_avoiding(
    graph: &SurfaceGraph,
    start: SurfaceCell,
    goal: SurfaceCell,
    max_snap_cells: Option<i32>,
    max_iterations: Option<usize>,
    blocked_cells: &[SurfaceCell],
) -> Option<Vec<SurfaceCell>> {
    graph_route_limited_avoiding_with_config(
        graph,
        start,
        goal,
        max_snap_cells,
        max_iterations,
        blocked_cells,
        GraphConfig::default(),
    )
}

#[cfg(test)]
fn graph_route_limited_avoiding_with_config(
    graph: &SurfaceGraph,
    start: SurfaceCell,
    goal: SurfaceCell,
    max_snap_cells: Option<i32>,
    max_iterations: Option<usize>,
    blocked_cells: &[SurfaceCell],
    config: GraphConfig,
) -> Option<Vec<SurfaceCell>> {
    let mut cells = graph.nodes.iter().map(|node| node.cell).collect::<Vec<_>>();
    cells.extend(
        graph
            .edges
            .iter()
            .flat_map(|edge| edge.corridor.iter())
            .copied(),
    );
    let state = PlannerState::from_graph(cells, Vec::new(), graph.clone(), config);
    let blocked = blocked_cells.iter().copied().collect::<HashSet<_>>();
    let mut path = query::plan(
        &state,
        start,
        goal,
        max_snap_cells.unwrap_or(i32::MAX),
        max_iterations.unwrap_or(usize::MAX),
        false,
        |from, to, kind| {
            if kind == 3 {
                return 1;
            }
            if blocked.contains(&from) || blocked.contains(&to) {
                return 0;
            }
            if kind != 0 && from.x == to.x && from.y == to.y && from.z != to.z {
                return 0;
            }
            if kind != 0 {
                let rise = (to.z - from.z).unsigned_abs() as f32;
                let run = ((to.x - from.x) as f32).hypot((to.y - from.y) as f32);
                if config.max_step_cells > 0 && rise > config.max_step_cells as f32 {
                    return 0;
                }
                if config.max_slope > 0.0 && rise > config.max_slope * run {
                    return 0;
                }
            }
            1
        },
    )
    .ok()?;
    if path.first() != Some(&start) {
        path.insert(0, start);
    }
    if path.last() != Some(&goal) {
        path.push(goal);
    }
    Some(path)
}

const ROUTE_OK: u32 = 0;
const ROUTE_EMPTY: u32 = 1;
const ROUTE_UNSUPPORTED: u32 = 2;
const ROUTE_OCCUPIED: u32 = 3;
const ROUTE_STEP: u32 = 4;
const ROUTE_SLOPE: u32 = 5;
const ROUTE_NO_GO: u32 = 6;
const ROUTE_SOURCE_UNAVAILABLE: u32 = 7;
const ROUTE_INVALID_CONFIG: u32 = 8;
const ROUTE_SWEEP: u32 = 9;

fn route_failure(kind: u32, segment: i32, cell: SurfaceCell) -> LtMlsRouteCheckResult {
    LtMlsRouteCheckResult {
        valid: 0,
        failure_kind: kind,
        failed_segment: segment,
        failed_cell: LtMlsCell {
            x: cell.x,
            y: cell.y,
            z: cell.z,
        },
    }
}

fn route_cell_in_region(cell: SurfaceCell, region: LtMlsRegion, radius: i32) -> bool {
    cell.x >= region.min_x.saturating_sub(radius)
        && cell.x <= region.max_x.saturating_add(radius)
        && cell.y >= region.min_y.saturating_sub(radius)
        && cell.y <= region.max_y.saturating_add(radius)
        && cell.z >= region.min_z
        && cell.z <= region.max_z
}

fn check_route(
    state: &PlannerState,
    route: &[SurfaceCell],
    config: LtMlsRouteCheckConfig,
) -> LtMlsRouteCheckResult {
    if route.is_empty() {
        return route_failure(ROUTE_EMPTY, -1, SurfaceCell::new(0, 0, 0));
    }

    let observed: HashSet<SurfaceCell> = state.artifact.surface.observed.iter().copied().collect();
    let inferred: HashSet<SurfaceCell> = state.artifact.surface.inferred.iter().copied().collect();
    let surface: HashSet<SurfaceCell> = observed.union(&inferred).copied().collect();
    for (index, &cell) in route.iter().enumerate() {
        if !observed.contains(&cell) && !inferred.contains(&cell) {
            return route_failure(ROUTE_UNSUPPORTED, index as i32, cell);
        }
        if config.has_no_go != 0
            && route_cell_in_region(cell, config.no_go, config.body_radius_cells.max(0))
        {
            return route_failure(ROUTE_NO_GO, index.saturating_sub(1) as i32, cell);
        }
    }

    if (config.body_radius_cells > 0 || config.body_height_cells > 0) && state.surface_source {
        return route_failure(ROUTE_SOURCE_UNAVAILABLE, 0, route[0]);
    }
    if config.body_radius_cells < 0
        || config.body_height_cells < 0
        || config.max_step_cells < 0
        || !config.max_slope.is_finite()
        || config.max_slope < 0.0
    {
        return route_failure(ROUTE_INVALID_CONFIG, -1, route[0]);
    }

    let body_blocked = |cell: SurfaceCell| {
        if config.body_radius_cells <= 0 && config.body_height_cells <= 0 {
            return false;
        }
        let radius = config.body_radius_cells;
        let min_z = cell.z.saturating_add(1);
        let max_z = min_z.saturating_add(config.body_height_cells);
        state.occupied.iter().any(|occupied| {
            let dx = i64::from(occupied.x) - i64::from(cell.x);
            let dy = i64::from(occupied.y) - i64::from(cell.y);
            dx * dx + dy * dy <= i64::from(radius) * i64::from(radius)
                && occupied.z >= min_z
                && occupied.z <= max_z
        })
    };

    for (index, &cell) in route.iter().enumerate() {
        if body_blocked(cell) {
            return route_failure(ROUTE_OCCUPIED, index as i32, cell);
        }
        if let Some(&next) = route.get(index + 1) {
            let dz = i64::from(next.z) - i64::from(cell.z);
            let dx = i64::from(next.x) - i64::from(cell.x);
            let dy = i64::from(next.y) - i64::from(cell.y);
            if dz.unsigned_abs() > config.max_step_cells as u64 {
                return route_failure(ROUTE_STEP, index as i32, next);
            }
            let horizontal = ((dx * dx + dy * dy) as f64).sqrt();
            if horizontal > 0.0
                && (dz.unsigned_abs() as f64) > f64::from(config.max_slope) * horizontal
            {
                return route_failure(ROUTE_SLOPE, index as i32, next);
            }

            let samples = dx
                .unsigned_abs()
                .max(dy.unsigned_abs())
                .max(dz.unsigned_abs());
            for step in 1..samples {
                let t = step as f64 / samples as f64;
                let sample = SurfaceCell::new(
                    (cell.x as f64 + dx as f64 * t).round() as i32,
                    (cell.y as f64 + dy as f64 * t).round() as i32,
                    (cell.z as f64 + dz as f64 * t).round() as i32,
                );
                if !surface.contains(&sample) {
                    return route_failure(ROUTE_SWEEP, index as i32, sample);
                }
                if config.has_no_go != 0
                    && route_cell_in_region(sample, config.no_go, config.body_radius_cells.max(0))
                {
                    return route_failure(ROUTE_NO_GO, index as i32, sample);
                }
                if body_blocked(sample) {
                    return route_failure(ROUTE_OCCUPIED, index as i32, sample);
                }
            }
        }
    }

    LtMlsRouteCheckResult {
        valid: 1,
        failure_kind: ROUTE_OK,
        failed_segment: -1,
        failed_cell: LtMlsCell::default(),
    }
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

unsafe fn read_graph(
    nodes: *const LtMlsGraphNode,
    nodes_len: u64,
    edges: *const LtMlsGraphEdge,
    edges_len: u64,
    corridor: *const LtMlsCell,
    corridor_len: u64,
) -> Result<SurfaceGraph, i32> {
    if (nodes_len > 0 && nodes.is_null())
        || (edges_len > 0 && edges.is_null())
        || (corridor_len > 0 && corridor.is_null())
    {
        return Err(-1);
    }
    let nodes_len = usize::try_from(nodes_len).map_err(|_| -3)?;
    let edges_len = usize::try_from(edges_len).map_err(|_| -3)?;
    let corridor_len = usize::try_from(corridor_len).map_err(|_| -3)?;
    let mut node_ids = std::collections::HashSet::with_capacity(nodes_len);
    let mut graph_nodes = Vec::with_capacity(nodes_len);
    let node_values = if nodes_len == 0 {
        &[][..]
    } else {
        std::slice::from_raw_parts(nodes, nodes_len)
    };
    for node in node_values {
        if !node_ids.insert(node.id) {
            return Err(-2);
        }
        graph_nodes.push(Node {
            id: node.id,
            cell: SurfaceCell::new(node.x, node.y, node.z),
        });
    }
    let node_ids = node_ids;
    let corridor_values = if corridor_len == 0 {
        &[][..]
    } else {
        std::slice::from_raw_parts(corridor, corridor_len)
    };
    let mut graph_edges = Vec::with_capacity(edges_len);
    let edge_values = if edges_len == 0 {
        &[][..]
    } else {
        std::slice::from_raw_parts(edges, edges_len)
    };
    for edge in edge_values {
        let end = edge
            .corridor_offset
            .checked_add(edge.corridor_len)
            .ok_or(-2)?;
        let end = usize::try_from(end).map_err(|_| -3)?;
        let start = usize::try_from(edge.corridor_offset).map_err(|_| -3)?;
        if end > corridor_values.len()
            || start >= end
            || edge.from == edge.to
            || !node_ids.contains(&edge.from)
            || !node_ids.contains(&edge.to)
            || !edge.cost.is_finite()
            || edge.cost <= 0.0
        {
            return Err(-2);
        }
        let expected_id = crate::stable_graph::edge_id(edge.from, edge.to);
        if edge.id != 0 && edge.id != expected_id {
            return Err(-2);
        }
        graph_edges.push(Edge {
            a: edge.from,
            b: edge.to,
            cost: edge.cost,
            corridor: corridor_values[start..end]
                .iter()
                .map(|cell| SurfaceCell::new(cell.x, cell.y, cell.z))
                .collect(),
        });
    }
    Ok(SurfaceGraph {
        nodes: graph_nodes,
        edges: graph_edges,
    })
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
            || !surface_config.closing_radius_cells.is_finite()
            || surface_config.closing_radius_cells < 0.0
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
                closing_radius_cells: surface_config.closing_radius_cells,
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
/// All input buffers must remain readable for the duration of the call.
pub unsafe extern "C" fn lt_mls_artifact_create_from_graph(
    observed: *const LtMlsCell,
    observed_len: u64,
    inferred: *const LtMlsCell,
    inferred_len: u64,
    nodes: *const LtMlsGraphNode,
    nodes_len: u64,
    edges: *const LtMlsGraphEdge,
    edges_len: u64,
    corridor: *const LtMlsCell,
    corridor_len: u64,
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
        let graph = read_graph(nodes, nodes_len, edges, edges_len, corridor, corridor_len)?;
        let graph_config = *graph_config;
        if graph_config.node_spacing_cells < 1 || graph_config.max_link_cells < 1 {
            return Err(-2);
        }
        let graph_config = GraphConfig {
            node_spacing_cells: graph_config.node_spacing_cells,
            max_link_cells: graph_config.max_link_cells,
            wall_clearance_cells: graph_config.wall_clearance_cells,
            wall_buffer_cells: graph_config.wall_buffer_cells,
            wall_buffer_weight: graph_config.wall_buffer_weight,
            step_penalty_weight: graph_config.step_penalty_weight,
            max_slope: graph_config.max_slope,
            max_step_cells: graph_config.max_step_cells,
            horizontal_hop_cells: graph_config.horizontal_hop_cells,
        };
        let state = PlannerState::from_graph(observed, inferred, graph, graph_config);
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

fn surface_column(cells: &[SurfaceCell], x: i32, y: i32) -> &[SurfaceCell] {
    let first = cells.partition_point(|cell| (cell.x, cell.y) < (x, y));
    let end = first + cells[first..].partition_point(|cell| (cell.x, cell.y) == (x, y));
    &cells[first..end]
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
    let surface = &artifact.state.artifact.surface;
    let floor = [&surface.observed, &surface.inferred]
        .into_iter()
        .filter_map(|cells| {
            let column = surface_column(cells, x, y);
            let end = column.partition_point(|cell| cell.z <= z);
            end.checked_sub(1).map(|index| column[index].z)
        })
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
    if surface_column(&artifact.state.artifact.surface.inferred, x, y)
        .iter()
        .any(|cell| cell.z >= min_z && cell.z <= max_z)
    {
        1
    } else {
        0
    }
}

#[no_mangle]
/// # Safety
///
/// `handle` must be a valid artifact pointer for the duration of the call.
pub unsafe extern "C" fn lt_mls_artifact_occupied(
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
    if artifact.state.surface_source {
        return -2;
    }
    if artifact
        .state
        .occupied()
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
/// `handle` and `result` must be valid pointers for the duration of the call.
pub unsafe extern "C" fn lt_mls_artifact_occupied_below(
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
    if artifact.state.surface_source {
        return -2;
    }
    let floor = artifact
        .state
        .occupied()
        .iter()
        .filter(|cell| cell.x == x && cell.y == y && cell.z < z)
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
pub unsafe extern "C" fn lt_mls_artifact_region_occupied(
    handle: *const LtMlsArtifactHandle,
    region: LtMlsRegion,
) -> i32 {
    if handle.is_null() {
        return -1;
    }
    let artifact = &*handle;
    if artifact.state.surface_source {
        return -2;
    }
    if artifact.state.occupied().iter().any(|cell| {
        cell.x >= region.min_x
            && cell.x <= region.max_x
            && cell.y >= region.min_y
            && cell.y <= region.max_y
            && cell.z >= region.min_z
            && cell.z <= region.max_z
    }) {
        1
    } else {
        0
    }
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
    let surface = &artifact.state.artifact.surface;
    for cell in surface_column(&surface.observed, x, y)
        .iter()
        .chain(surface_column(&surface.inferred, x, y))
    {
        // Select the observed voxel interval with the existing standing-height tolerance.
        // Its exposed top remains the physical height returned to the caller.
        let bottom = f64::from(cell.z) * resolution;
        let value = f64::from(cell.z + 1) * resolution;
        let distance = (bottom - expected_z).max(0.0).max(expected_z - value);
        if distance > tolerance + 1e-7 {
            continue;
        }
        if distance <= 1e-7 {
            *result = value;
            return 1;
        }
        if best.map_or(true, |(current, _)| distance < current) {
            best = Some((distance, value));
        }
    }
    match best {
        Some((_, value)) => {
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

pub type LtMlsSegmentCheck =
    unsafe extern "C" fn(LtMlsCell, LtMlsCell, u32, *mut std::ffi::c_void) -> i32;

fn cell_from_abi(cell: LtMlsCell) -> SurfaceCell {
    SurfaceCell::new(cell.x, cell.y, cell.z)
}

fn cell_to_abi(cell: SurfaceCell) -> LtMlsCell {
    LtMlsCell {
        x: cell.x,
        y: cell.y,
        z: cell.z,
    }
}

unsafe fn legacy_query(
    handle: *const LtMlsArtifactHandle,
    start: LtMlsCell,
    goal: LtMlsCell,
    max_snap_cells: i32,
    max_iterations: usize,
    blocked: &[SurfaceCell],
    route: *mut LtMlsBuffer,
) -> i32 {
    let route = &mut *route;
    lt_mls_free_buffer(route);
    let blocked = blocked.iter().copied().collect::<HashSet<_>>();
    let cells = graph_cells(&(*handle).state.artifact.surface);
    let nearest_distance = |target: LtMlsCell| {
        cells
            .iter()
            .map(|cell| {
                let dx = i64::from(cell.x) - i64::from(target.x);
                let dy = i64::from(cell.y) - i64::from(target.y);
                let dz = i64::from(cell.z) - i64::from(target.z);
                dx * dx + dy * dy + dz * dz
            })
            .min()
    };
    let start_distance = nearest_distance(start);
    let goal_distance = nearest_distance(goal);
    match query::plan(
        &(*handle).state,
        cell_from_abi(start),
        cell_from_abi(goal),
        max_snap_cells,
        max_iterations,
        false,
        |from, to, kind| {
            if kind == 3 {
                return 1;
            }
            if blocked.contains(&from) || blocked.contains(&to) {
                return 0;
            }
            let dx = i64::from(to.x) - i64::from(from.x);
            let dy = i64::from(to.y) - i64::from(from.y);
            let dz = i64::from(to.z) - i64::from(from.z);
            let distance = dx * dx + dy * dy + dz * dz;
            match kind {
                1 => i32::from(start_distance == Some(distance)),
                2 => i32::from(goal_distance == Some(distance)),
                _ => 1,
            }
        },
    ) {
        Ok(mut path) => {
            let start = cell_from_abi(start);
            let goal = cell_from_abi(goal);
            if path.first() != Some(&start) {
                path.insert(0, start);
            }
            if path.last() != Some(&goal) {
                path.push(goal);
            }
            put_buffer(path, route).map_or(-3, |_| 0)
        }
        Err(_) => -2,
    }
}

#[no_mangle]
/// # Safety
///
/// `handle` and `route` must be valid. The returned buffer belongs to the caller.
pub unsafe extern "C" fn lt_mls_artifact_query(
    handle: *const LtMlsArtifactHandle,
    start: LtMlsCell,
    goal: LtMlsCell,
    route: *mut LtMlsBuffer,
) -> i32 {
    if handle.is_null() || route.is_null() {
        return -1;
    }
    legacy_query(handle, start, goal, i32::MAX, usize::MAX, &[], route)
}

#[no_mangle]
/// # Safety
///
/// `handle` and `route` must be valid. The returned buffer belongs to the caller.
pub unsafe extern "C" fn lt_mls_artifact_query_limited(
    handle: *const LtMlsArtifactHandle,
    start: LtMlsCell,
    goal: LtMlsCell,
    max_snap_cells: i32,
    route: *mut LtMlsBuffer,
) -> i32 {
    if handle.is_null() || route.is_null() || max_snap_cells < 0 {
        return -1;
    }
    legacy_query(handle, start, goal, max_snap_cells, usize::MAX, &[], route)
}

#[no_mangle]
/// # Safety
///
/// `handle` and `route` must be valid. The returned buffer belongs to the caller.
pub unsafe extern "C" fn lt_mls_artifact_query_budgeted(
    handle: *const LtMlsArtifactHandle,
    start: LtMlsCell,
    goal: LtMlsCell,
    max_snap_cells: i32,
    max_iterations: i32,
    route: *mut LtMlsBuffer,
) -> i32 {
    if handle.is_null() || route.is_null() || max_snap_cells < 0 || max_iterations < 1 {
        return -1;
    }
    legacy_query(
        handle,
        start,
        goal,
        max_snap_cells,
        max_iterations as usize,
        &[],
        route,
    )
}

#[no_mangle]
/// # Safety
///
/// Input cells, `handle`, and `route` must be valid for the call.
pub unsafe extern "C" fn lt_mls_artifact_query_avoiding(
    handle: *const LtMlsArtifactHandle,
    start: LtMlsCell,
    goal: LtMlsCell,
    max_snap_cells: i32,
    max_iterations: i32,
    blocked_cells: *const LtMlsCell,
    blocked_cells_len: u64,
    route: *mut LtMlsBuffer,
) -> i32 {
    if handle.is_null() || route.is_null() || max_snap_cells < 0 || max_iterations < 1 {
        return -1;
    }
    let blocked = match read_cells(blocked_cells, blocked_cells_len) {
        Ok(value) => value,
        Err(code) => return code,
    };
    legacy_query(
        handle,
        start,
        goal,
        max_snap_cells,
        max_iterations as usize,
        &blocked,
        route,
    )
}

#[no_mangle]
/// # Safety
///
/// The handle, config and nonempty cell buffers must remain valid for the call.
pub unsafe extern "C" fn lt_mls_artifact_set_geometry(
    handle: *mut LtMlsArtifactHandle,
    occupied: *const LtMlsCell,
    occupied_len: u64,
    known_free: *const LtMlsCell,
    known_free_len: u64,
    config: *const LtMlsConfig,
) -> i32 {
    if handle.is_null() || config.is_null() {
        return -1;
    }
    let config = *config;
    if config.clearance_cells < 0
        || !config.closing_radius_cells.is_finite()
        || config.closing_radius_cells < 0.0
    {
        return -2;
    }
    let occupied = match read_cells(occupied, occupied_len) {
        Ok(c) => c,
        Err(e) => return e,
    };
    let known_free = match read_cells(known_free, known_free_len) {
        Ok(c) => c,
        Err(e) => return e,
    };
    (*handle).state.set_geometry(
        occupied,
        known_free,
        SurfaceConfig {
            clearance_cells: config.clearance_cells,
            closing_passes: config.closing_passes,
            closing_radius_cells: config.closing_radius_cells,
        },
    );
    0
}

#[no_mangle]
/// # Safety
///
/// `handle`, callback context, and `route` must remain valid throughout the call.
/// Callback kinds are segment=0, start connector=1, goal connector=2, cancel poll=3.
/// Callback results are pass=1, blocked=0, cancel=-1. Query status is found=0,
/// exhausted=1, budget=2, cancelled=3. Negative status denotes invalid ABI input.
pub unsafe extern "C" fn lt_mls_artifact_query_checked(
    handle: *const LtMlsArtifactHandle,
    start: LtMlsCell,
    goal: LtMlsCell,
    max_snap_cells: i32,
    max_iterations: i32,
    check: Option<LtMlsSegmentCheck>,
    context: *mut std::ffi::c_void,
    route: *mut LtMlsBuffer,
) -> i32 {
    if handle.is_null() || route.is_null() || max_snap_cells < 0 || max_iterations < 1 {
        return -1;
    }
    let Some(check) = check else {
        return -1;
    };
    let route = &mut *route;
    lt_mls_free_buffer(route);
    match query::plan(
        &(*handle).state,
        cell_from_abi(start),
        cell_from_abi(goal),
        max_snap_cells,
        max_iterations as usize,
        true,
        |from, to, kind| check(cell_to_abi(from), cell_to_abi(to), kind, context),
    ) {
        Ok(path) => put_buffer(path, route).map_or(-3, |_| 0),
        Err(
            query::QueryError::Exhausted
            | query::QueryError::StartUnreachable
            | query::QueryError::GoalUnreachable,
        ) => 1,
        Err(query::QueryError::Budget) => 2,
        Err(query::QueryError::Cancelled) => 3,
    }
}

#[no_mangle]
/// # Safety
///
/// `handle`, `route`, `config`, and `result` must be valid for the duration
/// of the call. The route is read-only and the result is written by Rust.
pub unsafe extern "C" fn lt_mls_artifact_check_route(
    handle: *const LtMlsArtifactHandle,
    route: *const LtMlsCell,
    route_len: u64,
    config: *const LtMlsRouteCheckConfig,
    result: *mut LtMlsRouteCheckResult,
) -> i32 {
    if handle.is_null()
        || (route_len > 0 && route.is_null())
        || config.is_null()
        || result.is_null()
    {
        return -1;
    }
    let route_len = match usize::try_from(route_len) {
        Ok(value) => value,
        Err(_) => return -2,
    };
    let cells = if route_len == 0 {
        &[]
    } else {
        std::slice::from_raw_parts(route, route_len)
    };
    let cells: Vec<SurfaceCell> = cells
        .iter()
        .map(|cell| SurfaceCell::new(cell.x, cell.y, cell.z))
        .collect();
    let check = check_route(&(*handle).state, &cells, *config);
    *result = check;
    0
}

#[no_mangle]
/// # Safety
///
/// The handle, config, result and nonempty input buffers must be valid for the call.
pub unsafe extern "C" fn lt_mls_artifact_check_metric_route(
    handle: *const LtMlsArtifactHandle,
    route: *const LtMlsPoint,
    route_len: u64,
    config: *const LtMlsMetricConfig,
    regions: *const LtMlsMetricRegion,
    regions_len: u64,
    result: *mut LtMlsRouteCheckResult,
) -> i32 {
    if handle.is_null()
        || config.is_null()
        || result.is_null()
        || (route_len > 0 && route.is_null())
        || (regions_len > 0 && regions.is_null())
    {
        return -1;
    }
    let (Ok(route_len), Ok(regions_len)) =
        (usize::try_from(route_len), usize::try_from(regions_len))
    else {
        return -2;
    };
    let route = if route_len == 0 {
        &[]
    } else {
        std::slice::from_raw_parts(route, route_len)
    };
    let regions = if regions_len == 0 {
        &[]
    } else {
        std::slice::from_raw_parts(regions, regions_len)
    };
    *result = route_check::check(&(*handle).state, route, *config, regions);
    0
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct LtMlsGeometry {
    pub config: LtMlsMetricConfig,
    pub regions: *const LtMlsMetricRegion,
    pub regions_len: u64,
    pub preblocked: *const LtMlsCell,
    pub preblocked_len: u64,
    pub cancel: Option<LtMlsCancelCheck>,
    pub context: *mut std::ffi::c_void,
}

unsafe fn metric_checker<'a>(
    state: &'a PlannerState,
    geometry: &'a LtMlsGeometry,
) -> Result<route_check::Checker<'a>, LtMlsRouteCheckResult> {
    if geometry.regions_len > 0 && geometry.regions.is_null() {
        return Err(route_failure(
            ROUTE_INVALID_CONFIG,
            -1,
            SurfaceCell::new(0, 0, 0),
        ));
    }
    let Ok(regions_len) = usize::try_from(geometry.regions_len) else {
        return Err(route_failure(
            ROUTE_INVALID_CONFIG,
            -1,
            SurfaceCell::new(0, 0, 0),
        ));
    };
    let regions = if regions_len == 0 {
        &[]
    } else {
        std::slice::from_raw_parts(geometry.regions, regions_len)
    };
    let preblocked = read_cells(geometry.preblocked, geometry.preblocked_len)
        .map_err(|_| route_failure(ROUTE_INVALID_CONFIG, -1, SurfaceCell::new(0, 0, 0)))?;
    route_check::prepare(
        state,
        geometry.config,
        regions,
        &preblocked,
        geometry.cancel,
        geometry.context,
    )
}

#[no_mangle]
/// # Safety
/// All nonempty buffers and the geometry callback context must be valid for this call.
pub unsafe extern "C" fn lt_mls_artifact_check_metric_poses(
    handle: *const LtMlsArtifactHandle,
    route: *const LtMlsPose,
    route_len: u64,
    geometry: *const LtMlsGeometry,
    result: *mut LtMlsRouteCheckResult,
) -> i32 {
    if handle.is_null()
        || geometry.is_null()
        || result.is_null()
        || (route_len > 0 && route.is_null())
    {
        return -1;
    }
    let Ok(len) = usize::try_from(route_len) else {
        return -2;
    };
    let route = if len == 0 {
        &[]
    } else {
        std::slice::from_raw_parts(route, len)
    };
    *result = match metric_checker(&(*handle).state, &*geometry) {
        Ok(checker) => checker.check_poses(route),
        Err(failure) => failure,
    };
    0
}

#[no_mangle]
/// # Safety
/// The handle, geometry buffers, output and callback context must be valid throughout this call.
pub unsafe extern "C" fn lt_mls_artifact_query_metric(
    handle: *const LtMlsArtifactHandle,
    start: LtMlsPose,
    goal: LtMlsPose,
    max_snap_cells: i32,
    max_iterations: i32,
    geometry: *const LtMlsGeometry,
    route: *mut LtMlsPoseBuffer,
) -> i32 {
    if handle.is_null()
        || geometry.is_null()
        || route.is_null()
        || max_snap_cells < 0
        || max_iterations < 1
    {
        return -1;
    }
    if [start, goal]
        .iter()
        .any(|pose| !pose.point.finite() || (pose.has_yaw != 0 && !pose.yaw.is_finite()))
    {
        return -1;
    }
    let geometry = &*geometry;
    let checker = match metric_checker(&(*handle).state, geometry) {
        Ok(c) => c,
        Err(_) => return -2,
    };
    let config = geometry.config;
    let output = &mut *route;
    lt_mls_free_pose_buffer(output);
    let mut budget = query::Budget {
        remaining: max_iterations as usize,
    };
    let goal = match checker.resolve_goal(start, goal, max_snap_cells, &mut budget) {
        Ok(goal) => goal,
        Err(query::QueryError::Budget) => return 2,
        Err(query::QueryError::Cancelled) => return 3,
        Err(_) => return 5,
    };
    let cell = |pose: LtMlsPose| {
        SurfaceCell::new(
            (pose.point.x / config.resolution).floor() as i32,
            (pose.point.y / config.resolution).floor() as i32,
            ((pose.point.z - config.support_height) / config.resolution).round() as i32 - 1,
        )
    };
    let start_attachment = checker.attachment(start.point.native());
    let goal_attachment = checker.attachment(goal.point.native());
    let start_cell = start_attachment.unwrap_or_else(|| cell(start));
    let goal_cell = goal_attachment.unwrap_or_else(|| cell(goal));
    let pose = |cell: SurfaceCell| LtMlsPose {
        point: LtMlsPoint {
            x: (cell.x as f64 + 0.5) * config.resolution,
            y: (cell.y as f64 + 0.5) * config.resolution,
            z: (cell.z as f64 + 1.0) * config.resolution + config.support_height,
        },
        ..LtMlsPose::default()
    };
    let same_xy = |a: LtMlsPoint, b: LtMlsPoint| {
        let a = a.native();
        let b = b.native();
        a.x == b.x && a.y == b.y
    };
    let virtual_start = |cell: SurfaceCell| {
        start_attachment == Some(cell) && same_xy(start.point, pose(cell).point)
    };
    let virtual_goal =
        |cell: SurfaceCell| goal_attachment == Some(cell) && same_xy(goal.point, pose(cell).point);
    let mut rejected = HashSet::<(SurfaceCell, SurfaceCell, u32)>::new();
    loop {
        let path = match query::plan_budgeted(
            &(*handle).state,
            start_cell,
            goal_cell,
            max_snap_cells,
            &mut budget,
            true,
            |a, b, kind| {
                if geometry
                    .cancel
                    .is_some_and(|cancel| cancel(geometry.context) != 0)
                {
                    return -1;
                }
                if kind == 3 {
                    return 1;
                }
                if rejected.contains(&(a, b, kind)) {
                    return 0;
                }
                let (from, to) = if kind == 1 && virtual_start(b) {
                    (start, start)
                } else if kind == 2 && virtual_goal(a) {
                    (goal, goal)
                } else {
                    (
                        if kind == 1 { start } else { pose(a) },
                        if kind == 2 { goal } else { pose(b) },
                    )
                };
                let actual =
                    config.actual_start != 0 && from.point.native() == start.point.native();
                let result = checker.check_connection(from, to, actual);
                if result.failure_kind == route_check::CANCELLED {
                    -1
                } else {
                    i32::from(result.valid != 0)
                }
            },
        ) {
            Ok(path) => path,
            Err(query::QueryError::Exhausted) => return 1,
            Err(query::QueryError::Budget) => return 2,
            Err(query::QueryError::Cancelled) => return 3,
            Err(query::QueryError::StartUnreachable) => return 4,
            Err(query::QueryError::GoalUnreachable) => return 5,
        };
        let mut poses = Vec::with_capacity(path.len() + 2);
        poses.push(start);
        poses.extend(path.iter().copied().map(pose));
        poses.push(goal);
        let mut keys = Vec::with_capacity(path.len() + 1);
        keys.push((start_cell, path[0], 1));
        keys.extend(path.windows(2).map(|edge| (edge[0], edge[1], 0)));
        keys.push((*path.last().unwrap(), goal_cell, 2));
        // Surface attachment keeps the physical endpoint height. It does not
        // command a vertical move to the nominal standing height at the same XY.
        let remove_goal = virtual_goal(*path.last().unwrap());
        if remove_goal {
            poses.remove(poses.len() - 2);
            keys.pop();
        }
        if virtual_start(path[0]) && !(remove_goal && path.len() == 1) {
            poses.remove(1);
            keys.remove(0);
        }
        let checked = checker.check_poses(&poses);
        if checked.valid != 0 {
            return put_raw(poses, &mut output.ptr, &mut output.len, &mut output.cap)
                .map_or(-3, |_| 0);
        }
        if checked.failure_kind == route_check::CANCELLED {
            return 3;
        }
        let index = checked.failed_segment.max(0) as usize;
        let edge = keys[index];
        if !rejected.insert(edge) {
            return 1;
        }
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
            .update_region(region_from_ffi(region), &occupied, &known_free)
            .map_err(|_| -2)?;
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
        if config.clearance_cells < 0
            || !config.closing_radius_cells.is_finite()
            || config.closing_radius_cells < 0.0
        {
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
        let map = extract_surfaces_with_free(
            &cells,
            &free_cells,
            SurfaceConfig {
                clearance_cells: config.clearance_cells,
                closing_passes: config.closing_passes,
                closing_radius_cells: config.closing_radius_cells,
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

#[no_mangle]
/// # Safety
/// The buffer must be returned by query_metric and must not be freed twice.
pub unsafe extern "C" fn lt_mls_free_pose_buffer(buffer: *mut LtMlsPoseBuffer) {
    if buffer.is_null() {
        return;
    }
    let buffer = &mut *buffer;
    if !buffer.ptr.is_null() {
        let _ = Vec::from_raw_parts(buffer.ptr, buffer.len as usize, buffer.cap as usize);
    }
    *buffer = LtMlsPoseBuffer::default();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn start_boundary() {
        let occupied = (0..26)
            .flat_map(|x| (-8..=8).map(move |y| SurfaceCell::new(x, y, -1)))
            .collect();
        let handle = LtMlsArtifactHandle {
            state: PlannerState::new(
                occupied,
                Vec::new(),
                SurfaceConfig {
                    clearance_cells: 11,
                    closing_passes: 4,
                    closing_radius_cells: 4.0,
                },
                GraphConfig {
                    node_spacing_cells: 2,
                    max_link_cells: 9,
                    max_step_cells: 9,
                    horizontal_hop_cells: 2,
                    wall_buffer_cells: 8,
                    wall_buffer_weight: 2.0,
                    ..GraphConfig::default()
                },
            ),
            revision: 1,
        };
        let start = LtMlsPose {
            point: LtMlsPoint {
                x: -0.02,
                y: 0.025,
                z: 0.35,
            },
            ..LtMlsPose::default()
        };
        let goal = LtMlsPose {
            point: LtMlsPoint {
                x: 0.975,
                ..start.point
            },
            ..LtMlsPose::default()
        };
        let geometry = LtMlsGeometry {
            config: LtMlsMetricConfig {
                resolution: 0.05,
                robot_radius: 0.1,
                below: 0.1,
                above: 0.2,
                support_height: 0.35,
                support_tolerance: 0.05,
                max_step: 0.45,
                ground_support: 1,
                actual_start: 1,
                start,
                min: LtMlsPoint {
                    x: -0.5,
                    y: -0.5,
                    z: -0.1,
                },
                max: LtMlsPoint {
                    x: 1.55,
                    y: 0.55,
                    z: 0.85,
                },
                ..LtMlsMetricConfig::default()
            },
            regions: std::ptr::null(),
            regions_len: 0,
            preblocked: std::ptr::null(),
            preblocked_len: 0,
            cancel: None,
            context: std::ptr::null_mut(),
        };
        let checker = unsafe { metric_checker(&handle.state, &geometry) }.unwrap();
        assert_eq!(
            checker
                .check_connection(
                    start,
                    LtMlsPose {
                        point: LtMlsPoint {
                            x: 0.025,
                            ..start.point
                        },
                        ..start
                    },
                    true
                )
                .valid,
            1
        );
        let mut route = LtMlsPoseBuffer::default();
        assert_eq!(
            unsafe {
                lt_mls_artifact_query_metric(&handle, start, goal, 24, 10000, &geometry, &mut route)
            },
            0
        );
        unsafe { lt_mls_free_pose_buffer(&mut route) };
    }

    #[test]
    fn standing_band_endpoints() {
        let occupied = (-16..=16)
            .flat_map(|x| {
                (-8..=8).map(move |y| SurfaceCell::new(x, y, if x < 0 { -7 } else { -3 }))
            })
            .collect();
        let state = PlannerState::new(
            occupied,
            Vec::new(),
            SurfaceConfig {
                clearance_cells: 9,
                ..SurfaceConfig::default()
            },
            GraphConfig {
                max_link_cells: 5,
                max_step_cells: 5,
                horizontal_hop_cells: 2,
                ..GraphConfig::default()
            },
        );
        let handle = LtMlsArtifactHandle { state, revision: 1 };
        let lower = LtMlsPose {
            point: LtMlsPoint {
                x: -0.225_f64 as f32 as f64,
                y: 0.025_f64 as f32 as f64,
                z: 0.025_f64 as f32 as f64,
            },
            ..LtMlsPose::default()
        };
        let upper = LtMlsPose {
            point: LtMlsPoint {
                x: 0.225_f64 as f32 as f64,
                y: 0.025_f64 as f32 as f64,
                z: 0.225_f64 as f32 as f64,
            },
            ..LtMlsPose::default()
        };
        for (start, goal) in [(lower, upper), (upper, lower)] {
            let geometry = LtMlsGeometry {
                config: LtMlsMetricConfig {
                    resolution: 0.05,
                    robot_radius: 0.155,
                    below: 0.1,
                    above: 0.1,
                    support_height: 0.35,
                    support_tolerance: 0.05,
                    max_step: 0.25,
                    ground_support: 1,
                    actual_start: 1,
                    start,
                    min: LtMlsPoint {
                        x: -1.0,
                        y: -1.0,
                        z: -1.0,
                    },
                    max: LtMlsPoint {
                        x: 1.0,
                        y: 1.0,
                        z: 1.0,
                    },
                    ..LtMlsMetricConfig::default()
                },
                regions: std::ptr::null(),
                regions_len: 0,
                preblocked: std::ptr::null(),
                preblocked_len: 0,
                cancel: None,
                context: std::ptr::null_mut(),
            };
            let mut route = LtMlsPoseBuffer::default();
            assert_eq!(
                unsafe {
                    lt_mls_artifact_query_metric(
                        &handle, start, goal, 0, 5000, &geometry, &mut route,
                    )
                },
                0
            );
            let poses = unsafe { std::slice::from_raw_parts(route.ptr, route.len as usize) };
            assert_eq!(poses.first().unwrap().point, start.point);
            assert_eq!(poses.last().unwrap().point, goal.point);
            assert!(poses.windows(2).all(|p| p[0].point.x != p[1].point.x
                || p[0].point.y != p[1].point.y
                || p[0].point.z == p[1].point.z));
            let checker = unsafe { metric_checker(&handle.state, &geometry) }.unwrap();
            assert_eq!(checker.check_poses(poses).valid, 1);
            unsafe { lt_mls_free_pose_buffer(&mut route) };
        }
    }

    #[test]
    fn surface_top_height() {
        for resolution in [0.1, 0.2] {
            for level in [-2, -1, 0, 7] {
                let cells = vec![SurfaceCell::new(0, 0, level), SurfaceCell::new(1, 0, level)];
                let mut state = PlannerState::from_graph(
                    cells.clone(),
                    Vec::new(),
                    SurfaceGraph::default(),
                    GraphConfig::default(),
                );
                state.set_geometry(cells, Vec::new(), SurfaceConfig::default());
                let handle = LtMlsArtifactHandle { state, revision: 1 };
                let top = (level + 1) as f64 * resolution;
                let start = LtMlsPose {
                    point: LtMlsPoint {
                        x: 0.5 * resolution,
                        y: 0.5 * resolution,
                        z: top + 0.435,
                    },
                    ..LtMlsPose::default()
                };
                let goal = LtMlsPose {
                    point: LtMlsPoint {
                        x: 1.5 * resolution,
                        ..start.point
                    },
                    ..LtMlsPose::default()
                };
                let geometry = LtMlsGeometry {
                    config: LtMlsMetricConfig {
                        resolution,
                        robot_radius: 0.32,
                        body_offset: 0.23,
                        below: 0.05,
                        above: 0.35,
                        support_height: 0.435,
                        support_tolerance: 0.1,
                        ground_support: 1,
                        min: LtMlsPoint {
                            x: -1.0,
                            y: -1.0,
                            z: top - 1.0,
                        },
                        max: LtMlsPoint {
                            x: 2.0,
                            y: 2.0,
                            z: top + 2.0,
                        },
                        ..LtMlsMetricConfig::default()
                    },
                    regions: std::ptr::null(),
                    regions_len: 0,
                    preblocked: std::ptr::null(),
                    preblocked_len: 0,
                    cancel: None,
                    context: std::ptr::null_mut(),
                };
                let mut route = LtMlsPoseBuffer::default();
                assert_eq!(
                    unsafe {
                        lt_mls_artifact_query_metric(
                            &handle, start, goal, 0, 100, &geometry, &mut route,
                        )
                    },
                    0
                );
                let route_poses =
                    unsafe { std::slice::from_raw_parts(route.ptr, route.len as usize) };
                assert!(route_poses
                    .iter()
                    .all(|pose| (pose.point.z - top - 0.435).abs() < 1e-12));
                unsafe { lt_mls_free_pose_buffer(&mut route) };
                let mut height = f64::NAN;
                assert_eq!(
                    unsafe {
                        lt_mls_artifact_height(
                            &handle,
                            0,
                            0,
                            top - 0.25 * resolution,
                            0.3 * resolution,
                            resolution,
                            &mut height,
                        )
                    },
                    1
                );
                assert!((height - top).abs() < 1e-12);
                assert_eq!(
                    unsafe {
                        lt_mls_artifact_height(
                            &handle,
                            0,
                            0,
                            top - 0.5 * resolution,
                            0.0,
                            resolution,
                            &mut height,
                        )
                    },
                    1
                );
                assert!((height - top).abs() < 1e-12);
                for (expected, tolerance, accepted) in [
                    (top - 1.25 * resolution, 0.3 * resolution, 1),
                    (top - 1.5 * resolution, 0.3 * resolution, 0),
                    (top + 0.25 * resolution, 0.3 * resolution, 1),
                    (top + 0.5 * resolution, 0.3 * resolution, 0),
                ] {
                    assert_eq!(
                        unsafe {
                            lt_mls_artifact_height(
                                &handle,
                                0,
                                0,
                                expected,
                                tolerance,
                                resolution,
                                &mut height,
                            )
                        },
                        accepted
                    );
                    if accepted == 1 {
                        assert!((height - top).abs() < 1e-12);
                    }
                }
            }
        }
    }

    #[test]
    fn metric_route_turns_at_vertex_without_pivot() {
        let corners = [
            SurfaceCell::new(-10, 0, 4),
            SurfaceCell::new(0, 0, 4),
            SurfaceCell::new(0, 10, 4),
            SurfaceCell::new(-10, 10, 4),
        ];
        let mut cells = Vec::new();
        let mut edges = Vec::new();
        for (a, b) in [(0, 1), (1, 2), (0, 3), (3, 2)] {
            let from = corners[a];
            let to = corners[b];
            let corridor = (0..=10)
                .map(|i| {
                    SurfaceCell::new(
                        from.x + (to.x - from.x) * i / 10,
                        from.y + (to.y - from.y) * i / 10,
                        4,
                    )
                })
                .collect::<Vec<_>>();
            cells.extend_from_slice(&corridor);
            edges.push(Edge {
                a: a as u32,
                b: b as u32,
                cost: 10.0,
                corridor,
            });
        }
        let graph = SurfaceGraph {
            nodes: corners
                .iter()
                .enumerate()
                .map(|(i, &cell)| Node { id: i as u32, cell })
                .collect(),
            edges,
        };
        let mut state = PlannerState::from_graph(cells, Vec::new(), graph, GraphConfig::default());
        let original_graph = state.artifact.graph.clone();
        state.set_geometry(
            vec![SurfaceCell::new(3, 3, 4)],
            Vec::new(),
            SurfaceConfig::default(),
        );
        assert_eq!(state.artifact.graph, original_graph);
        let handle = LtMlsArtifactHandle { state, revision: 1 };
        let start = LtMlsPose {
            point: LtMlsPoint {
                x: -0.95,
                y: 0.05,
                z: 0.5,
            },
            ..LtMlsPose::default()
        };
        let goal = LtMlsPose {
            point: LtMlsPoint {
                x: 0.05,
                y: 1.05,
                z: 0.5,
            },
            ..LtMlsPose::default()
        };
        let geometry = LtMlsGeometry {
            config: LtMlsMetricConfig {
                resolution: 0.1,
                robot_radius: 0.05,
                body_offset: 0.4,
                below: 0.05,
                above: 0.35,
                min: LtMlsPoint {
                    x: -2.0,
                    y: -2.0,
                    z: -2.0,
                },
                max: LtMlsPoint {
                    x: 4.0,
                    y: 4.0,
                    z: 4.0,
                },
                ..LtMlsMetricConfig::default()
            },
            regions: std::ptr::null(),
            regions_len: 0,
            preblocked: std::ptr::null(),
            preblocked_len: 0,
            cancel: None,
            context: std::ptr::null_mut(),
        };
        let checker = unsafe { metric_checker(&handle.state, &geometry) }.unwrap();
        let world = |c: SurfaceCell| LtMlsPose {
            point: LtMlsPoint {
                x: (c.x as f64 + 0.5) * 0.1,
                y: (c.y as f64 + 0.5) * 0.1,
                z: 0.5,
            },
            ..LtMlsPose::default()
        };
        let first = query::plan(
            &handle.state,
            corners[0],
            corners[2],
            0,
            2000,
            true,
            |a, b, kind| {
                if kind == 3 {
                    1
                } else {
                    i32::from(checker.check_connection(world(a), world(b), false).valid != 0)
                }
            },
        )
        .unwrap();
        let first_poses = first.iter().copied().map(world).collect::<Vec<_>>();
        // A pivot at corner 1 would hit the wall cell; the shorter corner
        // route is kept because vertices are not in-place turns.
        assert_eq!(checker.check_poses(&first_poses).valid, 1);
        let mut route = LtMlsPoseBuffer::default();
        let status = unsafe {
            lt_mls_artifact_query_metric(&handle, start, goal, 0, 2000, &geometry, &mut route)
        };
        assert_eq!(status, 0);
        let poses = unsafe { std::slice::from_raw_parts(route.ptr, route.len as usize) };
        assert!(poses
            .iter()
            .any(|p| (p.point.x - 0.05).abs() < 1e-12 && (p.point.y - 0.05).abs() < 1e-12));
        let checker = unsafe { metric_checker(&handle.state, &geometry) }.unwrap();
        assert_eq!(checker.check_poses(poses).valid, 1);
        unsafe { lt_mls_free_pose_buffer(&mut route) };
        assert_eq!(
            unsafe {
                lt_mls_artifact_query_metric(&handle, start, goal, 0, 2, &geometry, &mut route)
            },
            2
        );
        assert_eq!(route.len, 0);
    }
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
    fn sparse_nodes_preserve_same_column_surfaces() {
        let lower = SurfaceCell::new(0, 0, 0);
        let upper = SurfaceCell::new(0, 0, 2);
        let cells = [lower, upper, SurfaceCell::new(2, 0, 1)];
        let graph = edges::build_graph(
            &cells,
            GraphConfig {
                node_spacing_cells: 3,
                max_link_cells: 2,
                max_slope: 0.7,
                horizontal_hop_cells: 2,
                ..GraphConfig::default()
            },
        );
        assert!(graph.nodes.iter().any(|node| node.cell == lower));
        assert!(graph.nodes.iter().any(|node| node.cell == upper));
        let route = graph_route(&graph, lower, upper).unwrap();
        assert_eq!(route.first(), Some(&lower));
        assert_eq!(route.last(), Some(&upper));
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
    fn column_queries_preserve_layers_after_update() {
        let observed = vec![
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(0, 0, 10),
            SurfaceCell::new(0, 0, 30),
            SurfaceCell::new(1, 0, 50),
        ];
        let inferred = vec![SurfaceCell::new(0, 0, 5)];
        let mut handle = LtMlsArtifactHandle {
            state: PlannerState::from_surface(observed.clone(), inferred, GraphConfig::default()),
            revision: 1,
        };
        handle
            .state
            .set_geometry(observed, Vec::new(), SurfaceConfig::default());
        for pass in 0..2 {
            for x in -1..=2 {
                for z in -1..=52 {
                    let surface = &handle.state.artifact.surface;
                    let expected = surface
                        .observed
                        .iter()
                        .chain(&surface.inferred)
                        .filter(|cell| cell.x == x && cell.y == 0 && cell.z <= z)
                        .map(|cell| cell.z)
                        .max();
                    let mut found = -999;
                    let status = unsafe { lt_mls_artifact_below(&handle, x, 0, z, &mut found) };
                    assert_eq!(status, i32::from(expected.is_some()));
                    if let Some(expected) = expected {
                        assert_eq!(found, expected);
                    }
                }
            }
            if pass == 0 {
                handle.state.update_region(
                    RegionBounds::new(0, 0, 0, 0, 8, 14),
                    &[SurfaceCell::new(0, 0, 12)],
                    &[],
                ).unwrap();
            }
        }
    }

    #[test]
    fn endpoint_attachment_adds_a_surface_lead_in() {
        let graph = SurfaceGraph {
            nodes: vec![
                Node {
                    id: 0,
                    cell: SurfaceCell::new(0, 0, 0),
                },
                Node {
                    id: 1,
                    cell: SurfaceCell::new(2, 0, 0),
                },
            ],
            edges: vec![Edge {
                a: 0,
                b: 1,
                cost: 1.0,
                corridor: vec![SurfaceCell::new(1, 0, 0)],
            }],
        };
        let route = graph_route_limited(
            &graph,
            SurfaceCell::new(0, 1, 0),
            SurfaceCell::new(2, 1, 0),
            Some(2),
            Some(100),
        )
        .expect("endpoint connectors");
        assert_eq!(route.first(), Some(&SurfaceCell::new(0, 1, 0)));
        assert_eq!(route.last(), Some(&SurfaceCell::new(2, 1, 0)));
        assert!(route.contains(&SurfaceCell::new(0, 0, 0)));
        assert!(route.contains(&SurfaceCell::new(2, 0, 0)));
    }

    #[test]
    fn endpoint_attachment_rejects_a_vertical_jump() {
        let graph = SurfaceGraph {
            nodes: vec![
                Node {
                    id: 0,
                    cell: SurfaceCell::new(0, 0, 2),
                },
                Node {
                    id: 1,
                    cell: SurfaceCell::new(2, 0, 2),
                },
            ],
            edges: vec![Edge {
                a: 0,
                b: 1,
                cost: 1.0,
                corridor: vec![SurfaceCell::new(1, 0, 2)],
            }],
        };
        assert!(graph_route_limited_avoiding_with_config(
            &graph,
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(2, 0, 2),
            Some(3),
            Some(100),
            &[],
            GraphConfig {
                max_step_cells: 1,
                ..GraphConfig::default()
            },
        )
        .is_none());
    }

    #[test]
    fn endpoint_attachment_uses_graph_step_limits() {
        let graph = SurfaceGraph {
            nodes: vec![
                Node {
                    id: 0,
                    cell: SurfaceCell::new(0, 0, 2),
                },
                Node {
                    id: 1,
                    cell: SurfaceCell::new(2, 0, 2),
                },
            ],
            edges: vec![Edge {
                a: 0,
                b: 1,
                cost: 1.0,
                corridor: vec![SurfaceCell::new(1, 0, 2)],
            }],
        };
        let config = GraphConfig {
            max_step_cells: 2,
            max_slope: 2.0,
            ..GraphConfig::default()
        };
        assert!(graph_route_limited_avoiding_with_config(
            &graph,
            SurfaceCell::new(0, 1, 0),
            SurfaceCell::new(2, 1, 2),
            Some(3),
            Some(100),
            &[],
            config,
        )
        .is_some());
        assert!(graph_route_limited_avoiding_with_config(
            &graph,
            SurfaceCell::new(0, 1, 0),
            SurfaceCell::new(2, 1, 2),
            Some(3),
            Some(100),
            &[],
            GraphConfig {
                max_step_cells: 1,
                max_slope: 2.0,
                ..GraphConfig::default()
            },
        )
        .is_none());
    }

    #[test]
    fn geometry_blocked_corridor_uses_rust_detour() {
        let graph = SurfaceGraph {
            nodes: vec![
                Node {
                    id: 0,
                    cell: SurfaceCell::new(0, 0, 0),
                },
                Node {
                    id: 1,
                    cell: SurfaceCell::new(2, 0, 0),
                },
                Node {
                    id: 2,
                    cell: SurfaceCell::new(1, 1, 0),
                },
            ],
            edges: vec![
                Edge {
                    a: 0,
                    b: 1,
                    cost: 1.0,
                    corridor: vec![SurfaceCell::new(1, 0, 0)],
                },
                Edge {
                    a: 0,
                    b: 2,
                    cost: 2.0,
                    corridor: vec![SurfaceCell::new(0, 1, 0), SurfaceCell::new(1, 1, 0)],
                },
                Edge {
                    a: 2,
                    b: 1,
                    cost: 2.0,
                    corridor: vec![SurfaceCell::new(2, 1, 0)],
                },
            ],
        };
        let route = graph_route_limited_avoiding_with_config(
            &graph,
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(2, 0, 0),
            Some(0),
            Some(20),
            &[SurfaceCell::new(1, 0, 0)],
            GraphConfig::default(),
        )
        .expect("detour route");
        assert!(route.contains(&SurfaceCell::new(1, 1, 0)));
        assert_eq!(route.first(), Some(&SurfaceCell::new(0, 0, 0)));
        assert_eq!(route.last(), Some(&SurfaceCell::new(2, 0, 0)));
    }

    #[test]
    fn route_check_reports_first_failure() {
        let mut occupied = (0..4)
            .map(|x| SurfaceCell::new(x, 0, 0))
            .collect::<Vec<_>>();
        occupied.push(SurfaceCell::new(2, 0, 1));
        let state = PlannerState::new(
            occupied,
            Vec::new(),
            SurfaceConfig {
                clearance_cells: 0,
                ..SurfaceConfig::default()
            },
            GraphConfig::default(),
        );
        let route = (0..4)
            .map(|x| SurfaceCell::new(x, 0, 0))
            .collect::<Vec<_>>();
        let config = LtMlsRouteCheckConfig {
            body_radius_cells: 1,
            body_height_cells: 1,
            max_step_cells: 1,
            max_slope: 1.0,
            ..LtMlsRouteCheckConfig::default()
        };
        let result = check_route(&state, &route, config);
        assert_eq!(result.valid, 0);
        assert_eq!(result.failure_kind, ROUTE_OCCUPIED);
        assert_eq!(result.failed_segment, 1);

        let result = check_route(
            &state,
            &route,
            LtMlsRouteCheckConfig {
                has_no_go: 1,
                no_go: LtMlsRegion {
                    min_x: 1,
                    max_x: 1,
                    min_y: 0,
                    max_y: 0,
                    min_z: 0,
                    max_z: 0,
                },
                ..LtMlsRouteCheckConfig::default()
            },
        );
        assert_eq!(result.failure_kind, ROUTE_NO_GO);
        assert_eq!(result.failed_segment, 0);
    }

    #[test]
    fn route_check_rejects_missing_surface_during_sweep() {
        let state = PlannerState::from_surface(
            vec![SurfaceCell::new(0, 0, 0), SurfaceCell::new(2, 0, 0)],
            Vec::new(),
            GraphConfig::default(),
        );
        let result = check_route(
            &state,
            &[SurfaceCell::new(0, 0, 0), SurfaceCell::new(2, 0, 0)],
            LtMlsRouteCheckConfig {
                max_step_cells: 1,
                max_slope: 1.0,
                ..LtMlsRouteCheckConfig::default()
            },
        );
        assert_eq!(result.valid, 0);
        assert_eq!(result.failure_kind, ROUTE_SWEEP);
        assert_eq!(result.failed_segment, 0);
        assert_eq!(result.failed_cell.x, 1);
    }

    #[test]
    fn route_check_applies_body_test_to_swept_cells() {
        let state = PlannerState::new(
            vec![
                SurfaceCell::new(0, 0, 0),
                SurfaceCell::new(1, 0, 0),
                SurfaceCell::new(2, 0, 0),
                SurfaceCell::new(1, 1, 1),
            ],
            Vec::new(),
            SurfaceConfig {
                clearance_cells: 0,
                ..SurfaceConfig::default()
            },
            GraphConfig::default(),
        );
        let result = check_route(
            &state,
            &[SurfaceCell::new(0, 0, 0), SurfaceCell::new(2, 0, 0)],
            LtMlsRouteCheckConfig {
                body_radius_cells: 1,
                body_height_cells: 1,
                max_step_cells: 1,
                max_slope: 1.0,
                ..LtMlsRouteCheckConfig::default()
            },
        );
        assert_eq!(result.valid, 0);
        assert_eq!(result.failure_kind, ROUTE_OCCUPIED);
        assert_eq!(result.failed_segment, 0);
        assert_eq!(result.failed_cell.x, 1);
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
                ..SurfaceConfig::default()
            },
        );
        assert!(map.inferred.is_empty());
    }

    #[test]
    fn closing_respects_known_free_and_headroom() {
        let mut occupied = Vec::new();
        for y in 0..3 {
            for x in 0..3 {
                if x != 1 || y != 1 {
                    occupied.push(SurfaceCell::new(x, y, 0));
                }
            }
        }
        let known_free = [SurfaceCell::new(1, 1, 0)];
        let map = extract_surfaces_with_free(
            &occupied,
            &known_free,
            SurfaceConfig {
                clearance_cells: 1,
                closing_passes: 1,
                ..SurfaceConfig::default()
            },
        );
        assert!(map.inferred.is_empty());
        assert_eq!(map.observed.len(), occupied.len());

        occupied.push(SurfaceCell::new(1, 1, 1));
        let map = extract_surfaces(
            &occupied,
            SurfaceConfig {
                clearance_cells: 1,
                closing_passes: 1,
                ..SurfaceConfig::default()
            },
        );
        assert!(map.inferred.is_empty());
    }

    #[test]
    fn respects_fractional_closing_radius() {
        let mut occupied = Vec::new();
        for y in 0..6 {
            for x in 0..6 {
                if x == 0 || x == 5 || y == 0 || y == 5 {
                    occupied.push(SurfaceCell::new(x, y, 0));
                }
            }
        }
        let map = extract_surfaces(
            &occupied,
            SurfaceConfig {
                clearance_cells: 1,
                closing_radius_cells: 1.5,
                ..SurfaceConfig::default()
            },
        );
        assert!(map.inferred.contains(&SurfaceCell::new(1, 1, 0)));
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
    fn incremental_graph_preserves_unchanged_node_cells() {
        let cells = (0..24)
            .flat_map(|x| (0..3).map(move |y| SurfaceCell::new(x, y, 0)))
            .collect::<Vec<_>>();
        let config = GraphConfig {
            node_spacing_cells: 2,
            max_link_cells: 1,
            ..GraphConfig::default()
        };
        let mut graph = IncrementalGraph::from_cells(&cells, config);
        let original_ids = graph
            .nodes
            .values()
            .map(|node| (node.cell, node.id))
            .collect::<HashMap<_, _>>();
        let original_nodes = graph
            .nodes
            .iter()
            .map(|(&id, node)| (id, node.cell))
            .collect::<HashMap<_, _>>();
        let outside_edges = graph
            .edges
            .iter()
            .filter(|(_, edge)| {
                let Some(&from) = original_nodes.get(&edge.from) else {
                    return false;
                };
                let Some(&to) = original_nodes.get(&edge.to) else {
                    return false;
                };
                from.x.max(to.x) <= 5 || from.x.min(to.x) >= 17
            })
            .map(|(&id, edge)| (id, edge.clone()))
            .collect::<Vec<_>>();
        let mut updated = cells.clone();
        updated.retain(|cell| !(cell.x == 11 && cell.y == 1));
        graph.update(&updated, RegionBounds::new(11, 11, 1, 1, 0, 0), config);
        for node in graph.nodes.values() {
            if let Some(&id) = original_ids.get(&node.cell) {
                assert_eq!(node.id, id, "regional repair changed a stable node id");
            }
        }
        for (id, edge) in outside_edges {
            assert_eq!(
                graph.edges.get(&id),
                Some(&edge),
                "regional repair changed an outside edge"
            );
        }
        assert!(
            graph_route_limited(
                &graph.to_surface_graph(),
                SurfaceCell::new(0, 1, 0),
                SurfaceCell::new(23, 1, 0),
                None,
                None,
            )
            .is_some(),
            "regional Voronoi repair dropped a valid boundary route"
        );
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
        assert!(state.incremental_graph.is_none());
        let mut eager = PlannerState::new(
            state.occupied.clone(),
            state.known_free.clone(),
            state.surface_config,
            state.graph_config,
        );
        eager.incremental_graph = Some(IncrementalGraph::from_cells(
            &graph_cells(&eager.artifact.surface),
            eager.graph_config,
        ));
        let expected_report = eager
            .update_region(RegionBounds::new(1, 1, 1, 1, 0, 0), &[], &[])
            .unwrap();
        let report = state
            .update_region(RegionBounds::new(1, 1, 1, 1, 0, 0), &[], &[])
            .unwrap();
        assert!(state.incremental_graph.is_some());
        assert_eq!(report, expected_report);
        assert_eq!(state.artifact, eager.artifact);
        let start = SurfaceCell::new(0, 0, 0);
        let goal = SurfaceCell::new(3, 2, 0);
        let route = graph_route(&state.artifact.graph, start, goal).unwrap();
        assert_eq!(
            route,
            graph_route(&eager.artifact.graph, start, goal).unwrap()
        );
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
    fn surface_and_cached_graph_artifacts_reject_source_updates() {
        let observed = vec![SurfaceCell::new(0, 0, 0), SurfaceCell::new(2, 0, 0)];
        let inferred = vec![SurfaceCell::new(1, 0, 0)];
        let config = GraphConfig::default();
        let surface = PlannerState::from_surface(observed.clone(), inferred.clone(), config);
        let restored =
            PlannerState::from_graph(observed, inferred, surface.artifact.graph.clone(), config);
        for state in [surface, restored] {
            let before = state.artifact.clone();
            let mut handle = LtMlsArtifactHandle { state, revision: 7 };
            assert!(handle
                .state
                .update_region(RegionBounds::new(0, 0, 0, 0, 0, 0), &[], &[])
                .is_err());
            let free = LtMlsCell { x: 0, y: 0, z: 0 };
            let mut report = LtMlsUpdateReport {
                revision: 99,
                ..LtMlsUpdateReport::default()
            };
            let status = unsafe {
                lt_mls_artifact_update(
                    &mut handle,
                    LtMlsRegion::default(),
                    std::ptr::null(),
                    0,
                    &free,
                    1,
                    &mut report,
                )
            };
            assert_eq!(status, -2);
            assert_eq!(handle.state.artifact, before);
            assert!(handle.state.incremental_graph.is_none());
            assert_eq!(handle.revision, 7);
            assert_eq!(report.revision, 99);
        }
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

    #[test]
    fn update_tall_columns() {
        let mut occupied = (0..24)
            .flat_map(|x| (0..5).map(move |y| SurfaceCell::new(x, y, 0)))
            .collect::<Vec<_>>();
        occupied.extend((1..=100).map(|z| SurfaceCell::new(12, 2, z)));
        let surface_config = SurfaceConfig {
            clearance_cells: 8,
            ..SurfaceConfig::default()
        };
        for z in [1, 40, 99] {
            let mut state = PlannerState::new(
                occupied.clone(),
                Vec::new(),
                surface_config,
                GraphConfig::default(),
            );
            state.update_region(RegionBounds::new(12, 12, 2, 2, z, z), &[], &[]).unwrap();
            let full =
                extract_surfaces_with_free(state.occupied(), state.known_free(), surface_config);
            assert_eq!(
                state.artifact().surface,
                full,
                "truncated wall at edit height {z}"
            );
        }
    }

    #[test]
    fn update_closing_context() {
        let surface_config = SurfaceConfig {
            clearance_cells: 3,
            closing_passes: 2,
            ..SurfaceConfig::default()
        };
        let occupied = (0..24)
            .flat_map(|x| {
                (0..12).flat_map(move |y| {
                    [0, 12].into_iter().filter_map(move |z| {
                        let hole = (10..=11).contains(&x) && (5..=6).contains(&y);
                        (!hole).then_some(SurfaceCell::new(x, y, z))
                    })
                })
            })
            .collect::<Vec<_>>();
        let mut state =
            PlannerState::new(occupied, Vec::new(), surface_config, GraphConfig::default());
        for cell in [SurfaceCell::new(8, 4, 0), SurfaceCell::new(14, 7, 12)] {
            state.update_region(
                RegionBounds::new(cell.x, cell.x, cell.y, cell.y, cell.z, cell.z),
                &[],
                &[],
            ).unwrap();
            let full =
                extract_surfaces_with_free(state.occupied(), state.known_free(), surface_config);
            assert_eq!(state.artifact().surface, full);
        }
    }

    #[test]
    fn update_source_states() {
        let hole = SurfaceCell::new(4, 4, 0);
        let occupied = (0..9)
            .flat_map(|x| (0..9).map(move |y| SurfaceCell::new(x, y, 0)))
            .filter(|cell| *cell != hole)
            .collect::<Vec<_>>();
        let surface_config = SurfaceConfig {
            closing_passes: 1,
            ..SurfaceConfig::default()
        };
        let mut state =
            PlannerState::new(occupied, Vec::new(), surface_config, GraphConfig::default());
        let region = RegionBounds::new(4, 4, 4, 4, 0, 0);
        for (occupied, free) in [
            (vec![], vec![hole]),
            (vec![], vec![]),
            (vec![hole], vec![]),
            (vec![], vec![]),
        ] {
            state.update_region(region, &occupied, &free).unwrap();
            assert_eq!(state.known_free(), free.as_slice());
            assert_eq!(state.occupied().contains(&hole), occupied.contains(&hole));
            let full =
                extract_surfaces_with_free(state.occupied(), state.known_free(), surface_config);
            assert_eq!(state.artifact().surface, full);
        }
    }

    #[test]
    fn restored_ids() {
        let cells = (0..24)
            .flat_map(|x| (0..3).map(move |y| SurfaceCell::new(x, y, 0)))
            .collect::<Vec<_>>();
        let config = GraphConfig {
            node_spacing_cells: 2,
            max_link_cells: 1,
            ..GraphConfig::default()
        };
        let mut graph = build_graph(&cells, config);
        for node in &mut graph.nodes {
            node.id += 100;
        }
        for edge in &mut graph.edges {
            edge.a += 100;
            edge.b += 100;
        }
        let original_ids = graph
            .nodes
            .iter()
            .map(|node| (node.cell, node.id))
            .collect::<HashMap<_, _>>();
        let mut state = PlannerState::from_graph(cells.clone(), Vec::new(), graph, config);
        assert!(state.incremental_graph.is_none());
        state.set_geometry(cells, Vec::new(), SurfaceConfig::default());
        state.update_region(RegionBounds::new(11, 11, 1, 1, 0, 0), &[], &[]).unwrap();
        assert!(state.incremental_graph.is_some());
        for node in &state.artifact.graph.nodes {
            if let Some(&id) = original_ids.get(&node.cell) {
                assert_eq!(node.id, id);
            }
        }
    }
}
