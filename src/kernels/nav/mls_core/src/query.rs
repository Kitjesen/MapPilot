use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};

use crate::corridor::OrderedF32;
use crate::{adjacency, edges, graph_cells, nodes, GraphConfig, PlannerState, SurfaceCell};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum QueryError {
    Exhausted,
    Budget,
    Cancelled,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum QueryStage {
    #[default]
    None,
    StartAttach,
    GoalAttach,
    EndpointAttach,
    EdgeSearch,
}

pub(crate) struct Budget {
    pub(crate) remaining: usize,
}

impl Budget {
    pub(crate) fn spend(&mut self) -> Result<(), QueryError> {
        if self.remaining == 0 {
            return Err(QueryError::Budget);
        }
        self.remaining -= 1;
        Ok(())
    }
}

pub(crate) struct Surface {
    cells: Vec<SurfaceCell>,
    lookup: HashMap<SurfaceCell, usize>,
    adjacency: Vec<Vec<(usize, f32)>>,
    wall: Vec<f32>,
    config: GraphConfig,
    detour_offsets: Vec<(SurfaceCell, f32)>,
    node_indices: HashMap<u32, usize>,
    node_cells: Vec<Option<usize>>,
    node_links: Vec<Vec<(usize, usize, bool)>>,
}

impl Surface {
    fn new(state: &PlannerState) -> Self {
        let cells = graph_cells(&state.artifact.surface);
        let adjacency = adjacency::build_adjacency(&cells, state.graph_config);
        let columns = nodes::surface_columns(&cells);
        let wall = nodes::wall_distances(
            &cells,
            &adjacency,
            &columns,
            state.graph_config.max_link_cells.max(1),
        );
        let lookup: HashMap<SurfaceCell, usize> = cells
            .iter()
            .copied()
            .enumerate()
            .map(|(i, c)| (c, i))
            .collect();
        let graph = &state.artifact.graph;
        let node_indices: HashMap<u32, usize> = graph
            .nodes
            .iter()
            .enumerate()
            .map(|(i, node)| (node.id, i))
            .collect();
        let node_cells = graph
            .nodes
            .iter()
            .map(|node| lookup.get(&node.cell).copied())
            .collect();
        let mut node_links = vec![Vec::new(); graph.nodes.len()];
        for (i, edge) in graph.edges.iter().enumerate() {
            let (Some(&a), Some(&b)) = (node_indices.get(&edge.a), node_indices.get(&edge.b))
            else {
                continue;
            };
            node_links[a].push((b, i, false));
            node_links[b].push((a, i, true));
        }
        let mut surface = Self {
            cells,
            lookup,
            adjacency,
            wall,
            config: state.graph_config,
            detour_offsets: Vec::new(),
            node_indices,
            node_cells,
            node_links,
        };
        surface.detour_offsets = surface.build_detour_offsets();
        surface
    }

    fn allowed(&self, index: usize) -> bool {
        self.wall[index] + 1e-6 >= self.config.wall_clearance_cells.max(0) as f32
    }

    fn cost(&self, from: usize, to: usize, base: f32) -> f32 {
        base + edges::edge_penalty(
            self.cells[from],
            self.cells[to],
            self.wall[from].min(self.wall[to]),
            self.config,
        )
    }

    fn connector_within_limits(&self, from: SurfaceCell, to: SurfaceCell) -> bool {
        let rise = (i64::from(to.z) - i64::from(from.z)).unsigned_abs() as f32;
        let run = ((i64::from(to.x) - i64::from(from.x)) as f32)
            .hypot((i64::from(to.y) - i64::from(from.y)) as f32);
        (self.config.max_step_cells <= 0 || rise <= self.config.max_step_cells as f32)
            && (run == 0.0
                || self.config.max_slope <= 0.0
                || rise <= self.config.max_slope * run + 1e-6)
    }

    fn build_detour_offsets(&self) -> Vec<(SurfaceCell, f32)> {
        let horizontal = self.config.horizontal_hop_cells.max(1);
        let mut vertical = self.config.max_link_cells.max(1);
        if self.config.max_step_cells > 0 {
            vertical = vertical.min(self.config.max_step_cells);
        }
        let mut offsets = Vec::new();
        for dz in -vertical..=vertical {
            // The slope determines the minimum run. Body clearance can require
            // a longer or angled connection, so keep the per-rise hop square.
            let required_run = if self.config.max_slope > 0.0 {
                (dz.unsigned_abs() as f32 / self.config.max_slope).ceil() as i32
            } else {
                horizontal
            };
            let reach = horizontal
                .max(horizontal.saturating_mul(dz.abs()))
                .max(required_run);
            for dx in -reach..=reach {
                for dy in -reach..=reach {
                    let run = (dx as f32).hypot(dy as f32);
                    if run == 0.0 {
                        continue;
                    }
                    let offset = SurfaceCell::new(dx, dy, dz);
                    if self.connector_within_limits(SurfaceCell::new(0, 0, 0), offset) {
                        offsets.push((offset, run.hypot(dz as f32)));
                    }
                }
            }
        }
        offsets
    }

    fn detour_links(&self, index: usize, links: &mut Vec<(usize, f32)>) {
        links.clear();
        let from = self.cells[index];
        links.extend(self.detour_offsets.iter().filter_map(|&(offset, cost)| {
            let cell = SurfaceCell::new(from.x + offset.x, from.y + offset.y, from.z + offset.z);
            self.lookup.get(&cell).map(|&next| (next, cost))
        }));
    }
}

fn squared_distance(a: SurfaceCell, b: SurfaceCell) -> i64 {
    let dx = i64::from(a.x) - i64::from(b.x);
    let dy = i64::from(a.y) - i64::from(b.y);
    let dz = i64::from(a.z) - i64::from(b.z);
    dx * dx + dy * dy + dz * dz
}

fn checked(
    check: &mut impl FnMut(SurfaceCell, SurfaceCell, u32) -> i32,
    from: SurfaceCell,
    to: SurfaceCell,
    kind: u32,
) -> Result<bool, QueryError> {
    match check(from, to, kind) {
        value if value < 0 => Err(QueryError::Cancelled),
        0 => Ok(false),
        _ => Ok(true),
    }
}

struct Tree {
    distance: Vec<f32>,
    previous: Vec<usize>,
    seeds: HashSet<usize>,
}

impl Tree {
    fn path(&self, mut cell: usize) -> Vec<usize> {
        let mut result = vec![cell];
        while !self.seeds.contains(&cell) {
            cell = self.previous[cell];
            result.push(cell);
        }
        result.reverse();
        result
    }

    fn route(&self, surface: &Surface, goals: &[(usize, f32)]) -> Option<Vec<SurfaceCell>> {
        let finish = goals
            .iter()
            .copied()
            .filter(|&(i, _)| self.distance[i].is_finite())
            .min_by(|&(a, ac), &(b, bc)| {
                (self.distance[a] + ac).total_cmp(&(self.distance[b] + bc))
            })
            .map(|(index, _)| index)?;
        Some(
            self.path(finish)
                .into_iter()
                .map(|i| surface.cells[i])
                .collect(),
        )
    }
}

struct SnapCandidates {
    target: SurfaceCell,
    goal: bool,
    remaining: std::vec::IntoIter<(i64, usize)>,
    seeds: Vec<(usize, f32)>,
}

impl SnapCandidates {
    fn new(surface: &Surface, target: SurfaceCell, max_snap_cells: i32, goal: bool) -> Self {
        let limit = i64::from(max_snap_cells) * i64::from(max_snap_cells);
        let mut candidates = surface
            .cells
            .iter()
            .enumerate()
            .filter(|(index, _)| surface.allowed(*index))
            .filter(|(_, &cell)| surface.connector_within_limits(target, cell))
            .map(|(index, &cell)| (squared_distance(target, cell), index))
            .filter(|&(distance, _)| distance <= limit)
            .collect::<Vec<_>>();
        candidates.sort_unstable();
        Self {
            target,
            goal,
            remaining: candidates.into_iter(),
            seeds: Vec::new(),
        }
    }

    fn extend(
        &mut self,
        surface: &Surface,
        all: bool,
        budget: &mut Budget,
        check: &mut impl FnMut(SurfaceCell, SurfaceCell, u32) -> i32,
    ) -> Result<bool, QueryError> {
        let previous = self.seeds.len();
        for (distance, index) in self.remaining.by_ref() {
            budget.spend()?;
            let cell = surface.cells[index];
            let allowed = if self.goal {
                checked(check, cell, self.target, 2)?
            } else {
                checked(check, self.target, cell, 1)?
            };
            if allowed {
                self.seeds.push((index, (distance as f32).sqrt()));
                if !all {
                    break;
                }
            }
        }
        Ok(self.seeds.len() != previous)
    }
}

fn surface_search(
    surface: &Surface,
    seeds: &[(usize, f32)],
    centers: &[(SurfaceCell, i32)],
    goals: Option<&HashMap<usize, f32>>,
    reverse: bool,
    budget: &mut Budget,
    check: &mut impl FnMut(SurfaceCell, SurfaceCell, u32) -> i32,
    admitted: &mut HashMap<(usize, usize), bool>,
    checked_detour: bool,
) -> Result<Tree, QueryError> {
    let mut tree = Tree {
        distance: vec![f32::INFINITY; surface.cells.len()],
        previous: vec![usize::MAX; surface.cells.len()],
        seeds: HashSet::new(),
    };
    let estimate = |index: usize| {
        goals.map_or(0.0, |goals| {
            goals
                .iter()
                .map(|(&goal, &extra)| {
                    (squared_distance(surface.cells[index], surface.cells[goal]) as f32).sqrt()
                        + extra
                })
                .fold(f32::INFINITY, f32::min)
        })
    };
    // On equal lower bounds, advance one route instead of expanding every
    // equally good prefix before reaching a goal.
    let mut queue = BinaryHeap::<Reverse<(OrderedF32, Reverse<OrderedF32>, usize)>>::new();
    for &(seed, cost) in seeds {
        tree.distance[seed] = cost;
        tree.seeds.insert(seed);
        queue.push(Reverse((
            OrderedF32(cost + estimate(seed)),
            Reverse(OrderedF32(cost)),
            seed,
        )));
    }
    let mut best_goal = f32::INFINITY;
    let mut detour = Vec::new();
    while let Some(Reverse((OrderedF32(bound), Reverse(OrderedF32(cost)), current))) = queue.pop() {
        if cost > tree.distance[current] + 1e-6 {
            continue;
        }
        if bound >= best_goal - 1e-6 {
            break;
        }
        budget.spend()?;
        checked(check, surface.cells[current], surface.cells[current], 3)?;
        if let Some(extra) = goals.and_then(|goals| goals.get(&current)) {
            best_goal = best_goal.min(cost + extra);
        }
        let links = if checked_detour {
            surface.detour_links(current, &mut detour);
            &detour
        } else {
            &surface.adjacency[current]
        };
        for &(next, base) in links {
            if !surface.allowed(next)
                || (!centers.is_empty()
                    && !centers.iter().any(|&(point, radius)| {
                        squared_distance(point, surface.cells[next])
                            <= i64::from(radius) * i64::from(radius)
                    }))
            {
                continue;
            }
            let next_cost = cost + surface.cost(current, next, base);
            let next_bound = next_cost + estimate(next);
            if next_cost + 1e-6 >= tree.distance[next] || next_bound >= best_goal - 1e-6 {
                continue;
            }
            let pair = if reverse {
                (next, current)
            } else {
                (current, next)
            };
            let allowed = if let Some(&allowed) = admitted.get(&pair) {
                allowed
            } else {
                let allowed = checked(check, surface.cells[pair.0], surface.cells[pair.1], 0)?;
                admitted.insert(pair, allowed);
                allowed
            };
            if allowed {
                tree.distance[next] = next_cost;
                tree.previous[next] = current;
                tree.seeds.remove(&next);
                queue.push(Reverse((
                    OrderedF32(next_bound),
                    Reverse(OrderedF32(next_cost)),
                    next,
                )));
            }
        }
    }
    Ok(tree)
}

fn corridor_allowed(
    surface: &Surface,
    corridor: &[SurfaceCell],
    check: &mut impl FnMut(SurfaceCell, SurfaceCell, u32) -> i32,
    admitted: &mut HashMap<(usize, usize), bool>,
    budget: &mut Budget,
) -> Result<bool, QueryError> {
    for pair in corridor.windows(2) {
        let (Some(&a), Some(&b)) = (surface.lookup.get(&pair[0]), surface.lookup.get(&pair[1]))
        else {
            return Ok(false);
        };
        if !surface.allowed(a)
            || !surface.allowed(b)
            || !surface.adjacency[a].iter().any(|&(next, _)| next == b)
        {
            return Ok(false);
        }
        let allowed = if let Some(&allowed) = admitted.get(&(a, b)) {
            allowed
        } else {
            budget.spend()?;
            let allowed = checked(check, pair[0], pair[1], 0)?;
            admitted.insert((a, b), allowed);
            allowed
        };
        if !allowed {
            return Ok(false);
        }
    }
    Ok(!corridor.is_empty())
}

fn sparse_search(
    state: &PlannerState,
    surface: &Surface,
    starts: &Tree,
    goals: &Tree,
    budget: &mut Budget,
    check: &mut impl FnMut(SurfaceCell, SurfaceCell, u32) -> i32,
    admitted: &mut HashMap<(usize, usize), bool>,
) -> Result<Option<Vec<SurfaceCell>>, QueryError> {
    let graph = &state.artifact.graph;
    let indices = &surface.node_indices;
    let node_cells = &surface.node_cells;
    let links = &surface.node_links;

    // Geometry can only remove connections from this relaxed graph. Its
    // goal-rooted cost is a lower bound even for imported non-metric edge costs.
    let mut cost_to_go = node_cells
        .iter()
        .map(|cell| cell.map_or(f32::INFINITY, |cell| goals.distance[cell]))
        .collect::<Vec<_>>();
    let mut reverse_queue = BinaryHeap::<Reverse<(OrderedF32, usize)>>::new();
    for (node, &cost) in cost_to_go.iter().enumerate() {
        if cost.is_finite() {
            reverse_queue.push(Reverse((OrderedF32(cost), node)));
        }
    }
    while let Some(Reverse((OrderedF32(cost), current))) = reverse_queue.pop() {
        if cost > cost_to_go[current] + 1e-6 {
            continue;
        }
        budget.spend()?;
        checked(
            check,
            graph.nodes[current].cell,
            graph.nodes[current].cell,
            3,
        )?;
        for &(next, edge, _) in &links[current] {
            let next_cost = cost + graph.edges[edge].cost;
            if next_cost + 1e-6 < cost_to_go[next] {
                cost_to_go[next] = next_cost;
                reverse_queue.push(Reverse((OrderedF32(next_cost), next)));
            }
        }
    }

    let mut distance = vec![f32::INFINITY; graph.nodes.len()];
    let mut previous = vec![None::<(usize, bool)>; graph.nodes.len()];
    let mut queue = BinaryHeap::<Reverse<(OrderedF32, OrderedF32, usize)>>::new();
    for (node, cell) in node_cells.iter().enumerate() {
        if let Some(cell) = cell {
            if starts.distance[*cell].is_finite() && cost_to_go[node].is_finite() {
                distance[node] = starts.distance[*cell];
                queue.push(Reverse((
                    OrderedF32(distance[node] + cost_to_go[node]),
                    OrderedF32(distance[node]),
                    node,
                )));
            }
        }
    }
    let mut best = None::<(f32, usize)>;
    let mut edge_admission = HashMap::<(usize, bool), bool>::new();
    let mut corridor = Vec::new();
    while let Some(Reverse((OrderedF32(bound), OrderedF32(cost), current))) = queue.pop() {
        if cost > distance[current] + 1e-6 {
            continue;
        }
        if best.is_some_and(|(best, _)| bound >= best - 1e-6) {
            break;
        }
        budget.spend()?;
        let Some(cell) = node_cells[current] else {
            continue;
        };
        checked(check, surface.cells[cell], surface.cells[cell], 3)?;
        let total = cost + goals.distance[cell];
        if total.is_finite() && best.is_none_or(|(value, _)| total < value) {
            best = Some((total, current));
        }
        for &(next, edge, reverse) in &links[current] {
            let next_cost = cost + graph.edges[edge].cost;
            let next_bound = next_cost + cost_to_go[next];
            if next_cost + 1e-6 >= distance[next]
                || !next_bound.is_finite()
                || best.is_some_and(|(best, _)| next_bound >= best - 1e-6)
            {
                continue;
            }
            let allowed = if let Some(&allowed) = edge_admission.get(&(edge, reverse)) {
                allowed
            } else {
                corridor.clear();
                corridor.push(graph.nodes[current].cell);
                if reverse {
                    corridor.extend(graph.edges[edge].corridor.iter().rev().copied());
                } else {
                    corridor.extend(graph.edges[edge].corridor.iter().copied());
                }
                corridor.push(graph.nodes[next].cell);
                corridor.dedup();
                let allowed = corridor_allowed(surface, &corridor, check, admitted, budget)?;
                edge_admission.insert((edge, reverse), allowed);
                allowed
            };
            if allowed {
                distance[next] = next_cost;
                previous[next] = Some((edge, reverse));
                queue.push(Reverse((
                    OrderedF32(next_bound),
                    OrderedF32(next_cost),
                    next,
                )));
            }
        }
    }
    let Some((_, finish)) = best else {
        return Ok(None);
    };
    let mut node = finish;
    let mut segments = Vec::new();
    while let Some((edge, reverse)) = previous[node] {
        segments.push((edge, reverse));
        let parent_id = if reverse {
            graph.edges[edge].b
        } else {
            graph.edges[edge].a
        };
        node = indices[&parent_id];
    }
    let mut route = starts
        .path(node_cells[node].unwrap())
        .into_iter()
        .map(|i| surface.cells[i])
        .collect::<Vec<_>>();
    for (edge, reverse) in segments.into_iter().rev() {
        if reverse {
            route.extend(graph.edges[edge].corridor.iter().rev().copied());
        } else {
            route.extend(graph.edges[edge].corridor.iter().copied());
        }
        let end_id = if reverse {
            graph.edges[edge].a
        } else {
            graph.edges[edge].b
        };
        route.push(graph.nodes[indices[&end_id]].cell);
    }
    route.extend(
        goals
            .path(node_cells[finish].unwrap())
            .into_iter()
            .rev()
            .map(|i| surface.cells[i]),
    );
    route.dedup();
    Ok(Some(route))
}

pub(crate) fn plan(
    state: &PlannerState,
    start: SurfaceCell,
    goal: SurfaceCell,
    max_snap_cells: i32,
    max_iterations: usize,
    geometry_checked: bool,
    check: impl FnMut(SurfaceCell, SurfaceCell, u32) -> i32,
) -> Result<Vec<SurfaceCell>, QueryError> {
    plan_budgeted(
        state,
        start,
        goal,
        max_snap_cells,
        &mut Budget {
            remaining: max_iterations,
        },
        geometry_checked,
        check,
    )
}

pub(crate) fn plan_budgeted(
    state: &PlannerState,
    start: SurfaceCell,
    goal: SurfaceCell,
    max_snap_cells: i32,
    budget: &mut Budget,
    geometry_checked: bool,
    check: impl FnMut(SurfaceCell, SurfaceCell, u32) -> i32,
) -> Result<Vec<SurfaceCell>, QueryError> {
    let mut stage = QueryStage::None;
    plan_budgeted_with_stage(
        state,
        start,
        goal,
        max_snap_cells,
        budget,
        geometry_checked,
        &mut stage,
        check,
    )
}

pub(crate) fn plan_budgeted_with_stage(
    state: &PlannerState,
    start: SurfaceCell,
    goal: SurfaceCell,
    max_snap_cells: i32,
    budget: &mut Budget,
    geometry_checked: bool,
    stage: &mut QueryStage,
    mut check: impl FnMut(SurfaceCell, SurfaceCell, u32) -> i32,
) -> Result<Vec<SurfaceCell>, QueryError> {
    checked(&mut check, start, goal, 3)?;
    let surface = state.query_surface.get_or_init(|| Surface::new(state));
    let mut starts = SnapCandidates::new(surface, start, max_snap_cells, false);
    let mut goals = SnapCandidates::new(surface, goal, max_snap_cells, true);
    let mut admitted = HashMap::new();
    // Anchor the lead at the nearest executable surface. Other entries are
    // needed only when that surface cannot reach the goal.
    for all in [false, true] {
        *stage = QueryStage::EndpointAttach;
        let start_added = starts.extend(surface, all, budget, &mut check)?;
        let goal_added = goals.extend(surface, all, budget, &mut check)?;
        if starts.seeds.is_empty() {
            *stage = QueryStage::StartAttach;
            return Err(QueryError::Exhausted);
        }
        if goals.seeds.is_empty() {
            *stage = QueryStage::GoalAttach;
            return Err(QueryError::Exhausted);
        }
        if all && !start_added && !goal_added {
            *stage = QueryStage::EndpointAttach;
            return Err(QueryError::Exhausted);
        }
        *stage = QueryStage::EdgeSearch;
        let goal_costs = goals.seeds.iter().copied().collect::<HashMap<_, _>>();
        let radius = f64::from(max_snap_cells);
        if !all && squared_distance(start, goal) as f64 <= 4.0 * radius * radius {
            let tree = surface_search(
                &surface,
                &starts.seeds,
                &[(start, max_snap_cells), (goal, max_snap_cells)],
                Some(&goal_costs),
                false,
                budget,
                &mut check,
                &mut admitted,
                geometry_checked,
            )?;
            if let Some(route) = tree.route(surface, &goals.seeds) {
                return Ok(route);
            }
        }
        let start_tree = surface_search(
            &surface,
            &starts.seeds,
            &[(start, max_snap_cells)],
            None,
            false,
            budget,
            &mut check,
            &mut admitted,
            false,
        )?;
        let goal_tree = surface_search(
            &surface,
            &goals.seeds,
            &[(goal, max_snap_cells)],
            None,
            true,
            budget,
            &mut check,
            &mut admitted,
            false,
        )?;
        if let Some(route) = sparse_search(
            state,
            &surface,
            &start_tree,
            &goal_tree,
            budget,
            &mut check,
            &mut admitted,
        )? {
            return Ok(route);
        }
        let tree = surface_search(
            &surface,
            &starts.seeds,
            &[],
            Some(&goal_costs),
            false,
            budget,
            &mut check,
            &mut admitted,
            geometry_checked,
        )?;
        if let Some(route) = tree.route(surface, &goals.seeds) {
            return Ok(route);
        }
    }
    Err(QueryError::Exhausted)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Edge, Node, SurfaceGraph};

    #[test]
    fn updated_surface_changes_the_next_route() {
        let cells = (0..5).map(|x| SurfaceCell::new(x, 0, 0)).collect();
        let mut state = PlannerState::from_surface(cells, Vec::new(), GraphConfig::default());
        let start = SurfaceCell::new(0, 0, 0);
        let goal = SurfaceCell::new(4, 0, 0);
        assert!(plan(&state, start, goal, 0, 100, false, |_, _, _| 1).is_ok());
        state.update_region(crate::RegionBounds::new(2, 2, 0, 0, 0, 0), &[], &[]);
        assert_eq!(
            plan(&state, start, goal, 0, 100, false, |_, _, _| 1),
            Err(QueryError::Exhausted)
        );
        let added = (2..=5)
            .map(|x| SurfaceCell::new(x, 0, 0))
            .collect::<Vec<_>>();
        state.update_region(crate::RegionBounds::new(2, 5, 0, 0, 0, 0), &added, &[]);
        let restored = plan(
            &state,
            start,
            SurfaceCell::new(5, 0, 0),
            0,
            100,
            false,
            |_, _, _| 1,
        )
        .expect("restored graph reaches the new endpoint");
        assert_eq!(restored.first(), Some(&start));
        assert_eq!(restored.last(), Some(&SurfaceCell::new(5, 0, 0)));
    }

    fn state(cells: Vec<SurfaceCell>, graph: SurfaceGraph, config: GraphConfig) -> PlannerState {
        PlannerState::from_graph(cells, Vec::new(), graph, config)
    }

    #[test]
    fn prepared_detour_links_match_the_original_order_and_cost() {
        fn original(surface: &Surface, index: usize) -> Vec<(usize, f32)> {
            let from = surface.cells[index];
            let horizontal = surface.config.horizontal_hop_cells.max(1);
            let mut vertical = surface.config.max_link_cells.max(1);
            if surface.config.max_step_cells > 0 {
                vertical = vertical.min(surface.config.max_step_cells);
            }
            let mut links = Vec::new();
            for dz in -vertical..=vertical {
                let required_run = if surface.config.max_slope > 0.0 {
                    (dz.unsigned_abs() as f32 / surface.config.max_slope).ceil() as i32
                } else {
                    horizontal
                };
                let reach = horizontal
                    .max(horizontal.saturating_mul(dz.abs()))
                    .max(required_run);
                for dx in -reach..=reach {
                    for dy in -reach..=reach {
                        let run = (dx as f32).hypot(dy as f32);
                        if run == 0.0 {
                            continue;
                        }
                        let cell = SurfaceCell::new(from.x + dx, from.y + dy, from.z + dz);
                        let Some(&next) = surface.lookup.get(&cell) else {
                            continue;
                        };
                        if surface.connector_within_limits(from, cell) {
                            links.push((next, run.hypot(dz as f32)));
                        }
                    }
                }
            }
            links
        }
        let cells = (-4..=4)
            .flat_map(|x| {
                (-4..=4).flat_map(move |y| {
                    (-2..=2)
                        .filter(move |&z| (x + y + z) % 5 != 0)
                        .map(move |z| SurfaceCell::new(x, y, z))
                })
            })
            .collect::<Vec<_>>();
        for (vertical, step, horizontal, slope) in [
            (1, 0, 1, 0.0),
            (3, 2, 1, 0.4),
            (4, 3, 2, 0.57735026),
            (4, 4, 2, 0.7),
        ] {
            let config = GraphConfig {
                max_link_cells: vertical,
                max_step_cells: step,
                horizontal_hop_cells: horizontal,
                max_slope: slope,
                ..GraphConfig::default()
            };
            let state = state(cells.clone(), SurfaceGraph::default(), config);
            let surface = Surface::new(&state);
            let mut links = Vec::new();
            for index in (0..surface.cells.len()).step_by(13) {
                surface.detour_links(index, &mut links);
                let bits = |links: &[(usize, f32)]| {
                    links
                        .iter()
                        .map(|&(next, cost)| (next, cost.to_bits()))
                        .collect::<Vec<_>>()
                };
                assert_eq!(bits(&links), bits(&original(&surface, index)));
            }
        }
    }

    #[test]
    fn farther_snap_can_reach_the_goal() {
        let cells = vec![
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(0, 2, 0),
            SurfaceCell::new(1, 2, 0),
            SurfaceCell::new(2, 2, 0),
            SurfaceCell::new(3, 2, 0),
        ];
        let state = state(cells, SurfaceGraph::default(), GraphConfig::default());
        let goal = SurfaceCell::new(3, 2, 0);
        let route = plan(
            &state,
            SurfaceCell::new(0, 0, 0),
            goal,
            2,
            100,
            false,
            |from, _, kind| i32::from(kind != 2 || from == goal),
        )
        .expect("reachable alternate snap");
        assert_eq!(route.first(), Some(&SurfaceCell::new(0, 2, 0)));
        assert_eq!(route.last(), Some(&SurfaceCell::new(3, 2, 0)));
    }

    #[test]
    fn nearest_entries_avoid_unneeded_connector_checks() {
        let cells = (0..20)
            .flat_map(|x| (0..20).map(move |y| SurfaceCell::new(x, y, 0)))
            .collect::<Vec<_>>();
        let state = state(cells, SurfaceGraph::default(), GraphConfig::default());
        let mut connectors = [0; 2];
        let mut segments = 0;
        let start = SurfaceCell::new(8, 10, 0);
        let goal = SurfaceCell::new(12, 10, 0);
        let route = plan(&state, start, goal, 24, 10000, false, |_, _, kind| {
            if kind == 1 || kind == 2 {
                connectors[(kind - 1) as usize] += 1;
            }
            if kind == 0 {
                segments += 1;
            }
            1
        })
        .expect("nearest entries reach the goal");
        assert_eq!(connectors, [1, 1]);
        assert!(segments < 40, "goal search expanded the surrounding plane");
        assert_eq!(route.first(), Some(&start));
        assert_eq!(route.last(), Some(&goal));
    }

    #[test]
    fn nearest_entry_keeps_the_selected_floor() {
        let cells = [0, 4]
            .into_iter()
            .flat_map(|z| (0..5).map(move |x| SurfaceCell::new(x, 0, z)))
            .collect::<Vec<_>>();
        let state = state(cells, SurfaceGraph::default(), GraphConfig::default());
        let route = plan(
            &state,
            SurfaceCell::new(0, 0, 4),
            SurfaceCell::new(4, 0, 4),
            4,
            100,
            false,
            |_, _, _| 1,
        )
        .expect("upper floor route");
        assert!(route.iter().all(|cell| cell.z == 4));
    }

    #[test]
    fn alternate_entry_can_cancel_the_query() {
        let start = SurfaceCell::new(0, 0, 0);
        let alternate = SurfaceCell::new(0, 2, 0);
        let goal = SurfaceCell::new(1, 2, 0);
        let state = state(
            vec![start, alternate, goal],
            SurfaceGraph::default(),
            GraphConfig::default(),
        );
        let result = plan(&state, start, goal, 2, 100, false, |from, to, kind| {
            if kind == 1 && to == alternate {
                -1
            } else {
                i32::from(kind != 2 || from == goal)
            }
        });
        assert_eq!(result, Err(QueryError::Cancelled));
    }

    #[test]
    fn surface_detour_can_leave_the_endpoint_windows() {
        let cells = vec![
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(-1, 0, 0),
            SurfaceCell::new(-2, 0, 0),
            SurfaceCell::new(-2, 1, 0),
            SurfaceCell::new(-2, 2, 0),
            SurfaceCell::new(-1, 2, 0),
            SurfaceCell::new(0, 2, 0),
            SurfaceCell::new(1, 2, 0),
            SurfaceCell::new(2, 2, 0),
            SurfaceCell::new(2, 1, 0),
            SurfaceCell::new(2, 0, 0),
        ];
        let state = state(
            cells.clone(),
            SurfaceGraph::default(),
            GraphConfig::default(),
        );
        let route = plan(
            &state,
            cells[0],
            *cells.last().unwrap(),
            1,
            100,
            false,
            |_, _, _| 1,
        )
        .expect("route around the closed end of the corridor");
        assert_eq!(route, cells);
    }

    #[test]
    fn rejected_sparse_edge_uses_surface_detour() {
        let cells = (0..5)
            .flat_map(|x| (0..3).map(move |y| SurfaceCell::new(x, y, 0)))
            .collect::<Vec<_>>();
        let start = SurfaceCell::new(0, 1, 0);
        let goal = SurfaceCell::new(4, 1, 0);
        let graph = SurfaceGraph {
            nodes: vec![Node { id: 0, cell: start }, Node { id: 1, cell: goal }],
            edges: vec![Edge {
                a: 0,
                b: 1,
                cost: 4.0,
                corridor: (0..5).map(|x| SurfaceCell::new(x, 1, 0)).collect(),
            }],
        };
        let state = state(cells, graph, GraphConfig::default());
        let blocked = SurfaceCell::new(2, 1, 0);
        let route = plan(&state, start, goal, 0, 200, false, |a, b, kind| {
            if kind == 3 {
                1
            } else {
                i32::from(a != blocked && b != blocked)
            }
        })
        .expect("surface detour");
        assert!(!route.contains(&blocked));
        assert_eq!(route.first(), Some(&start));
        assert_eq!(route.last(), Some(&goal));
        let next = plan(&state, start, goal, 0, 200, false, |_, _, _| 1).expect("fresh query");
        assert!(next.contains(&blocked));
    }

    fn sparse_fixture(direct_cost: f32) -> (PlannerState, Surface, Tree, Tree) {
        let start = SurfaceCell::new(0, 0, 0);
        let goal = SurfaceCell::new(2, 0, 0);
        let detour = SurfaceCell::new(0, 2, 0);
        let cells = (0..3)
            .flat_map(|x| (0..3).map(move |y| SurfaceCell::new(x, y, 0)))
            .collect();
        let graph = SurfaceGraph {
            nodes: vec![
                Node { id: 0, cell: start },
                Node { id: 1, cell: goal },
                Node {
                    id: 2,
                    cell: detour,
                },
            ],
            edges: vec![
                Edge {
                    a: 0,
                    b: 1,
                    cost: direct_cost,
                    corridor: (0..3).map(|x| SurfaceCell::new(x, 0, 0)).collect(),
                },
                Edge {
                    a: 0,
                    b: 2,
                    cost: 0.1,
                    corridor: (0..3).map(|y| SurfaceCell::new(0, y, 0)).collect(),
                },
                Edge {
                    a: 2,
                    b: 1,
                    cost: 0.1,
                    corridor: vec![
                        detour,
                        SurfaceCell::new(1, 2, 0),
                        SurfaceCell::new(2, 2, 0),
                        SurfaceCell::new(2, 1, 0),
                        goal,
                    ],
                },
            ],
        };
        let state = state(cells, graph, GraphConfig::default());
        let surface = Surface::new(&state);
        let endpoint = |cell| {
            let index = surface.lookup[&cell];
            let mut distance = vec![f32::INFINITY; surface.cells.len()];
            distance[index] = 0.0;
            Tree {
                distance,
                previous: vec![usize::MAX; surface.cells.len()],
                seeds: HashSet::from([index]),
            }
        };
        let starts = endpoint(start);
        let goals = endpoint(goal);
        (state, surface, starts, goals)
    }

    #[test]
    fn sparse_search_uses_declared_edge_costs() {
        let (state, surface, starts, goals) = sparse_fixture(1.0);
        let route = sparse_search(
            &state,
            &surface,
            &starts,
            &goals,
            &mut Budget { remaining: 100 },
            &mut |_, _, _| 1,
            &mut HashMap::new(),
        )
        .unwrap()
        .expect("low-cost sparse detour");
        // The longer route costs 0.2. The geometric shortcut costs 1.0.
        assert!(route.contains(&SurfaceCell::new(0, 2, 0)));
    }

    #[test]
    fn sparse_route_reconstructs_imported_node_ids_in_both_directions() {
        let (mut state, _, starts, goals) = sparse_fixture(1.0);
        let graph = &mut state.artifact.graph;
        let ids = [91, 7, 305];
        for node in &mut graph.nodes {
            node.id = ids[node.id as usize];
        }
        for edge in &mut graph.edges {
            edge.a = ids[edge.a as usize];
            edge.b = ids[edge.b as usize];
        }
        let edge = &mut graph.edges[1];
        std::mem::swap(&mut edge.a, &mut edge.b);
        edge.corridor.reverse();
        graph.nodes.swap(0, 2);
        let surface = Surface::new(&state);
        let expected = vec![
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(0, 1, 0),
            SurfaceCell::new(0, 2, 0),
            SurfaceCell::new(1, 2, 0),
            SurfaceCell::new(2, 2, 0),
            SurfaceCell::new(2, 1, 0),
            SurfaceCell::new(2, 0, 0),
        ];
        for (starts, goals, expected) in [
            (&starts, &goals, expected.clone()),
            (&goals, &starts, expected.into_iter().rev().collect()),
        ] {
            let route = sparse_search(
                &state,
                &surface,
                starts,
                goals,
                &mut Budget { remaining: 100 },
                &mut |_, _, _| 1,
                &mut HashMap::new(),
            )
            .unwrap()
            .unwrap();
            assert_eq!(route, expected);
        }
    }

    #[test]
    fn sparse_search_routes_around_rejected_edge() {
        let (state, surface, starts, goals) = sparse_fixture(0.01);
        let blocked = SurfaceCell::new(1, 0, 0);
        let route = sparse_search(
            &state,
            &surface,
            &starts,
            &goals,
            &mut Budget { remaining: 100 },
            &mut |a, b, kind| i32::from(kind == 3 || (a != blocked && b != blocked)),
            &mut HashMap::new(),
        )
        .unwrap()
        .expect("detour after rejecting the cheapest corridor");
        assert!(route.contains(&SurfaceCell::new(0, 2, 0)));
        assert!(!route.contains(&blocked));
    }

    #[test]
    fn reverse_node_search_uses_the_query_budget_and_cancel() {
        let (state, surface, starts, goals) = sparse_fixture(1.0);
        assert_eq!(
            sparse_search(
                &state,
                &surface,
                &starts,
                &goals,
                &mut Budget { remaining: 0 },
                &mut |_, _, _| 1,
                &mut HashMap::new(),
            ),
            Err(QueryError::Budget)
        );
        assert_eq!(
            sparse_search(
                &state,
                &surface,
                &starts,
                &goals,
                &mut Budget { remaining: 100 },
                &mut |_, _, _| -1,
                &mut HashMap::new(),
            ),
            Err(QueryError::Cancelled)
        );
    }

    #[test]
    fn equal_cost_sparse_routes_keep_their_cost() {
        let side = 20;
        let cells = (0..side)
            .flat_map(|x| (0..side).map(move |y| SurfaceCell::new(x, y, 0)))
            .collect::<Vec<_>>();
        let nodes = cells
            .iter()
            .enumerate()
            .map(|(id, &cell)| Node {
                id: id as u32,
                cell,
            })
            .collect();
        let mut edges = Vec::new();
        for x in 0..side {
            for y in 0..side {
                for (dx, dy) in [(1, 0), (0, 1)] {
                    if x + dx < side && y + dy < side {
                        edges.push(Edge {
                            a: (x * side + y) as u32,
                            b: ((x + dx) * side + y + dy) as u32,
                            cost: 1.0,
                            corridor: vec![
                                SurfaceCell::new(x, y, 0),
                                SurfaceCell::new(x + dx, y + dy, 0),
                            ],
                        });
                    }
                }
            }
        }
        let state = state(cells, SurfaceGraph { nodes, edges }, GraphConfig::default());
        let surface = Surface::new(&state);
        let endpoint = |cell| {
            let index = surface.lookup[&cell];
            let mut distance = vec![f32::INFINITY; surface.cells.len()];
            distance[index] = 0.0;
            Tree {
                distance,
                previous: vec![usize::MAX; surface.cells.len()],
                seeds: HashSet::from([index]),
            }
        };
        let starts = endpoint(SurfaceCell::new(0, 0, 0));
        let goals = endpoint(SurfaceCell::new(side - 1, side - 1, 0));
        let mut checks = 0;
        let route = sparse_search(
            &state,
            &surface,
            &starts,
            &goals,
            &mut Budget { remaining: 5000 },
            &mut |_, _, kind| {
                if kind == 0 {
                    checks += 1;
                }
                1
            },
            &mut HashMap::new(),
        )
        .unwrap()
        .expect("equal-cost grid route");
        let cost = route
            .windows(2)
            .map(|pair| (squared_distance(pair[0], pair[1]) as f32).sqrt())
            .sum::<f32>();
        assert_eq!(cost, (2 * (side - 1)) as f32);
        eprintln!("equal-cost grid: cost={cost} checked_edges={checks}");
        assert!(checks < (side * side) as usize);
    }

    #[test]
    fn endpoint_searches_share_the_query_budget() {
        let cells = (0..8).map(|x| SurfaceCell::new(x, 0, 0)).collect();
        let state = state(cells, SurfaceGraph::default(), GraphConfig::default());
        let result = plan(
            &state,
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(7, 0, 0),
            0,
            2,
            false,
            |_, _, _| 1,
        );
        assert_eq!(result, Err(QueryError::Budget));
    }

    #[test]
    fn cancellation_is_distinct_from_exhaustion() {
        let cells = vec![SurfaceCell::new(0, 0, 0)];
        let state = state(cells, SurfaceGraph::default(), GraphConfig::default());
        let result = plan(
            &state,
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(0, 0, 0),
            0,
            20,
            false,
            |_, _, _| -1,
        );
        assert_eq!(result, Err(QueryError::Cancelled));
        let result = plan(
            &state,
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(0, 0, 0),
            0,
            20,
            false,
            |_, _, kind| i32::from(kind == 3),
        );
        assert_eq!(result, Err(QueryError::Exhausted));
    }

    #[test]
    fn endpoint_lead_in_obeys_the_graph_wall_distance() {
        let cells = (0..5)
            .flat_map(|x| (0..3).map(move |y| SurfaceCell::new(x, y, 0)))
            .collect::<Vec<_>>();
        let state = state(
            cells,
            SurfaceGraph::default(),
            GraphConfig {
                wall_clearance_cells: 1,
                ..GraphConfig::default()
            },
        );
        let result = plan(
            &state,
            SurfaceCell::new(0, 1, 0),
            SurfaceCell::new(4, 1, 0),
            0,
            100,
            false,
            |_, _, _| 1,
        );
        assert_eq!(result, Err(QueryError::Exhausted));
    }

    #[test]
    fn checked_abi_distinguishes_exact_endpoint_connectors() {
        unsafe extern "C" fn record(
            _from: crate::LtMlsCell,
            _to: crate::LtMlsCell,
            kind: u32,
            context: *mut std::ffi::c_void,
        ) -> i32 {
            (&mut *(context as *mut Vec<u32>)).push(kind);
            1
        }
        let state = state(
            vec![SurfaceCell::new(0, 0, 0)],
            SurfaceGraph::default(),
            GraphConfig::default(),
        );
        let handle = crate::LtMlsArtifactHandle { state, revision: 1 };
        let mut kinds = Vec::<u32>::new();
        let mut route = crate::LtMlsBuffer::default();
        let cell = crate::LtMlsCell { x: 0, y: 0, z: 0 };
        let status = unsafe {
            crate::lt_mls_artifact_query_checked(
                &handle,
                cell,
                cell,
                0,
                20,
                Some(record),
                &mut kinds as *mut _ as *mut std::ffi::c_void,
                &mut route,
            )
        };
        assert_eq!(status, 0);
        assert!(kinds.contains(&1));
        assert!(kinds.contains(&2));
        assert_eq!(route.len, 1);
        unsafe { crate::lt_mls_free_buffer(&mut route) };
    }

    #[test]
    fn legacy_query_keeps_disconnected_surfaces_separate() {
        let state = state(
            vec![
                SurfaceCell::new(0, 0, 0),
                SurfaceCell::new(1, 0, 0),
                SurfaceCell::new(4, 0, 0),
                SurfaceCell::new(5, 0, 0),
            ],
            SurfaceGraph::default(),
            GraphConfig::default(),
        );
        let handle = crate::LtMlsArtifactHandle { state, revision: 1 };
        let start = crate::LtMlsCell { x: 0, y: 0, z: 0 };
        let goal = crate::LtMlsCell { x: 5, y: 0, z: 0 };
        let mut route = crate::LtMlsBuffer::default();
        let status =
            unsafe { crate::lt_mls_artifact_query_limited(&handle, start, goal, 8, &mut route) };
        assert_eq!(status, -2);
        assert_eq!(route.len, 0);
    }

    #[test]
    fn surface_detour_stops_after_reaching_its_goal() {
        let cells = (0..100).map(|x| SurfaceCell::new(x, 0, 0)).collect();
        let state = state(cells, SurfaceGraph::default(), GraphConfig::default());
        let result = plan(
            &state,
            SurfaceCell::new(0, 0, 0),
            SurfaceCell::new(1, 0, 0),
            0,
            6,
            false,
            |_, _, _| 1,
        )
        .expect("goal reached inside budget");
        assert_eq!(
            result,
            vec![SurfaceCell::new(0, 0, 0), SurfaceCell::new(1, 0, 0)]
        );
    }

    #[test]
    fn checked_detour_connects_an_unequal_xy_stair_step() {
        let start = SurfaceCell::new(0, 0, 0);
        let goal = SurfaceCell::new(1, 2, 1);
        let config = GraphConfig {
            max_link_cells: 1,
            max_step_cells: 1,
            horizontal_hop_cells: 2,
            max_slope: 0.7,
            ..GraphConfig::default()
        };
        let stair = state(vec![start, goal], SurfaceGraph::default(), config);
        let route =
            plan(&stair, start, goal, 0, 20, true, |_, _, _| 1).expect("checked turn connection");
        assert_eq!(route, vec![start, goal]);
        let rejected = plan(&stair, start, goal, 0, 20, true, |_, _, kind| {
            i32::from(kind != 0)
        });
        assert_eq!(rejected, Err(QueryError::Exhausted));
        let limited = state(
            vec![start, goal],
            SurfaceGraph::default(),
            GraphConfig {
                max_slope: 0.4,
                ..config
            },
        );
        assert_eq!(
            plan(&limited, start, goal, 0, 20, true, |_, _, _| 1),
            Err(QueryError::Exhausted)
        );
    }

    #[test]
    fn angled_treads() {
        let start = SurfaceCell::new(0, 0, 0);
        let config = GraphConfig {
            max_link_cells: 5,
            max_step_cells: 5,
            horizontal_hop_cells: 2,
            max_slope: 0.577350269,
            ..GraphConfig::default()
        };
        for goal in [SurfaceCell::new(4, 4, 2), SurfaceCell::new(0, 8, 4)] {
            let stairs = state(vec![start, goal], SurfaceGraph::default(), config);
            for (from, to) in [(start, goal), (goal, start)] {
                assert_eq!(
                    plan(&stairs, from, to, 0, 30, true, |_, _, _| 1).unwrap(),
                    vec![from, to]
                );
                let mut checks = 0;
                assert_eq!(
                    plan(&stairs, from, to, 0, 30, true, |_, _, kind| {
                        if kind == 0 {
                            checks += 1;
                            0
                        } else {
                            1
                        }
                    }),
                    Err(QueryError::Exhausted)
                );
                assert!(checks > 0);
                assert_eq!(
                    plan(&stairs, from, to, 0, 30, false, |_, _, _| 1),
                    Err(QueryError::Exhausted)
                );
            }
        }
    }

    #[test]
    fn tall_treads() {
        let start = SurfaceCell::new(0, 0, 0);
        let goal = SurfaceCell::new(4, 0, 2);
        let config = GraphConfig {
            max_link_cells: 5,
            max_step_cells: 5,
            horizontal_hop_cells: 2,
            max_slope: 0.577350269,
            ..GraphConfig::default()
        };
        let ramp = state(vec![start, goal], SurfaceGraph::default(), config);
        for (from, to) in [(start, goal), (goal, start)] {
            let mut checked_segments = Vec::new();
            let route = plan(&ramp, from, to, 0, 30, true, |a, b, kind| {
                if kind == 0 {
                    checked_segments.push((a, b));
                }
                1
            })
            .expect("two-cell rise has a four-cell run");
            assert_eq!(route, vec![from, to]);
            assert_eq!(checked_segments, vec![(from, to)]);
        }
        for config in [
            GraphConfig {
                max_slope: 0.4,
                ..config
            },
            GraphConfig {
                max_step_cells: 1,
                ..config
            },
        ] {
            let limited = state(vec![start, goal], SurfaceGraph::default(), config);
            assert_eq!(
                plan(&limited, start, goal, 0, 30, true, |_, _, _| 1),
                Err(QueryError::Exhausted)
            );
        }
        // Wall and missing-support decisions remain with the segment checker.
        for (from, to) in [(start, goal), (goal, start)] {
            let mut rejected = false;
            assert_eq!(
                plan(&ramp, from, to, 0, 30, true, |a, b, kind| {
                    if kind == 0 && a != b {
                        rejected = true;
                        0
                    } else {
                        1
                    }
                }),
                Err(QueryError::Exhausted)
            );
            assert!(rejected);
        }
        assert_eq!(
            plan(&ramp, start, goal, 0, 30, false, |_, _, _| 1),
            Err(QueryError::Exhausted)
        );
        assert_eq!(
            plan(&ramp, start, goal, 0, 2, true, |_, _, _| 1),
            Err(QueryError::Budget)
        );
        assert_eq!(
            plan(&ramp, start, goal, 0, 30, true, |_, _, kind| {
                if kind == 0 {
                    -1
                } else {
                    1
                }
            }),
            Err(QueryError::Cancelled)
        );
        let flat_goal = SurfaceCell::new(4, 0, 0);
        let flat_gap = state(vec![start, flat_goal], SurfaceGraph::default(), config);
        assert_eq!(
            plan(&flat_gap, start, flat_goal, 0, 30, true, |_, _, _| 1),
            Err(QueryError::Exhausted)
        );
    }

    #[test]
    fn legacy_query_rejects_an_unchecked_xy_gap() {
        let start = SurfaceCell::new(0, 0, 0);
        let goal = SurfaceCell::new(1, 2, 1);
        let state = state(
            vec![start, goal],
            SurfaceGraph::default(),
            GraphConfig {
                max_link_cells: 1,
                max_step_cells: 1,
                horizontal_hop_cells: 2,
                max_slope: 0.7,
                ..GraphConfig::default()
            },
        );
        let handle = crate::LtMlsArtifactHandle { state, revision: 1 };
        let mut route = crate::LtMlsBuffer::default();
        let status = unsafe {
            crate::lt_mls_artifact_query_limited(
                &handle,
                crate::cell_to_abi(start),
                crate::cell_to_abi(goal),
                0,
                &mut route,
            )
        };
        assert_eq!(status, -2);
        assert_eq!(route.len, 0);
    }
}
