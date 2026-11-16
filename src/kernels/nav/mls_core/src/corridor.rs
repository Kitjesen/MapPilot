use std::collections::HashSet;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct OrderedF32(pub(crate) f32);

impl Eq for OrderedF32 {}

impl Ord for OrderedF32 {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&other.0)
    }
}

impl PartialOrd for OrderedF32 {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

pub(crate) fn chain_to_source(
    current: usize,
    predecessor: &[usize],
    node_cells: &[usize],
) -> Vec<usize> {
    let sources: HashSet<usize> = node_cells.iter().copied().collect();
    let mut chain = Vec::new();
    let mut cursor = current;
    chain.push(cursor);
    while !sources.contains(&cursor) {
        let next = predecessor[cursor];
        if next == usize::MAX || next == cursor {
            break;
        }
        cursor = next;
        chain.push(cursor);
    }
    chain
}

#[cfg(test)]
mod tests {
    use super::OrderedF32;
    use std::cmp::Reverse;
    use std::collections::BinaryHeap;

    #[test]
    fn ordered_cost_is_a_min_heap_key() {
        let mut queue = BinaryHeap::<Reverse<(OrderedF32, usize)>>::new();
        queue.push(Reverse((OrderedF32(2.0), 2)));
        queue.push(Reverse((OrderedF32(1.0), 1)));
        assert_eq!(queue.pop().unwrap().0 .0 .0, 1.0);
    }
}
