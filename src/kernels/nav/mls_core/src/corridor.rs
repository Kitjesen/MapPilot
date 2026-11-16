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
    sources: &HashSet<usize>,
) -> Vec<usize> {
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
    use super::{chain_to_source, OrderedF32};
    use std::cmp::Reverse;
    use std::collections::{BinaryHeap, HashSet};

    #[test]
    fn source_chains() {
        let predecessor = [usize::MAX, 0, 1, 2, 4, usize::MAX, 5, usize::MAX];
        let sources = HashSet::from([0, 5]);
        let expected = [
            vec![0],
            vec![1, 0],
            vec![2, 1, 0],
            vec![3, 2, 1, 0],
            vec![4],
            vec![5],
            vec![6, 5],
            vec![7],
        ];
        for (current, expected) in expected.into_iter().enumerate() {
            assert_eq!(chain_to_source(current, &predecessor, &sources), expected);
        }
    }

    #[test]
    fn ordered_cost_is_a_min_heap_key() {
        let mut queue = BinaryHeap::<Reverse<(OrderedF32, usize)>>::new();
        queue.push(Reverse((OrderedF32(2.0), 2)));
        queue.push(Reverse((OrderedF32(1.0), 1)));
        assert_eq!(queue.pop().unwrap().0 .0 .0, 1.0);
    }
}
