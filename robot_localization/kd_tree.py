"""
2D KD-tree built from scratch: median-split construction + nearest-neighbor
search with subtree pruning. This is the core data-structures deliverable.

WHAT TO WRITE
    class KDNode
        Fields: point (the (x, y) at this node), index (its row in the original
        array, so ICP can look up the matched map point), axis (0 = split on x,
        1 = split on y), left, right.

    class KDTree
        __init__(self, points)
            points: (M, 2) numpy array. Calls _build and stores the root.

        _build(self, indices, depth)            -> KDNode or None
            1. Base case: no points -> return None.
            2. axis = depth % 2  (alternate x, y, x, y, ...)
            3. Find the MEDIAN point along that axis (sort, or np.argpartition
               for O(n) median selection).
            4. Node = median point; left subtree = points below the median,
               right subtree = points above. Recurse with depth + 1.
            Complexity: O(M log M) with linear-time median (O(M log^2 M) if
            you fully sort at every level). Median split -> balanced tree,
            depth ~ log2(M).

        nearest(self, query)                    -> (index, distance)
            Recursive search, keeping track of "best so far":
            1. At a node, compute distance from query to node.point; update best.
            2. diff = query[axis] - node.point[axis]
               Go FIRST into the side the query falls on (near side).
            3. PRUNING: only visit the far side if |diff| < best distance.
               (The splitting line is |diff| away; if that's farther than the
               best point found, nothing on the far side can be closer.)
            Compare squared distances to skip sqrt until the end.
            Average O(log M); worst case O(M) (e.g. badly distributed points).

        nearest_batch(self, queries)            -> (indices (N,), distances (N,))
            Loop nearest() over each query. Same interface as BruteForceNN.

        depth(self)                             -> int
            Height of the tree. Needed for the "tree depth vs map size" plot.

    LATER (due 12/7):
        k_nearest(self, query, k)
            Keep a max-heap of size k (heapq with negated distances). Prune the
            far side when |diff| >= the k-th best distance.
        radius_search(self, query, r)
            Return all points within r. Prune the far side when |diff| > r.
            Useful for outlier rejection / correspondence gating in ICP.

TIPS
    - Store indices, not copies of points, so results map back to the array.
    - Python's default recursion limit (1000) is fine: depth ~ log2(M) ~ 17
      for 100k points.
    - Count nodes visited per query (optional counter) -- great evidence that
      pruning works, and a nice plot for the presentation.
"""
