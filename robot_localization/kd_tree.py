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

import numpy as np 

class KDNode: 
    """
    One node of a KD tree.

    Attributes:
    point: the point stored at this current node
    dim: the dimension this node splits on 
    left: subtree of poitns with smaller values on the same dimension
    right: subtree of points with larger values on the same dimension
    """

    def __init__ (self, point, index, dim, left=None, right = None) :
        self.point = point
        self.index = index
        self.dim = dim
        self.left = left
        self.right = right

class KDTree: 
    """
    A KD tree for nearest-neighbor search in 2D space. 

    The tree is built once from a list of points. 

    Args: 
        points: list of points to be stored in the tree, all must 
        have the same number of dimensions
    
    """

    def __init__(self, points):

        # convert points to numpy array
        self.points = np.asarray(points)

        # create a list of indices to keep track of each point's original position
        indices = list(range(len(points)))

        # check that array has 2 dims and that 2nd dim has its own 2 dimensions
        if self.points.ndim != 2 or self.points.shape[1] != 2: 
            raise ValueError("Invalid dimensions")

        # check that array of points isn't empty
        if len(self.points) == 0: 
            raise ValueError("Points are empty")

        # build tree
        self.root = self.build(indices, 0)

    # building the subtree 
    def build(self, indices, depth) -> KDNode | None: 
        """
        Recursively builds a subtree. Splits on dimensions corresponding to the 
        current depth while cycling through dimensions and uses the median 
        as the splitting point. 

        Args: 
            points: list of points belonging to this subtree
            depth: current depth in the tree 

        Returns: 
            The root node of the subtree or none if the provided list is
            empty.
        """

        if len(indices) == 0:
            return None

        dim = depth % 2

        # median 
        mid = len(indices) // 2

        # partially rearranges elements so median is in the right spot
        partitioned = np.argpartition(
            self.points[indices,dim], mid
        )

        # indexing using index array AKA fancy index 
        # apply rearrangement to original point indices 
        # original indices of the points arranged around median
        sorted_indices = np.asarray(indices)[partitioned]
        
        median_index = sorted_indices[mid]

        return KDNode(point = self.points[median_index], 
                      index = median_index, 
                      dim = dim, 
                      left = self.build(sorted_indices[:mid], depth + 1), 
                      right = self.build(sorted_indices[mid+1:], depth + 1))

    def nearest(self, query): 
        """
        Finds the nearest point to query. 

        Returns: 
            index: index of the point 
            distance: distance between points 
        
        """

        # convert to array 
        query = np.asarray(query)

        # dim check 
        if query.shape != (2, ):
            raise ValueError("Inavlid dimensions")

        # best dist is max value 
        best_index = None
        best_dist = float("inf")

        def search(node):

            # use the ones we just defined
            nonlocal best_index, best_dist

            if node is None: 
                return 

            # get euclidean distance 
            diff_vector = query - node.point

            # matrix multiplication 
            distance = diff_vector @ diff_vector

            # update best point 
            if distance < best_dist: 
                best_dist = distance 
                best_index = node.index

            # get near and far sides 
            diff = query[node.dim] - node.point[node.dim]

            # if difference is negative...
            if diff < 0: 
                near_side = node.left
                far_side = node.right
            else: 
                near_side = node.right
                far_side = node.left

            # search near first
            search(near_side)

            # don't waste time with far unless it contains a closer point
            if diff * diff  < best_dist: 
                search(far_side)

        search(self.root)

        # conver squared euclidean distance back to regular euc dist 
        return best_index, best_dist ** 0.5

    # nearest neighbor search
    def nearest_batch(self, queries):
        """
        Given an array of queries, return arrays of the indices and 
        distances for each of the queries

        Returns: 
            indices: array of nearest index for each query
            distances: array of the distances to these nearest
            points for each query 
        
        """
        queries = np.asarray(queries)

        # dim check
        if queries.ndim != 2 or queries.shape[1] != 2:
            raise ValueError("invalid dimensions")

        indices = []
        distances = []

        # get the nearest of each query point and store index + dist
        for query in queries: 
            index, distance = self.nearest(query)
            indices.append(index)
            distances.append(distance)

        # convert and return indices and distances to arrays
        return np.asarray(indices), np.asarray(distances)

        
    def depth(self):
        """
        Calculates the height (maximum depth) of the KD tree. 

        Returns: 
            int: The max number of levels in the tree, 0 if empty
        """

        def get_depth(node):
            """
            Given a node, recursievly calculates the height of a 
            subtree. Recursively called in depth(). 

                Returns: 
                    int: The height of the subtree. 0 if no node. 
            """

            # base case 
            if node is None: 
                return 0

            # get depth of left subtree 
            left_depth = get_depth(node.left)

            # get depth of right subtree 
            right_depth = get_depth(node.right)

            # return current dim + biggest depth between left and right subtree
            return 1 + max(left_depth, right_depth)

        return get_depth(self.root)
    

        




