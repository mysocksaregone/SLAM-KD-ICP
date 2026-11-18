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

        query = np.asarray(query)

        if query.shape != (2, ):
            raise ValueError("Inavlid dimensions")

        
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

        for query in queries: 
            index, distance = self.nearest(query)
            indices.append(index)
            distances.append(distance)

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
            if node is None: 
                return 0

            left_depth = get_depth(node.left)
            right_depth = get_depth(node.right)

            return 1 + max(left_depth, right_depth)

        return get_depth(self.root)
    

        




