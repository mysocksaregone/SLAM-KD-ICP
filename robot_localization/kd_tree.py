import numpy as np 

"""
Computes the squared distance between two points. 

Args: 
    a: The first point. 
    b: The second point. Must match dimensions 

Returns: 
    sum: the squared distance between a and b
"""
def squared_distance(a: np.ndarray, b: np.ndarray) -> float: 
    diff = a - b

    # matrix multiplication
    return diff @ diff 


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
    A KD tree for nearest-neighbor serach in k-dimensional space. 

    The tree is built once from a list of points. 

    Args: 
        points: list of points to be stored in the tree, all must 
        have the same number of dimensions
    
    """

    def __init__(self, points):

        if not points: 
            raise ValueError("Invalid points list: cannot be empty")

        self.points = np.asarray(points)

        # create a list of indices to keep track of each point's original position
        indices = list(range(len(points)))

        # dimension checks
        if self.points.ndim != 2 or self.points.shape[1] != 2: 
            raise ValueError("Invalid dimensions")

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

        # sort the indices based on specific dimension 
        sorted_indices = sorted(indices, key = lambda i: self.points[i][dim])

        # median 
        mid = len(sorted_indices) // 2

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
                near_side = node.right
            else: 
                far_side = node.right
                far_side = node.left

            # search near first
            search(near_side)

            # don't waste time with far unless it contains a closer point
            if diff * diff  < best_dist: 
                search(far_side)

            search(self.root)

            return best_index, best_dist ** 0.5






