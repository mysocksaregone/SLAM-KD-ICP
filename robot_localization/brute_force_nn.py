"""
Brute-force nearest-neighbor search (the "ground truth" baseline).

WHY THIS FILE EXISTS
    - It is the correct answer your KD-tree tests are checked against.
    - It is the O(N*M) baseline line on your benchmark plots.
    Keep it dead simple so you trust it 100%.

WHAT TO WRITE
    class BruteForceNN
        __init__(self, points)
            points: (M, 2) numpy array of map points. Just store it.
            "Build time" is basically zero -- worth noting in the write-up.

        nearest(self, query)
            query: a single (2,) point.
            Return (index, distance) of the closest stored point.
            Check every point -> O(M) per query.

        nearest_batch(self, queries)
            queries: (N, 2) array. Return (indices (N,), distances (N,)).
            Total cost O(N*M). This is what ICP step 1 would cost without a tree.

TIPS
    - Write a pure-Python-loop version first (fair comparison against your
      pure-Python KD-tree), then optionally a NumPy-vectorized version.
      Benchmark both and discuss: vectorized brute force can beat a Python
      KD-tree at small M because of interpreter overhead -- a good "surprise"
      to explain in the write-up.
    - Use the SAME method names as KDTree so ICP can take either one.
"""

import numpy as np


class BruteForceNN:
    """
    checks every point to find the nearest one
    slow but always right so we use it to test the kd tree
    """

    def __init__(self, points):

        # turn points into an array and save them
        self.points = np.asarray(points)

        # make sure points are n by 2
        if self.points.ndim != 2 or self.points.shape[1] != 2:
            raise ValueError("invalid dimensions")

    def nearest(self, query):
        """
        returns index and distance of the closest point to query
        """

        # turn query into an array
        query = np.asarray(query)

        # make sure query is one 2d point
        if query.shape != (2,):
            raise ValueError("invalid dimensions")

        # start with nothing found yet
        best_index = None
        best_dist = float("inf")

        # look at every point one by one
        for i in range(len(self.points)):

            # squared distance from query to this point
            diff = query - self.points[i]
            dist = diff @ diff

            # keep it if its the closest so far
            if dist < best_dist:
                best_dist = dist
                best_index = i

        # take the square root to get real distance
        return best_index, best_dist ** 0.5

    def nearest_batch(self, queries):
        """
        returns arrays of indices and distances for a bunch of queries
        """

        # turn queries into an array
        queries = np.asarray(queries)

        # make sure queries are n by 2
        if queries.ndim != 2 or queries.shape[1] != 2:
            raise ValueError("invalid dimensions")

        indices = []
        distances = []

        # find the nearest point for each query
        for query in queries:
            index, distance = self.nearest(query)
            indices.append(index)
            distances.append(distance)

        # return as arrays
        return np.asarray(indices), np.asarray(distances)
