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
