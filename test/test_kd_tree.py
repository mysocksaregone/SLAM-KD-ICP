"""
Unit tests for the KD-tree: every answer must match brute force.
Run with:  python3 -m pytest test/test_kd_tree.py -v   (from the repo root)

WHAT TO WRITE
    - Random points: build a KDTree and BruteForceNN on the same random (M, 2)
      points (fixed np.random seed so failures are reproducible). For many
      random queries, assert the nearest DISTANCES are equal (np.isclose).
      Compare distances, not indices -- ties can give different but equally
      correct indices.
    - Several sizes: M = 1, 2, 10, 1000.
    - Edge cases: a single point; duplicate points; all points on one
      vertical/horizontal line (all the same x or y); query exactly on a
      stored point (distance 0); query far outside the data.
    - Depth: for M points the median-split tree depth should be about
      ceil(log2(M + 1)). Assert depth <= that (+1 slack).
    - Pruning check (optional): count nodes visited; assert it's much less
      than M on average for large M.

    LATER: k_nearest matches brute force's k smallest distances (sorted);
    radius_search returns exactly the set brute force finds within r.
"""
