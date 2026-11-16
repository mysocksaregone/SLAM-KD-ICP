"""
Benchmarks: brute force vs our KDTree vs scikit-learn BallTree (12/7).
    python3 scripts/benchmark.py      -> saves plots to results/

WHAT TO WRITE
    - Map sizes M: e.g. [1e2, 3e2, 1e3, 3e3, 1e4, 3e4, 1e5] random 2D points
      (and also the real MAC map points as one data point).
    - Queries: a fixed number of query points (e.g. 360, like one scan).
    - For each structure and each M, measure with time.perf_counter
      (repeat a few times, take the median):
        * build time
        * total query time for the batch
    - Also record KDTree depth vs M (compare to log2 M).
    - ICP time per scan using each structure on real scans (from the .npz).
    - Plots (log-log axes make the complexity slopes visible):
        * build time vs M        (expect ~M log M for trees, ~0 for brute)
        * query time vs M        (expect ~log M for trees, ~M for brute)
        * tree depth vs M with a log2(M) reference line
        * bar chart of ICP time per scan
    - sklearn: sklearn.neighbors.BallTree(points).query(queries, k=1)
      (and optionally sklearn's KDTree too -- it's C-backed).
    - Discuss in the README: sklearn will be much faster in absolute terms
      (compiled C vs Python) -- compare SLOPES/growth, not raw seconds.
    - Verify correctness while benchmarking: all three must return the same
      distances.
"""
