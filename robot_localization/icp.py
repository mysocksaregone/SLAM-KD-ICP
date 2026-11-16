"""
Iterative Closest Point (ICP) in 2D: align a LIDAR scan to the map points.

ICP repeats two steps until it converges:
    1. Correspondences: for each scan point, find the nearest map point
       (using KDTree / BruteForceNN / sklearn BallTree -- all interchangeable).
    2. Alignment: compute the rigid transform (R, t) that best maps the scan
       points onto their matched map points (closed form via SVD).

WHAT TO WRITE
    best_fit_transform(A, B)                    -> (R (2x2), t (2,))
        A, B: (N, 2) arrays of matched pairs (A[i] should move onto B[i]).
        1. Centroids: ca = mean of A, cb = mean of B.
        2. Center both: A' = A - ca, B' = B - cb.
        3. Cross-covariance H = A'^T @ B'  (2x2).
        4. U, S, Vt = SVD(H);  R = Vt^T @ U^T.
        5. REFLECTION FIX: if det(R) < 0, negate the last row of Vt and
           recompute R (otherwise you can get a mirror image, not a rotation).
        6. t = cb - R @ ca.

    make_transform(x, y, theta) / transform_points(points, T)
        Helpers using 3x3 homogeneous matrices [[R, t], [0, 0, 1]].
        Composing transforms = matrix multiply; this makes accumulating the
        per-iteration corrections easy: T_total = T_step @ T_total.

    icp(scan_points, nn_search, map_points, init_T,
        max_iters=30, tol=1e-4, max_corr_dist=...)  -> (T_final, info)
        scan_points: (N, 2) points in the ROBOT frame.
        nn_search:   object with nearest_batch() (your KDTree etc).
        init_T:      3x3 initial guess (from odometry) for robot pose in map.
        Loop:
            a. Move scan into the map frame with the current T.
            b. nn_search.nearest_batch -> matched map points + distances.
            c. OUTLIER REJECTION: drop pairs with distance > max_corr_dist
               (or keep only the closest X% of pairs). Real LIDAR sees people,
               doors, glass, things not in the map -- without this ICP drifts.
            d. best_fit_transform on the kept pairs -> step transform.
            e. Accumulate: T = T_step @ T.
            f. CONVERGENCE: stop when the change in mean error (or the size of
               the step) is below tol, or max_iters hit.
        Return the final T plus info for debugging/benchmarks: iterations,
        final mean error, number of inliers, whether it converged.

TESTING (see test/test_icp.py)
    Synthetic first: take a shape (e.g. an L or a square outline), apply a
    known small rotation + translation, and check ICP recovers it.
    Note ICP only converges from a decent initial guess -- large rotations
    (> ~30 deg) can fall into a wrong local minimum. That's a failure case
    worth showing.

KEEP THIS FILE ROS-FREE (numpy only) so it can be tested with plain pytest.
"""
