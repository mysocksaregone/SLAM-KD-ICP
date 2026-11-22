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

import numpy as np


def best_fit_transform(A, B):
    """
    finds the rotation r and shift t that best moves points a onto points b
    a and b are n by 2 and a i should land on b i
    """

    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)

    # center of each set of points
    ca = A.mean(axis=0)
    cb = B.mean(axis=0)

    # move both sets so their centers are at zero
    A_centered = A - ca
    B_centered = B - cb

    # how the two sets line up with each other
    H = A_centered.T @ B_centered

    # svd gives us the best rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # if we got a mirror flip instead of a rotation fix it
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # shift that moves the rotated center of a onto the center of b
    t = cb - R @ ca

    return R, t


def make_transform(x, y, theta):
    """
    makes a 3 by 3 matrix from x y and angle
    """

    c = np.cos(theta)
    s = np.sin(theta)

    return np.array([[c, -s, x],
                     [s, c, y],
                     [0.0, 0.0, 1.0]])


def rt_to_transform(R, t):
    """
    makes a 3 by 3 matrix from a rotation and a shift
    """

    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = t
    return T


def transform_to_xytheta(T):
    """
    pulls x y and angle back out of a 3 by 3 matrix
    """

    return T[0, 2], T[1, 2], np.arctan2(T[1, 0], T[0, 0])


def transform_points(points, T):
    """
    moves n by 2 points by a 3 by 3 matrix
    """

    points = np.asarray(points, dtype=float)

    # rotate then shift
    return points @ T[:2, :2].T + T[:2, 2]


def icp(scan_points, nn_search, map_points, init_T,
        max_iters=30, tol=1e-4, max_corr_dist=0.5):
    """
    lines up the scan with the map starting from init_T
    nn_search is a kd tree or brute force built on map_points
    returns the final 3 by 3 matrix and an info dict
    """

    map_points = np.asarray(map_points, dtype=float)
    T = np.array(init_T, dtype=float)

    converged = False
    mean_error = None
    num_inliers = 0

    for iteration in range(1, max_iters + 1):

        # move the scan into the map frame with our current guess
        moved = transform_points(scan_points, T)

        # find the closest map point for each scan point
        indices, distances = nn_search.nearest_batch(moved)

        # throw out pairs that are too far apart since they are probably junk
        keep = distances <= max_corr_dist
        num_inliers = int(keep.sum())

        # need at least 3 pairs to get a good fit
        if num_inliers < 3:
            break

        mean_error = float(distances[keep].mean())

        # find the small fix that lines up the kept pairs
        R, t = best_fit_transform(moved[keep], map_points[indices[keep]])

        # add the fix on top of our guess
        T = rt_to_transform(R, t) @ T

        # stop when the fix is so small the scan barely moved
        step_angle = abs(np.arctan2(R[1, 0], R[0, 0]))
        if np.linalg.norm(t) < tol and step_angle < tol:
            converged = True
            break

    info = {
        "iterations": iteration,
        "mean_error": mean_error,
        "num_inliers": num_inliers,
        "converged": converged,
    }

    return T, info
