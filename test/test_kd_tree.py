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

import os
import sys

import numpy as np

# let the tests find the robot_localization folder
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robot_localization.kd_tree import KDTree
from robot_localization.brute_force_nn import BruteForceNN


def check_matches(points, queries):
    """
    builds both searchers and makes sure the distances are the same
    """

    tree = KDTree(points)
    brute = BruteForceNN(points)

    # get answers from both
    _, tree_dists = tree.nearest_batch(queries)
    _, brute_dists = brute.nearest_batch(queries)

    # compare distances not indices since ties can pick different points
    assert np.allclose(tree_dists, brute_dists)


def test_random_sizes():
    # same seed every time so failures can be repeated
    rng = np.random.default_rng(0)

    # try a few different sizes
    for m in [1, 2, 10, 1000]:
        points = rng.random((m, 2)) * 10
        queries = rng.random((200, 2)) * 10
        check_matches(points, queries)


def test_single_point():
    points = np.array([[3.0, 4.0]])
    tree = KDTree(points)

    # only one point so it has to be that one
    index, dist = tree.nearest([0.0, 0.0])
    assert index == 0
    assert np.isclose(dist, 5.0)


def test_duplicate_points():
    rng = np.random.default_rng(1)

    # lots of copies of the same few points
    points = np.repeat(rng.random((5, 2)), 20, axis=0)
    queries = rng.random((100, 2))
    check_matches(points, queries)


def test_points_on_a_line():
    rng = np.random.default_rng(2)
    queries = rng.random((100, 2)) * 10

    # all points have the same x
    vertical = np.column_stack([np.full(50, 5.0), rng.random(50) * 10])
    check_matches(vertical, queries)

    # all points have the same y
    horizontal = np.column_stack([rng.random(50) * 10, np.full(50, 5.0)])
    check_matches(horizontal, queries)


def test_query_on_a_point():
    rng = np.random.default_rng(3)
    points = rng.random((100, 2))
    tree = KDTree(points)

    # asking for a stored point should give distance zero
    for p in points:
        _, dist = tree.nearest(p)
        assert np.isclose(dist, 0.0)


def test_query_far_away():
    rng = np.random.default_rng(4)
    points = rng.random((100, 2))

    # queries way outside the data
    queries = np.array([[1000.0, 1000.0], [-500.0, 20.0], [0.5, -999.0]])
    check_matches(points, queries)


def test_depth():
    rng = np.random.default_rng(5)

    # median split should keep the tree about log2 deep
    for m in [1, 2, 10, 100, 1000]:
        tree = KDTree(rng.random((m, 2)))
        assert tree.depth() <= np.ceil(np.log2(m + 1)) + 1
