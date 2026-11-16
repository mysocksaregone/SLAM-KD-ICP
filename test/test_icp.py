"""
Unit tests for ICP on synthetic 2D point clouds (11/19 deliverable).
Run with:  python3 -m pytest test/test_icp.py -v

WHAT TO WRITE
    - best_fit_transform alone: random points A, a known R and t,
      B = (R @ A.T).T + t. With PERFECT correspondences, the recovered R, t
      should match exactly (np.allclose). Also check det(R) == +1.
    - Full icp(): make a shape with structure (an L shape, room outline,
      or random points), transform it by a known small motion (e.g.
      10 deg, (0.3, -0.2) m), run ICP from the identity guess, and assert
      the recovered transform is close to the true one.
    - Noise: add small Gaussian noise to the points; should still be close
      (looser tolerance).
    - Outliers: add some random junk points to the scan; with outlier
      rejection on, it should still recover the transform.
    - Failure case (documents a limitation, not a bug): a large rotation
      like 90 deg on a symmetric shape may converge to the wrong answer.
      Mark with pytest.mark.xfail or just record it for the write-up.
    - Run the same test with KDTree and BruteForceNN -> identical results
      (pytest.mark.parametrize over the two classes).
"""
