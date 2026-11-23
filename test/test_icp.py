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

import numpy as np
import pytest 
import icp 
import kd_tree
import brute_force_nn 

# creating that synthetic 2D cloud
def create_L_cloud(seed = 42, n=500):

  """
  Creates an L-shaped 2D point cloud.
  """

  rng = np.random.default_rng(seed)

  # |
  # |
  # |
  # |
  vertical_x = rng.uniform(-1.0, -0.7, n//2)
  vertical_y = rng.uniform(-0.15, 1.0, n//2)

  # _____
  horizontal_x = rng.uniform(-1.0, 1.0, n//2)
  horizontal_y = rng.uniform(-0.15, 0.15, n//2)

  vertical = np.column_stack((vertical_x, vertical_y))
  horizontal = np.column_stack((horizontal_x, horizontal_y))

  # combine arrays 
  return np.vstack((vertical, horizontal))

# make a map with a predetermined transformation 
def make_test_data():
  """
  Creates a map and scan using a predetermined transformation. 
  """

  map_points = create_L_cloud()

  # transformation is 10 deg, 0.5 x, and -0.3 y
  theta = np.deg2rad(10)
  transform = icp.make_transform(0.5, -0.3, theta)

  # reverse transformation to apply to larp robot points 
  scan_points = icp.transform_points(map_points, np.linalg.inv(transform))

  return scan_points, map_points, transform

# awesome niche tech incoming 
# reusable pytest function 
# allows us to test both kd and brute force 
@pytest.fixture(params=[kd_tree.KDTree, brute_force_nn.BruteForceNN])

# returns which of the nearest neighbor implementations it's doing currently 
def nn_class(request):
  return request.param


# test with perfect correspondence 
def test_best_fit_transform(): 

  # getting random points 
  rng = np.random.default_rng(10)
  A = rng.uniform(-2,2,(100,2))

  # rotate 25 degrees
  theta = np.deg2rad(25)

  # create rotation matrix 
  rotation = np.array([
    [np.cos(theta), -np.sin(theta)], 
    [np.sin(theta), np.cos(theta)]
  ])

  # translation
  translation = np.array([0.5, -0.3])

  # transform 
  B = (rotation @ A.T).T + translation 

  # test 
  R, t = icp.best_fit_transform(A, B)

  assert np.allclose(R, rotation, atol=1e-10)
  assert np.allclose(t, translation, atol=1e-10)

  # check that rotation matrix has a determinant of 1
  # -1 is reflection, not rotation
  assert np.isclose(np.linalg.det(R), 1.0)

def test_icp_basic(nn_class):

  # gimme fake data 
  scan_points, map_points, real_T = make_test_data()
  search = nn_class(map_points)

  # initial guess is identity matrix 
  # test icp with 100 max iterations and max_corr_dist of 0.5 m
  recovered_T, info = icp.icp(scan_points, search, map_points, np.eye(3), 100, 1e-6, 0.5)

  # okay fine u can be a little off 
  # allow a difference because icp never perfect (still using nearest neighbor)
  assert np.allclose(recovered_T, real_T, atol = 0.05)

  # at least 3 valid matched points found 
  assert info["num_inliers"] >= 3

  # make sure that ICP actually went through at least one iteration
  assert info["iterations"] > 0

def test_icp_guassy_noise(nn_class):
  """
  can icp still work with noise tho 
  """

  scan_points, map_points, real_T = make_test_data()

  # rng for noise 
  rng = np.random.default_rng(20)

  # apply gaussian noise to scan points 
  gaussy_noise = scan_points + rng.normal(0, 0.005, scan_points.shape)

  # go my nearest neighbor implementations 
  search = nn_class(map_points)

  # same as above, max_iter = 100, max_corr_dist is 0.5 m, identity matrix init guess
  recovered_T, info = icp.icp(gaussy_noise, search, map_points, np.eye(3), 100, 1e-6, 0.5)

  # more leeway for noise 
  assert np.allclose(recovered_T, real_T, atol = 0.05)

  # at least 3 points matched 
  assert info["num_inliers"] >= 3

def test_icp_with_outliers(nn_class):
  """
  can icp handle bs points tho 

  real world application: people walking by, random furniture
  """

  scan_points, map_points, real_T = make_test_data()
  rng = np.random.default_rng(30)

  # make bs points and add to original scan 
  outliers = rng.uniform(4.0, 6.0, (100,2))
  scan_with_outliers = np.vstack((scan_points, outliers))

  # tighten max_corr_dist since outliers r obvious 
  search = nn_class(map_points)
  recovered_T,info = icp.icp(scan_with_outliers, search, map_points, np.eye(3), 100, 1e-6, 0.2)

  assert np.allclose(recovered_T, real_T, atol = 0.03)

  # since we actually have outliers now, we should identify at least
  # 80 percent of our og data as inliers
  assert info["num_inliers"] >= 400

# since icp is sensitive to initial guess, create a test case that 
# documents this limitation 

# expected fail :(
@pytest.mark.xfail(reason = "ICP can converge to incorrect local minimum when initial guess " \
"is too far from true transformation")
def test_icp_large_rotation(nn_class):
  # make square point cloud 
  side = np.linspace(-1, 1, 50)
  top = np.column_stack((side, np.ones(50)))
  bottom = np.column_stack((side, -np.ones(50)))
  left = np.column_stack((-np.ones(50), side))
  right = np.column_stack((np.ones(50), side))
  map_points = np.vstack((top, bottom, left, right))

  # make trasformation with 90 degree rotation 
  real_T = icp.make_transform(0.3, -0.2, np.deg2rad(90))

  # fake robo data 
  scan_points = icp.transform_points(map_points, np.linalg.inv(real_T))

  search = nn_class(map_points)
  recovered_T, info = icp.icp(scan_points, search, map_points, np.eye(3), 100, 1e-6, 2.0)

  # will probably fail, rotation too large 
  assert np.allclose(recovered_T, real_T, atol=0.05)

# compare kd tree and brute force 
def test_epic_battle_nearest_neighbor_kdtree_vs_brute_force():
  scan_points, map_points, real_T = make_test_data()

  kd_tree_search = kd_tree.KDTree(map_points)
  brute_force_search = brute_force_nn.BruteForceNN(map_points)

  kd_T, kd_info = icp.icp(scan_points, kd_tree_search, map_points, np.eye(3), 100, 1e-6, 0.5)
  brute_T, brute_info = icp.icp(scan_points, brute_force_search, map_points, np.eye(3), 100, 1e-6, 0.5)

  # compare results
  assert np.allclose(kd_T, brute_T, atol = 1e-10)

  # compare valid matches
  assert kd_info["num_inliers"] == brute_info["num_inliers"]

  # compare valid transformation
  assert np.allclose(kd_T, real_T, atol = 0.03)

