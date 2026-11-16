"""
Run ICP offline on extracted bag data -- tune parameters without ROS.
    python3 scripts/offline_icp.py data/take1.npz maps/mac_1st_floor_final.yaml

WHAT TO WRITE
    - Load the map: read the yaml (resolution, origin, image), load the .pgm
      with matplotlib/PIL, convert to occupied points in meters. Careful: the
      image's row 0 is the TOP of the map, while the OccupancyGrid's row 0 is
      the BOTTOM -- flip vertically (np.flipud) before converting.
    - Build your KDTree on the map points.
    - Load the .npz from extract_bag.py.
    - Pick an initial pose for the first scan (find it by hand once, e.g.
      read it off RViz after using 2D Pose Estimate).
    - For each scan: initial guess = last correction applied to the odom
      pose, run icp(), store the result.
    - Plots (save to results/):
        * one scan before vs after alignment, overlaid on the map
        * odom-only path vs ICP path over the map
        * ICP mean error / iterations per scan (spikes = failure cases)
    - Expose params (max_corr_dist, max_iters, tol, downsampling) at the
      top so they're easy to tune.
"""
