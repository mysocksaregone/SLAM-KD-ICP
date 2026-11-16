"""
Helpers to turn ROS map/scan data into plain numpy point arrays (in METERS).

WHAT TO WRITE
    occupancy_grid_to_points(data, width, height, resolution, origin_xy,
                             occupied_threshold=50)       -> (M, 2) array
        Return the world (x, y) coordinate of every OCCUPIED cell.
        - The grid is row-major: cell (i = column, j = row) is at
          data[i + j * width].
        - Occupied = value > threshold (values are -1 unknown, 0..100).
        - Convert grid index -> meters (use the cell center):
              x = origin_x + (i + 0.5) * resolution
              y = origin_y + (j + 0.5) * resolution
        - For the MAC map: resolution 0.03 m, origin (-1.12, -39.4), yaw 0.
        NOTE: occupancy_field.py keeps points in grid INDICES -- ICP needs
        meters, so don't reuse its arrays directly.
        Tip: vectorize with np.array(data).reshape(height, width) + np.nonzero
        instead of a double for-loop (the starter code's loop is slow).
        Taking the args as plain values (not the ROS msg) keeps this testable
        without ROS; add a small wrapper that unpacks a nav_msgs/OccupancyGrid.

    scan_to_points(ranges, angle_min, angle_increment,
                   range_min, range_max)                  -> (N, 2) array
        Convert a LaserScan to (x, y) points in the ROBOT (laser) frame.
        - angle_i = angle_min + i * angle_increment
        - x = r cos(angle), y = r sin(angle)
        - DROP invalid readings: inf, nan, 0, < range_min, > range_max.
        - Optional: downsample (every k-th point) to speed up ICP.
        (helper_functions.TFHelper.convert_scan_to_polar_in_robot_frame does
        something similar if you'd rather reuse it inside the node.)

    pose_to_xytheta(pose_msg) / xytheta_to_pose(x, y, theta)
        Convert geometry_msgs/Pose <-> (x, y, yaw). See angle_helpers.py for
        quaternion <-> yaw conversion.
"""
