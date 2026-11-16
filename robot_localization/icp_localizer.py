"""
ROS 2 node: ICP-based localization against a known map.

(Put "#!/usr/bin/env python3" as the very first line, above this docstring,
 and run chmod +x on this file. Add it to install(PROGRAMS ...) in
 CMakeLists.txt.)

WHAT IT DOES
    Odometry gives a pose that drifts. Each new scan, use the odom pose as the
    initial guess, run ICP against the map, and publish the corrected pose.

WHAT TO WRITE
    class ICPLocalizer(Node)
        __init__
            - Get the map from map_server (copy the GetMap client pattern from
              occupancy_field.py), convert with map_utils, build the KDTree
              ONCE (map never changes).
            - Subscribe: /scan (LaserScan), /odom (Odometry),
              /initialpose (PoseWithCovarianceStamped, from RViz's
              "2D Pose Estimate" button -- the bag does NOT start at the map
              origin, so ICP needs this starting guess).
            - Publish: corrected pose (PoseStamped), /odom_path and /icp_path
              (nav_msgs/Path, append one pose per scan), and the
              map -> odom TF.
            - Frames: map, odom, base_footprint (same as pf.py).
            - Reuse TFHelper from helper_functions.py like pf.py does.

        scan_received(msg)
            1. Get the odom pose matching the scan's timestamp.
            2. Initial guess (map frame) = T_map_odom @ T_odom_base, where
               T_map_odom is the correction from the last ICP result.
            3. scan -> points (map_utils.scan_to_points), run icp().
            4. If ICP converged with enough inliers, accept it:
               T_map_odom = T_map_base_icp @ inverse(T_odom_base).
               Otherwise keep the old correction (log a warning -- this is
               where you'll find failure cases).
            5. Publish pose, append to paths, broadcast map -> odom.

        Log ICP time per scan (time.perf_counter) -- needed for benchmarks.

    main(): rclpy.init, spin ICPLocalizer, shutdown.

RUNNING
    ros2 launch robot_localization icp_localizer.py map_yaml:=<path to map yaml>
    ros2 bag play bags/macfirst_floor_take_1 --clock
    Then set "2D Pose Estimate" in RViz.
"""
