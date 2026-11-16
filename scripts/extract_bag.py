"""
Extract scans and odometry from a rosbag2 into numpy arrays for offline work
(11/19 deliverable). Run after sourcing ROS:
    python3 scripts/extract_bag.py bags/macfirst_floor_take_1 data/take1.npz

The MAC bags contain: /scan (LaserScan, ~716 msgs), /odom (Odometry, ~1849),
/tf, /tf_static, /cmd_vel, /robot_description.

WHAT TO WRITE
    - Open the bag with rosbag2_py:
        SequentialReader, StorageOptions(uri=..., storage_id='sqlite3'),
        ConverterOptions('cdr', 'cdr')
    - Loop reader.has_next() / read_next() -> (topic, raw bytes, timestamp).
      Deserialize with rclpy.serialization.deserialize_message and
      rosidl_runtime_py.utilities.get_message(type_string).
    - For /scan: save timestamp, ranges (as an array), and angle_min,
      angle_increment, range_min, range_max (same every scan -- save once).
    - For /odom: save timestamp and (x, y, yaw) from pose.pose
      (yaw from the quaternion, see angle_helpers.py).
    - np.savez the arrays: scan_times, scan_ranges (S, num_beams),
      odom_times, odom_xytheta (K, 3), plus the scan metadata.
    - Bonus: for each scan, also store the odom pose with the closest
      timestamp (np.searchsorted) so offline ICP has its initial guess ready.
"""
