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

import math

import numpy as np

# works both when run as a package and as a plain script
try:
    from robot_localization.angle_helpers import euler_from_quaternion, quaternion_from_euler
except ImportError:
    from angle_helpers import euler_from_quaternion, quaternion_from_euler


def occupancy_grid_to_points(data, width, height, resolution, origin_xy,
                             occupied_threshold=50):
    """
    returns the x y of every occupied cell in meters as an m by 2 array
    """

    # turn the flat list into rows and columns
    grid = np.asarray(data).reshape(height, width)

    # find rows and columns of cells that are occupied
    rows, cols = np.nonzero(grid > occupied_threshold)

    # use the middle of each cell and shift by the map origin
    x = origin_xy[0] + (cols + 0.5) * resolution
    y = origin_xy[1] + (rows + 0.5) * resolution

    # stack into an m by 2 array
    return np.column_stack([x, y])


def occupancy_grid_msg_to_points(msg, occupied_threshold=50):
    """
    same as above but takes a ros occupancy grid message
    """

    # pull the values out of the message
    info = msg.info
    origin_xy = (info.origin.position.x, info.origin.position.y)

    return occupancy_grid_to_points(msg.data, info.width, info.height,
                                    info.resolution, origin_xy,
                                    occupied_threshold)


def scan_to_points(ranges, angle_min, angle_increment,
                   range_min, range_max, step=1):
    """
    returns the x y of each good laser reading in the robot frame
    step lets you keep every kth point to go faster
    """

    ranges = np.asarray(ranges, dtype=float)

    # angle of each reading
    angles = angle_min + np.arange(len(ranges)) * angle_increment

    # keep only real readings inside the allowed range
    good = np.isfinite(ranges) & (ranges > 0)
    good &= (ranges >= range_min) & (ranges <= range_max)

    ranges = ranges[good]
    angles = angles[good]

    # turn polar into x y
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    points = np.column_stack([x, y])

    # keep every kth point
    return points[::step]


def pose_to_xytheta(pose_msg):
    """
    turns a ros pose into x y and yaw
    """

    q = pose_msg.orientation

    # we only need yaw since we are in 2d
    _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

    return pose_msg.position.x, pose_msg.position.y, yaw


def xytheta_to_pose(x, y, theta):
    """
    turns x y and yaw into a ros pose
    """

    # import here so the rest of the file works without ros
    from geometry_msgs.msg import Pose, Point, Quaternion

    # roll and pitch are zero in 2d
    q = quaternion_from_euler(0.0, 0.0, theta)

    return Pose(position=Point(x=float(x), y=float(y), z=0.0),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
