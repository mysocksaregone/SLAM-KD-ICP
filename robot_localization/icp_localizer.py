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

import time y
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time 
from rclpy.duration import Duration 
from rclpy.qos import qos_profile_sensor_data 

from nav_msgs.srv import GetMap
from nav.msgs.msg import Odometry, Path

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped, Pose)

from tf2_ros import TransformException

from helper_functions import TFHelper, stamped_transform_to_pose
from map_utils import occupancy_grid_msg_to_points, scan_to_points, pose_to_xytheta, xytheta_to_pose
from icp import icp, make_transform, transform_points

class ICPLocalizer(Node):
    def __init__(self):

        # initialize as ROS2 Node
        super().__init__('icp_localizer')

        # fixed global coordinate system 
        self.map_frame = "map"

        # coordinate system from wheel odometry
        self.odom_frame = "odom"

        # base coordinate system 
        self.base_frame = "base_footprint"

        self.scan_topic = "/scan" \

        # icp settings 
        self.max_iters = 30
        self.tol = 1e-4
        self.max_corr_dist = 0.5
        self.min_inliers = 10

        # transformation from map frame to odom frame 
        # initial guess received from RVIZ
        self.T_map_odom = np.eye(3)

        # remains false till we get something from RVIZ 
        self.intialized = False 

        # latest odometry message 
        self.latest_odom = None

        self.transform_helper = TFHelper(self)

        self.get_logger().info("waiting on map_server...")
        self.map_client = self.create_client(GetMap, 'map_server/map')

        # request occupancy grid 
        request = GetMap.Request()

        future = self.map_client.call_async(request)
        response = future.result()

        if response is None: 
            raise RuntimeError("no map received from map_server :(")

        self.map = response.map
        self.get_logger().info("Map received and dimensions are W = %d, H = %d" % (
            self.map.info.width, 
            self.map.info.height
        ))

        # convert map to points, each occupancy cell becomes a point representing center 
        self.map_points = occupancy_grid_msg_to_points(self.map, occupied_threshold = 50)

        if len(self.map_points) == 0:
            raise RuntimeError("No occupied points.")


        # building kd tree 
        self.get_logger().info("Starting KDTree build...")
        self.map_tree = KDTree(self.map_points)
        self.get_logger().info("KDTree built successfully!!")

        # receive LiDAR scans using subscriber nodes 
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_received, qos_profile_sensor_data)

        # receive latest odometry message using subscriber node with a history depth of 10 
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_received, 10)

        # receive initial guess from RViz using subscriber node with a history depth of 10 
        self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.initialpose_received, 10)

        # publish latest corrected robot pose using publisher node with depth of 10
        self.pose_pub = self.create_publisher(PoseStamped, "icp_pose", 10)

        # publish raw odometry path then corrected path 
        self.odom_path_pub = self.create_publisher(Path, "/odom_path", 10)
        self.icp_path_pub = self.create_publisher(Path, "/icp_path", 10)

        # for every scan processed, append a pose 
        self.odom_path = Path()
        self.icp_path = Path()
        self.odom_paath.header.frame_id = self.odom_frame
        self.icp_path.header.frame_id = self.map_frame

        self.get_logger().info("starting ICP localizer!")

    def odom_received(self, msg):
        self.latest_odom = msg

    def initialpose_received(self, msg):

    def scan_received(self,msg):





