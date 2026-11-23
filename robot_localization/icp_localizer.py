#!/usr/bin/env python3

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

import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time 
from rclpy.qos import qos_profile_sensor_data 

from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry, Path

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped, Pose)

from tf2_ros import TransformException

from helper_functions import TFHelper, stamped_transform_to_pose
from map_utils import occupancy_grid_msg_to_points, scan_to_points, pose_to_xytheta, xytheta_to_pose
from icp import icp, make_transform, transform_to_xytheta, transform_points 
from kd_tree import KDTree

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
        self.initialized = False 

        # latest odometry message 
        self.latest_odom = None

        self.transform_helper = TFHelper(self)

        self.get_logger().info("waiting on map_server...")
        self.map_client = self.create_client(GetMap, 'map_server/map')

        # request occupancy grid 
        request = GetMap.Request()

        # can only proceed to build kd tree if map has arrived 
        while not self.map_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("awaiting map service.")

        future = self.map_client.call_async(request)

        # give the system time to receive response 
        rclpy.spin_until_future_complete(self,future)
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
        self.odom_path.header.frame_id = self.odom_frame
        self.icp_path.header.frame_id = self.map_frame

        self.get_logger().info("starting ICP localizer!")

    def odom_received(self, msg):
        self.latest_odom = msg

    def initialpose_received(self, msg):
        self.get_logger().info("received initial pose estimate")

        # initial pose must be within the map frame 
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().info("this pose is not within the map frame")
            return 

        # establish initial map to odometry correction
        try: 
            odom_pose, delta_t = (
                self.transform_helper.get_matching_odom_pose(self.odom_frame, 
                                                             self.base_frame, 
                                                             msg.header.stamp)
            )
        except TransformException:
            self.get_logger().info("odometry transform failed")
            return 

        if odom_pose is None: 
            self.get_logger().info("try initial pose again, no odometry " \
            "transform available")
            return 

        map_pose = msg.pose.pose

        # convert ROS poses 
        map_xytheta = pose_to_xytheta(map_pose)
        odom_xytheta = pose_to_xytheta(odom_pose)

        # now into transformation matrices , unpack first 
        T_map_base = make_transform(*map_xytheta)
        T_odom_base = make_transform(*odom_xytheta)

        # calculate initial map and correct odometry map 
        self.T_map_odom = (T_map_base @ np.linalg.inv(T_odom_base))
        self.initialized = True
        self.get_logger().info("initial pose accepted.")
        

    def scan_received(self,msg):
        if not self.initialized:
            self.get_logger().info("initial pose has not been accepted yet.")
            return 

        # establish where the robot is prior to applying icp
        try: 
            odom_pose, delta_t = (self.transform_helper.get_matching_odom_pose(
                self.odom_frame, self.base_frame, 
                msg.header.stamp
            ))

        except TransformException: 
            self.get_logger().info("TF look up failed.")
            return 

        if odom_pose is None: 
            self.get_logger().info("no matching odometry transform for this scan")
            return 

        odom_xytheta = pose_to_xytheta(odom_pose)
        T_odom_base = make_transform(*odom_xytheta)

        # combine previous map with current odometry pose 
        initial_guess = self.T_map_odom @ T_odom_base

        # convert laser scan to points 

        scan_points = scan_to_points(msg.ranges, 
                                     msg.angle_min, msg.angle_increment, 
                                     msg.range_min, msg.range_max)

        # if LiDAR frame if offset relative to base footprint, we're 
        # in wrong coordinate frame
        # we don't know if LiDAR mounting is identical to base footprint
        # orientation or position 

        try: 
            laser_transform = self.transform_helper.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id, 
                Time.from_msg(msg.header.stamp)
            )

        except TransformException:
            self.get_logger().info("LiDAR to base frame transformation failed.") 
            return

        # convert ROS transform to a pose 
        laser_pose = stamped_transform_to_pose(laser_transform)

        # convert pose to transformation matrix 
        T_base_laser = make_transform(*pose_to_xytheta(laser_pose))

        # apply transformation to LiDAR points into base frame
        scan_points = transform_points(scan_points, T_base_laser)

        # skip if not enough valid points
        if len(scan_points) < self.min_inliers: 
            self.get_logger().info("skipping current scan. not enough LiDAR points.")
            return 

        start_time = time.perf_counter()

        # actually run icp 
        try: 
            T_map_base_icp, info = icp(scan_points, self.map_tree, self.map_points, 
                                       initial_guess,
                                       self.max_iters, self.tol, 
                                       self.max_corr_dist)

        except Exception: 
            self.get_logger().error("icp failed :(")
            return 

        # keep track of how long icp took to run 
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        self.get_logger().info("ICP time: %.2f ms \n Iterations: %d \n Inliers: %d" \
        "\n Error: %s \n Converged: %s" % (
            elapsed_ms, info["iterations"], info["num_inliers"],
            str(info["mean_error"]), str(info["converged"])
        ))

        # preemptively use odometry-based guess if icp fails 
        T_map_base = initial_guess

        # only accept icp if converged and enough valid points 
        if (info["converged"] and info["num_inliers"] >= self.min_inliers):

            # calculate new map with odom correction 
            self.T_map_odom = (T_map_base_icp @ np.linalg.inv(T_odom_base))

            # replace map base with ICP result. we now have an accepted pose 
            T_map_base = T_map_base_icp
        else: 
            self.get_logger().info("icp result not valid, keeping previous map")

        # convert new pose into xytheta 
        x, y, theta = transform_to_xytheta(T_map_base)

        # convert xytheta to ROS pose and publish 
        corrected_pose = xytheta_to_pose(x,y,theta)
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.map_frame
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.pose = corrected_pose
        self.pose_pub.publish(pose_msg)

        # update odometry path with odom pose and publish 
        odom_path_pose = PoseStamped() 
        odom_path_pose.header.frame_id = self.odom_frame
        odom_path_pose.header.stamp = msg.header.stamp 
        odom_path_pose.pose = odom_pose

        self.odom_path.header.stamp = msg.header.stamp
        self.odom_path.poses.append(odom_path_pose)
        self.odom_path_pub.publish(self.odom_path)

        # update icp path with corrected pose and publish 
        icp_path_pose = PoseStamped() 
        icp_path_pose.header.frame_id = self.map_frame
        icp_path_pose.header.stamp = msg.header.stamp
        icp_path_pose.pose = corrected_pose

        self.icp_path.header.stamp = msg.header.stamp 
        self.icp_path.poses.append(icp_path_pose)
        self.icp_path_pub.publish(self.icp_path)

        # broadcast map 
        # first, apply odometry transform 
        self.transform_helper.fix_map_to_odom_transform(corrected_pose, odom_pose)

        # convert scan timestamp to ROS time obj 
        timestamp = Time.from_msg(msg.header.stamp)

        # actually broadcast transform 
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, 
                                                              self.odom_frame, 
                                                              timestamp)

def main(args = None):
    # initialize ros
    rclpy.init(args = args)

    node = ICPLocalizer() 

    try: 
        # keep node running 
        rclpy.spin(node)

    # stop with keyboard input 
    except KeyboardInterrupt:
        pass 

    finally: 
        node.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()






