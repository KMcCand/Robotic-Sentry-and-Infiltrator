#!/usr/bin/env python3
#
#   localize.py
#
#   Locolization Node. This keeps tracks of the map to odom frame transform. 
#                      Subscribes to /odom and /scanfixed. Publishes /pose.
#   
#   Node: /localize
#   Publish:
#           /pose                       geometry_msgs/PoseStamped
#   Subscribe:
#           /odom                       Odometry
#           /scanfixed                  Scan
#           /initialpose                geometry_msgs/PoseWithCovarianceStamped

import rclpy
import traceback
import time

import tf2_ros
from tf2_ros.buffer             import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time                 import Time, Duration
from rclpy.qos                  import QoSProfile, DurabilityPolicy

from rclpy.executors            import MultiThreadedExecutor
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup

from math import pi, sin, cos

from rclpy.node         import Node
from tf2_ros            import TransformBroadcaster
from geometry_msgs.msg  import Point, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg  import TransformStamped, Vector3, Pose
from nav_msgs.msg       import Odometry, OccupancyGrid
from sensor_msgs.msg    import LaserScan
import numpy as np
from olaf169.planartransform import PlanarTransform
from visualization_msgs.msg import Marker

from rcl_interfaces.msg          import ParameterDescriptor, ParameterType


LOCALIZE_OUTLIER_THRESHOLD = 10
LOCALIZATION_SPEED = 0.1
LOAD_MAP = True

#
#   Localize Node Class
#
class LocalizeNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Declare the frame_prefix parameter, then get the value.
        # This makes the paremeter adjustable in the launch file.
        stringtype       = ParameterType.PARAMETER_STRING
        stringdescriptor = ParameterDescriptor(type=stringtype)

        self.declare_parameter('frame_prefix', descriptor=stringdescriptor, value ='')

        param       = self.get_parameter('frame_prefix')
        self.prefix = param.get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.tfbroadcaster = TransformBroadcaster(self)

        # Transforms
        self.map_to_odom = PlanarTransform.unity()
        self.odom_to_base = PlanarTransform.unity()
        self.odom_to_lidar = PlanarTransform.unity()

        # Mapping / Localization Variables
        self.uncertainty_map = None
        self.wallptmap = None
        self.map2grid = None
        self.map_built = False

        # Create publishers
        self.pubpose = self.create_publisher(PoseStamped, 'pose', 10)

        # First create a TF2 listener.  This implicitly fills a local
        # buffer, so we can quickly retrieve the transform information
        # we need.  The buffer is filled via incoming TF message
        # callbacks, so make sure this runs in a seperate thread.
        self.tfBuffer = Buffer()
        tflisten      = TransformListener(self.tfBuffer, self, spin_thread=True)

        # Use two different callback groups, which will run
        # independently in distinct threads.  Within a group,
        # subsequent callbacks are held until the prior ones complete.
        fastgroup = MutuallyExclusiveCallbackGroup()
        slowgroup = MutuallyExclusiveCallbackGroup()
        othergroup = MutuallyExclusiveCallbackGroup()

        # Subscribers
        ################################################################################
        # Create a subscriber for the map data.  Note this topic uses
        # a quality of service with durability TRANSIENT_LOCAL
        # allowing new subscribers to get the last sent message.
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.submap = self.create_subscription(
            OccupancyGrid, '/map', self.cb_mapmsg, quality, callback_group=othergroup)

        self.subodom = self.create_subscription(
            Odometry, 'odom', self.cb_odommsg, 10, callback_group=fastgroup)
        
        self.subinitial = self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.cb_initial, 1, callback_group=fastgroup)

        self.sublidar = self.create_subscription(
            LaserScan, 'scanfixed', self.cb_lidarmsg, 1, callback_group=slowgroup)
        ################################################################################

        self.pubmarker = self.create_publisher(Marker, "lidar_markers", 10) 

        # Report and return.
        self.get_logger().info("Localizer running")

    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node. 
        self.destroy_node()

    def cb_mapmsg(self, mapmsg):
        # Grab the map info.
        width    = mapmsg.info.width
        height   = mapmsg.info.height
        res      = mapmsg.info.resolution
        self.map2grid = PlanarTransform.fromPose(mapmsg.info.origin)
        self.uncertainty_map = np.reshape(mapmsg.data, (height, width))
        self.mapMetaData = mapmsg.info

        # Report.
        self.get_logger().info("Received the map: %dx%d" % (width, height))
        self.get_logger().info("  resolution %4.3fm -> %5.3fm x %5.3fm" %
                               (res, res * width, res * height))
        self.get_logger().info("  grid origin at %s" % self.map2grid)

        self.get_logger().info(f"FRAME OF MAP {mapmsg.header}")

        if LOAD_MAP:
            try:
                # Load map from numpy
                self.wallptmap = np.load('closest_walls_map_no_obstacles.npy')
            except Exception as e:
                self.get_logger().info(f"Exception in Loading Map my doods: {e}")
        else:
            # CODE TO MAKE THE CLOSEST WALL POINTS MAP
            # Get a list of all wall points
            WALLTHRESHOLD = 0.5
            wallpts = np.zeros((0,2), dtype=np.int)
            for v in range(height):
                for u in range(width):
                    if self.uncertainty_map[v,u] > WALLTHRESHOLD:
                        # Only counts as a wall point if not all of the adjacent points are wall points.
                        adjacent = self.uncertainty_map[max(0,v-1):min(height, v+2), max(0,u-1):min(width, u+2)]
                        if not np.all(adjacent > WALLTHRESHOLD):
                            wallpts = np.vstack([wallpts, np.array([u,v])])
                
            # Returns the nearest wall point for coordinates u, v
            def nearestwallpt(u,v):
                return wallpts[np.argmin(np.sum((np.array([u,v]) - wallpts)**2, axis=1))]

            # Find the nearest wall point for every point in the map and store in self.wallptmap
            self.wallptmap = np.zeros((height,width,2))
            for v in range(height):
                for u in range(width):
                    self.wallptmap[v,u] = nearestwallpt(u,v)

            # Save self.wallptmap to save future computation
            np.save('closest_walls_map_no_obstacles', self.wallptmap)
            self.get_logger().info("\n\n\nLocalize: THE CLOSEST POINTS MAP HAS BEEN BUILT\n\n\n")

        # If robot1, make the uncertainty map probabilities [0, 1]
        if self.prefix == 'robot1/':
            for v in range(height):
                for u in range(width):
                    self.uncertainty_map[v,u] /= 100
            
        if self.wallptmap is None:
            self.get_logger().info("\n\n\nLocalize: THE CLOSEST POINTS MAP IS BROKEN\n\n\n")
        else:
            self.get_logger().info("\n\n\nLocalize: THE CLOSEST POINTS MAP IS READY FOR USE")
            self.get_logger().info(f"Example map points: (0,0): {self.wallptmap[0,0]} (150,150): {self.wallptmap[150,150]}\n\n\n")        
        self.map_built = True

    # Get current robot position in u, v
    def get_robot_position_u_v(self):
        planartrans = self.map2grid.inv() * self.map_to_odom * self.odom_to_base
        x, y = planartrans.inParent(0, 0)
        res = 0.0254
        (u, v) = round(x / res), round(y / res) # grid frame

        return (u, v)
        
    def cb_odommsg(self, msg):
        # Get stuff from odom message
        timestamp = msg.header.stamp
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        self.odom_to_base = PlanarTransform(x, y, z, w)
        pose_msg = PoseStamped()

        pose_msg.pose = (self.map_to_odom * self.odom_to_base).toPose()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'map'

        self.pubpose.publish(pose_msg)
    
    def cb_initial(self, msg):
        assert msg.header.frame_id == 'map', "Localize: Message not in map frame!!!!"

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        initial_pose = PlanarTransform(x, y, z, w) # Transform from map to robot base right now
        self.map_to_odom = initial_pose * self.odom_to_base.inv()

        # Publish map to odom TF
        transform_msg = TransformStamped()
        transform_msg.transform = self.map_to_odom.toTransform()
        transform_msg.header.stamp = msg.header.stamp
        transform_msg.header.frame_id = 'map'
        transform_msg.child_frame_id = f'{self.prefix}odom'
        self.tfbroadcaster.sendTransform(transform_msg)
        
    def cb_lidarmsg(self, msg): 
        if not self.map_built:
            # Publish map to odom TF without localization if the map is not ready
            transform_msg = TransformStamped()
            transform_msg.transform = self.map_to_odom.toTransform()
            transform_msg.header.stamp = msg.header.stamp
            transform_msg.header.frame_id = 'map'
            transform_msg.child_frame_id = f'{self.prefix}odom'
            self.tfbroadcaster.sendTransform(transform_msg)
            return

        # Grab the initial and final transforms.  We again give 0.1s
        # in case the latest data has not quite arrived.
        parentframe = f'{self.prefix}odom'            # Set to whatever is appropriate
        childframe  = f'{msg.header.frame_id}'
        try:
            N = len(msg.ranges)

            t0 = Time().from_msg(msg.header.stamp)
            tN = t0 + Duration(seconds=(N*msg.time_increment))

            tfmsg0 = self.tfBuffer.lookup_transform(
                parentframe, childframe, t0, timeout=Duration(seconds=0.2))
            tfmsgN = self.tfBuffer.lookup_transform(
                parentframe, childframe, tN, timeout=Duration(seconds=0.2))

            # Set up the planar transforms.
            odom2scan0 = PlanarTransform.fromTransform(tfmsg0.transform)
            odom2scanN = PlanarTransform.fromTransform(tfmsgN.transform)

        except tf2_ros.TransformException as ex:
            self.get_logger().warn("Unable to get TF '%s' to '%s'" %
                                   (parentframe, childframe) +
                                   "  -- exiting scan callback")
            self.get_logger().warn("Exception: %s" % str(ex))
            return

        # Initialize variables
        a = np.zeros((len(msg.ranges), 1))
        J = np.zeros((len(msg.ranges), 3))
        weights = np.zeros(len(msg.ranges))

        # Initialize constants
        res = 0.0254

        rmin     = msg.range_min - 3 # Give it some extra range
        rmax     = msg.range_max 

        # Initialize Iterators
        alpha  = msg.angle_min
        dalpha = msg.angle_increment

        TF  = self.map2grid.inv() * self.map_to_odom * odom2scan0
        dTF = (odom2scan0.inv() * odom2scanN) ** (1/N)
        try:
            for i, dist in enumerate(msg.ranges):
                if (not np.isnan(dist)) and dist > rmin:
                    dist = min(dist, rmax)
                    # Shift into the (time-varying) parent frame then get (u, v) coords in grid frame.
                    (x,y) = TF.inParent(dist*cos(alpha), dist*sin(alpha)) # grid frame
                    (u, v) = round(x / res), round(y / res) # grid frame

                    if 0 <= u and u < 300 and 0 <= v and v < 300:
                        p = self.wallptmap[v,u]
                        r = np.array([float(u), float(v)])
                        norm = np.linalg.norm(p - r)

                        if norm != 0.0 and norm < LOCALIZE_OUTLIER_THRESHOLD:
                            # Update weights
                            # weights[i] = 1 if (dist > 0.2 and dist < 3.0) else 0.5
                            weights[i] = 1.0 / dist  # Change to more aggresive weights

                            # Update matrices
                            a[i][0] = norm
                            J[i][0] = (p[0] - r[0]) / norm
                            J[i][1] = (p[1] - r[1]) / norm
                            J[i][2] = (r[0] * p[1] - r[1] * p[0]) / norm
                        
                # Advance. Increment the angle, concatenate the TF.
                alpha += dalpha
                TF    *= dTF

        except Exception as e:
            self.get_logger().info(f"Exception During Localization: {e}")
           
        try: 
            if weights.any():
                delta = np.linalg.inv(J.T @ np.diag(weights) @ J) @ (J.T @ np.diag(weights) @ a)

                delta_t = PlanarTransform.basic(delta[0][0] * res, delta[1][0] * res, delta[2][0])
                self.map_to_odom = self.map2grid * (delta_t ** LOCALIZATION_SPEED) * self.map2grid.inv() * self.map_to_odom

        except Exception as e: 
            self.get_logger().info(f"Exception {e}")
            self.get_logger().info(f"Cannot Localize, No points")

        # Publish map to odom TF
        transform_msg = TransformStamped()
        transform_msg.transform = self.map_to_odom.toTransform()
        transform_msg.header.stamp = msg.header.stamp
        transform_msg.header.frame_id = 'map'
        transform_msg.child_frame_id = f'{self.prefix}odom'
        self.tfbroadcaster.sendTransform(transform_msg)

    def publish_all_markers(self, points, stamp):
        # Publish a marker message to show the waypoint trail
        markermsg = Marker()
        markermsg.header.frame_id = "map"
        markermsg.header.stamp = stamp
        markermsg.id = 0
        markermsg.type = Marker.POINTS
        markermsg.action = Marker.ADD # Should be same as Marker.MODIFY
        markermsg.pose = Pose()
        markermsg.scale = Vector3(x=0.04, y=0.04, z=0.04)
        markermsg.color.a = 1.0
        markermsg.color.r = 1.0
        markermsg.color.g = 0.0
        markermsg.color.b = 0.0
        markermsg.points = []
        for pt in points:
            markermsg.points.append(Point(x=pt[0], y=pt[1], z=0.0))
        self.pubmarker.publish(markermsg)

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = LocalizeNode('localize')

    # Spin the node until interrupted.  To support multiple callback
    # groups, use a multithreaded executor.  The callbacks in each
    # group will thus run in their respective thread.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
