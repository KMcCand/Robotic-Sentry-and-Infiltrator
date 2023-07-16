#!/usr/bin/env python3
#
#   mapping.py
#
#   Mapping Node. This keeps tracks of the map to odom frame transform. 
#                      Subscribes to /scanfixed. Publishes /uncertainty_map.
#   
#   Node: /mapping
#   Publish:
#           /uncertainty_map            nav_msgs/OccupancyGrid
#   Subscribe:
#           /scanfixed                  Scan

import rclpy
import traceback

import time

import tf2_ros
from std_msgs.msg import Bool
from tf2_ros.buffer             import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time                 import Time, Duration
from rclpy.qos                  import QoSProfile, DurabilityPolicy

from rclpy.executors            import MultiThreadedExecutor
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup

from math import pi, sin, cos

from rclpy.node         import Node
from tf2_ros            import TransformBroadcaster
from geometry_msgs.msg  import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg  import TransformStamped
from nav_msgs.msg       import OccupancyGrid, Odometry
from sensor_msgs.msg    import LaserScan
import numpy as np
from olaf169.planartransform import PlanarTransform

from rcl_interfaces.msg          import ParameterDescriptor, ParameterType


LOAD_MAP = True
LOCCUPIED = 20
LFREE = -3

#
#   Mapping Node Class
#
class MappingNode(Node):
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

        # Class Variables
        self.started = False
        self.uncertainty_map = None
        self.wallptmap = None
        self.robot_pose = None
        self.map2grid = None
        self.map_built = False

        if self.prefix == 'robot1/':
            # Create a subscriber for the map data.  Note this topic uses
            # a quality of service with durability TRANSIENT_LOCAL
            # allowing new subscribers to get the last sent message.
            quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
            self.pubuncertaintymap = self.create_publisher(OccupancyGrid, 'uncertainty_map', quality)

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

        self.subinitial = self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.cb_initial, 1, callback_group=fastgroup)

        self.sublidar = self.create_subscription(
            LaserScan, 'scanfixed', self.cb_lidarmsg, 1, callback_group=slowgroup)

        self.subodom = self.create_subscription(
            Odometry, 'odom', self.cb_odommsg, 10, callback_group=fastgroup)

        self.substartrobot = self.create_subscription(Bool, '/start_robot', self.cb_startrobot, 1)
        ################################################################################

        # Report and return.
        self.get_logger().info("Mapping running")

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
        # self.uncertainty_map = np.clip((np.reshape(mapmsg.data, (height, width)) + 0.5), 0, 1)
        self.uncertainty_map = np.zeros((height, width)) + 50
        self.mapMetaData = mapmsg.info
        actualMap = np.reshape(mapmsg.data, (height, width))

        for v in range(height):
            for u in range(width):
                if actualMap[v,u] > 0.5: # TODO CHECK PLS @ALL
                    self.uncertainty_map[v, u] = 100
                    # Only counts as a wall point if not all of the adjacent points are wall points.

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
            
        if self.wallptmap is None:
            self.get_logger().info("\n\n\nMapping: THE CLOSEST POINTS MAP IS BROKEN\n\n\n")
        else:
            self.get_logger().info("\n\n\nMapping: THE CLOSEST POINTS MAP IS READY FOR USE")
            self.get_logger().info(f"Example map points: (0,0): {self.wallptmap[0,0]} (150,150): {self.wallptmap[150,150]}\n\n\n")        
        self.map_built = True

    def cb_odommsg(self, msg):
        # Get stuff from odom message
        timestamp = msg.header.stamp
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        self.odom_to_base = PlanarTransform(x, y, z, w)

    # Get current robot position in u, v
    def get_robot_position_u_v(self):
        planartrans = self.map2grid.inv() * self.map_to_odom * self.odom_to_base
        x, y = planartrans.inParent(0, 0)
        res = 0.0254
        (u, v) = round(x / res), round(y / res) # grid frame

        return (u, v)
    
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

    def cb_startrobot(self, msg):
        self.started = True
        
    def cb_lidarmsg(self, msg): 
        # Grab the initial and final transforms.  We again give 0.1s
        # in case the latest data has not quite arrived.

        # Get odom to robot lidar
        parentframe = f'{self.prefix}odom'        # Set to whatever is appropriate
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
        
        # Get map to odom
        parentframe =  'map'      # Set to whatever is appropriate
        childframe  = f'{self.prefix}odom'
        try:
            t0 = Time(seconds=0) # 0 gets the latest
            self.map_to_odom = PlanarTransform.fromTransform(self.tfBuffer.lookup_transform(
                parentframe, childframe, t0, timeout=Duration(seconds=0.2)).transform)

        except tf2_ros.TransformException as ex:
            self.get_logger().warn("Unable to get TF '%s' to '%s'" %
                                   (parentframe, childframe) +
                                   "  -- exiting scan callback")
            self.get_logger().warn("Exception: %s" % str(ex))
            return

        # Initialize constants
        res = 0.0254

        if self.map2grid is None:
            return 

        robot_position = self.get_robot_position_u_v()

        rmin     = msg.range_min - 3 # Give it some extra range
        rmax     = msg.range_max 

        # Initialize Iterators
        alpha  = msg.angle_min
        dalpha = msg.angle_increment

        TF  = self.map2grid.inv() * self.map_to_odom * odom2scan0
        dTF = (odom2scan0.inv() * odom2scanN) ** (1 / N)
        try:
            for dist in msg.ranges:
                if not np.isnan(dist) and dist > rmin:
                    dist = min(dist, rmax)
                    # Shift into the (time-varying) parent frame then get (u, v) coords in grid frame.
                    (x,y) = TF.inParent(dist*cos(alpha), dist*sin(alpha)) # grid frame
                    (u, v) = round(x / res), round(y / res) # grid frame

                    if 0 <= u and u < 300 and 0 <= v and v < 300:
                        # If robot1, update the map using this scanned point
                        if self.prefix == 'robot1/' and self.started:
                            self.process_mapped_point(robot_position, (u, v))

                # Advance. Increment the angle, concatenate the TF.
                alpha += dalpha
                TF    *= dTF

        except Exception as e:
            self.get_logger().info(f"Exception During Mapping: {e}")

        # Also publish the updated uncertainty map
        self.publish_mapped_environment()

    # Takes in a point that the lidar scanned as (u, v). Updates the uncertainty map accordingly.
    def process_mapped_point(self, robot_position, point):
        uf, vf = point
        u0, v0 = robot_position

        # change to spread = [(0, 0)] to restore without spread
        # spread = [(0, 1), (1, 0), (1, 1), (0, 0), (-1, 0), (0, -1), (-1, -1), (-1, 1), (1, -1)]
        spread = [(0, 1), (1, 0), (0, 0), (-1, 0), (0, -1)]

        for dv, du in spread:
            if (vf+dv < self.uncertainty_map.shape[0] and vf+dv >= 0) and (uf+du < self.uncertainty_map.shape[0] and uf+du >=0):
                self.uncertainty_map[vf+dv, uf+du] += LOCCUPIED

                if self.uncertainty_map[vf+dv, uf+du] > 100:
                    self.uncertainty_map[vf+dv, uf+du] = 100
            
        self.bresenhams(u0, v0, uf, vf)
    
    def bresenhams(self, u1, v1, u2, v2):
        du = -abs(u2 - u1)
        dv = abs(v2 - v1)

        su = 1 if u1 < u2 else -1
        sv = 1 if v1 < v2 else -1

        err = du + dv
        uk, vk = u1, v1 # starting point

        while True:
            self.uncertainty_map[vk, uk] += LFREE
            if self.uncertainty_map[vk, uk] < 0:
                self.uncertainty_map[vk, uk] = 0
            err2 = 2 * err

            if (uk, vk) == (u2, v2):
                break
            
            if err2 >= du:
                if vk == v2:
                    break
                err = err + du
                vk = vk + sv
            
            if err2 <= dv:
                if uk == u2:
                    break
                err = err + dv
                uk = uk + su
        return
     
    def publish_mapped_environment(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"

        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        
        msg.info = self.mapMetaData
        msg.info.map_load_time = now.to_msg()

        # Change uncertainty map to probabilities here.
        
        msg.data = (self.uncertainty_map).astype(int).flatten().tolist()
        # self.get_logger().info(f"Time of processing: {time.time()}")

        self.pubuncertaintymap.publish(msg)

        
    def find_points_inside(self, x1, y1, x2, y2):
        pts = []
        dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** (1/2)
        for d in np.linspace(0, 1, int(2 * dist)):
            x, y = x2 * d + x1 * (1-d),  y2 * d + y1 * (1-d)
            xi, yi = int(x), int(y)
            if (xi, yi) not in pts:
                pts.append((xi, yi))
            
        return pts 

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = MappingNode('mapping')

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
