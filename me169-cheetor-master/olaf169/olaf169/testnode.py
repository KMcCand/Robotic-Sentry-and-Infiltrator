#!/usr/bin/env python3
#
#   odometry_skeleton.py
#
#   THIS IS A SKELETON ONLY.  PLEASE COPY/RENAME AND THEN EDIT!
#
#   Odometry node.  This
#   (a) converts both a body velocity command to wheel velocity commands.
#   (b) estimates the body velocity and pose from the wheel motions
#       and the gyroscope.
#
#   Node:       /odometry
#   Publish:    /odom                   geometry_msgs/TransJointState
#               TF odom -> base         geometry_msgs/TransformStamped
#               /wheel_command          sensor_msgs/JointState
#   Subscribe:  /cmd_vel                geometry_msgs/Twist
#               /wheel_state            sensor_msgs/JointState
#
import rclpy
import traceback

import tf2_ros
from tf2_ros                    import TransformException
from tf2_ros.buffer             import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time                 import Time, Duration

from rclpy.executors            import MultiThreadedExecutor
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup

import time
from scipy.spatial.transform import Rotation
from math import pi, sin, cos

from rclpy.node         import Node
from tf2_ros            import TransformBroadcaster
from geometry_msgs.msg  import Point, Quaternion, Twist, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg  import TransformStamped, Vector3
from nav_msgs.msg       import Odometry
from sensor_msgs.msg    import JointState, LaserScan
import numpy as np
from olaf169.planartransform import PlanarTransform


#
#   TestNode Node Class
#
class TestNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Initialize the transform broadcaster
        self.tfbroadcaster = TransformBroadcaster(self)

        # The big bad map
        self.map_to_odom = PlanarTransform.unity()
        self.odom_to_base = PlanarTransform.unity()

        # Create publishers
        self.pubpose = self.create_publisher(PoseStamped, '/pose', 10)

        # First create a TF2 listener.  This implicitly fills a local
        # buffer, so we can quickly retrieve the transform information
        # we need.  The buffer is filled via incoming TF message
        # callbacks, so make sure this runs in a seperate thread.
        self.tfBuffer = Buffer()
        tflisten      = TransformListener(self.tfBuffer, self, spin_thread=True)

        # With TF being in a seperate thread, we can already request
        # and wait for a transform.  For example, fetch the URDF-based
        # transform from 'base' to 'lidar'.  Note, the unspecified
        # Time() (being time=0) requests the most recently available
        # data - entirely appropriate for constant transforms.  We
        # give it 5s in case the startup takes a while.
        try:
            tfmsg = self.tfBuffer.lookup_transform(
                'base', 'lidar', Time(), timeout = Duration(seconds=5.0))
        except tf2_ros.TransformException as ex:
            self.get_logger().error("Unable to get TF 'base' to 'lidar'")
            raise

        # Get base_to_lidar as a planar transform
        self.base_to_lidar = PlanarTransform.fromTransform(tfmsg.transform)

        # Use two different callback groups, which will run
        # independently in distinct threads.  Within a group,
        # subsequent callbacks are held until the prior ones complete.
        fastgroup = MutuallyExclusiveCallbackGroup()
        slowgroup = MutuallyExclusiveCallbackGroup()
        # othergroup = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.subodom = self.create_subscription(
            Odometry, '/odom', self.cb_odommsg, 10, callback_group=fastgroup)
        
        self.subinitial = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.cb_initial, 5, callback_group=fastgroup)

        # self.subgoal = self.create_subscription(
        #     PoseStamped, '/goal_pose', self.cb_goalmsg, 3, callback_group=fastgroup)
        
        self.sublidar = self.create_subscription(
            LaserScan, '/scanfixed', self.cb_lidarmsg, 1, callback_group=slowgroup)

        # Report and return.s
        self.get_logger().info("Localizer running")

    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()

    def cb_odommsg(self, msg):
        print("odom")
        # Get stuff from odom message
        timestamp = msg.header.stamp
        # frame_id = msg.header.frame_id 'odom'
        # child_frame_id = msg.child_frame_id 'base'
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        self.odom_to_base = PlanarTransform(x, y, z, w)
        pose_msg = PoseStamped()

        pose_msg.pose = (self.map_to_odom * self.odom_to_base).toPose()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'map'
        # self.pubpose.publish(pose_msg)
        
    def cb_initial(self, msg):
        self.get_logger().info('Service call failed')
        print("WE MADE IT!!!!!")

    def cb_lidarmsg(self, msg):
        print("lidar")
        # Grab the transform: to data's frame and at the data's time
        # (as specified in the message).  We give it 0.1s in case the
        # latest data hasn't quite come in yet.
        parentframe = 'odom'            # Set to whatever is appropriate
        childframe  = msg.header.frame_id
        time        = Time().from_msg(msg.header.stamp)
        try:
            tfmsg = self.tfBuffer.lookup_transform(
                parentframe, childframe, time, timeout=Duration(seconds=0.2))
        except tf2_ros.TransformException as ex:
            self.get_logger().warn("Unable to get TF '%s' to '%s'" %
                                   (parentframe, childframe) +
                                   "  -- exiting scan callback")
            self.get_logger().warn("Exception: %s" % str(ex))
            return

        # For ease of processing, convert into a planar transform
        pt = PlanarTransform.fromTransform(tfmsg.transform)

        # Publish map to odom TF
        transform_msg = TransformStamped()
        transform_msg.transform = self.map_to_odom.toTransform()
        transform_msg.header.stamp = msg.header.stamp
        transform_msg.header.frame_id = 'map'
        transform_msg.child_frame_id = 'odom'
        # self.tfbroadcaster.sendTransform(transform_msg)


#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # # Instantiate the DEMO node.
    # node = TestNode('testnode')

    # # Spin the node until interrupted.
    # try:
    #     rclpy.spin(node)
    # except BaseException as ex:
    #     print("Ending due to exception: %s" % repr(ex))
    #     traceback.print_exc()

    # # Shutdown the node and ROS.
    # node.shutdown()
    # rclpy.shutdown()

    # Instantiate the DEMO node.
    node = TestNode('testnode')

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
