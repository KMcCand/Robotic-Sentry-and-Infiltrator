#!/usr/bin/env python3
#
#   check_complete.py
#
#   Checking Complete Node. This node
#   (a) Subscribes to the 2 goal poses 
#   (b) Checks if either if sentry caught the infiltrator or
#       if the infiltrator reached the goal
#   
#   Node: /check_complete
#   Publish:
#           /stop_sentry                std_msgs/Bool
#           /stop_infiltrator           std_msgs/Bool
#   Subscribe:
#           /robot1/pose                  geometry_msgs/PoseStamped
#           /robot2/pose                  geometry_msgs/PoseStamped

import rclpy
from std_msgs.msg import ColorRGBA

from rclpy.qos                  import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool

from math import sin, cos, atan2

from rclpy.node         import Node
from nav_msgs.msg       import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3

import numpy as np
import random 
import os
from olaf169.planartransform import PlanarTransform
from visualization_msgs.msg import Marker
from rcl_interfaces.msg          import ParameterDescriptor, ParameterType
from std_msgs.msg import Bool

# Global constants
CONNECTS_TO_GRANULARITY = 6 # Number of grids to skip when checking connectsTo
GRIDS_TO_WALL = 6 # Robot half width in grids
RES = 0.0254
LOAD_MAP = True

#
#   Planner Node Class
#
class CheckCompleteNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        stringtype       = ParameterType.PARAMETER_STRING
        stringdescriptor = ParameterDescriptor(type=stringtype)

        self.declare_parameter('frame_prefix', descriptor=stringdescriptor, value = '')

        param       = self.get_parameter('frame_prefix')
        self.prefix = param.get_parameter_value().string_value

        self.robot1_pose = None
        self.robot2_pose = None

        # Centroid of [(2.0, -0.5), (0.0, -0.3), (0.0, 0.2), (1.5, 1.0), (2.5, 0.8)]
        self.robot1_goal = np.array([1.2, 0.24])
        self.reached_goal = False
        self.caught = False

        # Class variables
        self.started = False
        self.current_pose = None
        
        self.subrobot1pose = self.create_subscription(PoseStamped, '/robot1/pose', self.cb_robot1pose, 10)
        self.subrobot2pose = self.create_subscription(PoseStamped, '/robot2/pose', self.cb_robot2pose, 10)

        self.pub_stopsentry = self.create_publisher(Bool, '/stop_sentry', 3)
        self.pub_stopinfiltrator = self.create_publisher(Bool, '/stop_infiltrator', 3)

        rate = 10
        self.timer = self.create_timer(1/rate, self.check_game_status)

        # Report and return. 
        self.get_logger().info("Planner running")

    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()

    def check_game_status(self):

        if (self.robot1_pose is not None) and (self.robot2_pose is not None):

            d_interrobot = np.linalg.norm(self.robot1_pose - self.robot2_pose)
            d_togoal = np.linalg.norm(self.robot1_pose - self.robot1_goal)
            
            # what if both happen at the exact same time?
            if self.reached_goal:
                self.get_logger().info('Infiltrator won!')
                return None
            elif self.caught:
                self.get_logger().info('Infiltrator was caught :(')
                return None
            
            if d_togoal < 0.2 and not self.caught:
                self.reached_goal = True
                self.get_logger().info('------ STOPPING ROBOTS ------')
                msg = Bool()
                msg.data = True
                self.pub_stopinfiltrator.publish(msg)
                self.pub_stopsentry.publish(msg)

            if d_interrobot < 0.15 and not self.reached_goal:
                self.caught = True
                self.get_logger().info('------ STOPPING ROBOTS ------')
                msg = Bool()
                msg.data = True
                self.pub_stopsentry.publish(msg)
                self.pub_stopinfiltrator.publish(msg)
            
            self.get_logger().info(f"Interrobot distance: {d_interrobot}")

    # Keep track of the robots current position
    def cb_robot1pose(self, msg):
        assert msg.header.frame_id == 'map', "Planner: Robot 1 Pose essage not in map frame!!!!"
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        self.robot1_pose = np.array([x, y])

    def cb_robot2pose(self, msg):
        assert msg.header.frame_id == 'map', "Planner: Robot 2 Pose Message not in map frame!!!!"
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        self.robot2_pose = np.array([x, y])


    def cb_startrobot(self, msg):
        self.started = True

        msg = Bool()
        msg.data = True
        self.pub_get_global_waypoint.publish(msg)
        self.get_logger().info("Requesting for global waypoint")

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = CheckCompleteNode('check_complete')

    # Spin the node until interrupted.  To support multiple callback
    # groups, use a multithreaded executor.  The callbacks in each
    # group will thus run in their respective thread.

    # Spin the node until interrupted.
    try:
        rclpy.spin(node)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
