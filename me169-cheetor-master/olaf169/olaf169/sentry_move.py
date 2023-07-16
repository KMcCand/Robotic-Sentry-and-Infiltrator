#!/usr/bin/env python3
#
#   sentry_move.py
#   Move the sentry along 4 waypoints, splining with respect to time
#   (An alternative to drive_auto, so no planning for now)
#   
#   Node: /sentrymove
#   Publish:
#           cmd_vel                    geometry_msgs/Twist
#   Subscribe:
#           /odom                       Odometry
#           /scanfixed                  Scan

import rclpy
import traceback


from std_msgs.msg import Bool
from rclpy.node         import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from visualization_msgs.msg import Marker
from rcl_interfaces.msg          import ParameterDescriptor, ParameterType
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup
from rclpy.executors            import MultiThreadedExecutor

#
#   Sentry Node Class
#
class SentryMove(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        stringtype       = ParameterType.PARAMETER_STRING
        stringdescriptor = ParameterDescriptor(type=stringtype)

        self.declare_parameter('frame_prefix', descriptor=stringdescriptor, value = '')

        param       = self.get_parameter('frame_prefix')
        self.prefix = param.get_parameter_value().string_value

        # Class variables
        self.waypoints = [(-0.6, 1.0), (-0.6, 2.0), (-2.0, 2.0), (-2.0, 1.0)]
        self.i = 0
        
        # Publishers
        self.pub_waypoint = self.create_publisher(PoseStamped, 'waypoint', 1) # Waypoints
        self.pub_markers = self.create_publisher(Marker, "markers", 10)       # Markers for Waypoints

        fastgroup = MutuallyExclusiveCallbackGroup()
        slowgroup = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.sub_nextwaypoint = self.create_subscription(Bool, 'next_waypoint', self.cb_nextwaypoint, 1, callback_group=fastgroup)
        self.marker_sender = self.create_timer(0.2,  self.cb_marker_sender, callback_group=slowgroup)
        self.sub_startrobot = self.create_subscription(Bool, '/start_robot', self.cb_nextwaypoint, 1, callback_group=fastgroup)

        # Report and return.
        self.get_logger().info("Sentry Mover running")

    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()
    
    def cb_marker_sender(self):
        # Places 4 Markers
        markermsg = Marker()
        markermsg.header.frame_id = "map"

        now = self.get_clock().now()
        markermsg.header.stamp = now.to_msg()

        markermsg.ns = "sentry_waypoints"
        markermsg.id = 0
        markermsg.type = Marker.POINTS
        markermsg.action = Marker.ADD # Should be same as Marker.MODIFY
        markermsg.pose = Pose()
        markermsg.scale = Vector3(x=0.07, y=0.07, z=0.07)
        markermsg.color.a = 1.0

        # Dark green markers for robot2
        markermsg.color.r = 0.0
        markermsg.color.g = 1.0
        markermsg.color.b = 0.0
        markermsg.points = []

        for pt in self.waypoints:
            markermsg.points.append(Point(x=pt[0], y=pt[1], z=0.0))

        self.pub_markers.publish(markermsg)

    # When drive_auto sends a Bool indicating it has reached its current waypoint,
    # send it a new waypoint if we have one
    def cb_nextwaypoint(self, msg):
        if self.waypoints:
            current_waypoint = self.waypoints[self.i % len(self.waypoints)]
            self.i += 1

        # Publish the waypoint
        waypoint_msg = PoseStamped()
        waypoint_msg.pose.position.x = current_waypoint[0]
        waypoint_msg.pose.position.y = current_waypoint[1]
        waypoint_msg.pose.position.z = 0.0
        waypoint_msg.header.frame_id = 'map'
        self.pub_waypoint.publish(waypoint_msg)
        self.get_logger().info("PUBLISHING WAYPOINT")

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = SentryMove('sentrymove')

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
