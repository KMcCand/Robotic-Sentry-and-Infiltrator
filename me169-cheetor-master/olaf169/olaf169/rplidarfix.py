#!/usr/bin/env python3
#
#   rplidarfix.py
#
#   Process the RPLidar scan from /scan to /scanfixed.  This addresses
#   two bugs in the plain RPLidar scan:
#
#     (a) The plain scan presents the samples ordered from angles
#         (-pi) to (pi). But the actual scanner rotates clockwise (in
#         opposite direction), so the temporal order is backwards.
#         This is inconsistent with the time increment parameter.
#         FIX: Reverse the order to match the actual temporal
#         sampling.  This means the time increment parameter works.
#
#     (b) The timestamp should be the time of the first sample.  But
#         the stamp appears to be the time when the software reordered
#         the samples and generated the ROS message.  FIX: Offset the
#         time stamp.  This shifts by the scan time (being the time
#         between scans) plus a fixed delay incurred by the RPLidar
#         software (defaulting to 0.033s)
#
#   To launch, use:
#
#       node_lidarfix = Node(
#           name       = 'lidarfix',
#           package    = 'shared169',
#           executable = 'rplidarfix',
#           output     = 'screen',
#           parameters = [{'delay': 0.033])
#
#
#   Node:       /rplidarfix
#   Subscribe:  /scan                   sensor_msgs/LaserScan
#   Publish:    /scanfixed              sensor_msgs/LaserScan
#
import rclpy
import traceback

from rclpy.node                 import Node
from rclpy.time                 import Time, Duration
from rcl_interfaces.msg         import ParameterDescriptor, ParameterType
from sensor_msgs.msg            import LaserScan
import os


# Declare the name and default value of the delay parameter: the delay
# between the original RPLidar scan time and the ROS LaserScan msg.
DELAYNAME    = 'delay'
DELAYDEFAULT = 0.033

#
#   RP Lidar Fix Node Class
#
class RPNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        stringtype       = ParameterType.PARAMETER_STRING
        stringdescriptor = ParameterDescriptor(type=stringtype)

        self.declare_parameter('frame_prefix', descriptor=stringdescriptor, value='')

        param       = self.get_parameter('frame_prefix')
        self.prefix = param.get_parameter_value().string_value

        # Declare the delay parameter, then get the value.  This makes
        # the paremeter adjustable in the launch file.
        doubletype       = ParameterType.PARAMETER_DOUBLE
        doubledescriptor = ParameterDescriptor(type = doubletype)

        self.declare_parameter(
            DELAYNAME, descriptor = doubledescriptor, value = DELAYDEFAULT)

        param      = self.get_parameter(DELAYNAME)
        self.delay = param.get_parameter_value().double_value

        # Create the publisher for the republished scan messages.
        self.pubscan = self.create_publisher(
            LaserScan, 'scanfixed', 10)

        # Then create a subscriber to listen to the scan.
        self.subscan = self.create_subscription(
            LaserScan, 'scan', self.cb_scanmsg, 10)

        # Report and return.
        self.get_logger().info("RPLidarFix running (fixed delay %5.3fs)" %
                               self.delay)

    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()


    # Scan Message Callback
    def cb_scanmsg(self, msg):
        # Shift the start time by the scan time (which is the time
        # from the previous scan) as well as a fixed (additional) delay.
        msg.header.stamp = \
            (Time().from_msg(msg.header.stamp) -
             Duration(seconds=(msg.scan_time + self.delay))
            ).to_msg()

        # Edit the frame id to have namespace
        msg.header.frame_id = f'{self.prefix}lidar'

        # Reverse the alpha sample order.
        (msg.angle_min, msg.angle_increment, msg.angle_max) = \
            (msg.angle_max, -msg.angle_increment, msg.angle_min)

        # Reverse the range order.
        msg.ranges = msg.ranges[::-1]

        # Also reverse the intensity order.
        msg.intensities = msg.intensities[::-1]

        # Re-publish the scan data.
        self.pubscan.publish(msg)

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the custom node.
    node = RPNode('rplidarfix')

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
