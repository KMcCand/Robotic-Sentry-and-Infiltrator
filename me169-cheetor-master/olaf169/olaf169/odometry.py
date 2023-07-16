#!/usr/bin/env python3
#
#   odometry_skeleton.py
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

from math import pi, sin, cos

from rclpy.node         import Node
from tf2_ros            import TransformBroadcaster
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import TransformStamped
from nav_msgs.msg       import Odometry
from sensor_msgs.msg    import JointState
import numpy as np
from rcl_interfaces.msg          import ParameterDescriptor, ParameterType


#
#   Constants
#
R = 0.03315           # Wheel radius
d = 0.065           # Halfwidth between wheels

PHI_LEFT_NAME = 'leftwheel'
PHI_RIGHT_NAME = 'rightwheel'


#
#   Odometry Node Class
#
class OdometryNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        stringtype       = ParameterType.PARAMETER_STRING
        stringdescriptor = ParameterDescriptor(type=stringtype)

        self.declare_parameter('frame_prefix', descriptor=stringdescriptor, value = '')

        param       = self.get_parameter('frame_prefix')
        self.prefix = param.get_parameter_value().string_value

        # Set the initial pose to zero.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # self.encoder_only_theta = 0  # For testing only, theta estimate without gyro values

        # Keep track of previous lpsi, rsi, theta_gyro
        self.prev_lpsi = 0
        self.prev_rpsi = 0
        self.prev_lpsi_dot = 0
        self.prev_rpsi_dot = 0
        self.prev_theta_gyro = 0

        # Create the publishers for the wheel commands and the
        # odometry information.
        self.pubwcmd = self.create_publisher(JointState, 'wheel_command', 3)
        self.pubodom = self.create_publisher(Odometry, 'odom', 10)

        # Initialize the transform broadcaster
        self.tfbroadcaster = TransformBroadcaster(self)

        # Create the subscribers to listen to wheel state and twist
        # comamnds.
        self.subwact = self.create_subscription(
            JointState, 'wheel_state', self.cb_wheelmsg, 10)
        self.subvcmd = self.create_subscription(
            Twist, 'cmd_vel', self.cb_vcmdmsg, 10)

        # Report and return.
        self.get_logger().info("Odometry running")

    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()

    # Velocity Command Message Callback
    def cb_vcmdmsg(self, msg):
        # Grab the forward and spin (velocity) commands.
        vx = msg.linear.x
        wz = msg.angular.z

        # Find the wheel's rotational speed
        phi_left = 1 / R * (vx - d * wz)
        phi_right = -1 / R * (vx + d * wz)

        # Create the wheel command msg and publish.  Note the incoming
        # message does not have a time stamp, so generate one here.
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name         = [PHI_LEFT_NAME,  PHI_RIGHT_NAME]        
        msg.velocity     = [phi_left, phi_right]
        self.pubwcmd.publish(msg)


    # Wheel State Message Callback
    def cb_wheelmsg(self, msg):
        # Grab the timestamp, wheel and gyro position/velocities.
        try:
            timestamp = msg.header.stamp
            lpsi      = msg.position[msg.name.index('leftwheel')]
            lpsi_dot  = msg.velocity[msg.name.index('leftwheel')]
            rpsi      = msg.position[msg.name.index('rightwheel')]
            rpsi_dot  = msg.velocity[msg.name.index('rightwheel')]
            theta_gyro= msg.position[msg.name.index('gyro')]
            omega_gyro= msg.velocity[msg.name.index('gyro')]
        except:
            self.get_logger().error("Ill-formed /wheel_state message!")
            return

        # Convert to get a forward distance and heading change.
        vx = R / 2 * lpsi_dot - R / 2 * rpsi_dot
        wz = -R / (2 * d) * lpsi_dot - R / (2 * d) * rpsi_dot
       
        # Calculate psi values
        delta_lpsi = lpsi - self.prev_lpsi
        delta_rpsi = rpsi - self.prev_rpsi
        delta_lpsi_dot = lpsi_dot - self.prev_lpsi_dot
        delta_rpsi_dot = rpsi_dot - self.prev_rpsi_dot

        # Update previous psi values
        self.prev_lpsi = lpsi
        self.prev_rpsi = rpsi
        self.prev_lpsi_dot = lpsi_dot
        self.prev_rpsi_dot = rpsi_dot

        delta_theta_encoders = R / (2 * d) * (-delta_lpsi - delta_rpsi) # Encoder Calculation
        delta_theta_gyro = theta_gyro - self.prev_theta_gyro            # Gyro reading
        self.prev_theta_gyro = theta_gyro

        # Use gyro readings to correct for possible wheel slipping if acceleration is high
        if abs(delta_lpsi_dot) < 0.8 and abs(delta_rpsi_dot) < 0.8:
            # If motors are both accelerating slowly, assume no slippage
            slippage = 0
        elif abs(delta_lpsi_dot) < abs(delta_rpsi_dot):
            # If delta_lpsi_dot is smaller, base delta_p on delta_lpsi and gyro only.
            slippage = 2 * d / R * (delta_theta_gyro - delta_theta_encoders)
        else:
            # Else base delta_p on delta_rpsi and gyro only. See Goals #3 Submission for the formula math.
            slippage = 2 * d / R * (delta_theta_gyro + delta_theta_encoders)
        
        delta_p = R / 2 * (delta_lpsi - delta_rpsi + slippage)
        
        self.x     += delta_p * cos(self.theta + delta_theta_gyro / 2) * np.sinc(delta_theta_gyro / 2)
        self.y     += delta_p * sin(self.theta + delta_theta_gyro / 2) * np.sinc(delta_theta_gyro / 2)
        self.theta += delta_theta_gyro
        # self.encoder_only_theta += delta_theta_encoders # For testing only

        # Create the odometry msg and publish (reuse the time stamp).
        msg = Odometry()
        msg.header.stamp            = timestamp
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'base'
        msg.pose.pose.position.x    = self.x
        msg.pose.pose.position.y    = self.y
        msg.pose.pose.orientation.z = sin(self.theta / 2)
        msg.pose.pose.orientation.w = cos(self.theta / 2)
        msg.twist.twist.linear.x    = vx
        msg.twist.twist.angular.z   = wz
        self.pubodom.publish(msg)

        # Create the transform and broadcast (reuse the time stamp).
        trans = TransformStamped()
        trans.header.stamp            = timestamp
        trans.header.frame_id         = f'{self.prefix}odom'
        trans.child_frame_id          = f'{self.prefix}base'
        trans.transform.translation.x = self.x # We edited these 4 lines
        trans.transform.translation.y = self.y
        trans.transform.rotation.z    = sin(self.theta / 2)
        trans.transform.rotation.w    = cos(self.theta / 2) 
        self.tfbroadcaster.sendTransform(trans)

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = OdometryNode('odometry')

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
