#!/usr/bin/env python3
#
#   drive_auto.py
#
#   Auto Driving Node. This
#   (a) Reads the /odom topic to recieve the robot position and orientation and the goal pose 
#   (b) Publishes velocities to /cmd_vel
#   (c) Recomputes this continously 
#   
#   Node: /drive_auto
#   Publish:
#           /cmd_vel                    geometry_msgs/Twist
#           /next_waypoint              std_msgs/Bool
#   Subscribe:
#           /odom                       Odometry
#           /waypoint                   geometry_msgs/PoseStamped
#           /scanfixed                  Scan
#

import rclpy
import traceback
import time

from tf2_ros                    import TransformException
from tf2_ros.buffer             import Buffer
from tf2_ros.transform_listener import TransformListener

from rclpy.executors            import MultiThreadedExecutor
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup

import time
from scipy.spatial.transform import Rotation
from math import pi, sin, cos

import os
from rclpy.node         import Node
from tf2_ros            import TransformBroadcaster
from geometry_msgs.msg  import Point, Quaternion, Twist, PoseStamped, Pose
from geometry_msgs.msg  import TransformStamped, Vector3



from nav_msgs.msg       import Odometry
from sensor_msgs.msg    import JointState, LaserScan
import numpy as np
from std_msgs.msg import Bool, Float64
from olaf169.planartransform import PlanarTransform
from rcl_interfaces.msg          import ParameterDescriptor, ParameterType


STOP_FILTER_T = 2


# Wraps an angle t to [-pi, pi]
def wrap(t):
    return ((t + np.pi) % (2 * np.pi)) - np.pi

#
#   Odometry Node Class
#
class DriveAutoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Class variables
        self.goal_pose = None # Goal pose is (pos, orientation)
        #if os.uname()[1] == 'olaf': # HEY @NISHKA change this to olafrobot2 later
        #    self.waiting_for_waypoint = True # True if sent waypoint request but have not received yet
        #else:
        self.waiting_for_replan = False
        self.waiting_for_waypoint = False
        self.waiting_for_replan = False
        self.robot_heading = np.array([1, 0])
        self.robot_position = np.array([0, 0])

        self.robot_moving = 1 # 1 for moving, 0 for stopped
        self.cb_posemsg_prev_timestamp = None

        stringtype       = ParameterType.PARAMETER_STRING
        stringdescriptor = ParameterDescriptor(type=stringtype)

        self.declare_parameter('frame_prefix', descriptor=stringdescriptor, value = '')

        param       = self.get_parameter('frame_prefix')
        self.prefix = param.get_parameter_value().string_value
        if self.prefix == 'robot1':
            simple_move = False
        else:
            simple_move = True

        # Creating the generator functions (Change constant here)
        if False:
            # Configuration to become a point turn robot
            self.v_from_theta = self.generate_v_from_theta_func(np.pi / 15, np.pi / 30)
            self.w_from_theta = self.generate_w_from_theta_func(np.pi / 6)
            self.v_from_D = self.generate_v_from_D_func(0.75, 0.25, 1)
        else:
            # Configuraiton for smoooooth driving curves
            self.v_from_theta = self.generate_v_from_theta_func(np.pi / 3, np.pi / 6)
            self.w_from_theta = self.generate_w_from_theta_func(np.pi / 4)
            self.v_from_D = self.generate_v_from_D_func(0.75, 0.25, 1)

        self.omega_max = 1.5       # Maximum angular velocity drive_auto is allowed to send
        self.vel_max = 0.5          # Maximum linear velocity drive_auto is allowed to send
        self.accepted_radius = 0.05 # Within accepted_radius, the bot is considered at the target
        self.goal_reached = False
        self.goal_angle_reached = False
        self.close_points = []

        self.vd = 0
        self.vt = 0
        self.wt = 0

        self.no_orientation = False

        self.distance_scalar = 1


        # Create publishers
        self.pubvcmd = self.create_publisher(Twist, 'cmd_vel', 10) # Velocity command
        self.pub_next_waypoint = self.create_publisher(Bool, 'next_waypoint', 1) # Ask global planner for next waypoint
        self.pub_new_rrt = self.create_publisher(Bool, 'new_rrt', 1) # Ask global planner to re-plan because we are stuck

        self.pub_vd = self.create_publisher(Float64, 'vd', 10)
        self.pub_vt = self.create_publisher(Float64, 'vt', 10)
        self.pub_wt = self.create_publisher(Float64, 'wt', 10)
        self.pub_distance_scalar = self.create_publisher(Float64, 'ds', 10)

        # Creating the message
        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0

        # Creating information for movement time
        self.max_movement_time = 0
        self.initial_movement_time = time.time()

        # Instantiate TF Listener and Buffer
        self.tfBuffer   = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

        # Use two different callback groups, which will run
        # independently in distinct threads.  Within a group,
        # subsequent callbacks are held until the prior ones complete.
        fastgroup = MutuallyExclusiveCallbackGroup()
        slowgroup = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.subpose = self.create_subscription(
            PoseStamped, 'pose', self.cb_posemsg, 10, callback_group=fastgroup)
        
        self.subgoal = self.create_subscription(
            PoseStamped, 'waypoint', self.cb_goalmsg, 1, callback_group=fastgroup)

        self.sublidar = self.create_subscription(
            LaserScan, 'scanfixed', self.cb_lidarmsg, 1, callback_group=slowgroup)

        self.pub_new_rrt = self.create_publisher(Bool, 'new_rrt', 2, callback_group=fastgroup) # Ask global planner to re-plan because we are stuck

        self.timer = self.create_timer(1/4, self.print_values)


        # Report and return.
        self.get_logger().info("Drive Auto running")

    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()

    # Generates a function to get linear velocity v from deadzone alpha and speed slowing error gamma
    def generate_v_from_theta_func(self, alpha, gamma):
        return lambda theta: min(max(1.0 + (np.abs(theta) - gamma)/(gamma - alpha), 0.0), 1.0)

    def generate_w_from_theta_func(self, beta):
        return lambda theta: np.sign(theta) * min(1, np.abs(theta) / beta)

    def generate_v_from_D_func(self, cmin, a, b):
        return lambda D: max(min(1, cmin + (D - a) * (1 - cmin) / (b - a)), cmin)

    def find_distance_scalar(self):
        if len(self.close_points) == 0:
            return 1
        if len(self.close_points) > 2:
            dist = self.close_points[1]
        else:
            dist = self.close_points[0]

        return max(0, min(1, (dist - 0.25) / (0.15)))
    
    def signed_angle(self, f, t):
        # from - f, to - t
        return np.arctan2(t[1] * f[0] - t[0] * f[1], np.dot(f, t))

    def cb_posemsg(self, msg):
        # Get stuff from odom message
        timestamp = msg.header.stamp

        # dt = timestamp - self.cb_posemsg_prev_timestamp if self.cb_posemsg_prev_timestamp else 0
        dt = 0.02

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w

        # Create robot info vectors
        self.robot_position = np.array([x, y])
        rotZ = Rotation.from_quat(np.array([0, 0, z, w])).as_euler('xyz', degrees=False)[2]
        self.robot_heading = np.array([np.cos(rotZ), np.sin(rotZ)])

        if self.goal_pose is None: 
            return None

        # Can do this more efficiently
        if self.no_orientation:
            target_heading = None
            target_position = np.array(self.goal_pose[0])

        else:
            target_position, target_orientation = np.array(self.goal_pose[0]), np.array(self.goal_pose[1])        
            rotZTarget = Rotation.from_quat(target_orientation).as_euler('xyz', degrees=False)[2]
            target_heading  = np.array([np.cos(rotZTarget), np.sin(rotZTarget)])

        target_vec = target_position - self.robot_position
        theta = self.signed_angle(self.robot_heading, target_vec)
        target_distance = np.linalg.norm(target_vec)

        self.wt = self.w_from_theta(theta)
        omega = self.omega_max * self.wt

        self.vd = self.v_from_D(target_distance) 
        self.vt = self.v_from_theta(theta)

        self.get_logger().info(f"Theta: {theta}")

        self.distance_scalar = self.find_distance_scalar()
        
        self.robot_moving += dt / STOP_FILTER_T * (self.distance_scalar - self.robot_moving)

        if self.robot_moving < 0.3 and not self.waiting_for_replan:
            msg = Bool()
            msg.data = True
            self.pub_new_rrt.publish(msg)
            self.get_logger().info("Stuck! Waiting for new RRT...")
            self.waiting_for_replan = True


        # self.get_logger().info(f"Stop Filter: {self.robot_moving}")
        # CHECK IF DISTANCE SCALAR IS ON AVERAGE 0 FOR LAST FEW SECONDS

        vel = self.vel_max * self.vd * self.vt * self.distance_scalar
        # print(f"Omega: {omega}, vel: {vel}")

        # If we are at the target position, move towards the target orientation with 0 linear velocity
        if target_distance < self.accepted_radius or (target_distance < self.accepted_radius * 6 and self.goal_reached):
            if (target_distance < self.accepted_radius):
                self.goal_reached = True

            # Ask global planner for a new waypoint!
            # print('Drive_auto: Waypoint reached!')
            if not self.waiting_for_waypoint:
                msg = Bool()
                msg.data = True
                self.pub_next_waypoint.publish(msg)
                self.waiting_for_waypoint = True

            vel = 0.0

            if (target_heading is None):
                angle_to_target = 0
            else:
                angle_to_target = self.signed_angle(self.robot_heading, target_heading)

            if self.goal_angle_reached:
                if abs(angle_to_target) > 0.30: # BIG DEADZONE TO ACCOUNT FOR GETTING OUT OF RANGE
                    self.goal_angle_reached = False
                    omega = self.w_from_theta(angle_to_target) * self.omega_max
                else:
                    omega = 0.0
            else:
                if abs(angle_to_target) < 0.05:
                    self.goal_angle_reached = True 
                    omega = 0.0
                else:
                    omega = self.w_from_theta(angle_to_target) * self.omega_max

        if target_distance > self.accepted_radius * 6:
            self.goal_reached = False
        
        self.msg.linear.x  = vel
        self.msg.angular.z = omega
        self.pubvcmd.publish(self.msg)

        vd_msg = Float64(data=float(self.vd))
        vt_msg = Float64(data=float(self.vt))
        wt_msg = Float64(data=float(self.wt))
        dist_s_msg = Float64(data=float(self.distance_scalar))

        self.pub_vd.publish(vd_msg)
        self.pub_vt.publish(vt_msg)
        self.pub_wt.publish(wt_msg)
        self.pub_distance_scalar.publish(dist_s_msg)

    def cb_goalmsg(self, msg):
        self.get_logger().info(f'Got new msg {msg}')
        assert msg.header.frame_id == 'map', "Message not in map frame!!!!"

        pose = msg.pose
        # Only Storing the X-Y and the Z,W
        self.goal_pose = (np.array([pose.position.x, pose.position.y]),
            np.array([0, 0, pose.orientation.z, pose.orientation.w]))

        if (pose.orientation.z == 0 and pose.orientation.w == 0):
            self.no_orientation = True 

        else:
            self.no_orientation = False
        
        self.waiting_for_waypoint = False

        self.waiting_for_replan = False
        
        # Reset filter for stopping
        self.robot_moving = 0.7

        # Setting movement times 
        self.initial_movement_time = time.time()

        if (pose.orientation.x == 0.0):
            self.max_movement_time = np.Inf 

        else:
            self.max_movement_time = pose.orientation.x
            


    def print_values(self):
        self.get_logger().info(f"VD: {self.vd} VT: {self.vt} DS: {self.distance_scalar} WT: {self.wt}")

    def cb_lidarmsg(self, msg):
        try:
            tf_map_msg = self.tfBuffer.lookup_transform(
                'map', f'{msg.header.frame_id}', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn("Unable to get transform: %s" % (ex,))
            return

        map2lidar = PlanarTransform.fromTransform(tf_map_msg.transform)
        
        # Grab the rays: each ray's range and angle relative to the
        # turtlebot's position and orientation.
        rmin     = msg.range_min        # Sensor minimum range to be valid
        rmax     = msg.range_max        # Sensor maximum range to be valid
        ranges   = msg.ranges           # List of ranges for each angle

        thetamin = msg.angle_min        # Min angle (-pi)
        thetamax = msg.angle_max        # Max angle (pi)
        thetainc = msg.angle_increment  # Delta between angles (2pi/360)
        thetas   = np.arange(thetamin, thetamax, thetainc)
        
        ##################################################################
        # New auto stopping code!
        ROBOT_STOPPING_WIDTH = 0.18
        ROBOT_SLOWING_DISTANCE = 0.4
        ROBOT_STOPPING_NPOINTS = 2

        front_points_close = 0
        front_points_all = 0
        self.close_points = []

        for (r_lidar, t) in zip(ranges, thetas):
            todom = wrap(t)

            if rmin - 5 < r_lidar: # Giving it some extra range on the near end
                # If hit detected
                r_lidar = min(r_lidar, rmax)
                x_lidar, y_lidar = r_lidar * np.cos(todom), r_lidar * np.sin(todom)
                map_point = np.array(map2lidar.inParent(x_lidar, y_lidar))

                # Check if in front
                point_angle = self.signed_angle(self.robot_heading, map_point - self.robot_position)

                r = np.linalg.norm(map_point - self.robot_position)

                if r * abs(sin(point_angle)) < ROBOT_STOPPING_WIDTH and 0.0 <= r * cos(point_angle):
                    # Point is ahead of the robot. Defines the sides of a rectangle.
                    front_points_all += 1
                    if r * cos(point_angle) < ROBOT_SLOWING_DISTANCE:
                        # Point is close! Defines the opposite edge of the rectangle.
                        front_points_close += 1
                        self.close_points.append(r * cos(point_angle))
        
        self.close_points = sorted(self.close_points)
        #################################################################################

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = DriveAutoNode('drive_auto')

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
