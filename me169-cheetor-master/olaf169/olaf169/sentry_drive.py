#!/usr/bin/env python3
#
#   sentry_drive.py
#
#   Sentry Driving Node. This
#   (a) Calculates next position using a time based spline
#   (b) Publishes velocities to /cmd_vel
#   (c) Recomputes this continously
#   
#   Node: /sentry_drive
#   Publish:
#           /cmd_vel                    geometry_msgs/Twist
#   Subscribe:
#           /odom                       Odometry
#           /scanfixed                  Scan
#

import rclpy
import traceback

from tf2_ros                    import TransformException
from tf2_ros.buffer             import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker

from rclpy.executors            import MultiThreadedExecutor
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup

import time
from scipy.spatial.transform import Rotation
from math import pi, sin, cos, sqrt

import os
from std_msgs.msg import Bool
from rclpy.node         import Node
from tf2_ros            import TransformBroadcaster
from geometry_msgs.msg  import Point, Quaternion, Twist, PoseStamped, Pose
from geometry_msgs.msg  import TransformStamped, Vector3
from nav_msgs.msg       import Odometry
from sensor_msgs.msg    import JointState, LaserScan
import numpy as np
from std_msgs.msg import Bool
from olaf169.planartransform import PlanarTransform
from rcl_interfaces.msg          import ParameterDescriptor, ParameterType
from math import sqrt

STOP_FILTER_T = 2
# WAYPOINTS = [(-0.6, 1.0), (-0.6, 2.0), (-2.0, 2.0), (-2.0, 1.0)]
WAYPOINTS = [(2.0, -0.5), (0.0, -0.3), (0.0, 0.2), (1.5, 1.0), (2.5, 0.8)]
SENTRY_EXPECTED_SPEED = 0.4

# Times is the array of sentry time to travel between points, based on constant velocity
# Times[0] is the time to travel from WAYPOINTS[0] to WAYPOINTS[1]
TIMES = []
for i in range(len(WAYPOINTS)):
    curr_pt = WAYPOINTS[i]
    next_pt = WAYPOINTS[(i + 1) % len(WAYPOINTS)]
    dist = sqrt((curr_pt[0] - next_pt[0]) ** 2 + (curr_pt[1] - next_pt[1]) ** 2)
    TIMES.append(dist / SENTRY_EXPECTED_SPEED) # Expected time starting at waypoint[i] going to waypoint[i+1]

# Wraps an angle t to [-pi, pi]
def wrap(t):
    return ((t + np.pi) % (2 * np.pi)) - np.pi

def get_sentry_position(t):
    num_seg = len(WAYPOINTS)
    t = t % sum(TIMES)

    cumsum_times = np.cumsum(np.insert(TIMES, 0, 0))

    next_pt_idx = np.argmax(cumsum_times > t) % num_seg # point you're going towards
    last_pt_idx = (next_pt_idx - 1) % num_seg

    segment_duration = float(TIMES[last_pt_idx])
    percent_through_motion = float(t  - cumsum_times[last_pt_idx]) / segment_duration

    past_pt = WAYPOINTS[last_pt_idx]
    next_pt = WAYPOINTS[next_pt_idx]

    x = past_pt[0] + percent_through_motion * (next_pt[0] - past_pt[0])
    y = past_pt[1] + percent_through_motion * (next_pt[1] - past_pt[1])
    return x, y

#
#   Sentry Drive Auto Node Class
#
class SentryDriveAutoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Class variables
        self.START_TIME = None
        self.goal_pose = None # Goal pose is (pos, orientation)
        self.started = False

        self.robot_heading = np.array([1, 0])
        self.robot_position = np.array([0, 0])

        self.robot_moving = 1 # 1 for moving, 0 for stopped

        # Variables for sentry movement
        self.current_waypoint_id = 0
        self.last_waypoint_id = len(WAYPOINTS) - 1
        self.last_waypoint_switch_time = float(time.time())

        stringtype       = ParameterType.PARAMETER_STRING
        stringdescriptor = ParameterDescriptor(type=stringtype)

        self.declare_parameter('frame_prefix', descriptor=stringdescriptor, value = '')
        param       = self.get_parameter('frame_prefix')
        self.prefix = param.get_parameter_value().string_value

        # Creating the generator functions (Change constant here)
        if False:
            # Configuration to become a point turn robot
            self.v_from_theta = self.generate_v_from_theta_func(np.pi / 15, np.pi / 30)
            self.w_from_theta = self.generate_w_from_theta_func(np.pi / 6)
            self.v_from_D = self.generate_v_from_D_func(0.4, 0.5, 2)
        else:
            # Configuraiton for smoooooth driving curves
            self.v_from_theta = self.generate_v_from_theta_func(np.pi / 2, np.pi / 10)
            self.w_from_theta = self.generate_w_from_theta_func(np.pi / 3)
            self.v_from_D = self.generate_v_from_D_func(0.5, 0, 1)

        self.omega_max = 2          # Maximum angular velocity drive_auto is allowed to send
        self.vel_max = 0.6          # Maximum linear velocity drive_auto is allowed to send
        self.close_points = []

        # Create publishers
        self.pubvcmd = self.create_publisher(Twist, 'cmd_vel', 10)      # Velocity command
        self.pub_markers = self.create_publisher(Marker, "markers", 10) # Markers for Waypoints
        self.pub_carrot_marker = self.create_publisher(Marker, "carrot", 10) # Markers for Waypoints

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
        
        self.sublidar = self.create_subscription(
            LaserScan, 'scanfixed', self.cb_lidarmsg, 1, callback_group=slowgroup)

        self.spline_callback = self.create_timer(0.05,  self.split, callback_group=fastgroup)

        self.substartrobot = self.create_subscription(Bool, '/start_robot', self.cb_startrobot, 1)
        self.substopsentry = self.create_subscription(Bool, '/stop_sentry', self.cb_stop_sentry, 1)

        # Report and return.
        self.get_logger().info("Sentry Drive running")

    # Go linear from point1 to point2
    def linear(self, point1, point2, cur_t, t_this_side):
        x = point1[0] + (point2[0] - point1[0]) * cur_t / t_this_side
        y = point1[1] + (point2[1] - point1[1]) * cur_t / t_this_side
        return x, y

    # Returns the time it should take to travel from curr_pt to next_pt at self.vel_max
    def get_t_this_side(self, curr_pt, next_pt):
        dist = sqrt((next_pt[0] - curr_pt[0])**2 + (next_pt[1] - curr_pt[1])**2)
        return dist / (self.vel_max * 2/3)
    
    def split(self):
        if self.started:
            n_pts = len(WAYPOINTS)
            curr_time = float(time.time()) - self.last_waypoint_switch_time # The time we have been on this side for
            t_this_side = self.get_t_this_side(WAYPOINTS[self.current_waypoint_id], WAYPOINTS[(self.current_waypoint_id + 1) % n_pts]) # The total time that this side should take

            if curr_time >= t_this_side:
                # Reached a new side. Update state variables.
                self.last_waypoint_id = self.current_waypoint_id
                self.current_waypoint_id = (self.current_waypoint_id + 1) % n_pts
                self.last_waypoint_switch_time = float(time.time())

                curr_time = curr_time - t_this_side
                t_this_side = self.get_t_this_side(WAYPOINTS[self.current_waypoint_id], WAYPOINTS[(self.current_waypoint_id + 1) % n_pts]) # The total time that this side should take
        
            x, y = self.linear(WAYPOINTS[self.current_waypoint_id], WAYPOINTS[(self.current_waypoint_id + 1) % n_pts], curr_time, t_this_side)
            
            self.goal_pose = np.array([float(x), float(y)])
            self.send_carrot()
    
    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()

    # Generates a function to get linear velocity v from deadzone alpha and speed slowing error gamma
    def generate_v_from_theta_func(self, alpha, gamma):
        return lambda theta: min(max((np.abs(theta) - alpha)/(gamma - alpha), 0.0), 1.0)

    def generate_w_from_theta_func(self, beta):
        return lambda theta: np.sign(theta) * min(1.0, np.abs(theta) / beta)

    def generate_v_from_D_func(self, cmin, a, b):
        return lambda D: max(min(1.0, cmin + (D - a) * (1 - cmin) / (b - a)), cmin)

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

    # Publish a red marker for the goal pose
    def send_carrot(self):
        markermsg = Marker()
        markermsg.header.frame_id = "map"

        now = self.get_clock().now()
        markermsg.header.stamp = now.to_msg()

        markermsg.ns = "sentry_goal_pose"
        markermsg.id = 0
        markermsg.type = Marker.POINTS
        markermsg.action = Marker.ADD # Should be same as Marker.MODIFY
        markermsg.pose = Pose()
        markermsg.scale = Vector3(x=0.05, y=0.05, z=0.05)
        markermsg.color.a = 1.0

        # Dark green markers for robot2
        markermsg.color.r = 1.0
        markermsg.color.g = 0.0
        markermsg.color.b = 0.0
        markermsg.points = []

        markermsg.points.append(Point(x=self.goal_pose[0], y=self.goal_pose[1], z=0.0))

        self.pub_carrot_marker.publish(markermsg)

    def send_markers(self):
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

        for pt in WAYPOINTS:
            markermsg.points.append(Point(x=pt[0], y=pt[1], z=0.0))

        self.pub_markers.publish(markermsg)

    def cb_posemsg(self, msg):
        # No sentry movements if splines haven't set goal pose yet
        if self.goal_pose is None:
            return None

        # Send the markers
        self.send_markers()

        # Get stuff from odom message
        timestamp = msg.header.stamp
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w

        dt = 0.02

        # Robot info vectors
        self.robot_position = np.array([x, y])
        rotZ = Rotation.from_quat(np.array([0, 0, z, w])).as_euler('xyz', degrees=False)[2]
        self.robot_heading = np.array([np.cos(rotZ), np.sin(rotZ)])

        # Target info vectors
        target_vec = self.goal_pose - self.robot_position
        theta = self.signed_angle(self.robot_heading, target_vec)
        target_distance = np.linalg.norm(target_vec)

        wt = self.w_from_theta(theta)
        omega = self.omega_max * wt

        vd = self.v_from_D(target_distance) 
        vt = self.v_from_theta(theta)

        self.distance_scalar = 1 # self.find_distance_scalar() 
        self.robot_moving += dt / STOP_FILTER_T * (self.distance_scalar - self.robot_moving)

        vel = self.vel_max * vd * vt * self.distance_scalar
        
        # Publish the command
        msg = Twist()
        msg.linear.x = vel if self.started else 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = omega if self.started else 0.0

        self.pubvcmd.publish(msg)

    # Starts the sentry movement
    def cb_startrobot(self, msg):
        self.current_waypoint_id = 0
        self.last_waypoint_id = len(WAYPOINTS) - 1
        self.last_waypoint_switch_time = float(time.time())
        self.started = True

    # Stops the sentry movement
    def cb_stop_sentry(self, msg):
        self.started = False

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

                if r * abs(sin(point_angle)) < ROBOT_STOPPING_WIDTH and 0 <= r * cos(point_angle):
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
    node = SentryDriveAutoNode('sentry_drive')

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
