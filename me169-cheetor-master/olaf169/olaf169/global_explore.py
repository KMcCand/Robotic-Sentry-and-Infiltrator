#!/usr/bin/env python3
#
#   global_explore.py
#
#   Global Exploration Node. This node
#   (a) Subscribes to robot poses from /pose, the current map from /uncertainty_map and 
#   (b) Publishes next exploration target to /goal_pose
#   
#   Node: /global_explore
#   Publish:
#
#           /marker_explore                     visualization_msgs/Marker
#           /goal_pose                  geometry_msgs/PoseStamped
#   Subscribe:
#           /pose                       geometry_msgs/PoseStamped
#           /uncertainty_map                        OccupancyGrid
import time
import rclpy
import traceback

from rclpy.executors            import MultiThreadedExecutor
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup

from rclpy.qos                  import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool

from math import pi, sin, cos

from rclpy.node         import Node
from nav_msgs.msg       import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3

from std_msgs.msg import ColorRGBA

import numpy as np
import random 
import os
from olaf169.planartransform import PlanarTransform
from visualization_msgs.msg import Marker
from rcl_interfaces.msg          import ParameterDescriptor, ParameterType

# Global constants
RES = 0.0254
LOAD_MAP = True

xmin, xmax = 0, 300
ymin, ymax = 0, 300

SENTRY_POINTS = [(2, -0.5), (0, -0.3), (0, 0.2), (1.5, 1), (2.5, 0.8)]

def generate_edges(points):
    edges = []
    for i in range(len(points)):
        p1 = points[i]
        p2 = points[(i + 1) % len(points)]
        edges.append((np.array(p1), np.array(p2)))

    return edges 

# Helper functions 

def distanceToEdge(pnt, p1, p2):
    line_vec = p2 - p1
    pnt_vec = pnt - p1
    line_len = np.linalg.norm(p2 - p1)
    line_unitvec = (p2 - p1) / line_len
    pnt_vec_scaled = pnt_vec * (1.0/line_len)
    t = np.dot(line_unitvec, pnt_vec_scaled)    
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = line_vec * t
    dist = np.linalg.norm(nearest - pnt_vec)
    return dist

class GlobalExplore(Node):
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
        self.started = False
        self.got_map = False
        self.current_pose = None

        self.edges = generate_edges(SENTRY_POINTS)
        self.center = np.mean(self.edges, 0)

        # HAS TO BE A THREADED NODE, CANNOT BLOCK UNCERTAINTY MAP INPUT
        fastgroup = MutuallyExclusiveCallbackGroup()
        slowgroup = MutuallyExclusiveCallbackGroup()
        othergroup = MutuallyExclusiveCallbackGroup()
        
        # Publishers
        self.pub_goalpose = self.create_publisher(PoseStamped, 'goal_pose', 1, callback_group=othergroup) # Waypoints
        self.pub_markers = self.create_publisher(Marker, "marker_explore", 10, callback_group=othergroup)      # Markers for Waypoints

        # Subscribers
        # Create a subscriber for the map data.  Note this topic uses
        # a quality of service with durability TRANSIENT_LOCAL
        # allowing new subscribers to get the last sent message.
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        # self.submap = self.create_subscription(OccupancyGrid, '/map', self.cb_mapmsg, quality)

        self.subuncertainty_map = self.create_subscription(OccupancyGrid, 'uncertainty_map', self.cb_uncertainty_map, quality, callback_group=fastgroup)

        self.subrobotpose = self.create_subscription(PoseStamped, 'pose', self.cb_robotpose, 10, callback_group=fastgroup)

        self.substartrobot = self.create_subscription(Bool, '/start_robot', self.cb_startrobot, 1, callback_group=fastgroup)

        self.subget_global_waypoint = self.create_subscription(Bool, "get_global_waypoint", self.cb_get_global_waypoint, 1, callback_group=slowgroup)

        # Recomputing 
        # self.timer = self.create_timer(4, self.cb_get_global_waypoint_test)

        # Report and return.
        self.get_logger().info("Global Explore running")

    def distance_to_path(self, point):
        distance_to_center = np.linalg.norm(self.center - point)

        distances_to_edges = [distanceToEdge(point, p1, p2) for (p1, p2) in self.edges]
        min_distance = min(min(distances_to_edges), distance_to_center)

        return min_distance
           

    # Shutdown
    def shutdown(self):
        # Nothing to do except shut foun the node.
        self.destroy_node()

    # Return grid coords u, v for map coords x, y
    def grid_coords_from_map(self, x, y):
        u, v = self.map2grid.inv().inParent(x, y) # Get grid frame
        return round(u / RES), round(v / RES)

    # Return map coords x, y for grid coords u, v
    def map_coords_from_grid(self, u, v):
        return self.map2grid.inParent(u * RES, v * RES) # Get grid frame
    
    # Getting uncertainty map from mapping node
    def cb_uncertainty_map(self, msg):
        self.height, self.width = msg.info.height, msg.info.width
        self.uncertainty_map = np.reshape(msg.data, (self.height, self.width))
        self.map2grid = PlanarTransform.fromPose(msg.info.origin)
        self.got_map = True

    # Keep track of the robots current position
    def cb_robotpose(self, msg):
        assert msg.header.frame_id == 'map', "Planner: Robot Pose Message not in map frame!!!!"
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        self.current_pose = (x, y)

    def cb_startrobot(self, msg):
        self.get_logger().info("Global Planner Starting")
        self.started = True

    def generate_points(self):
        if not self.got_map:
            self.get_logger().info("No Map Recieved")
            return None

        NO_ANGLES = 30
        NO_POINTS = 100
        angles = [i * (2 * pi / NO_ANGLES) for i in range(NO_ANGLES)]

        points = []
        strengths = []
        skipped = 0

        complete_metric = []
        sentry_distances = []


        st = time.time()

        self.get_logger().info("Starting Loop")

        for i in range(NO_POINTS):
            points_checked = 0 

            while True: 
                u, v = random.randint(xmin, xmax - 1), random.randint(ymin, ymax - 1)
                points_checked += 1
                if self.uncertainty_map[v, u] < 30:
                    break

                if points_checked > 1000: # Try to see if at max 1000 points are valid, if we can't find any then just ditch the loop
                    self.get_logger().info("Something help")
                    return None

            metrics = [self.bresenhams(u, v, angle) for angle in angles]
            point_value = sum([score for (score, temp) in metrics])

            min_distance = min([dist for (temp, dist) in metrics])

            robot_u, robot_v = self.grid_coords_from_map(self.current_pose[0], self.current_pose[1])
            distance_weight = max((((u - robot_u)**2) + ((v - robot_v) ** 2)) ** (1/2), 7)

            point_value /= ((distance_weight) ** (1/4))
            point_value = max(0, point_value)

            map_coords = self.map_coords_from_grid(u, v)

            distance_from_sentry = self.distance_to_path(np.array(map_coords))
            sentry_distances.append(distance_from_sentry)

            if min_distance < 10:
                skipped += 1
                point_value = -1

            elif distance_from_sentry < 0.5: # MAP SPACE
                skipped += 1
                point_value = -2

            else:
                complete_metric.append(point_value)

            points.append(self.map_coords_from_grid(u, v))
            strengths.append(point_value)

        dt = time.time() - st
        
        self.get_logger().info(f"Time to compute: {NO_POINTS * NO_ANGLES} bresenhams {dt}")
        self.get_logger().info(f"Complete Metric: {complete_metric}. ")
        self.get_logger().info(f"Average: {sum(complete_metric) / (len(complete_metric) + 1)} Min: {min(complete_metric)} Max: {max(complete_metric)}")

        self.publish_markers(points, strengths)

        if len(points) == 0:
            self.get_logger().info("No valid Points, help me krishy")

        bestp, maxstrength = -1, -100000
        for (point, strength) in zip(points, strengths):
            if strength >= maxstrength:
                bestp = point
                maxstrength = strength

        return bestp
        
    def bresenhams(self, u1, v1, angle):
        value = 0

        LINE_LENGTH = 300
        u2, v2 = int(u1 + LINE_LENGTH * cos(angle)), int(v1 + LINE_LENGTH * sin(angle))

        du = -abs(u2 - u1)
        dv = abs(v2 - v1)

        su = 1 if u1 < u2 else -1
        sv = 1 if v1 < v2 else -1

        err = du + dv
        uk, vk = u1, v1 # starting point

        distance = None

        while True:
            if (uk >= xmax) or (uk <= xmin) or (vk >= ymax) or (vk <= ymin):
                d = (((u1 - uk)**2) + ((v1 - vk) ** 2)) ** (1/2)

                if distance == None:
                    return value, d
                else:
                    return value, min(distance, d)
            
            if self.uncertainty_map[vk, uk] > 90:
                # We have hit a wall
                d = (((u1 - uk)**2) + ((v1 - vk) ** 2)) ** (1/2)
                
                if distance == None:
                    return value, d
                else:
                    return value, min(distance, d) 
            
            elif self.uncertainty_map[vk, uk] > 33:
                # Unknown spot, so add value 
                value += 1

                if distance == None:
                    distance = (((u1 - uk)**2) + ((v1 - vk) ** 2)) ** (1/2)

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

        return value, distance

    def publish_markers(self, points, strengths):
        max_strength = max(strengths)

        markermsg = Marker()
        markermsg.header.frame_id = "map"
        markermsg.header.stamp = self.get_clock().now().to_msg()
        markermsg.ns = "global_mapping"
        markermsg.id = 0
        markermsg.type = Marker.POINTS
        markermsg.action = Marker.ADD # Should be same as Marker.MODIFY
        markermsg.pose = Pose()
        markermsg.scale = Vector3(x=0.07, y=0.07, z=0.07)

        markermsg.color.r = 0.0
        markermsg.color.g = 0.0
        markermsg.color.b = 1.0
        markermsg.color.a = 1.0

        markermsg.colors = []
        markermsg.points = []

        for (pt, strength) in zip(points, strengths):
            markermsg.points.append(Point(x=pt[0], y=pt[1], z=0.0))
            col = ColorRGBA()

            if strength == -1:
                col.r = 1.0
                col.g = 1.0
                col.b = 1.0
                col.a = 1.0

            elif strength == -2:
                col.r = 0.0
                col.g = 0.0
                col.b = 0.0
                col.a = 1.0
            
            else:
                col.r = float(strength / max_strength)
                col.g = 1 - float(strength / max_strength)
                col.b = 0.0
                col.a = 1.0
                
            markermsg.colors.append(col)
        self.pub_markers.publish(markermsg)

    def cb_get_global_waypoint(self, msg):
        time.sleep(0.5)
        if self.started:
            self.get_logger().info("Computing new explorer waypoints!")
            # Compute the best point
            best_point = None
            while best_point is None:
                best_point = self.generate_points()
                time.sleep(2)

            self.get_logger().info("Generated Points")
            
            # Publish the waypoint - do we care about theta???
            waypoint_msg = PoseStamped()
            waypoint_msg.pose.position.x = best_point[0]
            waypoint_msg.pose.position.y = best_point[1]
            waypoint_msg.pose.position.z = 0.0
        
            # Send a 0 quaternion to indicate we have no desired orientation.
            # Real quaternions would not work if they are all 0's
            waypoint_msg.pose.orientation.z = 0.0
            waypoint_msg.pose.orientation.w = 0.0
                    
            waypoint_msg.header.frame_id = 'map'
            waypoint_msg.header.stamp = self.get_clock().now().to_msg()

            self.pub_goalpose.publish(waypoint_msg)
            self.get_logger().info("Publishing Global Mapping Point")

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = GlobalExplore('global_explore')

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin the node until interrupted.  To support multiple callback
    # groups, use a multithreaded executor.  The callbacks in each
    # group will thus run in their respective thread.

    # Spin the node until interrupted.
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
