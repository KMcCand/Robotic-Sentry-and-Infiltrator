#!/usr/bin/env python3
#
#   planner.py
#
#   Global Planning Node. This node
#   (a) Subscribes to goal poses from map, waypoint requests from drive_auto, and robot poses from /pose
#   (b) Publishes waypoints for drive_auto (local planner) to drive to
#   (c) Maintains a list of these waypoints to get to the goal point
#   
#   Node: /planner
#   Publish:
#           /waypoint                   geometry_msgs/PoseStamped
#           /marker                     visualization_msgs/Marker
#           /new_target                 std_msgs/Bool
#   Subscribe:
#           /goal_pose                  geometry_msgs/PoseStamped
#           /next_waypoint              std_msgs/Bool
#           /pose                       geometry_msgs/PoseStamped
#           /map                        OccupancyGrid

import traceback
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

# RRT Parameters
Nmax = 1000
dstep = 10
xmin, xmax = 0, 300
ymin, ymax = 0, 300

# Global constants
CONNECTS_TO_GRANULARITY = 5 # Number of grids to skip when checking connectsTo
GRIDS_TO_WALL = 10 # Robot object avoidance radius in grids
RES = 0.0254
LOAD_MAP = True

class RRTGraphNode():
    # Initialize with coordinates.
    def __init__(self, u, v, nearest_neighbours):
        # Define/remember the state/coordinates (x,y).
        self.u = u
        self.v = v

        # Clear any parent information.
        self.parent = None
        
        self.nearest_neighbours = nearest_neighbours

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Point %d,%d>" % (self.u, self.v))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate(self, other, alpha):
        return RRTGraphNode(int(self.u + alpha * (other.u - self.u)),
                    int(self.v + alpha * (other.v - self.v)))

    # Compute the relative distance to another node.
    def distance(self, other):
        # is there a better distance measure in grid space??
        return ((self.u - other.u)**2 + (self.v - other.v)**2) ** (0.5)

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        return (self.u, self.v)

    ######################
    # Collision functions:
    # Check whether in free space.
    def inFreespace(self):
        if not ((0 <= self.v and self.v < 300) and (0 <= self.u and self.u < 300)):
        # if not (self.v, self.u) in self.nearest_neighbours:
            print(f"Planner RRT InFreeSpace: pts outside map {self.u, self.v}")
            return False
        # Get nearest wall point by accessing self.wallptmap in grid frame
        u_wall, v_wall = self.nearest_neighbours[self.v, self.u] # Nearest wall point in Grid Frame
        return self.distance(RRTGraphNode(u_wall, v_wall, self.nearest_neighbours)) > GRIDS_TO_WALL

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        return self.interpolateConnectsTo((self.u, self.v), (other.u, other.v))
        
    def interpolateConnectsTo(self, start, end):
        queue = [(start, end)]

        while len(queue) != 0:
            s, e = queue.pop(0)
            m = (int((s[0] + e[0]) / 2), int((s[1] + e[1]) / 2))

            if (abs(e[0] - s[0]) ** 2 + abs(e[1] - s[1]) ** 2) ** 0.5 < CONNECTS_TO_GRANULARITY:
                return True

            # CHECK INFREESPACE
            if not RRTGraphNode(m[0], m[1], self.nearest_neighbours).inFreespace():
                return False 
            
            queue.append((s, m))
            queue.append((m, e))

# Performs rrt on the map stored in self.wallpts
def rrt(start, goal, nearest_neighbours):
    print("\nRRT Started!")

    # Start the tree with the start (set no parent just in case).
    start.parent = None
    tree = [start]

    # Function to attach a new node to an existing node: attach the
    # parent, add to the tree, and show in the figure.
    def addtotree(oldnode, newnode):
        newnode.parent = oldnode
        tree.append(newnode)

    # Loop - keep growing the tree.
    while True:
        # Determine the target state.
        if np.random.uniform() < 0.05:
            target = goal
        else:
            target = RRTGraphNode(random.randint(xmin, xmax), random.randint(ymin, ymax), nearest_neighbours)

        # Directly determine the distances to the target node.
        distances = np.array([node.distance(target) for node in tree])
        index     = np.argmin(distances)
        nearnode  = tree[index]
        d         = distances[index]

        # Determine the next node.
        if d == 0.0:
            continue
        nextnode = RRTGraphNode(int(nearnode.u + dstep / d * (target.u - nearnode.u)),
                        int(nearnode.v + dstep / d *  (target.v - nearnode.v)), nearest_neighbours)

        # Check whether to attach.
        if nearnode.connectsTo(nextnode) and nextnode.inFreespace() and nextnode.u >= xmin and nextnode.u <= xmax and nextnode.v >= ymin and nextnode.v <= ymax:
            addtotree(nearnode, nextnode)

            # If within dstep, also try connecting to the goal.  If
            # the connection is made, break the loop to stop growing.
            if nextnode.distance(goal) <= dstep and nextnode.connectsTo(goal):
                addtotree(nextnode, goal)
                break

        # Check whether we should abort (tree has gotten too large).
        if (len(tree) >= Nmax):
            print(f"PLANNER: RRT FAILED TO FIND A PATH IN {Nmax} RRTGraphNodes!!!")
            return None

    # Build and return the path.
    path = [goal]
    while path[0].parent is not None:
        path.insert(0, path[0].parent)
    
    # Post process the path
    print(f'Pre-processed RRT Path of length {len(path)} FOUND!')
    post_process(path)
    return path
    
# Post Process the Path
def post_process(path):
    i = 0
    while (i < len(path)-2):
        if path[i].connectsTo(path[i+2]):
            path.pop(i+1)
        else:
            i = i+1

#
#   Planner Node Class
#
class PlannerNode(Node):
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
        self.got_map = False
        self.started = False
        self.current_pose = None
        self.waypoints = []
        self.current_waypoint = None
        self.goalx, self.goaly = None, None
        self.goalz, self.goalw = None, None
        
        # Publishers
        self.pubwaypoint = self.create_publisher(PoseStamped, 'waypoint', 1) # Waypoints
        self.pub_markers = self.create_publisher(Marker, "markers", 10)      # Markers for Waypoints
        self.pub_get_global_waypoint = self.create_publisher(Bool, 'get_global_waypoint', 1)

        # Subscribers
        # Create a subscriber for the map data.  Note this topic uses
        # a quality of service with durability TRANSIENT_LOCAL
        # allowing new subscribers to get the last sent message.
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        # self.submap = self.create_subscription(OccupancyGrid, '/map', self.cb_mapmsg, quality)

        self.subuncertainty_map = self.create_subscription(OccupancyGrid, 'uncertainty_map', self.cb_uncertainty_map, quality)

        self.subrobotpose = self.create_subscription(PoseStamped, 'pose', self.cb_robotpose, 10)

        self.subnextwaypoint = self.create_subscription(Bool, 'next_waypoint', self.cb_nextwaypoint, 1)
        self.subnewrrt = self.create_subscription(Bool, 'new_rrt', self.cb_newrrt, 1)

        self.subgoal = self.create_subscription(PoseStamped, 'goal_pose', self.cb_goalmsg, 10)

        self.substartrobot = self.create_subscription(Bool, '/start_robot', self.cb_startrobot, 1)

        # Report and return.
        self.get_logger().info("Planner running")

    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
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
        self.started = True

        msg = Bool()
        msg.data = True
        self.pub_get_global_waypoint.publish(msg)
        self.get_logger().info("Requesting for global waypoint")

    def cb_newrrt(self, msg):     
        msg = PoseStamped()
        msg.pose.position.x = float(self.goalx)
        msg.pose.position.y = float(self.goaly)
        msg.pose.position.z = float(self.goalz)
        msg.pose.orientation.w = float(self.goalw)
        
        msg.header.stamp = msg.header.stamp
        msg.header.frame_id = 'map'
        self.get_logger().info('Planner got stuck robot request for new RRT.')
        self.cb_goalmsg(msg)

    # When drive_auto sends a Bool indicating it has reached its current waypoint,
    # send it a new waypoint if we have one

    # If no waypoints, request for a new goal pose
    def cb_nextwaypoint(self, msg):
        if self.waypoints:
            self.current_waypoint = self.waypoints.pop(0)

            # Publish the waypoint - do we care about theta???
            waypoint_msg = PoseStamped()
            waypoint_msg.pose.position.x = self.current_waypoint[0]
            waypoint_msg.pose.position.y = self.current_waypoint[1]
            waypoint_msg.pose.position.z = 0.0
            
            if not self.waypoints:
                # Last waypoint, so send orientation
                waypoint_msg.pose.orientation.z = self.goalz
                waypoint_msg.pose.orientation.w = self.goalw
                      
            waypoint_msg.header.frame_id = 'map'
            self.pubwaypoint.publish(waypoint_msg)
            self.get_logger().info("PUBLISHING WAYPOINT")

        else:
            msg = Bool()
            msg.data = True
            self.pub_get_global_waypoint.publish(msg)
            self.get_logger().info("Requesting new global position")

    def compute_nearest_neighbours(self):
        WALLTHRESHOLD_NEAREST = 80
        wallpts = np.zeros((0,2), dtype=np.int)
        for v in range(self.height):
            for u in range(self.width):
                if self.uncertainty_map[v,u] > WALLTHRESHOLD_NEAREST:
                # Also check the adjacent pixels in a 3x3 grid.
                    adjacent = self.uncertainty_map[max(0,v-1):min(self.height, v+2), max(0,u-1):min(self.width, u+2)]
                    if not np.all(adjacent > WALLTHRESHOLD_NEAREST):
                        wallpts = np.vstack([wallpts, np.array([u,v])])
            
        # Returns the nearest wall point for coordinates u, v
        def nearestwallpt(u,v):
            return wallpts[np.argmin(np.sum((np.array([u,v]) - wallpts)**2, axis=1))]

        # Find the nearest wall point for every point in the map and store in wallptmap
        wallptmap = np.zeros((self.height, self.width, 2))
        for v in range(self.height):
            for u in range(self.width):
                wallptmap[v,u] = nearestwallpt(u,v)

        self.nearest_neighbours = wallptmap
    
    def cb_goalmsg(self, msg):
        assert msg.header.frame_id == 'map', "Planner: Message not in map frame!!!!"

        if self.got_map:
            # RECOMPUTE NEAREST NEIGHBOURS 
            self.compute_nearest_neighbours()
            pose = msg.pose
        
            # Start is robot position, goal is pose, in map frame
            startx, starty = self.current_pose[0], self.current_pose[1]
            self.goalx, self.goaly = pose.position.x, pose.position.y
            self.goalz, self.goalw = float(pose.orientation.z), float(pose.orientation.w)

            # Get start and goal in grid frame
            startu, startv = self.grid_coords_from_map(startx, starty)
            goalu, goalv = self.grid_coords_from_map(self.goalx, self.goaly)

            # Get the waypoints from RRT
            self.waypoints = rrt(RRTGraphNode(startu, startv, self.nearest_neighbours), RRTGraphNode(goalu, goalv, self.nearest_neighbours), self.nearest_neighbours)
            self.get_logger().info('RRT Computed')
            if self.waypoints:
                # Get waypoints into map frame
                for i, pt in enumerate(self.waypoints):
                    x, y = self.map_coords_from_grid(float(pt.u), float(pt.v))
                    self.waypoints[i] = x, y

                self.get_logger().info(f'Post-Processed RRT Path in map coords:\n{self.waypoints}')
                    
                # Publish a marker message to show the waypoint trail
                markermsg = Marker()
                markermsg.header.frame_id = "map"
                markermsg.header.stamp = msg.header.stamp
                markermsg.ns = "rrt_waypoints"
                markermsg.id = 0
                markermsg.type = Marker.POINTS
                markermsg.action = Marker.ADD # Should be same as Marker.MODIFY
                markermsg.pose = Pose()
                markermsg.scale = Vector3(x=0.12, y=0.12, z=0.12)
               
                markermsg.points = []
                markermsg.colors = []

                for (i, point) in enumerate(self.waypoints):
                    markermsg.points.append(Point(x=point[0], y=point[1], z=0.0))
                    
                    col = ColorRGBA()
                    col.r = 0.0
                    col.g = 0.0
                    col.b = 1.0 - float(i / len(self.waypoints))
                    col.a = 1.0
                    markermsg.colors.append(col)

                self.pub_markers.publish(markermsg)

                if self.started:
                    # Only dequeue and publish if we have entered the go command in pose_toggle
                    # Pop current waypoint
                    self.current_waypoint = self.waypoints.pop(0)

                    # Publish current waypoint
                    waypoint_msg = PoseStamped()
                    waypoint_msg.pose.position.x = float(self.current_waypoint[0])
                    waypoint_msg.pose.position.y = float(self.current_waypoint[1])
                    waypoint_msg.pose.position.z = 0.0

                    # Publish an orientation for the last waypoint
                    if not self.waypoints:
                        if self.goalz == 0.0 and self.goalw == 0.0:
                            # If this was called from global_explore, there is
                            # no desired orientation
                            dx, dy = self.goalx - self.current_pose.x, self.goaly - self.current_pose.y
                            theta = atan2(dy, dx)
                            waypoint_msg.pose.orientation.z = sin(theta / 2)
                            waypoint_msg.pose.orientation.w = cos(theta / 2)
                        else:
                            # Send the desired goal if there was one
                            waypoint_msg.pose.orientation.z = self.goalz
                            waypoint_msg.pose.orientation.w = self.goalw
                    waypoint_msg.header.stamp = msg.header.stamp
                    waypoint_msg.header.frame_id = 'map'

                    self.get_logger().info(f"z: {waypoint_msg.pose.orientation.z} w: {waypoint_msg.pose.orientation.w}")
                    self.get_logger().info("PUBLISHING WAYPOINT")
                    self.pubwaypoint.publish(waypoint_msg)   

            else:
                # NO RRT PATH TO POINT SO ASK FOR NEW GLOBAL 
                self.get_logger().info("RRT Path not found to point, requesting for new global waypoint")
                msg = Bool()
                msg.data = True
                self.pub_get_global_waypoint.publish(msg)
               

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = PlannerNode('planner')

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
