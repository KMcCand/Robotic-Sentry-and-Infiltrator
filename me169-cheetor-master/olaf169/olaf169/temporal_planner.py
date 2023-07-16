#!/usr/bin/env python3
#
#   temporal_planner.py
#
#   Global Temporal Planning Node. This node
#   (a) Subscribes to goal poses from map, waypoint requests from drive_auto, and robot poses from /pose
#   (b) Publishes waypoints for drive_auto (local planner) to drive to without going near the sentry trajectory
#   (c) Maintains a list of these waypoints to get to the goal point
#   
#   Node: /temporal_planner
#   Publish:
#           /waypoint                   geometry_msgs/PoseStamped
#           /marker                     visualization_msgs/Marker
#           /new_target                 std_msgs/Bool
#   Subscribe:
#           /goal_pose                  geometry_msgs/PoseStamped
#           /next_waypoint              std_msgs/Bool
#           /pose                       geometry_msgs/PoseStamped
#           /map                        OccupancyGrid

import time
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
from math import sqrt

from rclpy.executors            import MultiThreadedExecutor
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup

# TEMPORAL RRT CONSTANTS
##############################################################################
# RRT Parameters
Nmax = 300
dstep = 20
xmin, xmax = 0, 300
ymin, ymax = 0, 300
RRT_MAX_PATH_SEGMENT_DIST = 50 # Maximum allowed distance_space between two nodes in RRT path

# Global constants
CONNECTS_TO_GRANULARITY = 6 # Number of grids to skip when checking connectsTo
GRIDS_TO_WALL = 8 # Robot half width in grids
RES = 0.0254
LOAD_MAP = True

# RRT Temporal planning constants
VEL_MIN = 5
VEL_MAX = 20
TURN_TIME = 0.5

# Sentry constants
SENTRY_LAG = 3
ALLOWED_SENTRY_RADIUS = 30
SAMPLE_TIME = 10
SENTRY_REACH = 10
SENTRY_CONE_WIDTH_SCALE = 2/3
TIME_GRANULARITY = 0.2

COMPUTATIONAL_ESTIMATE = 15

# WAYPOINTS = [(2.0, -0.5), (0.0, -0.3), (0.0, 0.2), (1.5, 1.0), (2.5, 0.8)] old waypoints

WAYPOINTS = np.array([[ 2.54,  0.28],
       [ 2.05, -0.74],
       [ 0.95, -0.99],
       [ 0.07, -0.28],
       [ 0.07,  0.84],
       [ 0.95,  1.55],
       [ 2.05,  1.3 ]]) + 0.08

small_polygon = np.array([[ 2.24,  0.28],
       [ 1.86, -0.5 ],
       [ 1.02, -0.69],
       [ 0.34, -0.15],
       [ 0.34,  0.71],
       [ 1.02,  1.25],
       [ 1.86,  1.06]]) + 0.08

SENTRY_EXPECTED_SPEED = 0.3 

# Times is the array of sentry time to travel between points, based on constant velocity
# Times[0] is the time to travel from WAYPOINTS[0] to WAYPOINTS[1]
TIMES = []
for i in range(len(WAYPOINTS)):
    curr_pt = WAYPOINTS[i]
    next_pt = WAYPOINTS[(i + 1) % len(WAYPOINTS)]
    dist = sqrt((curr_pt[0] - next_pt[0]) ** 2 + (curr_pt[1] - next_pt[1]) ** 2)
    TIMES.append(dist / SENTRY_EXPECTED_SPEED) # Expected time starting at waypoint[i] going to waypoint[i+1]

#########################################################################################
# End temporal RRT constants

class RRTGraphNode():
    # Initialize with coordinates.
    def __init__(self, u, v, t, nearest_neighbours, get_sentry_position_function):
        # Define/remember the state/coordinates (x,y).
        self.u = u
        self.v = v
        self.t = t

        self.x = None
        self.y = None 
        self.dt = None
        # Clear any parent information.
        self.parent = None
        
        self.nearest_neighbours = nearest_neighbours
        self.get_sentry_position_function = get_sentry_position_function

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return (f"<Point {self.u}, {self.v}, {self.dt}>")

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate(self, other, alpha):
        return RRTGraphNode(int(self.u + alpha * (other.u - self.u)),
                    int(self.v + alpha * (other.v - self.v)),
                    self.t + alpha * (other.t - self.t),
                    self.nearest_neighbours,
                    self.get_sentry_position_function)

    # Compute the relative distance to another node.
    def distance(self, other):
        if self.t and other.t:
            return sqrt((self.u - other.u)**2 + (self.v - other.v)**2 + 0.1*(self.t - other.t)**2)
        else:
            return ((self.u - other.u)**2 + (self.v - other.v)**2) ** (0.5)

    def distance_space(self, other):
        return sqrt((self.u - other.u)**2 + (self.v - other.v)**2)

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        return (self.u, self.v)

    def currently_collided_with_sentry(self):
        # Check if too close to sentry
        su, sv = self.get_sentry_position_function(self.t)
        too_close_to_sentry = sqrt((su - self.u) ** 2 + (sv - self.v) ** 2) <= ALLOWED_SENTRY_RADIUS

        # Check if within the sentry's view
        # u, v = self.grid_coords_from_map(self.current_pose)
        # sentry_orientation = self.sentry_orientation(self.t)

        return too_close_to_sentry # or in_sentry_view

    def in_free_space_only(self):
        if (self.u <= xmin or self.u >= xmax or
            self.v <= ymin or self.v >= ymax):
            print(f"Planner RRT InFreeSpace: pts outside map {self.u, self.v}")
            return False

        u_wall, v_wall = self.nearest_neighbours[self.v, self.u] # Nearest wall point in Grid Frame
        return self.distance_space(RRTGraphNode(u_wall, v_wall, None, self.nearest_neighbours, self.get_sentry_position_function)) > GRIDS_TO_WALL # TIME CAN BE NONE FOR distance_space
        
    ######################
    # Collision functions:
    # Check whether in free space.
    def inFreespace(self):
        if (self.u <= xmin or self.u >= xmax or
            self.v <= ymin or self.v >= ymax):
            print(f"Planner RRT InFreeSpace: pts outside map {self.u, self.v}")
            return False

        if self.currently_collided_with_sentry():
            return False

        # Get nearest wall point by accessing self.wallptmap in grid frame
        u_wall, v_wall = self.nearest_neighbours[self.v, self.u] # Nearest wall point in Grid Frame
        return self.distance_space(RRTGraphNode(u_wall, v_wall, None, self.nearest_neighbours, self.get_sentry_position_function)) > GRIDS_TO_WALL # TIME CAN BE NONE FOR distance_space

    def check_sentry_collision(self, other):
        ''' 
        DEPRACATED in favor of calling infreespace, which contains this check
        '''

        return False
        '''
        Checks between self and other to see if sentry will catch you on the way, doesn't account for turn time TODO: Krish
        '''
        temp_t = other.t
        
        if other.t == None: # at the goal checking sentry collision
            temp_t = self.t + self.distance_space(other) / VEL_MAX

        # Returns true if sentry collides with line segment
        for alpha in np.arange(0.0, 1.0, 0.01): # checks 10 intermediate points in connects to
            inter_pt = self.u + alpha*(other.u-self.u), self.v + alpha*(other.v-self.v)
            inter_t = self.t + alpha*(temp_t-self.t)
            sentry_at_t = get_sentry_position(inter_t - SENTRY_LAG)
            pt1, pt2, pt3 = get_sentry_triangle(inter_t - SENTRY_LAG)

            # colliding with sentry check
            if (sentry_at_t[0] - inter_pt[0]) ** 2 + (sentry_at_t[1] - inter_pt[1]) ** 2 < (ALLOWED_SENTRY_RADIUS+3) ** 2:
                return True # collides
                
            # check collision with sentry's field of view
            if self.triangle_intersect(pt1[0], pt1[1], pt2[0], pt2[1], pt3[0], pt3[1], inter_pt[0], inter_pt[1]):
                return True
            # if PointInTriangle(inter_pt, get_sentry_triangle(inter_t - SENTRY_LAG)):
            #     return True

        return False
    
    def triangle_intersect(self, x1, y1, x2, y2, x3, y3, xp, yp):
        '''DEPRACATED'''
        c1 = (x2-x1)*(yp-y1)-(y2-y1)*(xp-x1)
        c2 = (x3-x2)*(yp-y2)-(y3-y2)*(xp-x2)
        c3 = (x1-x3)*(yp-y3)-(y1-y3)*(xp-x3)
        if (c1<0 and c2<0 and c3<0) or (c1>0 and c2>0 and c3>0):
            return True
        else:
            return False

    def connectsTo(self, other, alt_time=None):
        # IF ALT TIME IS NOT NONE, THAT MEANS THAT IT IS A GOAL NODE
        if other.t == None:
            other_time = alt_time
        else:
            other_time = other.t

        if self.t + TURN_TIME + (self.distance_space(other) / VEL_MAX) > other_time:
            # We cannot possibly get there
            return False
                
        # Find out if we will cross obstacles
        return self.interpolateConnectsTo((self.u, self.v, self.t), (other.u, other.v, other_time))

    def get_pos(self, s, e, t):
        if s[2] + TURN_TIME > t:
            return (s[0], s[1])

        else:
            total_movement_time = e[2] - s[2] - TURN_TIME
            movement_time = t - s[2] - TURN_TIME
            movement_distance = e[0] - s[0], e[1] - s[1]        

            return (movement_time / total_movement_time) * movement_distance[0] + s[0], (movement_time / total_movement_time) * movement_distance[1] + s[1]

        
    def interpolateConnectsTo(self, start, end):
        assert not (start[2] is None) and not (end[2] is None), "None passed into the times for interpolate connects to" 
            
        queue = [(start[2], end[2])]

        while len(queue) != 0:
            st, et = queue.pop(0)
            mt = (st + et) / 2
            pos = self.get_pos(start, end, mt)
            
            if not RRTGraphNode(int(pos[0]), int(pos[1]), mt, self.nearest_neighbours, self.get_sentry_position_function).inFreespace():
                return False 

            queue.append((st, mt))
            queue.append((mt, et))

            if (et - st < TIME_GRANULARITY):
                return True

# Performs rrt on the map stored in self.wallpts
def temporal_rrt(start, goal, nearest_neighbours, logger, get_sentry_position_function):
    rrt_in_function_start = time.time()
    # Check that goal is in free space
    if not goal.in_free_space_only():
        logger("TEMPORAL PLANNER: GOAL IS NOT IN FREE SPACE (not counting sentry movements). Returning None.")
        return None
    
    # If start is not in free space, just send a waypoint behind us
    # not implemented yet

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
        if np.random.uniform() < 0.2:
            target = goal
        else:
            # Passing None as time, we query time after finding nearest nodes
            target = RRTGraphNode(random.randint(xmin, xmax), random.randint(ymin, ymax), None, nearest_neighbours, get_sentry_position_function)

        # Directly determine the distances to the target node.
        distances = np.array([node.distance_space(target) for node in tree])
        index     = np.argmin(distances)
        nearnode  = tree[index]
        d         = distances[index]

        # Determine the next node.
        if d == 0.0:
            continue

        next_node_t = nearnode.t + TURN_TIME + random.uniform(dstep / VEL_MAX, dstep / VEL_MIN)           
        nextnode = RRTGraphNode(int(nearnode.u + dstep / d * (target.u - nearnode.u)),
                        int(nearnode.v + dstep / d *  (target.v - nearnode.v)),
                        next_node_t,
                        nearest_neighbours, get_sentry_position_function)

        # Check whether to attach.
        if nextnode.u >= xmin and nextnode.u <= xmax and nextnode.v >= ymin and nextnode.v <= ymax:
            if nextnode.inFreespace():
                if nearnode.connectsTo(nextnode):
                    addtotree(nearnode, nextnode)

                    # If within dstep, also try connecting to the goal.  If
                    # the connection is made, break the loop to stop growing.
                    dist_to_goal = nextnode.distance_space(goal)
                    goal_alt_time = nextnode.t + TURN_TIME + random.uniform(dist_to_goal / VEL_MAX, dist_to_goal / VEL_MIN)
                    if dist_to_goal <= dstep and nextnode.connectsTo(goal, alt_time=goal_alt_time):
                        # After this point, the goal_node has a time
                        goal.t = goal_alt_time
                        addtotree(nextnode, goal)
                        break

        if time.time() - rrt_in_function_start >= 12:
            logger(f"TEMPORAL PLANNER: TEMPORAL RRT FAILED TO FIND A PATH IN 12 SECONDS!!!")
            # RRT HAS TAKEN TOO LONG, REPLAN
            return None

        # Check whether we should abort (tree has gotten too large).
        if (len(tree) >= Nmax):
            logger(f"TEMPORAL PLANNER: TEMPORAL RRT FAILED TO FIND A PATH IN {Nmax} RRTGraphNodes!!!")
            return None
        elif len(tree) % 100 == 0:
            logger(f'   Temporal RRT tree size: {len(tree)}')

    # Build and return the path.
    path = [goal]
    while path[0].parent is not None:
        path.insert(0, path[0].parent)
    
    # Post process the path
    logger(f'Pre-processed Temporal RRT Path of length {len(path)} FOUND!')
    post_process(path)
    temporal_add_intermediate_nodes(path)
    return path
    
# Post Process the Path
def post_process(path):
    i = 0
    while (i < len(path)-2):
        if path[i].connectsTo(path[i+2]):
            path.pop(i+1)
        else:
            i = i+1

# Make sure that no two nodes are further apart than RRT_MAX_PATH_SEGMENT_DIST
def temporal_add_intermediate_nodes(path):
    i = 0
    while i < len(path) - 1:
        if path[i].distance_space(path[i+1]) > RRT_MAX_PATH_SEGMENT_DIST:
            # Insert a node in the middle, and stay at the current node
            path.insert(i+1, path[i].intermediate(path[i+1], 0.5))
        else:
            # Go to the next node
            i += 1

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
        self.started = True # Ignore 3 commands for now, MUUUUUSTTTTTTT CHANGE TO FALSE LATER!!!!!!
        self.current_pose = None
        self.waypoints = []
        self.current_waypoint = None
        self.goalx, self.goaly = None, None
        self.goalz, self.goalw = None, None
        
        self.motion_start_time = None
        self.waiting_for_global_waypoint = False 

        # Publishers
        self.pubwaypoint = self.create_publisher(PoseStamped, 'waypoint', 1) # Waypoints
        self.pub_markers = self.create_publisher(Marker, "markers", 10)      # Markers for Waypoints
        self.pub_get_global_waypoint = self.create_publisher(Bool, 'get_global_waypoint', 1)

        self.pub_sentry_position_estimate = self.create_publisher(PoseStamped, 'carrot_final', 10) # Sentry position estimate
        self.pub_sentry_position_estimate_marker = self.create_publisher(Marker, 'carrot_final_marker', 10) # Sentry position estimate marker

        # Subscribers
        fastgroup = MutuallyExclusiveCallbackGroup()
        slowgroup = MutuallyExclusiveCallbackGroup()
        othergroup = MutuallyExclusiveCallbackGroup()

        self.publish_waypoint_timer = self.create_timer(0.05, self.cb_waypoint_publish)
        self.timer = self.create_timer(0.03, self.publish_sentry_marker)
        
        # Create a subscriber for the map data.  Note this topic uses
        # a quality of service with durability TRANSIENT_LOCAL
        # allowing new subscribers to get the last sent message.
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        # self.submap = self.create_subscription(OccupancyGrid, '/map', self.cb_mapmsg, quality)

        self.subuncertainty_map = self.create_subscription(OccupancyGrid, 'uncertainty_map', self.cb_uncertainty_map, quality, callback_group=othergroup)

        self.subrobotpose = self.create_subscription(PoseStamped, 'pose', self.cb_robotpose, 10, callback_group=fastgroup)

        # Depracted -- planner now sends waypoints at the correct time and drive_auto just listens
        # self.subnextwaypoint = self.create_subscription(Bool, 'next_waypoint', self.cb_nextwaypoint, 1)
        self.subnewrrt = self.create_subscription(Bool, 'new_rrt', self.cb_newrrt, 1, callback_group=slowgroup)

        self.subgoal = self.create_subscription(PoseStamped, 'goal_pose', self.cb_goalmsg, 10, callback_group=slowgroup)

        self.substartrobot = self.create_subscription(Bool, '/start_robot', self.cb_startrobot, 1, callback_group=fastgroup)

        # Report and return.
        self.get_logger().info("Temporal Planner running")

    def publish_sentry_marker(self):

        if not self.got_map:
            return 

        t = time.time()
        u, v = self.get_sentry_position_smallpoly(t)
        x, y = self.map_coords_from_grid(u, v)
        sentry_orientation = self.sentry_orientation(t)

        angle = atan2(sentry_orientation[1], sentry_orientation[0])
        w = np.cos(angle / 2)
        z = np.sin(angle / 2)
        
        # Publish the sentry's current position, orientation as a PoseStamped
        posemsg = PoseStamped()

        posemsg.pose.position.x = float(x)
        posemsg.pose.position.y = float(y)
        posemsg.pose.orientation.z = float(z)
        posemsg.pose.orientation.w = float(w)
       
        posemsg.header.stamp = self.get_clock().now().to_msg()
        posemsg.header.frame_id = 'map'

        self.pub_sentry_position_estimate.publish(posemsg)

        # Publish a marker on the sentry's current position, orientation
        markermsg = Marker()
        markermsg.header.frame_id = "map"
        markermsg.header.stamp = self.get_clock().now().to_msg()
        markermsg.ns = "sentry_estimate"
        markermsg.id = 0
        markermsg.type = Marker.CYLINDER
        markermsg.action = Marker.ADD # Should be same as Marker.MODIFY
        markermsg.pose = Pose()
        markermsg.pose.position.x = x
        markermsg.pose.position.y = y
        markermsg.pose.position.z = 0.0

        markermsg.color.a = 0.3
        markermsg.color.g = 1.0

        markermsg.pose.orientation.z = np.sin(angle / 2)
        markermsg.pose.orientation.w = np.cos(angle / 2)

        markermsg.scale = Vector3(x=ALLOWED_SENTRY_RADIUS * RES, y=ALLOWED_SENTRY_RADIUS * RES, z=0.0)
        
        self.pub_sentry_position_estimate_marker.publish(markermsg)

    def get_sentry_position(self, t):
        t -= SENTRY_LAG
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

        # return self.grid_coords_from_map(1.03, 0.68) # DELETE THIS LATER

        return self.grid_coords_from_map(x, y)
    
    def get_sentry_position_smallpoly(self, t):
        t -= SENTRY_LAG
        num_seg = len(WAYPOINTS)
        t = t % sum(TIMES)

        cumsum_times = np.cumsum(np.insert(TIMES, 0, 0))

        next_pt_idx = np.argmax(cumsum_times > t) % num_seg # point you're going towards
        last_pt_idx = (next_pt_idx - 1) % num_seg

        segment_duration = float(TIMES[last_pt_idx])
        percent_through_motion = float(t  - cumsum_times[last_pt_idx]) / segment_duration

        past_pt = small_polygon[last_pt_idx]
        next_pt = small_polygon[next_pt_idx]

        x = past_pt[0] + percent_through_motion * (next_pt[0] - past_pt[0])
        y = past_pt[1] + percent_through_motion * (next_pt[1] - past_pt[1])

        # return self.grid_coords_from_map(1.03, 0.68) # DELETE THIS LATER

        return self.grid_coords_from_map(x, y)

    def get_sentry_position_map(self, t):
        t -= SENTRY_LAG
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

        # return self.grid_coords_from_map(1.03, 0.68) # DELETE THIS LATER

        return (x, y)


    def sentry_orientation(self, t):
        sentry_pos = self.get_sentry_position(t)
        carot_pos = self.get_sentry_position(t + SENTRY_LAG)
        thetas = np.array((carot_pos[0] - sentry_pos[0], carot_pos[1] - sentry_pos[1]))
        return thetas / sqrt(thetas[0]**2 + thetas[1] ** 2)

    def get_sentry_triangle(self, t):
        orientation = SENTRY_REACH * self.sentry_orientation(t)
        position = self.get_sentry_position(t)
        tip = (position[0] + orientation[0], position[1] + orientation[1])

        pt1 = (tip[0] + SENTRY_CONE_WIDTH_SCALE * orientation[1], tip[1] - SENTRY_CONE_WIDTH_SCALE * orientation[0])
        pt2 = (tip[0] - SENTRY_CONE_WIDTH_SCALE * orientation[1], tip[1] + SENTRY_CONE_WIDTH_SCALE * orientation[0])
        pt3 = (position[0], position[1])

        return pt1, pt2, pt3

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
        assert msg.header.frame_id == 'map', "Temporal Planner: Robot Pose Message not in map frame!!!!"
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        self.current_pose = (x, y)

    def cb_startrobot(self, msg):
        self.started = True

        msg = Bool()
        msg.data = True
        self.pub_get_global_waypoint.publish(msg)
        self.get_logger().info("Temporal Planner Requesting for global waypoint")

    def cb_newrrt(self, msg):     
        msg = PoseStamped()
        msg.pose.position.x = float(self.goalx)
        msg.pose.position.y = float(self.goaly)
        msg.pose.position.z = float(self.goalz)
        msg.pose.orientation.w = float(self.goalw)
        
        msg.header.stamp = msg.header.stamp
        msg.header.frame_id = 'map'
        self.get_logger().info('Temporal Planner got stuck robot request for new RRT.')
        self.cb_goalmsg(msg)

    # When drive_auto sends a Bool indicating it has reached its current waypoint,
    # send it a new waypoint if we have one

    # DEPRACTED -- PLANNER NOW SENDS WAYPOINTS WHENEVER, DRIVE_AUTO JUST LISTENS
    # If no waypoints, request for a new goal pose
    # def cb_nextwaypoint(self, msg):
    #     if self.waypoints:
    #         self.current_waypoint = self.waypoints.pop(0)

    #         # Publish the waypoint - do we care about theta???
    #         waypoint_msg = PoseStamped()
    #         waypoint_msg.pose.position.x = self.current_waypoint[0]
    #         waypoint_msg.pose.position.y = self.current_waypoint[1]
    #         waypoint_msg.pose.position.z = 0.0
            
    #         if not self.waypoints:
    #             # Last waypoint, so send orientation
    #             waypoint_msg.pose.orientation.z = self.goalz
    #             waypoint_msg.pose.orientation.w = self.goalw
                      
    #         waypoint_msg.header.frame_id = 'map'
    #         self.pubwaypoint.publish(waypoint_msg)
    #         self.get_logger().info("TEMPORAL PLANNER PUBLISHING WAYPOINT")

    #     else:
    #         msg = Bool()
    #         msg.data = True
    #         self.pub_get_global_waypoint.publish(msg)
    #         self.get_logger().info("Requesting new global position")

    def compute_nearest_neighbours(self):
        WALLTHRESHOLD_NEAREST = 40
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
        
    def cb_waypoint_publish(self):
        if self.motion_start_time: # Maybe check if we have waypoints available? Else do we care?
            time_into_motion = time.time() - self.motion_start_time
            if time_into_motion > self.current_waypoint.dt:
                # Publish new waypoint and reset self.motion_start_time
                if self.waypoints:
                    self.get_logger().info(f"time_into_motion: {time_into_motion} current_waypoint dt: {self.current_waypoint.dt}")
                    self.current_waypoint = self.waypoints.pop(0)

                    # Publish the waypoint - do we care about theta???
                    waypoint_msg = PoseStamped()
                    waypoint_msg.pose.position.x = float(self.current_waypoint.x)
                    waypoint_msg.pose.position.y = float(self.current_waypoint.y)
                    waypoint_msg.pose.position.z = 0.0
                    
                    waypoint_msg.pose.orientation.x = float(self.current_waypoint.dt)
                    if not self.waypoints:
                        # Last waypoint, so send orientation
                        waypoint_msg.pose.orientation.z = self.goalz
                        waypoint_msg.pose.orientation.w = self.goalw
                    else:
                        waypoint_msg.pose.orientation.z = 0.0
                        waypoint_msg.pose.orientation.w = 0.0
                            
                    waypoint_msg.header.frame_id = 'map'
                    self.pubwaypoint.publish(waypoint_msg)
                    self.motion_start_time = time.time()
                    self.get_logger().info("TEMPORAL PLANNER PUBLISHING NEXT WAYPOINT")

                elif not self.waiting_for_global_waypoint:
                    msg = Bool()
                    msg.data = True
                    self.pub_get_global_waypoint.publish(msg)
                    self.get_logger().info("Requesting new global position")
                    self.waiting_for_global_waypoint = True
                
    
    def cb_goalmsg(self, msg):
        self.get_logger().info("Temporal planner got goal messsage.")
        self.current_waypoint = None
        self.waypoints = []

        rrt_start_time = time.time()
        assert msg.header.frame_id == 'map', "Temporal Planner: Message not in map frame!!!!"

        if self.got_map:
            self.waiting_for_global_waypoint = False

            # RECOMPUTE NEAREST NEIGHBOURS 
            self.get_logger().info("Temporal Planner: Recomputing nearest wall points...")
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
            startnode = RRTGraphNode(startu, startv, COMPUTATIONAL_ESTIMATE + rrt_start_time, self.nearest_neighbours, self.get_sentry_position)
            goalnode = RRTGraphNode(goalu, goalv, None, self.nearest_neighbours, self.get_sentry_position)
            self.get_logger().info("Starting Temporal RRT...")
            self.waypoints = temporal_rrt(startnode, goalnode, self.nearest_neighbours, self.get_logger().info, self.get_sentry_position)

            rrt_computational_time = time.time() - rrt_start_time
            self.get_logger().info(f'Temporal RRT Computed in {rrt_computational_time}.')

            if self.waypoints:
                # Get waypoints into map frame
                for i, pt in enumerate(self.waypoints):
                    x, y = self.map_coords_from_grid(float(pt.u), float(pt.v))
                    pt.x = x
                    pt.y = y
                    pt.dt = COMPUTATIONAL_ESTIMATE - rrt_computational_time if i == 0 else self.waypoints[i].t - self.waypoints[i - 1].t

                self.get_logger().info(f'Post-Processed Temporal RRT Path in (grid, grid, dt) form:\n{self.waypoints}')
                    
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
                    markermsg.points.append(Point(x=point.x, y=point.y, z=0.0))
                    
                    col = ColorRGBA()
                    col.r = 0.0
                    col.g = 0.0
                    col.b = 1.0 - float(i / len(self.waypoints))
                    col.a = 1.0
                    markermsg.colors.append(col)

                self.pub_markers.publish(markermsg)

                # Only dequeue and publish if we have entered the go command in pose_toggle
                if self.started:
                    # Pop current waypoint
                    self.current_waypoint = self.waypoints.pop(0)

                    # Publish current waypoint
                    waypoint_msg = PoseStamped()
                    waypoint_msg.pose.position.x = float(self.current_waypoint.x)
                    waypoint_msg.pose.position.y = float(self.current_waypoint.y)
                    waypoint_msg.pose.position.z = 0.0

                    waypoint_msg.pose.orientation.x = float(self.current_waypoint.dt)
                    if not self.waypoints:
                        # Last waypoint, so send orientation
                        waypoint_msg.pose.orientation.z = self.goalz
                        waypoint_msg.pose.orientation.w = self.goalw
                    else:
                        waypoint_msg.pose.orientation.z = 0.0
                        waypoint_msg.pose.orientation.w = 0.0

                    waypoint_msg.header.stamp = msg.header.stamp
                    waypoint_msg.header.frame_id = 'map'
                    self.pubwaypoint.publish(waypoint_msg)
                    self.get_logger().info("TEMPORAL PLANNER PUBLISHED FIRST WAYPOINT")
                    self.motion_start_time = time.time()
            elif not self.waiting_for_global_waypoint:
                # NO RRT PATH TO POINT, ask global planner for new one
                self.get_logger().info("Temporal RRT Path not found to point, requesting for new global waypoint")
                msg = Bool()
                msg.data = True
                self.pub_get_global_waypoint.publish(msg)
                self.waiting_for_global_waypoint = True
               

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
