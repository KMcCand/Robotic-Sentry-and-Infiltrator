#!/usr/bin/env python3
#
#   temporal_prm.py
#
#   Global Temporal Planning Node. This node
#   (a) Subscribes to goal poses from map, waypoint requests from drive_auto, and robot poses from /pose
#   (b) Publishes waypoints for drive_auto (local planner) to drive to without going near the sentry trajectory
#   (c) Maintains a list of these waypoints to get to the goal point
#   
#   Node: /temporal_prm
#   Publish:
#           /waypoint                   geometry_msgs/PoseStamped
#           /markers                    visualization_msgs/Marker
#           /get_global_waypoint        std_msgs/Bool
#   Subscribe:
#           /goal_pose                  geometry_msgs/PoseStamped
#           /next_waypoint              std_msgs/Bool
#           /new_rrt                    std_msgs/Bool
#           /start_robot                std_msgs/Bool
#           /pose                       geometry_msgs/PoseStamped
#           /uncertainty_map            OccupancyGrid
# 

import rclpy
from std_msgs.msg import ColorRGBA
import traceback

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
from scipy.spatial import KDTree
import bisect
import math
from math import sqrt

# Temporal PRM Parameters
K = 20
N = 1500
T_SAMPLE_MAX = 30
xmin, xmax = 0, 300
ymin, ymax = 0, 300

# Global constants
CONNECTS_TO_GRANULARITY = 6 # Number of grids to skip when checking connectsTo
TIME_GRANULARITY = 0.5 # TIME GRANULARITY FOR TEMPORAL RRT

GRIDS_TO_WALL = 6 # Robot half width in grids
RES = 0.0254
LOAD_MAP = True

# Temporal PRM constants
SENTRY_LAG = 0.1
TURN_TIME = 1

VEL_MAX = (2.5) * 39.3 # input m/s in braces

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


######################################################################
#
#   Node Definition
#
class PRMSample:
    def __init__(self, u, v, t, nearest_neighbours):
        # Edges = set of neighbors.  You need to fill in.
        self.neighbors = set()
        self.nearest_neighbours = nearest_neighbours

        # Reset the A* search tree information
        self.reset()

        # Define/remember the state/coordinates (x,y).
        self.u = u
        self.v = v
        self.t = t
        self.x = None
        self.y = None

    def reset(self):
        # Clear the status, connection, and costs for the A* search tree.
        #   TRUNK:  done = True
        #   LEAF:   done = False, seen = True
        #   AIR:    done = False, seen = False
        self.done   = False
        self.seen   = False
        self.parent = []
        self.creach = 0
        self.ctogo  = math.inf
    
    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return (f"X: {self.u} Y: {self.v} T: {self.t}")

    # Define the "less-than" to enable sorting in A*. Use total cost estimate.
    def __lt__(self, other):
        return (self.creach + self.ctogo) < (other.creach + other.ctogo)

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate(self, other, alpha):
        return PRMSample(self.u + alpha * (other.u - self.u),
                    self.v + alpha * (other.v - self.v),
                    self.t+alpha * (other.t - self.t), self.nearest_neighbours)

    # Compute the relative distance to another node.
    def distance(self, other):
        if (self.t is None) or (other.t is None):
            return self.distance_space(other)

        return ((10 * (self.t - other.t)) ** 2 + (self.u - other.u) ** 2 + (self.v - other.v) ** 2) ** 1/2

    def distance_space(self, other):
        return ((self.u - other.u) ** 2 + (self.v - other.v) ** 2) ** 1/2

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        return (self.u, self.v, self.t)

    ###############
    # A* functions:
    # Actual and Estimated costs.
    def costToConnect(self, other):
        return self.distance(other)

    def costToGoEst(self, other):
        return self.distance(other)
    
    def collides_with_sentry(self):
        sentry_pos = get_sentry_position(self.t - SENTRY_LAG)
        return sqrt((sentry_pos[0] - self.u) ** 2 + (sentry_pos[1] - self.v) ** 2) < 1

    ################
    # PRM functions:
    # Check whether in free space.
    def inFreespace(self):
        # Get nearest wall point by accessing self.wallptmap in grid frame
        if not ((0 <= self.v and self.v < 300) and (0 <= self.u and self.u < 300)):        
            print(f"Temporal PRM inFreeSpace: pts outside map {self.u, self.v}")
            return False
        
        u_wall, v_wall = self.nearest_neighbours[self.v, self.u] # Nearest wall point in Grid Frame

        away_from_wall = self.distance_space(PRMSample(u_wall, v_wall, None, self.nearest_neighbours)) > GRIDS_TO_WALL
        away_from_sentry = not self.collides_with_sentry()
        return away_from_wall and away_from_sentry

    # IF YOU CONNECT TO OTHER NODE
    def connectsTo(self, other):
        if (other.t != None and self.t != None):
            if (self.t + TURN_TIME + (self.distance_space(other) / VEL_MAX) > other.t):
                # Cannot possible make it there in time even driving at max speed
                return False

        return self.interpolateConnectsTo((self.u, self.v, self.t), (other.u, other.v, other.t))

    def get_pos(self, s, e, t):
        if s[2] + TURN_TIME > t:
            return (s[0], s[1])

        else:
            total_movement_time = e[2] - s[2] - TURN_TIME
            movement_time = t - s[2] - TURN_TIME
            movement_distance = e[0] - s[0], e[1] - s[1]        

            return (movement_time / total_movement_time) * movement_distance[0] + s[0], (movement_time / total_movement_time) * movement_distance[1] + s[1]

    def interpolateConnectsTo(self, start, end):
        assert start[2] is not None, "interpolate got goal node as a start"
        
        if end[2] is None:
            distance = ((start[0] - end[0]) ** 2 + (end[1] - start[1]) ** 2) ** 1/2
            endtime = start[0] + TURN_TIME + distance / VEL_MAX
            end = (end[0], end[1], endtime)

    
        queue = [(start[2], end[2])]

        while len(queue) != 0:
            st, et = queue.pop(0)
            mt = (st + et) / 2
            pos = self.get_pos(start, end, mt)
            
            if not PRMSample(int(pos[0]), int(pos[1]), mt, self.nearest_neighbours).inFreespace():
                return False 

            queue.append((st, mt))
            queue.append((mt, et))

            if (et - st < TIME_GRANULARITY):
                return True

######################################################################
#   PRM Functions

# Create the list of nodes.
def create_nodes(N, nearest_neighbours):
    # Add nodes sampled uniformly across the space.
    nodes = []
    while len(nodes) < N:
        node = PRMSample(random.randint(xmin, xmax - 1),
                    random.randint(ymin, ymax - 1),
                    random.uniform(0, T_SAMPLE_MAX),
                    nearest_neighbours)
        if node.inFreespace():
            nodes.append(node)
    return nodes

# Create the list of nodes.
def create_nodes_timesampling(N, nearest_neighbours, start, time_std):
    # Add nodes sampled uniformly across the space.
    nodes = []
    while len(nodes) < N:
        x_rand = random.randint(xmin, xmax - 1)
        y_rand = random.randint(ymin, ymax - 1)
        x_cur, y_cur = start.u, start.v
        t_exp = sqrt((x_cur - x_rand) ** 2 + (y_cur - y_rand) ** 2) / VEL_MAX
        t_rand = np.random.normal(t_exp, time_std)
        node = PRMSample(x_rand,
                    y_rand,
                    t_rand,
                    nearest_neighbours)
        if node.inFreespace():
            nodes.append(node)
    return nodes

# Connect the nearest neighbors
def connect_nearest_neighbours(nodes, K, goal, logger):
    # Clear any existing neighbors.  Use a set to add below.
    for node in nodes:
        node.neighbors = set()

    # Determine the indices for the K nearest neighbors.  Distance is
    # computed as the Euclidean distance of the coordinates.  This
    # also reports the node itself as the closest neighbor, so add one
    # extra here and ignore the first element below.
    X = np.array([node.coordinates() for node in nodes])

    [dist, idx] = KDTree(X).query(X, k=2*K)
    logger("        Queried KD Tree.")

    # Add the edges.  Ignore the first neighbor (being itself).
    for i, nbrs in enumerate(idx):
        count = 0
        for n in nbrs[1:]:
            if nodes[i].connectsTo(nodes[n]):
                nodes[i].neighbors.add(nodes[n])
                count += 1
                if count >= K:
                    break

        if i % 100 == 0: 
            logger(f"       Connected with {i} nodes")
            
            # nodes[n].neighbors.add(nodes[i])

    close_to_goal = sorted([(i, node.distance_space(goal)) for (i, node) in enumerate(nodes)], key=lambda x:x[1])
    
    count = 0
    for (idx, distance) in close_to_goal:
        if nodes[idx].connectsTo(goal):
            nodes[idx].neighbors.add(goal)
            count += 1
        if count >= K:
            break

# Post Process the Path
def post_process(path):
    i = 0
    while (i < len(path)-2):
        if path[i].connectsTo(path[i+2]):
            path.pop(i+1)
        else:
            i = i+1

######################################################################

#
#   A* Planning Algorithm
#
def astar(nodes, start, goal):
    # Clear the A* search tree information.
    for node in nodes:
        node.reset()

    # Prepare the still empty *sorted* on-deck queue.
    onDeck = []

    # Begin with the start node on-deck.
    start.done   = False
    start.seen   = True
    start.parent = None
    start.creach = 0
    start.ctogo  = start.costToGoEst(goal)
    bisect.insort(onDeck, start)

    # Continually expand/build the search tree.
    while True:
        # Grab the next node (first on deck).
        node = onDeck.pop(0)

        # Add the neighbors to the on-deck queue (or update)
        for neighbor in node.neighbors:
            # Skip if already done.
            if neighbor.done:
                continue

            # Compute the cost to reach the neighbor via this new path.
            creach = node.creach + node.costToConnect(neighbor)

            # Just add to on-deck if not yet seen (in correct order).
            if not neighbor.seen:
                neighbor.seen   = True
                neighbor.parent = node
                neighbor.creach = creach
                neighbor.ctogo  = neighbor.costToGoEst(goal)
                bisect.insort(onDeck, neighbor)
                continue

            # Skip if the previous path to reach (cost) was better!
            if neighbor.creach <= creach:
                continue

            # Update the neighbor's connection and resort the on-deck queue.
            neighbor.parent = node
            neighbor.creach = creach
            onDeck.remove(neighbor)
            bisect.insort(onDeck, neighbor)

        # Declare this node done.
        node.done = True

        # Check whether we have processed the goal (now done).
        if (goal.done):
            break

        # Also make sure we still have something to look at!
        if not (len(onDeck) > 0):
            return []

    # Build the path.
    path = [goal]
    while path[0].parent is not None:
        path.insert(0, path[0].parent)

    # Return the path.
    return path

def temporal_prm(start, goal, nearest_neighbours, logger):
    logger(f"    Temporal PRM: Sampling {N} nodes...")
    nodes = create_nodes(N, nearest_neighbours)
    #nodes = create_nodes_timesampling(N, nearest_neighbours, start, 4)
    nodes.append(start)
    logger(f"    Temporal PRM: Connecting {K} neighbors...")
    
    connect_nearest_neighbours(nodes, K, goal, logger)
    logger(f"    Temporal PRM: Running astar...")
    
    path = astar(nodes, start, goal)
    post_process(path)
    
    return path

#
#   Temporal PRM Node Class
#
class TemporalPRMNode(Node):
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
        
        self.subuncertainty_map = self.create_subscription(OccupancyGrid, 'uncertainty_map', self.cb_uncertainty_map, quality)

        self.subrobotpose = self.create_subscription(PoseStamped, 'pose', self.cb_robotpose, 10)

        self.subnextwaypoint = self.create_subscription(Bool, 'next_waypoint', self.cb_nextwaypoint, 1)
        
        self.subnewrrt = self.create_subscription(Bool, 'new_rrt', self.cb_newrrt, 1) # can this be the same channel as the other new_rrt?

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
            waypoint_msg.pose.position.x = float(self.current_waypoint.x)
            waypoint_msg.pose.position.y = float(self.current_waypoint.y)
            waypoint_msg.pose.position.z = 0.0
            waypoint_msg.pose.orientation.x = float(self.current_waypoint.t)
            
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
        if self.got_map:
            assert msg.header.frame_id == 'map', "Planner: Message not in map frame!!!!"
            self.get_logger().info('Temporal PRM Planner got goal pose.')
            
            # RECOMPUTE NEAREST NEIGHBOURS
            self.compute_nearest_neighbours()
            pose = msg.pose
            self.get_logger().info('Temporal PRM Planner recomputed nearest neighbors.')
        
            # Start is robot position, goal is pose, in map frame
            startx, starty = self.current_pose[0], self.current_pose[1]
            self.goalx, self.goaly = pose.position.x, pose.position.y
            self.goalz, self.goalw = float(pose.orientation.z), float(pose.orientation.w)

            # Get start and goal in grid frame
            startu, startv = self.grid_coords_from_map(startx, starty)
            goalu, goalv = self.grid_coords_from_map(self.goalx, self.goaly)

            # Get the waypoints from Temporal PRM
            start_node = PRMSample(startu, startv, 0, self.nearest_neighbours)
            end_node = PRMSample(goalu, goalv, None, self.nearest_neighbours)
            self.get_logger().info('Starting Temporal PRM...')
            self.waypoints = temporal_prm(start_node, end_node, self.nearest_neighbours, self.get_logger().info)
            self.get_logger().info('Temporal PRM Computed')

            
            if self.waypoints:
                
                # Get waypoints into map frame
                for i, pt in enumerate(self.waypoints):
                    x, y = self.map_coords_from_grid(float(pt.u), float(pt.v))
                    pt.x = x
                    pt.y = y

                    if self.waypoints[i].t is None:
                        self.waypoints[i].t = 0.0
                     
                        continue # if t is none, this is the goal node

                    if i == 0:
                        self.waypoints[i].t = 0.0
                    else:
                        self.waypoints[i].t = pt.t - self.waypoints[i-1].t

                self.get_logger().info(f'Post-Processed Temporal PRM Path in grid coords:\n{self.waypoints}')
                    
                # Publish a marker message to show the waypoint trail
                markermsg = Marker()
                markermsg.header.frame_id = "map"
                markermsg.header.stamp = msg.header.stamp
                markermsg.ns = "temporal_prm_waypoints"
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

                # DEQUEUING AND PUBLISHING NEXT WAYPOINT

                self.get_logger().info(f"Reaching Here, {self.started}")

                if self.started:
                    # Only dequeue and publish if we have entered the go command in pose_toggle
                    # Pop current waypoint
                    self.current_waypoint = self.waypoints.pop(0)

                    # Publish current waypoint
                    waypoint_msg = PoseStamped()
                    waypoint_msg.pose.position.x = float(self.current_waypoint.x)
                    waypoint_msg.pose.position.y = float(self.current_waypoint.y)
                    waypoint_msg.pose.position.z = 0.0
                    waypoint_msg.pose.orientation.x = float(self.current_waypoint.t)

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

                    # self.get_logger().info(f"z: {waypoint_msg.pose.orientation.z} w: {waypoint_msg.pose.orientation.w}")
                    self.get_logger().info("PUBLISHING WAYPOINT")
                    self.pubwaypoint.publish(waypoint_msg)   

            else:
                # NO TEMPORAL PRM PATH TO POINT SO ASK FOR NEW GLOBAL 
                self.get_logger().info("Temporal PRM Path not found to point, requesting for new global waypoint")
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
    node = TemporalPRMNode('temporal_prm') # used to be "planner"

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
