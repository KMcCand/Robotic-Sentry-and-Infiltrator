#!/usr/bin/env python3
#
#   prmtriangles.py
#
#   Use PRM to find a path around triangular obstacles.
#
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math          import pi, sin, cos, sqrt, ceil
from scipy.spatial import KDTree
from temporal_astar         import AStarNode, astar
from planarutils   import *


######################################################################
#
#   Parameters
#
#   Define the N/K...
#
N = 2000
K = 40

VEL_MAX = 2
TURN_TIME = 0.5
SENTRY_LAG = 0.3
ALLOWED_SENTRY_DIST = 1

WAYPOINTS = ((10, 0.2), (13.8, 5), (9, 8))
SENTRY_EXPECTED_SPEED = 2

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
#   World Definitions
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (0, 14)
(ymin, ymax) = (0, 10)
(tmin, tmax) = (0, 30)

triangles = ((( 2, 6), ( 3, 2), ( 4, 6)),
             (( 6, 5), ( 7, 7), ( 8, 5)),
             (( 6, 9), ( 8, 9), ( 8, 7)),
             ((10, 3), (11, 6), (12, 3)))

(startx, starty, startt) = (1, 5, 0)
(goalx,  goaly, goalt)  = (13, 5, None)


######################################################################
#
#   Node Definition
#
class Node(AStarNode):
    def __init__(self, x, y, t):
        # Setup the basic A* node.
        super().__init__()

        # Define/remember the state/coordinates (x,y).
        self.x = x
        self.y = y
        self.t = t
    
    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return (f"X: {self.x} Y: {self.y} T: {self.t}")

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate(self, other, alpha):
        return Node(self.x + alpha * (other.x - self.x),
                    self.y + alpha * (other.y - self.y),
                    self.t+alpha * (other.t - self.t))

    # Compute the relative distance to another node.
    def distance(self, other):
        if (self.t is None) or (other.t is None):
            return self.distance_space(other)

        return ((self.t - other.t) ** 2 + (self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 1/2

    def distance_space(self, other):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 1/2

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        return (self.x, self.y, self.t)

    ###############
    # A* functions:
    # Actual and Estimated costs.
    def costToConnect(self, other):
        return self.distance(other)

    def costToGoEst(self, other):
        return self.distance(other)
    
    def collidesWithSentry(self):
        sentry_pos = get_sentry_position(self.t - SENTRY_LAG)
        return sqrt((sentry_pos[0] - self.x) ** 2 + (sentry_pos[1] - self.y) ** 2) < ALLOWED_SENTRY_DIST

    ################
    # PRM functions:
    # Check whether in free space.
    def inFreespace(self):
        for t in triangles:
            if PointInTriangle((self.x, self.y), t):
                return False
        
        return not self.collidesWithSentry()

    # Check the local planner - whether this connects to another node

    # IF YOU CONNECT TO OTHER TREE
    def connectsTo(self, other):
        if (other.t != None and self.t != None):
            if (self.t + TURN_TIME + (self.distance_space(other) / VEL_MAX) > other.t):
                # Cannot make it even if you go at max velocity
                return False
             
            # Find out if we will cross the sentry
            robot_travel = (self.x, self.y), (other.x, other.y)
            sentry_travel = get_sentry_position(self.t - SENTRY_LAG), get_sentry_position(other.t - SENTRY_LAG)
            if SegmentNearSegment(1, robot_travel, sentry_travel):
                return False
        
        s = ((self.x, self.y), (other.x, other.y))

        for t in triangles:
            if SegmentCrossTriangle(s, t):
                return False 

        return True

######################################################################
#
#   Visualization
#
class Visualization:
    def __init__(self):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.grid(True)
        plt.gca().axis('on')
        plt.gca().set_xlim(xmin, xmax)
        plt.gca().set_ylim(ymin, ymax)
        plt.gca().set_aspect('equal')

        # Show the triangles.
        for tr in triangles:
            plt.plot((tr[0][0], tr[1][0], tr[2][0], tr[0][0]),
                     (tr[0][1], tr[1][1], tr[2][1], tr[0][1]),
                     'k-', linewidth=2)

        plt.plot((WAYPOINTS[0][0], WAYPOINTS[1][0], WAYPOINTS[2][0], WAYPOINTS[0][0]),
                    (WAYPOINTS[0][1], WAYPOINTS[1][1], WAYPOINTS[2][1], WAYPOINTS[0][1]),
                    'r-', linewidth=2)

        # Show.
        self.show()

    def show(self, text = ''):
        # Show the plot.
        plt.pause(0.001)
        # If text is specified, print and wait for confirmation.
        if len(text)>0:
            input(text + ' (hit return to continue)')

    def drawNode(self, node, *args, **kwargs):
        return plt.plot(node.x, node.y, *args, **kwargs)

    def drawEdge(self, head, tail, *args, **kwargs):
        plt.plot((head.x, tail.x),
                 (head.y, tail.y), *args, **kwargs)

    def drawPath(self, path, *args, **kwargs):
        for i in range(len(path)-1):
            self.drawEdge(path[i], path[i+1], *args, **kwargs)


######################################################################
#
#   PRM Functions
#
# Create the list of nodes.
def createNodes(N):
    # Add nodes sampled uniformly across the space.
    nodes = []
    while len(nodes) < N:
        node = Node(random.uniform(xmin, xmax),
                    random.uniform(ymin, ymax),
                    random.uniform(0, 30))
        if node.inFreespace():
            nodes.append(node)
    return nodes

# Connect the nearest neighbors
def connectNearestNeighbors(nodes, K, goal):
    # Clear any existing neighbors.  Use a set to add below.
    for node in nodes:
        node.neighbors = set()

    # Determine the indices for the K nearest neighbors.  Distance is
    # computed as the Euclidean distance of the coordinates.  This
    # also reports the node itself as the closest neighbor, so add one
    # extra here and ignore the first element below.
    X = np.array([node.coordinates() for node in nodes])

    [dist, idx] = KDTree(X).query(X, k=(K+1))

    # Add the edges.  Ignore the first neighbor (being itself).
    for i, nbrs in enumerate(idx):
        count = 0
        for n in nbrs[1:]:
            if nodes[i].connectsTo(nodes[n]):
                nodes[i].neighbors.add(nodes[n])
                count += 1
                if count >= K:
                    break
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
def PostProcess(path):
    i = 0
    
    while (i < len(path) - 1):
        p = path[i]

        j = i + 1
        while (j + 1< len(path)):
            if p.connectsTo(path[j + 1]):
                path.pop(j) 
            else:
                break 

        i += 1

######################################################################
#
#  Main Code
#
def main():
    # Report the parameters.
    print('Running with', N, 'nodes and', K, 'neighbors.')

    # Create the figure.
    visual = Visualization()

    # Create the start/goal nodes.
    startnode = Node(startx, starty, 0)
    goalnode  = Node(goalx,  goaly, None)

    # Show the start/goal nodes.
    visual.drawNode(startnode, color='r', marker='o')
    visual.drawNode(goalnode,  color='r', marker='o')
    visual.show("Showing basic world")


    # Create the list of nodes.
    print("Sampling the nodes...")
    nodes = createNodes(N)

    # Show the sample nodes.
    for node in nodes:
        visual.drawNode(node, color='k', marker='x')
    visual.show("Showing the nodes")

    # Add the start/goal nodes.
    nodes.append(startnode)
    # nodes.append(goalnode)


    # Connect to the nearest neighbors.
    print("Connecting the nodes...")
    connectNearestNeighbors(nodes, K, goalnode)

    # Show the neighbor connections.  Yes, each edge is drawn twice...
    for node in nodes:
        for neighbor in node.neighbors:
            visual.drawEdge(node, neighbor, color='g', linewidth=0.5)
    visual.show("Showing the full graph")


    # Run the A* planner.
    print("Running A*...")
    path = astar(nodes, startnode, goalnode)

    # If unable to connect, show the part explored.
    if not path:
        print("UNABLE TO FIND A PATH")
        for node in nodes:
            if node.done:
                visual.drawNode(node, color='b', marker='o')
        visual.show("Showing DONE nodes")
        return

    # Show the path.
    visual.drawPath(path, color='r', linewidth=2)
    visual.show("Showing the raw path")

    # Post Process the path.
    PostProcess(path)

    # Show the post-processed path.
    visual.drawPath(path, color='b', linewidth=2)
    visual.show("Showing the post-processed path")

    # Show the path traversal live with moving sentry
    # Reset the visual for speed
    visual = Visualization()
    visual.drawNode(startnode, color='r', marker='o')
    visual.drawNode(goalnode,  color='r', marker='o')
    visual.drawPath(path, color='b', linewidth=2)
    
    T_STEP = 0.05
    curr_path_index = 0
    dist_along_path = 0
    t = 0
    while True:
        path_dist = path[curr_path_index].distance_space(path[curr_path_index + 1])

        if dist_along_path > path_dist:
            # Reached the end of the path.
            # Draw to the end of the current path segment
            intermediate_x = (1 - dist_along_path / path_dist) * path[curr_path_index].x + (dist_along_path / path_dist) * path[curr_path_index+1].x
            intermediate_y = (1 - dist_along_path / path_dist) * path[curr_path_index].y + (dist_along_path / path_dist) * path[curr_path_index+1].y
            visual.drawEdge(Node(intermediate_x, intermediate_y, None), path[curr_path_index + 1], color='c')

            # Go to the next segment
            dist_along_path -= path_dist
            curr_path_index += 1
            if curr_path_index == len(path) - 1:
                # Finished the segment to goal, break
                break
            
            # Draw progress in the new segment
            path_dist = path[curr_path_index].distance_space(path[curr_path_index + 1])
            intermediate_x = (1 - dist_along_path / path_dist) * path[curr_path_index].x + (dist_along_path / path_dist) * path[curr_path_index+1].x
            intermediate_y = (1 - dist_along_path / path_dist) * path[curr_path_index].y + (dist_along_path / path_dist) * path[curr_path_index+1].y
            visual.drawEdge(path[curr_path_index], Node(intermediate_x, intermediate_y, None), color='c')

        else:
            # Intermediate between two path points, draw the intermediate
            intermediate_x = (1 - dist_along_path / path_dist) * path[curr_path_index].x + (dist_along_path / path_dist) * path[curr_path_index+1].x
            intermediate_y = (1 - dist_along_path / path_dist) * path[curr_path_index].y + (dist_along_path / path_dist) * path[curr_path_index+1].y
            visual.drawEdge(path[curr_path_index], Node(intermediate_x, intermediate_y, None), color='c')

        # Render sentry
        sentry_pos = get_sentry_position(t - SENTRY_LAG)
        carrot_pos = get_sentry_position(t)
        if sqrt((sentry_pos[0] - intermediate_x)**2 + (sentry_pos[1] - intermediate_y)**2) < ALLOWED_SENTRY_DIST:
            visual.show("HIT SENTRY!!!!")

        if t > 0.0:
            save_old_line = old_line
            save_old_carrot = old_carrot
        old_line = visual.drawNode(Node(sentry_pos[0], sentry_pos[1], None), color='navy', marker='o', markersize=10)
        old_carrot = visual.drawNode(Node(carrot_pos[0], carrot_pos[1], None), color='r', marker='o', markersize=3)
        if t > 0.0:
            save_old_line.pop().remove()
            save_old_carrot.pop().remove()
        
        visual.show()
        dist_along_path += VEL_MAX * T_STEP
        t += T_STEP


if __name__== "__main__":
    main()
