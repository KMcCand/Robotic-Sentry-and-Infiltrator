#!/usr/bin/env python3
#
#   rrttriangles.py
#
#   Use RRT to find a path around triangular obstacles.
#
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math          import pi, sin, cos, sqrt, ceil
from planarutils   import *


######################################################################
#
#   Parameters
#
#   Define the step size.  Also set a maximum number of nodes...
#
dstep = 1
Nmax  = 1000
MIN_VEL = 0.5

SHOW_VISUAL = False
SHOW_RRT = False
NUM_TRIALS = 200

VEL_MAX = 2
TURN_TIME = 0
SENTRY_LAG = 0.5
ALLOWED_SENTRY_RADIUS = 0.5
SAMPLE_TIME = 10
SENTRY_REACH = 2
SENTRY_CONE_WIDTH_SCALE = 2/3

WAYPOINTS = ((8, 1), (10, 6), (4, 6))
SENTRY_EXPECTED_SPEED = 1.5

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

def sentry_orientation(t):
    sentry_pos = get_sentry_position(t)
    carot_pos = get_sentry_position(t + SENTRY_LAG)
    thetas = np.array((carot_pos[0] - sentry_pos[0], carot_pos[1] - sentry_pos[1]))
    return thetas / sqrt(thetas[0]**2 + thetas[1] ** 2)

def get_sentry_triangle(t):
    print(SENTRY_REACH)
    orientation = SENTRY_REACH * sentry_orientation(t)
    position = get_sentry_position(t)
    tip = (position[0] + orientation[0], position[1] + orientation[1])

    pt1 = (tip[0] + SENTRY_CONE_WIDTH_SCALE * orientation[1], tip[1] - SENTRY_CONE_WIDTH_SCALE * orientation[0])
    pt2 = (tip[0] - SENTRY_CONE_WIDTH_SCALE * orientation[1], tip[1] + SENTRY_CONE_WIDTH_SCALE * orientation[0])
    pt3 = (position[0], position[1])

    return pt1, pt2, pt3
    

######################################################################
#
#   World Definitions
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (0, 14)
(ymin, ymax) = (0, 10)

triangles = (# (( 2, 6), ( 4, 9), ( 3.5, 6)),
            #  (( 2, 4), ( 4, 2), ( 2, 2)),
             (( 5, 7), ( 10, 7), ( 8, 8)),
             ((10, 5.5), (6, 5.5), (8, 4)))

(startx, starty) = (4.2, 6)
(goalx,  goaly)  = (8, 3)


######################################################################
#
#   Node Definition
#
class Node:
    # Initialize with coordinates.
    def __init__(self, x, y, t):
        # Define/remember the state/coordinates (x,y).
        self.x = x
        self.y = y
        self.t = t

        # Clear any parent information.
        self.parent = None

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return (f"X: {self.x} Y: {self.y} T: {self.t}")

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    # def intermediate(self, other, alpha):
    #     return Node(self.x + alpha * (other.x - self.x),
    #                 self.y + alpha * (other.y - self.y),
    #                 self.t + alpha * (other.t - self.t))

    # Compute the relative distance to another node.
    # def distance(self, other):
    #     if self.t and other.t:
    #         return sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + 0.1*(self.t - other.t)**2)
    #     else:
    #         return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
        
    def distance_space(self, other):
            return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        return (self.x, self.y)
    
    def currently_collided_with_sentry(self):
        sentry_pos = get_sentry_position(self.t - SENTRY_LAG)
        return sqrt((sentry_pos[0] - self.x) ** 2 + (sentry_pos[1] - self.y) ** 2) <= ALLOWED_SENTRY_RADIUS + 0.1

    ######################
    # Collision functions:
    # Check whether in free space.
    def inFreespace(self):
        if (self.x <= xmin or self.x >= xmax or
            self.y <= ymin or self.y >= ymax):
            return False
        if self.currently_collided_with_sentry():
            return False
        for triangle in triangles:
            if PointInTriangle((self.x, self.y), triangle):
                return False
        return True

    def check_sentry_collision(self, other):
        if other.t == None: # at the goal checking sentry collision
            other.t = self.t + self.distance_space(other) / VEL_MAX

        # Returns true if sentry collides with line segment
        for alpha in np.arange(0.0, 1.0, 0.01): # checks 10 intermediate points in connects to
            inter_pt = self.x + alpha*(other.x-self.x), self.y + alpha*(other.y-self.y)
            inter_t = self.t + alpha*(other.t-self.t)
            sentry_at_t = get_sentry_position(inter_t - SENTRY_LAG)
            # pt1, pt2, pt3 = get_sentry_triangle(inter_t - SENTRY_LAG)

            # colliding with sentry check
            if (sentry_at_t[0] - inter_pt[0]) ** 2 + (sentry_at_t[1] - inter_pt[1]) ** 2 < (ALLOWED_SENTRY_RADIUS+0.1) ** 2:
                return True # collides

            # colliding with sentry field of view check
            # if self.triangle_intersect(pt1[0], pt1[1], pt2[0], pt2[1], pt3[0], pt3[1], inter_pt[0], inter_pt[1]):
            #     return True
            if PointInTriangle(inter_pt, get_sentry_triangle(inter_t - SENTRY_LAG)):
                return True

        return False
    
    # def triangle_intersect(self, x1, y1, x2, y2, x3, y3, xp, yp):
    #     c1 = (x2-x1)*(yp-y1)-(y2-y1)*(xp-x1)
    #     c2 = (x3-x2)*(yp-y2)-(y3-y2)*(xp-x2)
    #     c3 = (x1-x3)*(yp-y3)-(y1-y3)*(xp-x3)
    #     if (c1<0 and c2<0 and c3<0) or (c1>0 and c2>0 and c3>0):
    #         return True
    #     else:
    #         return False

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        if not (other.t == None or self.t == None): 
            if self.t + TURN_TIME + (self.distance_space(other) / VEL_MAX) > other.t:
                # Cannot possibly get there
                return False
            
        # Find out if we will cross the sentry
        if self.check_sentry_collision(other):
            return False

        for triangle in triangles:
            if SegmentCrossTriangle(((self.x, self.y), (other.x, other.y)),
                                    triangle):
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
        # self.show()

    def show(self, text = ''):
        # Show the plot.
        plt.pause(0.001)
        # If text is specified, print and wait for confirmation.
        if len(text)>0:
            input(text + ' (hit return to continue)')

    def drawNode(self, node, *args, **kwargs):
        return plt.plot(node.x, node.y, *args, **kwargs)

    # def drawDot(self, dot_pos):
    #     plt.plot(dot_pos[0], dot_pos[1], marker="o")

    def drawEdge(self, head, tail, *args, **kwargs):
        return plt.plot((head.x, tail.x),
                 (head.y, tail.y), *args, **kwargs)

    def drawPath(self, path, *args, **kwargs):
        for i in range(len(path)-1):
            self.drawEdge(path[i], path[i+1], *args, **kwargs)


######################################################################
#
#   RRT Functions
#
def rrt(startnode, goalnode, visual):
    # Start the tree with the startnode (set no parent just in case).
    startnode.parent = None
    tree = [startnode]

    # Function to attach a new node to an existing node: attach the
    # parent, add to the tree, and show in the figure.
    def addtotree(oldnode, newnode):
        newnode.parent = oldnode
        tree.append(newnode)
        if SHOW_VISUAL:
            visual.drawEdge(oldnode, newnode, color='g', linewidth=1)
            if SHOW_RRT:
                visual.show()

    # Loop - keep growing the tree.
    while True:
        # Determine the target state.
        if np.random.uniform() < 0.05:
            targetnode = goalnode
        else:
            targetnode = Node(random.uniform(xmin, xmax),random.uniform(ymin, ymax), random.uniform(startnode.t, startnode.t + SAMPLE_TIME))

        # Directly determine the distances to the target node.
        distances = np.array([node.distance_space(targetnode) for node in tree])
        index     = np.argmin(distances)
        nearnode  = tree[index]
        d         = distances[index]

        # Determine the next node.
        if targetnode.t:
            next_node_t = nearnode.t + (targetnode.t - nearnode.t) * dstep / (nearnode.distance_space(targetnode))
        else:
            next_node_t = nearnode.t + dstep / VEL_MAX
        
        nextnode = Node(nearnode.x + dstep / d * (targetnode.x - nearnode.x),
                        nearnode.y + dstep / d *  (targetnode.y - nearnode.y),
                        next_node_t)
                        # nearnode.t + random.uniform(dstep / VEL_MAX, dstep / MIN_VEL))
                        # nearnode.t + dstep / MIN_VEL)

        # Check whether to attach.
        if nearnode.connectsTo(nextnode) and nextnode.x >= xmin and nextnode.x <= xmax and nextnode.y >= ymin and nextnode.y <= ymax:
            addtotree(nearnode, nextnode)

            # If within dstep, also try connecting to the goal.  If
            # the connection is made, break the loop to stop growing.
            if nextnode.distance_space(goalnode) <= dstep and nextnode.connectsTo(goalnode):
                addtotree(nextnode, goalnode)
                break

        # Check whether we should abort (tree has gotten too large).
        if (len(tree) >= Nmax):
            return (None, len(tree))

    # Build and return the path.
    path = [goalnode]
    while path[0].parent is not None:
        path.insert(0, path[0].parent)
    return (path, len(tree))


# Post Process the Path
def PostProcess(path):
    i = 0
    while (i < len(path)-2):
        if path[i].connectsTo(path[i+2]):
            path.pop(i+1)
        else:
            i = i+1


######################################################################
#
#  Main Code
#
def main():
    # Report the parameters.
    print(f'Running {NUM_TRIALS} trials with step size ', dstep, ' and up to ', Nmax, ' nodes.')
    num_collisions = 0

    for trial in range(NUM_TRIALS):
        # Create the figure.
        visual = Visualization()

        # Create the start/goal nodes.
        startnode = Node(startx, starty, 0)
        goalnode  = Node(goalx,  goaly, None)

        # Show the start/goal nodes.
        visual.drawNode(startnode, color='r', marker='o')
        visual.drawNode(goalnode,  color='r', marker='o')
        # if SHOW_VISUAL:
        #     visual.show("Showing basic world")

        # Run the RRT planner.
        if SHOW_VISUAL:
            print("Running Temporal RRT...")
        (path, N) = rrt(startnode, goalnode, visual)

        # If unable to connect, show what we have.
        if not path:
            if SHOW_VISUAL:
                visual.show("UNABLE TO FIND A PATH")
            print('rrt failed to find path')
            continue

        # Show the path.
        print(f"Trial {trial}: PATH found after %d nodes" % N)
        visual.drawPath(path, color='r', linewidth=2)
        # if SHOW_VISUAL:
        #     visual.show("Showing the raw path")

        # Post Process the path.
        PostProcess(path)
        if SHOW_VISUAL:
            for pt in path:
                print(pt)

        # Show the post-processed path.
        visual.drawPath(path, color='b', linewidth=2)
        # if SHOW_VISUAL:
        #     visual.show("Showing the post-processed path")

        # Show the path traversal live with moving sentry
        # Reset the visual for speed
        visual = Visualization()
        visual.drawNode(startnode, color='r', marker='o')
        visual.drawNode(goalnode,  color='r', marker='o')
        visual.drawPath(path, color='b', linewidth=2)
        
        T_STEP = 0.05
        curr_path_index = 0
        dist_along_path = 0
        start_time = time.time()
        prev_time = start_time
        t = 0
        while True:
            path_dist = path[curr_path_index].distance_space(path[curr_path_index + 1])
            if path[curr_path_index + 1].t == None:
                vel_cur = VEL_MAX
            else:
                path_time = path[curr_path_index + 1].t - path[curr_path_index].t
                vel_cur = (path_dist/path_time)

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
            within_sentry_dot = sqrt((sentry_pos[0] - intermediate_x)**2 + (sentry_pos[1] - intermediate_y)**2) < ALLOWED_SENTRY_RADIUS
            within_sentry_triangle = PointInTriangle((intermediate_x, intermediate_y), get_sentry_triangle(t - SENTRY_LAG))
            if within_sentry_dot or within_sentry_triangle:
                visual.show("HIT SENTRY!!!!")
                num_collisions += 1

            if t > 0.0:
                # On all but the first timestep, save the old markers to delete them
                save_old_carrot = old_carrot
                save_old_cone = old_cone
                save_old_dot = old_dot
                
            # Draw markers: carrot, sentry view cone, sentry
            old_carrot = visual.drawNode(Node(carrot_pos[0], carrot_pos[1], None), color='red', marker='o', markersize=5)
            pt1, pt2, pt3 = get_sentry_triangle(t - SENTRY_LAG)
            old_cone = plt.plot((pt1[0], pt2[0], pt3[0], pt1[0]), (pt1[1], pt2[1], pt3[1], pt1[1]), 'orange', linewidth=2)
            old_dot = visual.drawNode(Node(sentry_pos[0], sentry_pos[1], None), color='navy', marker='o', markersize=ALLOWED_SENTRY_RADIUS*50)

            if t > 0.0:
                # On all but the first timestep, remove old markers
                save_old_carrot.pop().remove()  
                save_old_cone.pop().remove()
                save_old_dot.pop().remove()
            
            if SHOW_VISUAL:
                visual.show()
                curr_time = time.time()
                dist_along_path += vel_cur * (curr_time - prev_time)
                t = curr_time - start_time
                prev_time = curr_time
            else:
                dist_along_path += vel_cur * T_STEP
                t += T_STEP
            
        if SHOW_VISUAL:
            visual.show("SUCCESS!!!! (return to go again)")
    print(f'{NUM_TRIALS} trials concluded with {num_collisions} collisions.')

if __name__== "__main__":
    main()
