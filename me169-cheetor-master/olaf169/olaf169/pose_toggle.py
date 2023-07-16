"""
Take in a goal pose and output robot1/goal_pose or robot2/goal_pose, depending on toggle
value passed in continuously.
"""

#!/usr/bin/env python3
#
#   pose_toggle.py
#
#  Pose Toggle node. This keeps tracks of the map to odom frame transform. 
#                      Subscribes to /odom and /scanfixed. Publishes /pose.
#   
#   Node: /pose_toggle
#   Publish:
#           /robot1/goal_pose                  geometry_msgs/PoseStamped
#           /robot2/goal_pose                  geometry_msgs/PoseStamped
#           /robot1/initialpose                geometry_msgs/PoseWithCovarianceStamped
#           /robot2/initialpose                geometry_msgs/PoseWithCovarianceStamped
#   Subscribe:
#           /initialpose                       geometry_msgs/PoseWithCovarianceStamped
#           /goal_pose                         geometry_msgs/PoseStamped


import rclpy
import traceback

from rclpy.executors            import MultiThreadedExecutor
from rclpy.callback_groups      import MutuallyExclusiveCallbackGroup

from rclpy.node         import Node
from geometry_msgs.msg  import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool


class PoseToggleNode(Node):
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Class variables
        self.current_toggle = 1 # 1 for robot 1, 2 for robot 2
        
        # Publishers
        self.pub_robot1goalpose = self.create_publisher(PoseStamped, '/robot1/goal_pose', 3) # Robot 1 pose
        self.pub_robot2goalpose = self.create_publisher(PoseStamped, '/robot2/goal_pose', 3) # Robot 2 pose

        self.pub_initrobot1pose = self.create_publisher(PoseWithCovarianceStamped, '/robot1/initialpose', 3) # Robot 1 init pose
        self.pub_initrobot2pose = self.create_publisher(PoseWithCovarianceStamped, '/robot2/initialpose', 3) # Robot 2 init pose

        self.pub_startrobot = self.create_publisher(Bool, '/start_robot', 3) # for both bots
        self.pub_stopsentry = self.create_publisher(Bool, '/stop_sentry', 3)

        fastgroup = MutuallyExclusiveCallbackGroup()
        slowgroup = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.initialpose = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.cb_initialpose, 10, callback_group=fastgroup)
        self.goalpose = self.create_subscription(PoseStamped, '/goal_pose', self.cb_goalpose, 10, callback_group=fastgroup)
        self.toggleinputcallback = self.create_timer(0.2,  self.get_toggle_input, callback_group=slowgroup)

        # Report and return.
        self.get_logger().info("Pose Toggle running")

    def cb_initialpose(self, msg):
        assert msg.header.frame_id == 'map', "Pose Toggle: Message not in map frame!!!!"
        
        if self.current_toggle == 1:
            self.pub_initrobot1pose.publish(msg)
        else:
            self.pub_initrobot2pose.publish(msg)
    
    def cb_goalpose(self, msg):
        assert msg.header.frame_id == 'map', "Pose Toggle: Message not in map frame!!!!"

        if self.current_toggle == 1:
            self.pub_robot1goalpose.publish(msg)
        else:
            self.pub_robot2goalpose.publish(msg)

    def get_toggle_input(self):
        while True:
            val = int(input("Requesting which robot to publish to: "))

            if val == 3:
                print('Robots are starting!')
                msg = Bool()
                msg.data = True
                self.pub_startrobot.publish(msg)
            elif val in(1, 2):
                self.current_toggle = val
                print(f"Setting goal and initial pose to go to robot {val}.")
            elif val == 4:
                print('Stopping sentry.')
                msg = Bool()
                msg.data = True
                self.pub_stopsentry.publish(msg)
            else:
                print(f"Invalid toggle value {val}.")

    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()
  
#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = PoseToggleNode('pose_toggle')

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