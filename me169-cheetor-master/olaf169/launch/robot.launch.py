"""Launch RVIZ to show the bot URDF

   ros2 launch olaf169 robot.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                      import LaunchDescription
from launch.actions              import Shutdown
from launch_ros.actions          import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # SELECT THE OPTIONS

    # Locate the URDF default file.
    urdf = os.path.join(pkgdir('olaf169'), 'urdf/bot_front_wheels.urdf')

    # Load the robot's URDF file (XML).
    with open(urdf, 'r') as file:
        robot_description = file.read()

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS
    if os.uname()[1] == 'olafrobot2':
        namespace = "robot2"
        namespaceFS = "robot2/"
    else:
        namespace = "robot1"
        namespaceFS = "robot1/"

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher',
        namespace  = namespace,
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}, 
                      {'frame_prefix': namespaceFS}],
    )

    node_wheelcommand = Node(
        name       = 'wheelcommand',
        namespace  = namespace,
        package    = 'olaf169',
        executable = 'wheelcontrol',
        output     = 'screen',
    )    

    node_odometry = Node(
        name       = 'odometry',
        namespace  = namespace,
        package    = 'olaf169',
        executable = 'odometry',
        output     = 'screen',
        parameters = [{'frame_prefix': namespaceFS}],
    )

    node_drive_auto = Node(
        name       = 'drive_auto',
        namespace  = namespace,
        package    = 'olaf169',
        executable = 'drive_auto',
        output     = 'screen',
        parameters = [{'frame_prefix': namespaceFS}],
    )

    node_sentry_drive = Node(
        name       = 'sentry_drive',
        namespace  = namespace,
        package    = 'olaf169',
        executable = 'sentry_drive',
        output     = 'screen',
        parameters = [{'frame_prefix': namespaceFS}],
    )
        
    node_lidar = Node(
        name = 'rplidar_composition', 
        namespace  = namespace,
        package = 'rplidar_ros',
        executable = 'rplidar_composition',
        output = 'screen',
        parameters = [{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'lidar',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    node_lidar_fix = Node(
        name       = 'rplidarfix',
        namespace  = namespace,
        package    = 'olaf169',
        executable = 'lidar',
        output     = 'screen',
        parameters = [{'frame_prefix': namespaceFS}],
    )

    node_localize = Node(
        name       = 'localize',
        namespace  = namespace,
        package    = 'olaf169',
        executable = 'localize',
        output     = 'screen',
        parameters = [{'frame_prefix': namespaceFS}],
    )

    node_sentry_move = Node(
        name = 'sentry_move',
        namespace= namespace,
        package = 'olaf169',
        executable = 'sentry_move',
        output = 'screen',
        parameters = [{'frame_prefix': namespaceFS}]
    )

    # node_planner = Node(
    #     name       = 'planner',
    #     namespace  = namespace,
    #     package    = 'olaf169',
    #     executable = 'planner',
    #     output     = 'screen',
    #     parameters = [{'frame_prefix': namespaceFS}],
    # )

    node_mapping = Node(
        name       = 'mapping',
        namespace  = namespace,
        package    = 'olaf169',
        executable = 'mapping',
        output     = 'screen',
        parameters = [{'frame_prefix': namespaceFS}],
    )

    if namespace == 'robot1':
        description_list = [
            # Start dem nodez. map_server and lifecycle have been moved to the common launch file.
            node_robot_state_publisher,
            node_wheelcommand,
            node_odometry,
            node_drive_auto,
            node_lidar,
            node_lidar_fix,
            # node_localize,
            #node_planner, # Planner for interceptor has been moved to the Nuuc
            #node_sentry_move, # for now!!
            node_mapping, # Mapping for interceptor
        ]

    else:
        description_list = [

            # Start dem nodez. map_server and lifecycle have been moved to the common launch file.
            node_robot_state_publisher,
            node_wheelcommand,
            node_odometry,
            node_lidar,
            node_lidar_fix,
            node_localize,

            # new drive_auto for sentry
            node_sentry_drive,

            # uncomment these for the old sentry driving - cycling between 4 waypoints
            #node_sentry_move,
            #node_drive_auto,
        ]

    return LaunchDescription(description_list)
