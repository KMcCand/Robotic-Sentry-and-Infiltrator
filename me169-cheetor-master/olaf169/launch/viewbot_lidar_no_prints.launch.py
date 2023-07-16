"""Launch RVIZ to show the bot URDF

   ros2 launch olaf169 viewbot_lidar.launch.py

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                             import LaunchDescription
from launch.actions                     import Shutdown
from launch_ros.actions                 import Node


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

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('olaf169'), 'rviz/viewbot.rviz')

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher',
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description': robot_description}])

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz',
        package    = 'rviz2',
        executable = 'rviz2',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown()
        )

    node_wheelcommand = Node(
        name       = 'wheelcommand',
        package    = 'olaf169',
        executable = 'wheelcontrol',
        )    

    node_odometry = Node(
        name       = 'odometry',
        package    = 'olaf169',
        executable = 'odometry',
        )

    node_drive_auto = Node(
        name       = 'drive_auto',
        package    = 'olaf169',
        executable = 'drive_auto',
        )
        
    node_lidar = Node(
        name = 'rplidar_composition', 
        package = 'rplidar_ros',
        executable = 'rplidar_composition',
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
        package    = 'olaf169',
        executable = 'lidar',
    )

    node_localize = Node(
        name       = 'localize',
        package    = 'olaf169',
        executable = 'localize',
        output     = 'screen',
    )

    # Locate the map
    mapfile = os.path.join(pkgdir('shared169'), 'maps/keck308corner.yaml')

    ######################################################################
    # Define the Map Server and Lifecycle manager.
    # -------------------------------------------
    # Being part of the navigation stack, the map server needs to be
    # explicitly activated by the lifecycle manager.  This is intended for
    # a smooth bring-up and shutdown of many components.

    ### MAPPING
    # The map server publishes an existing map.  It is part of the
    # navigation stack, which requires a lifecycle manager to start.
    node_map_server = Node(
        name       = 'map_server',
        package    = 'nav2_map_server',
        executable = 'map_server',
        parameters = [{'yaml_filename': mapfile},
                      {'topic_name':    "map"},
                      {'frame_id':      "map"}])

    node_lifecycle = Node(
        name       = 'lifecycle_manager_localization',
        package    = 'nav2_lifecycle_manager',
        executable = 'lifecycle_manager',
        parameters = [{'autostart':    True},
                      {'node_names':   ['map_server']}])


    return LaunchDescription([
        # Start the robot_state_publisher and RVIZ.
        node_robot_state_publisher,
        node_rviz,
        node_wheelcommand,
        node_odometry,
        node_drive_auto,
        node_lidar,
        node_lidar_fix,
        node_localize,
        node_map_server,
        node_lifecycle,
    ])
