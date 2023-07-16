"""Launch RVIZ to show the bot URDF

   ros2 launch olaf169 viewbot.launch.py

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
    urdf = os.path.join(pkgdir('olaf169'),
                        'urdf/bot_front_wheels.urdf')

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
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz',
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown()
        )

    node_wheelcommand = Node(
        name       = 'wheelcommand',
        package    = 'olaf169',
        executable = 'wheelcontrol',
        output     = 'screen',
        )    

    node_odometry = Node(
        name       = 'odometry',
        package    = 'olaf169',
        executable = 'odometry',
        output     = 'screen',
        )

    node_drive_auto = Node(
        name       = 'drive_auto',
        package    = 'olaf169',
        executable = 'drive_auto',
        output     = 'screen',
        )

    node_lidar = Node(
        name       = 'rplidarfix',
        package    = 'olaf169',
        executable = 'rplidarfix',
        output     = 'screen',
    )


    ######################################################################
    # RETURN THE ELEMENTS, built into a Launch Description list

    return LaunchDescription([
        # Start the robot_state_publisher and RVIZ.
        node_robot_state_publisher,
        node_rviz,
        node_wheelcommand,
        node_odometry,
        # node_drive_auto,
        node_lidar
    ])
