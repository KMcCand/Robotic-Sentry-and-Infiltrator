"""Launch RVIZ and all shared nodes

   ros2 launch olaf169 shared_nodes.launch.py

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

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('olaf169'), 'rviz/viewbot.rviz')

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz',
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown()
        )
    
    # mapfile = os.path.join(pkgdir('olaf169'), 'maps/keck308obstacles.yaml')
    mapfile = os.path.join(pkgdir('olaf169'), 'maps/keck308corner.yaml')
    
    node_map_server = Node(
        name       = 'map_server',
        package    = 'nav2_map_server',
        executable = 'map_server',
        output     = 'screen',
        parameters = [{'yaml_filename': mapfile},
                      {'topic_name':    "map"},
                      {'frame_id':      "map"}])

    node_lifecycle = Node(
        name       = 'lifecycle_manager_localization',
        package    = 'nav2_lifecycle_manager',
        executable = 'lifecycle_manager',
        output     = 'screen',
        parameters = [{'autostart':    True},
                      {'node_names':   ['map_server']}])

    node_planner = Node(
        name       = 'planner',
        namespace  = 'robot1',
        package    = 'olaf169',
        executable = 'planner',
        output     = 'screen',
        parameters = [{'frame_prefix': 'robot1/'}],
    )
    # Only for robot 1

    node_localize1 = Node(
        name       = 'localize',
        namespace  = 'robot1',
        package    = 'olaf169',
        executable = 'localize',
        output     = 'screen',
        parameters = [{'frame_prefix': 'robot1/'}],
    )

    node_planner2 = Node(
        name       = 'temporal_planner',
        namespace  = 'robot1',
        package    = 'olaf169',
        executable = 'temporal_planner',
        output     = 'screen',
        parameters = [{'frame_prefix': 'robot1/'}],
    )
    
    return LaunchDescription([
        # Start dem nodez. map_server and lifecycle first because they take a long time.
        # node_map_server,
        # node_lifecycle,
        # node_rviz,
        # node_planner,
        node_planner2, 
        # node_localize1
    ])
