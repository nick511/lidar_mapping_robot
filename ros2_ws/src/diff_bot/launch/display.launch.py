from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)

import os
import xacro

def generate_launch_description():

    pkg_path = get_package_share_directory(
        'diff_bot'
    )

    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'diffbot.urdf.xacro'
    )

    robot_desc = xacro.process_file(
        xacro_file
    ).toxml()

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_desc}
            ]
        ),

        Node(
            package='diff_bot',
            executable='odom_tf_broadcaster.py',
            name='odom_tf_broadcaster'
        ),

        Node(
            package='rviz2',
            executable='rviz2'
        )

    ])
