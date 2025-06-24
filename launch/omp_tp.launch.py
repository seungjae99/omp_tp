#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('omp_tp')

    rviz_config = os.path.join(pkg_share, 'rviz', 'omp_tp_rviz2.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        Node(
            package='omp_tp',
            executable='obs_avoid_3',
            name='obs_avoid_3',
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                '/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/my_boat/thrust1@std_msgs/msg/Float64@ignition.msgs.Double',
                '/my_boat/thrust2@std_msgs/msg/Float64@ignition.msgs.Double',
            ],
            output='screen'
        ),
    ])
