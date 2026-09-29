"""Assetto Corsa bridge, telemetry, lidar, drive, and RViz. No autonomy nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    ac_share = get_package_share_directory('uvis_ac')
    bridge = os.path.join(ac_share, 'scripts', 'ac_shm_bridge.sh')
    return LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        ExecuteProcess(cmd=[bridge], output='screen'),
        Node(
            package='uvis_ac',
            executable='ac_telemetry',
            name='ac_telemetry',
            output='screen',
        ),
        Node(
            package='uvis_ac',
            executable='ac_lidar',
            name='ac_lidar',
            output='screen',
        ),
        Node(
            package='uvis_ac',
            executable='ac_drive',
            name='ac_drive',
            output='screen',
        ),
        Node(
            package='uvis_perception',
            executable='cone_visualizer',
            name='cone_visualizer',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='ac_rviz',
            arguments=['-d', os.path.join(ac_share, 'rviz', 'ac.rviz')],
            parameters=[{'use_sim_time': False}],
            output='screen',
        ),
    ])
