from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uvis_ac',
            executable='ac_telemetry',
            name='ac_telemetry',
            output='screen',
        ),
    ])
