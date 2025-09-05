from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kuka_lbr_iiwa14_marlab',
            executable='high_level_control_node',
            name='high_level_control_node',
            output='screen',
            parameters=[{'target_object_rate_hz': 20.0}],
        )
    ])     
