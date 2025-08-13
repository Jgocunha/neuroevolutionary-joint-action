from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kuka_lbr_iiwa14_marlab',
            executable='dnf_control_architecture',
            name='dnf_control_architecture',
            output='screen',
            parameters=[{'target_object_rate_hz': 30.0}],
        )
    ])     
