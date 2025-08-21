from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kuka_lbr_iiwa14_marlab',
            executable='onrobot_rg2_control',
            name='onrobot_rg2_control',
            output='screen',
        )
    ])     
