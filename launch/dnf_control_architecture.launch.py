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
            # If you have parameters:
            # parameters=[{'some_param': 42}],
            # If using Wayland and GLFW prefers X11, you may need:
            # env={'DISPLAY': os.environ.get('DISPLAY', ':0')}
        )
    ])