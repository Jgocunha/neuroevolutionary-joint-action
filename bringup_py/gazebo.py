from typing import List, Optional, Union

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, TextSubstitution
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch import LaunchDescription


class GazeboMixin:
    @staticmethod
    def include_gazebo(**kwargs) -> LaunchDescription:
        marlab_env_desc_pkg = get_package_share_directory('marlab_env_description')
        world_file = PathJoinSubstitution([marlab_env_desc_pkg, 'worlds', 'marlab.sdf'])
        models_path = PathJoinSubstitution([marlab_env_desc_pkg, 'models'])

        return LaunchDescription([
            SetEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                [
                    EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
                    TextSubstitution(text=':'),
                    models_path
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ros_gz_sim"),
                            "launch",
                            "gz_sim.launch.py",
                        ]
                    ),
                ),
                launch_arguments={
                    "gz_args": [TextSubstitution(text="-r "), world_file]
                }.items(),
                **kwargs,
            ),
        ])

    @staticmethod
    def node_create(
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        tf: List[float] = [0, 0, 0, 0, 0, 0],
        **kwargs,
    ) -> Node:
        label = ["-x", "-y", "-z", "-R", "-P", "-Y"]
        tf = [str(x) for x in tf]
        return Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic",
                "robot_description",
                "-name",
                robot_name,
                "-allow_renaming",
            ]
            + [item for pair in zip(label, tf) for item in pair],
            output="screen",
            namespace=robot_name,
            **kwargs,
        )

    @staticmethod
    def node_clock_bridge(**kwargs) -> Node:
        return Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
            **kwargs,
        )
