from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Paths to ros_gz_sim launch and your package
    ros_gz_pkg = get_package_share_directory('ros_gz_sim')
    my_pkg = get_package_share_directory('kuka_lbr_iiwa14_marlab')
    gz_launch = PathJoinSubstitution([ros_gz_pkg, 'launch', 'gz_sim.launch.py'])
    world_file = PathJoinSubstitution([my_pkg, 'worlds', 'marlab.sdf'])
    # (Optional) set GZ resource path to your models directory
    models_path = PathJoinSubstitution([my_pkg, 'models'])
    return LaunchDescription([
        # Make sure Gazebo sees your models
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),
        # Launch Gazebo server+client with your world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': [world_file]}.items()
        ),
    ])
