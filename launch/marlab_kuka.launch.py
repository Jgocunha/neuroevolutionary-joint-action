from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable, 
    IncludeLaunchDescription, 
    AppendEnvironmentVariable,
    TimerAction,
    ExecuteProcess,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.gazebo import GazeboMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Get package paths
    ros_gz_pkg = get_package_share_directory('ros_gz_sim')
    my_pkg = get_package_share_directory('kuka_lbr_iiwa14_marlab')
    lbr_description_pkg = get_package_share_directory('lbr_description')
    
    gz_launch = PathJoinSubstitution([ros_gz_pkg, 'launch', 'gz_sim.launch.py'])
    world_file = PathJoinSubstitution([my_pkg, 'worlds', 'marlab.sdf'])
    
    # Set environment variables for mesh loading
    ld.add_action(AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH', 
        os.path.join(lbr_description_pkg, '..')
    ))
    
    ld.add_action(AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join([
            os.path.join(my_pkg, 'models'),
            os.path.join(lbr_description_pkg, '..'),
            lbr_description_pkg
        ])
    ))
    
    ld.add_action(AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        ':'.join([
            os.path.join(my_pkg, 'models'),
            os.path.join(lbr_description_pkg, '..'),
            lbr_description_pkg
        ])
    ))

    # launch arguments
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())

    # robot description
    robot_description = LBRDescriptionMixin.param_robot_description(mode="gazebo")

    # robot state publisher
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=True
    )
    ld.add_action(robot_state_publisher)

    # Launch Gazebo with custom world
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': [world_file]}.items()
        )
    )
    
    # Gazebo bridges
    ld.add_action(GazeboMixin.node_clock_bridge())
    
    # Spawn robot
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="Spawning robot in Gazebo..."),
            GazeboMixin.node_create(tf=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        ]
    )
    ld.add_action(spawn_entity)

    # Load and configure controllers with proper state transitions
    
    # Load joint state broadcaster
    load_jsb = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg="Loading joint_state_broadcaster..."),
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller',
                    '-c', '/lbr/controller_manager',
                    'joint_state_broadcaster'
                ],
                output='screen'
            )
        ]
    )
    ld.add_action(load_jsb)
    
    # Configure joint state broadcaster (unconfigured -> inactive)
    configure_jsb = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg="Configuring joint_state_broadcaster..."),
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'set_controller_state',
                    '-c', '/lbr/controller_manager',
                    'joint_state_broadcaster',
                    'inactive'
                ],
                output='screen'
            )
        ]
    )
    ld.add_action(configure_jsb)
    
    # Load main controller
    load_ctrl = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="Loading main controller..."),
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller',
                    '-c', '/lbr/controller_manager',
                    LaunchConfiguration("ctrl")
                ],
                output='screen'
            )
        ]
    )
    ld.add_action(load_ctrl)
    
    # Configure main controller (unconfigured -> inactive)
    configure_ctrl = TimerAction(
        period=9.0,
        actions=[
            LogInfo(msg="Configuring main controller..."),
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'set_controller_state',
                    '-c', '/lbr/controller_manager',
                    LaunchConfiguration("ctrl"),
                    'inactive'
                ],
                output='screen'
            )
        ]
    )
    ld.add_action(configure_ctrl)
    
    # Now activate both controllers (inactive -> active)
    activate_controllers = TimerAction(
        period=11.0,
        actions=[
            LogInfo(msg="Activating controllers..."),
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'switch_controllers',
                    '-c', '/lbr/controller_manager',
                    '--activate', 'joint_state_broadcaster', LaunchConfiguration("ctrl"),
                    '--strict'
                ],
                output='screen'
            )
        ]
    )
    ld.add_action(activate_controllers)
    
    # Verify controllers are active
    verify_controllers = TimerAction(
        period=13.0,
        actions=[
            LogInfo(msg="Verifying controller status..."),
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'list_controllers',
                    '-c', '/lbr/controller_manager'
                ],
                output='screen'
            )
        ]
    )
    ld.add_action(verify_controllers)
    
    return ld