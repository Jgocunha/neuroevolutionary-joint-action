from typing import List
import os

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.moveit import LBRMoveGroupMixin
from lbr_bringup.rviz import RVizMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin
from bringup_py.gazebo import GazeboMixin
from ament_index_python.packages import get_package_share_directory

def hidden_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRMoveGroupMixin.arg_allow_trajectory_execution())
    ld.add_action(LBRMoveGroupMixin.arg_capabilities())
    ld.add_action(LBRMoveGroupMixin.arg_disable_capabilities())
    ld.add_action(LBRMoveGroupMixin.arg_monitor_dynamics())
    ld.add_action(LBRMoveGroupMixin.args_publish_monitored_planning_scene())

    model = LaunchConfiguration("model").perform(context)
    moveit_configs_builder = LBRMoveGroupMixin.moveit_configs_builder(
        robot_name=model,
        package_name=f"{model}_moveit_config",
    )
    move_group_params = LBRMoveGroupMixin.params_move_group()

    mode = LaunchConfiguration("mode").perform(context)
    use_sim_time = False
    if mode == "gazebo":
        use_sim_time = True

    # MoveGroup
    robot_name = LaunchConfiguration("robot_name")
    ld.add_action(
        LBRMoveGroupMixin.node_move_group(
            parameters=[
                moveit_configs_builder.to_dict(),
                move_group_params,
                {"use_sim_time": use_sim_time},
            ],
            namespace=robot_name,
        )
    )

    # RViz if desired
    rviz = RVizMixin.node_rviz(
        rviz_cfg_pkg=f"{model}_moveit_config",
        rviz_cfg="config/moveit.rviz",
        parameters=LBRMoveGroupMixin.params_rviz(
            moveit_configs=moveit_configs_builder.to_moveit_configs()
        )
        + [{"use_sim_time": use_sim_time}],
        remappings=[
            (
                "display_planned_path",
                PathJoinSubstitution([robot_name, "display_planned_path"]),
            ),
            ("joint_states", PathJoinSubstitution([robot_name, "joint_states"])),
            (
                "monitored_planning_scene",
                PathJoinSubstitution([robot_name, "monitored_planning_scene"]),
            ),
            ("planning_scene", PathJoinSubstitution([robot_name, "planning_scene"])),
            (
                "planning_scene_world",
                PathJoinSubstitution([robot_name, "planning_scene_world"]),
            ),
            (
                "robot_description",
                PathJoinSubstitution([robot_name, "robot_description"]),
            ),
            (
                "robot_description_semantic",
                PathJoinSubstitution([robot_name, "robot_description_semantic"]),
            ),
            (
                "recognized_object_array",
                PathJoinSubstitution([robot_name, "recognized_object_array"]),
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    ld.add_action(rviz)
    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_mode())
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(RVizMixin.arg_rviz())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())

    robot_description = LBRDescriptionMixin.param_robot_description(mode="gazebo")
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(robot_description=robot_description, 
                                                                           use_sim_time=True)
    ld.add_action(robot_state_publisher)  
    # Gazebo
    ld.add_action(GazeboMixin.include_gazebo())
    ld.add_action(GazeboMixin.node_clock_bridge())
    ld.add_action(
        GazeboMixin.node_create(tf=[0, 0, 0, 0, 0, 0]) # world origin
    )  # spawns robot in Gazebo through robot_description topic of robot_state_publisher

    # Set multiple environment variables for better compatibility
    pkg_onrobot_description = get_package_share_directory('onrobot_description')
    workspace_install = os.path.dirname(os.path.dirname(pkg_onrobot_description))
    share_path = os.path.join(workspace_install, 'share')
    ld.add_action(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', share_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',  share_path))
    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH',      share_path))


    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(controller="joint_state_broadcaster")
    ld.add_action(joint_state_broadcaster)
    ld.add_action(
        LBRROS2ControlMixin.node_controller_spawner(
            controller=LaunchConfiguration("ctrl")
        )
    )

    ld.add_action(OpaqueFunction(function=hidden_setup))
    return ld
