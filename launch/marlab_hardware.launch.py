from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.moveit import LBRMoveGroupMixin
from bringup_py.gazebo import GazeboMixin
from lbr_bringup.rviz import RVizMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin
from launch_ros.actions import Node
from bringup_py.gazebo import GazeboMixin
from launch.event_handlers import OnProcessStart


def hidden_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

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

    # launch arguments
    ld.add_action(LBRDescriptionMixin.arg_mode())
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(RVizMixin.arg_rviz())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())  
    ld.add_action(LBRDescriptionMixin.arg_robot_name())

    ld.add_action(LBRROS2ControlMixin.arg_sys_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_sys_cfg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg())

    # robot description
    robot_description = LBRDescriptionMixin.param_robot_description(mode="hardware")

    # robot state publisher
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=False
    )
    ld.add_action(
        robot_state_publisher
    )  # Do not condition robot state publisher on joint state broadcaster as Gazebo uses robot state publisher to retrieve robot description

    # Gazebo
    #ld.add_action(GazeboMixin.include_gazebo())  # Gazebo has its own controller manager
    # ld.add_action(GazeboMixin.node_clock_bridge())
    #ld.add_action(
        #GazeboMixin.node_create(tf=[0, 0, 0, 0, 0, 0]) # world origin
        #GazeboMixin.node_create(tf=[4.687, -0.86, 1.585, -1.57, 0, 2.615]) # right arm
        #GazeboMixin.node_create(tf=[10.0389, 5.106, 1.586, 1.57, 0, -2.615]) # left arm 
    #)  # spawns robot in Gazebo through robot_description topic of robot_state_publisher

    # ros2 control node
    ros2_control_node = LBRROS2ControlMixin.node_ros2_control(
        use_sim_time=False, robot_description=robot_description
    )
    ld.add_action(ros2_control_node)

    # joint state broad caster and controller on ros2 control node start
    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster"
    )
    force_torque_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="force_torque_broadcaster"
    )
    lbr_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="lbr_state_broadcaster"
    )
    controller = LBRROS2ControlMixin.node_controller_spawner(
        controller=LaunchConfiguration("ctrl")
    )

    controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                joint_state_broadcaster,
                force_torque_broadcaster,
                lbr_state_broadcaster,
                controller,
            ],
        )
    )
    ld.add_action(controller_event_handler)

    ld.add_action(OpaqueFunction(function=hidden_setup))

    return ld
