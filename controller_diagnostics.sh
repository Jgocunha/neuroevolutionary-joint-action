#!/bin/bash

echo "=== ROS2 Control Diagnostics ==="
echo

echo "1. Checking controller manager:"
ros2 node list | grep controller_manager
echo

echo "2. Controller manager namespace:"
ros2 node info /lbr/controller_manager 2>/dev/null || echo "Controller manager not found at /lbr/controller_manager"
echo

echo "3. Available controllers:"
ros2 control list_controllers -c /lbr/controller_manager
echo

echo "4. Available hardware interfaces:"
ros2 control list_hardware_interfaces -c /lbr/controller_manager
echo

echo "5. Controller types available:"
ros2 control list_controller_types -c /lbr/controller_manager
echo

echo "6. Checking if robot is spawned in Gazebo:"
ros2 topic list | grep -E "(joint_states|lbr)"
echo

echo "7. Robot description parameter:"
ros2 param get /robot_state_publisher robot_description | head -20
echo

echo "=== To manually load and activate controllers: ==="
echo "ros2 control load_controller -c /lbr/controller_manager joint_state_broadcaster"
echo "ros2 control set_controller_state -c /lbr/controller_manager joint_state_broadcaster active"
echo
echo "ros2 control load_controller -c /lbr/controller_manager joint_trajectory_controller"
echo "ros2 control set_controller_state -c /lbr/controller_manager joint_trajectory_controller active"